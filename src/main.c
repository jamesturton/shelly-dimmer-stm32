/* 
 * This file is part of the shelly-dimmer-stm32 project.
 * https://github.com/jamesturton/shelly-dimmer-stm32
 * Copyright (c) 2020 James Turton.
 * 
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */


#include <string.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

#define SHD_DRIVER_MAJOR_VERSION            51
#define SHD_DRIVER_MINOR_VERSION            6

#define SHD_SWITCH_CMD                      0x01
#define SHD_SWITCH_FADE_CMD                 0x02
#define SHD_POLL_CMD                        0x10
#define SHD_VERSION_CMD                     0x11
#define SHD_SETTINGS_CMD                    0x20
#define SHD_WARMUP_CMD                      0x21
#define SHD_CALIBRATION1_CMD                0x30
#define SHD_CALIBRATION2_CMD                0x31

#define SHD_SWITCH_SIZE                     2
#define SHD_SWITCH_FADE_SIZE                6
#define SHD_SETTINGS_SIZE                   10
#define SHD_WARMUP_SIZE                     4
#define SHD_CALIBRATION_SIZE                200

#define SHD_START_BYTE                      0x01
#define SHD_END_BYTE                        0x04

#define SHD_BUFFER_SIZE                     256

#define SHD_CF1_PULSE_TIMEOUT               10000
#define SHD_CF1_PULSE_MIN                   1
#define SHD_CF1_PULSE_MAX                   1000

#define ADC_NUM_CHANNELS                    2

typedef int32_t ring_size_t;

struct ring {
    uint8_t *data;
    ring_size_t size;
    uint32_t begin;
    uint32_t end;
};

struct ring     output_ring;
static uint8_t  output_ring_buffer[SHD_BUFFER_SIZE];

static uint8_t  rx_data[SHD_BUFFER_SIZE]    = {0};
static uint8_t  byte_counter                = 0;

static uint8_t  id                          = 0;
static uint8_t  cmd                         = 0;

static uint32_t systick_ms                  = 0;
static uint32_t line_freq                   = 1000 * 64; // Guess we are at 50 Hz (x 64 for enhanced precision in IIR filter)
static uint32_t line_freq_counter           = 1000; // Guess we are at 50 Hz

static uint32_t tim_ccr1_now                = 0;
static uint32_t tim_ccr1_last               = 0;
static uint32_t tim_ccr2_now                = 0;
static uint32_t tim_ccr2_last               = 0;
static uint32_t last_cf_interrupt           = 0;
static uint32_t first_cf1_interrupt         = 0;
static uint32_t last_cf1_interrupt          = 0;
static uint32_t cf1_pulse_count             = 0;
static uint32_t current_pulse_width         = 0;
static uint32_t voltage_pulse_width         = 0;
static uint32_t power_pulse_width           = 0;
static uint32_t power_pulse_count           = 0;
static bool     current_mode                = false;

typedef enum
{
    dimmer1,
    dimmer2,
}
hw_types;

static hw_types hw_version                  = dimmer2;

static uint16_t brightness                  = 0;
static uint16_t brightness_req              = 0;
static uint32_t brightness_adj              = 0;

static bool     leading_edge                = false;
static uint32_t low_brightness_threshold    = 0; // switch on the mosfets later for low brightness (default to 0)

static uint16_t adc_data[ADC_NUM_CHANNELS]  = {0};
static uint8_t  adc_channels[ADC_NUM_CHANNELS] =
{
    3,                  // ADC_CHANNEL3 - Live pin sense
    8,                  // ADC_CHANNEL8 - Current sense
};

static uint16_t adc_count                   = 0;
static uint16_t current_max                 = 0;
static uint16_t current_max_period          = 0;
static uint32_t current_total               = 0;
static float    current_total_mag_period    = 0;
static uint32_t current_total_mag           = 0;
static uint16_t voltage_max                 = 0;
static uint16_t voltage_max_period          = 0;
static uint32_t voltage_total               = 0;
static uint16_t max_brightness              = 500;

static void ring_init(struct ring *ring, uint8_t *buf, ring_size_t size)
{
    ring->data = buf;
    ring->size = size;
    ring->begin = 0;
    ring->end = 0;
}

static int32_t ring_write_ch(struct ring *ring, uint8_t ch)
{
    if (((ring->end + 1) % ring->size) != ring->begin)
    {
        ring->data[ring->end++] = ch;
        ring->end %= ring->size;
        return (uint32_t)ch;
    }

    return -1;
}

static int32_t ring_read_ch(struct ring *ring, uint8_t *ch)
{
    int32_t ret = -1;

    if (ring->begin != ring->end)
    {
        ret = ring->data[ring->begin++];
        ring->begin %= ring->size;
        if (ch)
            *ch = ret;
    }

    return ret;
}

static uint16_t checksum(uint8_t *buf, int len)
{
    uint16_t chksm = 0;
    for (uint8_t i = 0; i < len; i++)
        chksm += buf[i];
    return chksm;
}

static int check_byte(uint8_t *buf, uint8_t index)
{
    uint8_t byte = buf[index];

    if (index == 0)
        return byte == SHD_START_BYTE;

    if (index < 4)
        return 1;   // We can't do any checking yet

    uint8_t data_length = buf[3];
    if ((4 + data_length + 3) > SHD_BUFFER_SIZE)
        return 0;   // Expected packet length too long to handle

    if (index < 4 + data_length + 1)
        return 1;   // Again, we don't have any more to check

    if (index == 4 + data_length + 1)
    {
        // Compare checksum matches what it should do
        uint16_t chksm = (buf[index - 1] << 8 | buf[index]);
        uint16_t chksm_calc = checksum(&buf[1], 3 + data_length);
        if (chksm != chksm_calc)
            return 0;

        return 1;
    }

    if ((index == (4 + data_length + 2)) && (byte == SHD_END_BYTE))
        return index;   // Check end byte is valid
    
    return 0;
}

static void packet_process(uint8_t *buf)
{
    uint8_t pos = 1; // Skip start byte
    uint8_t len = 0;

    id = buf[pos++];
    cmd = buf[pos++];
    len = buf[pos++];

    (void)len;  // We don't use the packet length here, but maybe in the future
    
    switch (cmd)
    {
        case SHD_SWITCH_CMD:
            {
                brightness_req = buf[pos + 1] << 8 | buf[pos + 0];
            }
            break;
        case SHD_SETTINGS_CMD:
            {
                leading_edge = 2 - buf[pos + 2];
                low_brightness_threshold = buf[pos + 7] << 8 | buf[pos + 6]; // this is warmup_brightness
            }
            break;
        default:
            break;  // There are other valid commands, but for now ignore them
    }
}

static void generate_packet(uint8_t len, uint8_t *payload)
{
    // 4 bytes pre-amble, 3 bytes post-amble
    uint8_t data[4 + len + 3];
    uint16_t chksm;
    uint8_t pos = 0;

    data[0] = SHD_START_BYTE;
    data[1] = id;
    data[2] = cmd;
    data[3] = len;

    pos += 4;

    if (payload)
    {
        memcpy(data + 4, payload, len);
        pos += len;
    }

    // Calculate checksum from id and onwards
    chksm = checksum(data + 1, 3 + len); 
    data[pos++] = chksm >> 8;
    data[pos++] = chksm & 0xff;
    data[pos++] = SHD_END_BYTE;

    for (int i = 0; i < pos; i++)
        ring_write_ch(&output_ring, data[i]);
}

static void check_cf_signal(void)
{
    // Have we timed out waiting for CF pulse?
    if ((systick_ms - last_cf_interrupt) > SHD_CF1_PULSE_TIMEOUT)
        power_pulse_width = 0;
}

static void check_cf1_signal(void)
{
    // Have we timed out waiting for CF1 pulse?
    if ((systick_ms - last_cf1_interrupt) > SHD_CF1_PULSE_TIMEOUT)
    {
        if (current_mode)
            current_pulse_width = 0;
        else
            voltage_pulse_width = 0;

        // Check the other current mode instead
        current_mode = !current_mode;
        if (current_mode)
            gpio_clear(GPIOB, GPIO8);
        else
            gpio_set(GPIOB, GPIO8);

        last_cf1_interrupt = first_cf1_interrupt = systick_ms;
    }
}

static uint32_t get_current(void)
{
    if (hw_version == dimmer1)
    {
        // Power measurements are more sensitive to switch offs,
        // so we first check if power is 0 to set current_pulse_width to 0 too
        if (power_pulse_width == 0)
            current_pulse_width = 0;
        else
            check_cf1_signal();

        return current_pulse_width;
    }
    else
    {
        // Todo(jamesturton): Fix these magic numbers!
        if (adc_data[1] == 0)
            return 0;
        else
            return (1448 * 556) / (float)current_total_mag_period;
    }
}

static uint32_t get_voltage(void)
{
    if (hw_version == dimmer1)
    {
        check_cf1_signal();
        return voltage_pulse_width;
    }
    else
    {
        // Todo(jamesturton): Fix these magic numbers!
        if (adc_data[0] == 0)
            return 0;
        else
            return (347800 * 15.5) / (float)voltage_max_period;
    }
}

static uint32_t get_active_power(void)
{
    if (hw_version == dimmer1)
    {
        check_cf_signal();
        return power_pulse_width;
    }
    else
    {
        // Todo(jamesturton): Fix these magic numbers!
        if ((adc_data[0] == 0) || (adc_data[1] == 0))
            return 0;
        else
            return ((float)880373 * (float)15.5 * (float)556) / (float)((float)voltage_max_period * (float)current_total_mag_period);
    }
}

static void generate_reply(void)
{
    uint8_t data[SHD_BUFFER_SIZE] = {0};
    uint8_t len = 0;
    uint32_t voltage = get_voltage();
    uint32_t current = get_current();
    uint32_t wattage = get_active_power();

    // Construct our packet depending on the cmd
    switch (cmd)
    {
    case SHD_SWITCH_CMD:
    case SHD_SETTINGS_CMD:
        {
            len = 1;
            data[0] = 0x01;
        }
        break;
    
    case SHD_VERSION_CMD:
        {
            len = 2;
            data[0] = SHD_DRIVER_MINOR_VERSION;
            data[1] = SHD_DRIVER_MAJOR_VERSION;
        }
        break;

    case SHD_POLL_CMD:
        {
            len      = 17;
            data[0]  = hw_version;              // ??
            data[1]  = 0;                       // ??
            data[2]  = brightness_req & 0xff;   // brightness
            data[3]  = brightness_req >> 8;     // brightness
            data[4]  = wattage;                 // active power
            data[5]  = wattage >> 8;            // active power
            data[6]  = wattage >> 16;           // active power
            data[7]  = wattage >> 24;           // active power
            data[8]  = voltage;                 // voltage
            data[9]  = voltage >> 8;            // voltage
            data[10] = voltage >> 16;           // voltage
            data[11] = voltage >> 24;           // voltage
            data[12] = current;                 // current
            data[13] = current >> 8;            // current
            data[14] = current >> 16;           // current
            data[15] = current >> 24;           // current
            data[16] = leading_edge;            // leading edge
        }
        break;

    default:
        return;
    }

    // Now send it off
    generate_packet(len, data);
}

static bool read_serial(uint8_t *buf, uint8_t *index)
{
    uint8_t serial_in_byte = usart_recv(USART1);
    buf[*index] = serial_in_byte;
    
    int check = check_byte(buf, *index);

    if (check > 1)
    {
        // Finished
        packet_process(buf);
        *index = 0;
        return true;
    }
    else if (check == 0)
        *index = 0; // Wrong data, reset byte_counter
    else
        (*index)++; // Not finished recieving yet, increment byte_counter

    return false;
}

void usart1_isr(void)
{
    // Check if we were called because of RXNE
    if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) &&
        ((USART_ISR(USART1) & USART_ISR_RXNE) != 0))
    {
        // Retrieve the data from the peripheral
        if (read_serial(rx_data, &byte_counter))
        {
            // Generate our reply
            generate_reply();

            // Enable transmit interrupt so it sends back the data
            USART_CR1(USART1) |= USART_CR1_TXEIE;
        }
    }

    // Check if we were called because of TXE
    if (((USART_CR1(USART1) & USART_CR1_TXEIE) != 0) &&
        ((USART_ISR(USART1) & USART_ISR_TXE) != 0))
    {
        int32_t data;

        data = ring_read_ch(&output_ring, NULL);

        if (data == -1)
        {
            // Disable the TXE interrupt as we don't need it anymore
            USART_CR1(USART1) &= ~USART_CR1_TXEIE;
        }
        else
        {
            // Put data into the transmit register
            usart_send(USART1, data);
        }
    }
}

static void clock_setup(void)
{
    // Enable 48 MHz high speed internal oscillator
    rcc_clock_setup_in_hsi_out_48mhz();

    // Enable GPIOA and GPIOB clocks
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);

    // Enable clocks for USART1
    rcc_periph_clock_enable(RCC_USART1);

    // Enable clocks for TIM1, TIM2 and TIM3
    rcc_periph_clock_enable(RCC_TIM1);
    rcc_periph_clock_enable(RCC_TIM2);
    rcc_periph_clock_enable(RCC_TIM3);

    // Enable clocks for EXTI
    rcc_periph_clock_enable(RCC_SYSCFG_COMP);

    // Enable clocks for ADC1
    rcc_periph_clock_enable(RCC_ADC1);

    // Enable clocks for DMA
    rcc_periph_clock_enable(RCC_DMA);
}

static void usart_setup(void)
{
    // Initialize output ring buffer
    ring_init(&output_ring, output_ring_buffer, SHD_BUFFER_SIZE);

    // Setup GPIO pins for USART1 transmit and receive
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO10);

    // Setup USART1 TX pin and RX pin as alternate function
    gpio_set_af(GPIOA, GPIO_AF1, GPIO9 | GPIO10);

    // Enable the USART1 interrupt
    nvic_set_priority(NVIC_USART1_IRQ, 255);
    nvic_enable_irq(NVIC_USART1_IRQ);

    // Setup USART1 parameters
    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 8);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_stopbits(USART1, USART_CR2_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

    // Enable USART1 receive interrupt
    USART_CR1(USART1) |= USART_CR1_RXNEIE;

    // Finally enable the USART
    usart_enable(USART1);
}

static void gpio_setup(void)
{
    // Setup GPIO pins for MOSFET outputs on both HW1 and HW2
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                    GPIO8 | GPIO11 | GPIO12);
    gpio_set(GPIOA, GPIO11);

    // Setup GPIO pins for test pads
    // gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO6 | GPIO7);

    // Setup GPIO pins for PWM inputs from HLW8012 as TIM2_CH1 and TIM2_CH2
    // alternate functions
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLDOWN, GPIO0 | GPIO1);
    gpio_set_af(GPIOA, GPIO_AF2, GPIO0 | GPIO1);

    // Setup GPIO pins for SEL pin on HLW8012
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, GPIO8);
    // Start in voltage measuring mode
    gpio_set(GPIOB, GPIO8);

    // Setup GPIO pins for mains detect pins
    gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO2);
    gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO7);

    // Setup analog GPIO pins
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO3); // ADC_IN3
    gpio_mode_setup(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0); // ADC_IN8
}

static void systick_setup(void)
{
    // Setup clock as source external reference clock
    systick_set_clocksource(STK_CSR_CLKSOURCE_EXT);

    // Clear counter so it starts right away
    STK_CVR = 0;

    // Divide systick clock down to milliseconds
    systick_set_reload(rcc_ahb_frequency / 8 / 10000);

    // Enable systick interrupt
    systick_interrupt_enable();

    // Finally enable the systick timer
    systick_counter_enable();
}

static void timer1_setup(void)
{
    // Disable and reset timer 1
    timer_disable_counter(TIM1);
    rcc_periph_reset_pulse(RST_TIM1);

    // Enable timer 1 interupt
    nvic_enable_irq(NVIC_TIM1_CC_IRQ);

    // Initialise timer 1 as up counting on clock edge
    timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

    // Divide systick clock down to 100 kHz
    timer_set_prescaler(TIM1, 480);

    // Only needed for advanced timers:
    // timer_set_repetition_counter(TIM1, 0);
    timer_enable_break_main_output(TIM1);

    // Configure timer 1 with preload and continuous mode
    timer_enable_preload(TIM1);
    timer_continuous_mode(TIM1);
    timer_set_period(TIM1, 65535);

    timer_enable_irq(TIM1, TIM_DIER_CC1IE);
    timer_enable_irq(TIM1, TIM_DIER_CC2IE);

    // Finally enable timer 1
    timer_enable_counter(TIM1);
}

static void timer2_setup(void)
{
    // Disable and reset timer 2
    timer_disable_counter(TIM2);
    rcc_periph_reset_pulse(RST_TIM2);

    // Enable timer 2 interupt
    nvic_enable_irq(NVIC_TIM2_IRQ);

    // Initialise timer 2 as up counting on clock edge
    timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

    // Configure timer 2 with preload and continuous mode
    timer_set_prescaler(TIM2, 48);
    timer_enable_preload(TIM2);
    timer_continuous_mode(TIM2);

    // Clock timer 2 with internal clock, don't use external trigger
    timer_slave_set_mode(TIM2, TIM_SMCR_SMS_OFF);

    // Setup channel 1, PA0
    timer_ic_set_input(TIM2, TIM_IC1, TIM_IC_IN_TI1);
    timer_ic_set_filter(TIM2, TIM_IC_IN_TI1, TIM_IC_CK_INT_N_2);
    timer_ic_set_prescaler(TIM2, TIM_IC1, TIM_IC_PSC_OFF);
    timer_disable_oc_output(TIM2, TIM_OC1);
    timer_set_oc_polarity_high(TIM2, TIM_OC1);
    timer_enable_oc_output(TIM2, TIM_OC1);
    timer_ic_enable(TIM2, TIM_IC1);
    timer_enable_irq(TIM2, TIM_DIER_CC1IE);
    
    // Setup channel 2, PA1
    timer_ic_set_input(TIM2, TIM_IC2, TIM_IC_IN_TI2);
    timer_ic_set_filter(TIM2, TIM_IC_IN_TI2, TIM_IC_CK_INT_N_2);
    timer_ic_set_prescaler(TIM2, TIM_IC2, TIM_IC_PSC_OFF);
    timer_disable_oc_output(TIM2, TIM_OC2);
    timer_set_oc_polarity_high(TIM2, TIM_OC2);
    timer_enable_oc_output(TIM2, TIM_OC2);
    timer_ic_enable(TIM2, TIM_IC2);
    timer_enable_irq(TIM2, TIM_DIER_CC2IE);

    // Set timer so it resets right away
    timer_set_counter(TIM2, 0xffffffff);

    // Finally enable timer 2
    timer_enable_counter(TIM2);
}

static void timer3_setup(void)
{
    // Disable and reset timer 3
    timer_disable_counter(TIM3);
    rcc_periph_reset_pulse(RST_TIM3);

    // Enable timer 3 interupt
    nvic_enable_irq(NVIC_TIM3_IRQ);

    // Initialise timer 3 as up counting on clock edge
    timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

    // Divide systick clock down to 100 kHz
    timer_set_prescaler(TIM3, 480);

    // Configure timer 3 with preload and continuous mode
    timer_enable_preload(TIM3);
    timer_continuous_mode(TIM3);
    timer_set_period(TIM3, 10);
    timer_set_oc_value(TIM3, TIM_OC1, 5);

    // timer_set_master_mode(TIM3, TIM_CR2_MMS_UPDATE);

    timer_enable_irq(TIM3, TIM_DIER_CC1IE);

    // Finally enable timer 3
    timer_enable_counter(TIM3);
}

static void exti_setup(void)
{
    // Enable external interrupts 2 and 7
    nvic_enable_irq(NVIC_EXTI2_3_IRQ);
    nvic_enable_irq(NVIC_EXTI4_15_IRQ);

    // Configure the EXTI subsystem
    exti_select_source(EXTI2, GPIOB);
    exti_select_source(EXTI7, GPIOB);

    exti_set_trigger(EXTI2, EXTI_TRIGGER_BOTH);
    exti_set_trigger(EXTI7, EXTI_TRIGGER_BOTH);

    // Finally enable EXTI2 and EXTI7
    exti_enable_request(EXTI2);
    exti_enable_request(EXTI7);
}

static void adc_setup(void)
{
    adc_power_off(ADC1);

    adc_set_operation_mode(ADC1, ADC_MODE_SCAN);
    adc_set_single_conversion_mode(ADC1);
    adc_disable_external_trigger_regular(ADC1);
    // TRG1 = TIM1_CC4
    // adc_enable_external_trigger_regular(ADC1, ADC_CFGR1_EXTSEL_VAL(3), ADC_CFGR1_EXTEN_RISING_EDGE);
    adc_set_right_aligned(ADC1);
    adc_enable_temperature_sensor();
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPTIME_071DOT5);
    adc_set_regular_sequence(ADC1, ADC_NUM_CHANNELS, adc_channels);
    adc_set_resolution(ADC1, ADC_RESOLUTION_12BIT);
    adc_disable_analog_watchdog(ADC1);

    adc_calibrate(ADC1);

    // Configure DMA for ADC
    // nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);

    dma_channel_reset(DMA1, DMA_CHANNEL1);
    dma_set_priority(DMA1, DMA_CHANNEL1, DMA_CCR_PL_LOW);
    dma_set_memory_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_16BIT);
    dma_set_peripheral_size(DMA1, DMA_CHANNEL1, DMA_CCR_PSIZE_16BIT);
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL1);
    dma_enable_circular_mode(DMA1, DMA_CHANNEL1);
    dma_set_read_from_peripheral(DMA1, DMA_CHANNEL1);
    dma_set_peripheral_address(DMA1, DMA_CHANNEL1, (uint32_t) &ADC1_DR);
    // The array adc_data[] is filled with the waveform data to be output
    dma_set_memory_address(DMA1, DMA_CHANNEL1, (uint32_t) adc_data);
    dma_set_number_of_data(DMA1, DMA_CHANNEL1, ADC_NUM_CHANNELS);
    // dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL1);
    dma_enable_channel(DMA1, DMA_CHANNEL1);

    adc_enable_dma(ADC1);

    adc_power_on(ADC1);

    adc_start_conversion_regular(ADC1);
}

static void mosfet_on(void) {
    // Turn on the MOSFETs
    if(leading_edge != (bool)(hw_version == dimmer1))
    {
        gpio_set(GPIOA, GPIO8);
        gpio_set(GPIOA, GPIO11);
        gpio_set(GPIOA, GPIO12);
    }
    else
    {
        gpio_clear(GPIOA, GPIO8);
        gpio_clear(GPIOA, GPIO11);
        gpio_clear(GPIOA, GPIO12);
    }
}

static void mosfet_off(void) {
    if(leading_edge != (bool)(hw_version == dimmer1))
    {
        gpio_clear(GPIOA, GPIO8);
        gpio_clear(GPIOA, GPIO11);
        gpio_clear(GPIOA, GPIO12);
    }
    else
    {
        gpio_set(GPIOA, GPIO8);
        gpio_set(GPIOA, GPIO11);
        gpio_set(GPIOA, GPIO12);
    }
}

static void on_trigger(uint32_t gpio_bank, uint32_t gpio_pin)
{
    // Have we triggered too early? If so return straight away
    if (timer_get_counter(TIM1) < 750)
        return;

    if (gpio_get(gpio_bank, gpio_pin))
        line_freq_counter = timer_get_counter(TIM1);
    else
    {
        line_freq_counter = (line_freq_counter + timer_get_counter(TIM1)) / 2;
        line_freq = line_freq - line_freq / 64 + line_freq_counter;
    }

    // Change ouput polarity if needed depending on leading edge mode
    brightness = brightness_req * max_brightness / 1000;
    brightness = leading_edge ? 1000 - brightness : brightness;

    // Adjust the brigtness value according to the mains frequency
    brightness_adj = brightness * line_freq / 64000;
    if (brightness_adj < low_brightness_threshold)
    {
        timer_set_oc_value(TIM1, TIM_OC1, low_brightness_threshold);
        timer_set_oc_value(TIM1, TIM_OC2, low_brightness_threshold - brightness_adj);
    }
    else if (brightness_adj > 0)
    {
        timer_set_oc_value(TIM1, TIM_OC1, brightness_adj);
        timer_set_oc_value(TIM1, TIM_OC2, 0);
        mosfet_on();
    }
    timer_set_counter(TIM1, 0);

    // Do the rest after setting up the timer, as this may cause jitter

    // Update only once per full line cycle
    if (gpio_get(gpio_bank, gpio_pin))
    {
        current_max_period = current_total / adc_count;
        current_total = 0;
        current_total_mag_period = current_total_mag / adc_count;
        current_total_mag = 0;
        voltage_max_period = voltage_total / adc_count;
        voltage_total = 0;
        adc_count = 0;
    }
}

int main(void)
{
    // Setup all subsystems
    // Setup internal oscillator and peripheral clocks
    clock_setup();
    // Setup GPIO input and output modes
    gpio_setup();
    // Setup ADC for measuring voltage/current on HW 2
    adc_setup();
    // Setup UART for comunication with ESP8266
    usart_setup();
    // Setup timer 1 for PWM outout controling output MOSFETs
    timer1_setup();
    // Setup timer 2 for measuring voltage/power data from HLW8012
    timer2_setup();
    // Setup timer 3 for measuring voltage/current data ADC
    timer3_setup();
    // Setup external trigger for synchronising PWM output with zero-crossing
    // point of mains voltage
    exti_setup();
    // Setup systick for general timekeeping
    systick_setup();

    // Keep doing this forever
    while (1)
        __asm__("nop");

    return 0;
}

// Interupts

void sys_tick_handler(void)
{
    // Increment systick value
    systick_ms++;
}

void tim1_cc_isr(void)
{
    if (timer_get_flag(TIM1, TIM_SR_CC1IF))
    {
        // Reset interupt flag
        timer_clear_flag(TIM1, TIM_SR_CC1IF);

        // No need to turn off if brightness is full
        if (brightness == 1000)
            return;

        // Turn off the MOSFETs
        mosfet_off();
    }

    if (timer_get_flag(TIM1, TIM_SR_CC2IF))
    {
        // Reset interupt flag
        timer_clear_flag(TIM1, TIM_SR_CC2IF);
        if (brightness == 0)
            return;
        mosfet_on();
    }
}

void tim2_isr(void)
{
    // Check which capture/compare interrupt triggered
    if (timer_get_flag(TIM2, TIM_SR_CC1IF))
    {
        // CF1 triggered
        // Reset interupt flag
        timer_clear_flag(TIM2, TIM_SR_CC1IF);

        hw_version = dimmer1;
        max_brightness = 1000;

        // Calculate period of CF1 signal
        tim_ccr1_now = TIM_CCR1(TIM2);
        uint32_t pulse_width = tim_ccr1_now - tim_ccr1_last;
        tim_ccr1_last = tim_ccr1_now;
        uint32_t now = systick_ms;

        // Update value for voltage or current reading
        if ((last_cf1_interrupt != first_cf1_interrupt) &&
            (cf1_pulse_count > SHD_CF1_PULSE_MIN))
        {
            if (current_mode)
                current_pulse_width = pulse_width;
            else
                voltage_pulse_width = pulse_width;
        }

        cf1_pulse_count++;

        // Check if it is time to swap between measuring voltage and current
        if (((now - first_cf1_interrupt) > SHD_CF1_PULSE_TIMEOUT) ||
            (cf1_pulse_count > SHD_CF1_PULSE_MAX))
        {
            // Check the other current mode instead
            current_mode = !current_mode;
            if (current_mode)
                gpio_clear(GPIOB, GPIO8);
            else
                gpio_set(GPIOB, GPIO8);

            first_cf1_interrupt = now;
            cf1_pulse_count = 0;
        }

        last_cf1_interrupt = now;
    }
    if (timer_get_flag(TIM2, TIM_SR_CC2IF))
    {
        // CF triggered
        // Reset interupt flag
        timer_clear_flag(TIM2, TIM_SR_CC2IF);

        // Calculate period of CF signal
        tim_ccr2_now = TIM_CCR2(TIM2);
        power_pulse_width = tim_ccr2_now - tim_ccr2_last;
        tim_ccr2_last = tim_ccr2_now;

        ++power_pulse_count;
        last_cf_interrupt = systick_ms;
    }
}

void tim3_isr(void)
{
    // Check if capture/compare interrupt triggered
    if (timer_get_flag(TIM3, TIM_SR_CC1IF))
    {
        // Reset interupt flag
        timer_clear_flag(TIM3, TIM_SR_CC1IF);

        // Check if ADC has finished converting
        if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL1, DMA_TCIF))
        {
            dma_clear_interrupt_flags(DMA1, DMA_CHANNEL1, DMA_TCIF);

            voltage_total += adc_data[0];
            current_total += adc_data[1];
            if (adc_data[1] > current_max_period)
                current_total_mag += adc_data[1] - current_max_period;
            else
                current_total_mag += current_max_period - adc_data[1];
            adc_count++;

            adc_start_conversion_regular(ADC1);
        }
    }
}

void exti2_3_isr(void)
{
    if (!exti_get_flag_status(EXTI2))
        return;

    // Reset interupt request
    exti_reset_request(EXTI2);

    on_trigger(GPIOB, GPIO2);
}

void exti4_15_isr(void)
{
    if (!exti_get_flag_status(EXTI7))
        return;

    // Reset interupt request
    exti_reset_request(EXTI7);

    // We must be on dimmer2
    // Ignore EXTI2 interupts so we don't trigger twice
    exti_disable_request(EXTI2);

    on_trigger(GPIOB, GPIO7);
}
