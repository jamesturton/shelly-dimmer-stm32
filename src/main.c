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
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

#define SHD_DRIVER_MAJOR_VERSION            50
#define SHD_DRIVER_MINOR_VERSION            1

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

static uint8_t  rx_data[SHD_BUFFER_SIZE]    = {0};
static uint8_t  byte_counter                = 0;

static uint8_t  id                          = 0;
static uint8_t  cmd                         = 0;

static uint32_t systick_ms                  = 0;
static uint32_t line_freq                   = 10000; // Guess we are at 50 Hz

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

static uint16_t brightness                  = 0;
static uint32_t brightness_adj              = 0;
static bool     leading_edge                = true;

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
                brightness = buf[pos + 1] << 8 | buf[pos + 0];
            }
            break;
        case SHD_SETTINGS_CMD:
            {
                leading_edge = buf[pos + 2] - 1;
            }
            break;
        default:
            break;  // There are other valid commands, but for now ignore them
    }
}

static void send_packet(uint8_t *buf, int len)
{
    for (int i = 0; i < len; i++)
        usart_send_blocking(USART1, buf[i]);    // Should we use async sending?
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

    send_packet(data, pos);
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
    // Power measurements are more sensitive to switch offs,
    // so we first check if power is 0 to set current_pulse_width to 0 too
    if (power_pulse_width == 0)
        current_pulse_width = 0;
    else
        check_cf1_signal();

    return current_pulse_width;
}

static uint32_t get_voltage(void)
{
    check_cf1_signal();
    return voltage_pulse_width;
}

static uint32_t get_active_power(void)
{
    check_cf_signal();
    return power_pulse_width;
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
            data[0]  = 0;                    // ??
            data[1]  = 0;                    // ??
            data[2]  = brightness & 0xff;    // brightness
            data[3]  = brightness >> 8;      // brightness
            data[4]  = wattage;              // active power
            data[5]  = wattage >> 8;         // active power
            data[6]  = wattage >> 16;        // active power
            data[7]  = wattage >> 24;        // active power
            data[8]  = voltage;              // voltage
            data[9]  = voltage >> 8;         // voltage
            data[10] = voltage >> 16;        // voltage
            data[11] = voltage >> 24;        // voltage
            data[12] = current;              // current
            data[13] = current >> 8;         // current
            data[14] = current >> 16;        // current
            data[15] = current >> 24;        // current
            data[16] = leading_edge;         // fade rate
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
            // Enable transmit interrupt so it sends back the data
            USART_CR1(USART1) |= USART_CR1_TXEIE;
        }
    }

    // Check if we were called because of TXE
    if (((USART_CR1(USART1) & USART_CR1_TXEIE) != 0) &&
        ((USART_ISR(USART1) & USART_ISR_TXE) != 0))
    {
        // Generate our reply
        generate_reply();

        // Disable the TXE interrupt as we don't need it anymore
        USART_CR1(USART1) &= ~USART_CR1_TXEIE;
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

    // Enable clocks for TIM1 and TIM2
    rcc_periph_clock_enable(RCC_TIM1);
    rcc_periph_clock_enable(RCC_TIM2);

    // Enable clocks for EXTI
    rcc_periph_clock_enable(RCC_SYSCFG_COMP);
}

static void usart_setup(void)
{
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
    // Setup GPIO pins for MOSFET outputs as TIM1_CH1 and TIM1_CH4 alternate
    // functions
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8 | GPIO11);
    // gpio_set_output_options(GPIOA, GPIO_OTYPE_PP,
    //                         GPIO_OSPEED_HIGH, GPIO8 | GPIO11);
    gpio_set_af(GPIOA, GPIO_AF2, GPIO8 | GPIO11);

    // Setup GPIO pins for test pads
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO6 | GPIO7);

    // Setup GPIO pins for PWM inputs from HLW8012 as TIM2_CH1 and TIM2_CH2
    // alternate functions
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO0 | GPIO1);
    gpio_set_af(GPIOA, GPIO_AF2, GPIO0 | GPIO1);

    // Setup GPIO pins for SEL pin on HLW8012
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, GPIO8);
    // Start in voltage measuring mode
    gpio_set(GPIOB, GPIO8);

    // Setup GPIO pins for mains detect pin
    gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO2);
}

static void systick_setup(void)
{
    // Setup clock as source external reference clock
    systick_set_clocksource(STK_CSR_CLKSOURCE_EXT);

    // Clear counter so it starts right away
    STK_CVR = 0;

    // Divide systick clock down to milliseconds
    systick_set_reload(rcc_ahb_frequency / 8 / 1000);

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
    // timer_set_period(TIM1, 2000);

    // Setup channel 1, PA8
    timer_disable_oc_output(TIM1, TIM_OC1);
    timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM2);
    timer_set_oc_polarity_high(TIM1, TIM_OC1);
    timer_enable_oc_output(TIM1, TIM_OC1);
    timer_set_oc_value(TIM1, TIM_OC1, 0xffffffff);

    // Setup channel 4, PA11
    timer_disable_oc_output(TIM1, TIM_OC4);
    timer_set_oc_mode(TIM1, TIM_OC4, TIM_OCM_PWM2);
    timer_set_oc_polarity_high(TIM1, TIM_OC4);
    timer_enable_oc_output(TIM1, TIM_OC4);
    timer_set_oc_value(TIM1, TIM_OC4, 0xffffffff);

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

static void exti_setup(void)
{
	// Enable external interrupt 2
	nvic_enable_irq(NVIC_EXTI2_3_IRQ);

	// Configure the EXTI subsystem
	exti_select_source(EXTI2, GPIOB);
	exti_set_trigger(EXTI2, EXTI_TRIGGER_BOTH);

    // Finally enable EXTI2
	exti_enable_request(EXTI2);
}

int main(void)
{
    // Setup all subsystems
    // Setup internal oscillator and peripheral clocks
    clock_setup();
    // Setup GPIO input and output modes
    gpio_setup();
    // Setup UART for comunication with ESP8266
    usart_setup();
    // Setup timer 1 for PWM outout controling output MOSFETs
    timer1_setup();
    // Setup timer 2 for measuring voltage/power data from HLW8012
    timer2_setup();
    // Setup external trigger for synchronising PWM output with zero-crossing
    // point of mains voltage
    exti_setup();
    // Setup systick for general timekeeping
    systick_setup();

    // Keep doing this forever
    while (1);

    return 0;
}


// Interupts

void tim2_isr(void)
{
    // Read timer 2 status register
    uint32_t sr = TIM_SR(TIM2);

    // Check which capture/compare interrupt triggered
    if (sr & TIM_SR_CC1IF)
    {
        // CF1 triggered

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
        // Reset interupt flag
        timer_clear_flag(TIM2, TIM_SR_CC1IF);
    }
    if (sr & TIM_SR_CC2IF)
    {
        // CF triggered

        // Calculate period of CF signal
        tim_ccr2_now = TIM_CCR2(TIM2);
        power_pulse_width = tim_ccr2_now - tim_ccr2_last;
        tim_ccr2_last = tim_ccr2_now;

        ++power_pulse_count;
        last_cf_interrupt = systick_ms;
        // Reset interupt flag
        timer_clear_flag(TIM2, TIM_SR_CC2IF);
    }
}

void sys_tick_handler(void)
{
    // Increment systick value
    systick_ms++;
}

void exti2_3_isr(void)
{
    line_freq = timer_get_counter(TIM1);

    // Change ouput polarity if needed depending on leading edge mode
    if (leading_edge)
    {
        timer_disable_oc_output(TIM1, TIM_OC1);
        timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM2);
        timer_enable_oc_output(TIM1, TIM_OC1);

        timer_disable_oc_output(TIM1, TIM_OC4);
        timer_set_oc_mode(TIM1, TIM_OC4, TIM_OCM_PWM2);
        timer_enable_oc_output(TIM1, TIM_OC4);

        brightness_adj = 1000 - brightness;
    }
    else
    {
        timer_disable_oc_output(TIM1, TIM_OC1);
        timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM1);
        timer_enable_oc_output(TIM1, TIM_OC1);

        timer_disable_oc_output(TIM1, TIM_OC4);
        timer_set_oc_mode(TIM1, TIM_OC4, TIM_OCM_PWM1);
        timer_enable_oc_output(TIM1, TIM_OC4);

        brightness_adj = brightness;
    }

    // Adjust the brigtness value so we will always be fully on when the
    // requested value is 1000
    brightness_adj = brightness_adj * 1.02;
    timer_set_oc_value(TIM1, TIM_OC1, brightness_adj);
    timer_set_oc_value(TIM1, TIM_OC4, brightness_adj);
    timer_set_counter(TIM1, 0);

    // Reset interupt request
	exti_reset_request(EXTI2);
}
