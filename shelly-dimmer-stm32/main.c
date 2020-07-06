#include <string.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

#define SHD_SWITCH_CMD              0x01
#define SHD_SWITCH_FADE_CMD         0x02
#define SHD_POLL_CMD                0x10
#define SHD_VERSION_CMD             0x11
#define SHD_SETTINGS_CMD            0x20
#define SHD_WARMUP_CMD              0x21
#define SHD_CALIBRATION1_CMD        0x30
#define SHD_CALIBRATION2_CMD        0x31

#define SHD_SWITCH_SIZE             2
#define SHD_SWITCH_FADE_SIZE        6
#define SHD_SETTINGS_SIZE           10
#define SHD_WARMUP_SIZE             4
#define SHD_CALIBRATION_SIZE        200

#define SHD_START_BYTE              0x01
#define SHD_END_BYTE                0x04

#define SHD_BUFFER_SIZE             256

static volatile uint8_t  rx_data[SHD_BUFFER_SIZE] = {0};
static volatile uint8_t  byte_counter             = 0;
static volatile uint8_t  systick_ms               = 0;
// static volatile uint32_t freq                     = 0;
static volatile uint32_t cc1if = 0, cc2if = 0, c1count = 0, c2count = 0;


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
        return 1;

    uint8_t data_length = buf[3];
    if ((4 + data_length + 3) > SHD_BUFFER_SIZE)
        return 0;

    if (index < 4 + data_length + 1)
        return 1;

    if (index == 4 + data_length + 1)
    {
        uint16_t chksm = (buf[index - 1] << 8 | buf[index]);
        uint16_t chksm_calc = checksum(&buf[1], 3 + data_length);
        if (chksm != chksm_calc)
            return 0;

        return 1;
    }

    if (index == 4 + data_length + 2 && byte == SHD_END_BYTE)
        return index;
    
    return 0;
}

static void packet_process(uint8_t* buf)
{
    return;
}

static void send_packet(uint8_t *buf, int len)
{
    for (int i = 0; i < len; i++)
        usart_send_blocking(USART1, buf[i]);
}

static void generate_packet(uint8_t id, uint8_t cmd, uint8_t len, uint8_t *payload)
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

    // calculate checksum from id and onwards
    chksm = checksum(data + 1, 3 + len); 
    data[pos++] = chksm >> 8;
    data[pos++] = chksm & 0xff;
    data[pos++] = SHD_END_BYTE;

    send_packet(data, pos);
}

static bool read_serial(uint8_t *buf, uint8_t *index)
{
    uint8_t serial_in_byte = usart_recv(USART1);
    buf[*index] = serial_in_byte;
    
    int check = check_byte(buf, *index);

    if (check > 1)
    {
        // finished
        packet_process(buf);
        *index = 0;
        return true;
    }
    else if (check == 0)
        *index = 0; // wrong data
    else
        (*index)++;  // not finished recieving yet

    return false;
}

void usart1_isr(void)
{
    static uint8_t payload[4] = {0};

    // Check if we were called because of RXNE
    if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) && ((USART_ISR(USART1) & USART_ISR_RXNE) != 0))
    {
        // Retrieve the data from the peripheral
        if (read_serial(rx_data, &byte_counter))
        {
            // Enable transmit interrupt so it sends back the data
            USART_CR1(USART1) |= USART_CR1_TXEIE;
        }
    }

    // Check if we were called because of TXE
    if (((USART_CR1(USART1) & USART_CR1_TXEIE) != 0) && ((USART_ISR(USART1) & USART_ISR_TXE) != 0))
    {
        // Put data into the transmit register
        for (int i = 0; i < 4; i++)
            payload[i] = (cc1if >> (8 * i)) & 0xff;
        generate_packet(rx_data[1], SHD_VERSION_CMD, 4, payload);

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
}

static void usart_setup(void)
{
    // Setup GPIO pins for USART1 transmit and receive
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO10);

    // Setup USART1 TX pin and RX pin as alternate function
    gpio_set_af(GPIOA, GPIO_AF1, GPIO9 | GPIO10);

    // Enable the USART2 interrupt
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
    // Setup GPIO pins for MOSFET outputs as TIM1_CH1 and TIM1_CH4 alternate functions
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8 | GPIO11);
    // gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, GPIO8 | GPIO11);
    gpio_set_af(GPIOA, GPIO_AF2, GPIO8 | GPIO11);

    // Setup GPIO pins for test pads
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO6 | GPIO7);

    // Setup GPIO pins for PWM inputs from HLW8012 as TIM2_CH1 and TIM2_CH2 alternate functions
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO0 | GPIO1);
    gpio_set_af(GPIOA, GPIO_AF2, GPIO0 | GPIO1);

    // Setup GPIO pins for SEL pin on HLW8012
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8);
}

static void systick_setup(int xms)
{
    systick_set_clocksource(STK_CSR_CLKSOURCE_EXT);
    STK_CVR = 0; // clear counter so it starts right away
    // ToDo(jamesturton): Hardcode this value
    systick_set_reload(rcc_ahb_frequency / 8 / 1000 * xms);
    systick_interrupt_enable();
    systick_counter_enable();
}

static void timer1_setup(void)
{
    timer_disable_counter(TIM1);
    rcc_periph_reset_pulse(RST_TIM1);
    timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM1, 48 * 1000 * 20 / UINT16_MAX / 2);
    // Only needed for advanced timers:
    // timer_set_repetition_counter(TIM1, 0);
    timer_enable_break_main_output(TIM1);
    timer_enable_preload(TIM1);
    timer_continuous_mode(TIM1);
    timer_set_period(TIM1, UINT16_MAX);

    timer_disable_oc_output(TIM1, TIM_OC1);
    timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM2);
    timer_enable_oc_output(TIM1, TIM_OC1);

    timer_disable_oc_output(TIM1, TIM_OC4);
    timer_set_oc_mode(TIM1, TIM_OC4, TIM_OCM_PWM2);
    timer_enable_oc_output(TIM1, TIM_OC4);

    timer_enable_counter(TIM1);
    timer_set_oc_value(TIM1, TIM_OC1, UINT16_MAX/2);
    timer_set_oc_value(TIM1, TIM_OC4, UINT16_MAX/2);
}

static void timer2_setup(void)
{
    timer_disable_counter(TIM2);
    rcc_periph_reset_pulse(RST_TIM2);
    // nvic_set_priority(NVIC_DMA1_CHANNEL3_IRQ, 2);
    nvic_enable_irq(NVIC_TIM2_IRQ);
    timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM2, 48);
    timer_ic_set_input(TIM2, TIM_IC1, TIM_IC_IN_TI1);
    timer_ic_set_input(TIM2, TIM_IC2, TIM_IC_IN_TI1);
    timer_ic_set_filter(TIM2, TIM_IC_IN_TI1, TIM_IC_CK_INT_N_2);
    timer_ic_set_prescaler(TIM2, TIM_IC1, TIM_IC_PSC_OFF);
    timer_slave_set_mode(TIM2, TIM_SMCR_SMS_RM);
    timer_slave_set_trigger(TIM2, TIM_SMCR_TS_TI1FP1);
    TIM_CCER(TIM2) &= ~(TIM_CCER_CC2P|TIM_CCER_CC2E|TIM_CCER_CC1P|TIM_CCER_CC1E);
    TIM_CCER(TIM2) |= TIM_CCER_CC2P|TIM_CCER_CC2E|TIM_CCER_CC1E;
    timer_ic_enable(TIM2, TIM_IC1);
    timer_ic_enable(TIM2, TIM_IC2);
    timer_enable_irq(TIM2, TIM_DIER_CC1IE|TIM_DIER_CC2IE);
    timer_enable_counter(TIM2);
}

int main(void)
{
    clock_setup();
    gpio_setup();
    usart_setup();

    timer1_setup();
    timer2_setup();
    systick_setup(1000);

    gpio_set(GPIOB, GPIO8);
    // gpio_set(GPIOA, GPIO8 | GPIO11);

    while (1);

    return 0;
}


// Interupts

void tim2_isr(void)
{
    uint32_t sr = TIM_SR(TIM2);

    if (sr & TIM_SR_CC1IF)
    {
        cc1if = TIM_CCR1(TIM2);
        ++c1count;
        timer_clear_flag(TIM2, TIM_SR_CC1IF);
    }
    if (sr & TIM_SR_CC2IF)
    {
        cc2if = TIM_CCR2(TIM2);
        ++c2count;
        timer_clear_flag(TIM2, TIM_SR_CC2IF);
    }
}

void sys_tick_handler(void)
{
    systick_ms++;

    // gpio_toggle(GPIOA, GPIO8 | GPIO11);
    // gpio_toggle(GPIOB, GPIO8);

    // freq = freq_temp + timer_get_counter(TIM2);

    // /* Reset the counter. This will generate one extra overflow for next measurement. */
    // /* In case of nothing got counted, manually generate a reset to keep consistency. */
    // timer_set_counter(TIM2, 1);
    // timer_set_counter(TIM2, 0);
    // freq_temp = 0;
}
