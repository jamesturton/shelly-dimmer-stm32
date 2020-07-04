#include <string.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

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

uint8_t rx_data[SHD_BUFFER_SIZE] = {0};
uint8_t byte_counter = 0;

static void sleep(uint16_t time)
{
    for (int i = 0; i < (1000 * time); i++)
        __asm__("NOP");
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
		*index = 0;	// wrong data
	else
		(*index)++;  // not finished recieving yet

    return false;
}

void usart1_isr(void)
{
	static uint8_t payload[2] = {0xff, 0xf1};

	// Check if we were called because of RXNE
	if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_ISR(USART1) & USART_ISR_RXNE) != 0)) {

		// Retrieve the data from the peripheral
		if (read_serial(rx_data, &byte_counter))
		{
			// Enable transmit interrupt so it sends back the data
			USART_CR1(USART1) |= USART_CR1_TXEIE;
		}
	}

	// Check if we were called because of TXE
	if (((USART_CR1(USART1) & USART_CR1_TXEIE) != 0) &&
	    ((USART_ISR(USART1) & USART_ISR_TXE) != 0)) {

		// Put data into the transmit register
		generate_packet(rx_data[1], SHD_VERSION_CMD, 2, payload);

		// Disable the TXE interrupt as we don't need it anymore
		USART_CR1(USART1) &= ~USART_CR1_TXEIE;
	}
}

static void clock_setup(void)
{
	// Enable GPIOC clock for USART
	rcc_periph_clock_enable(RCC_GPIOA);

	// Enable clocks for USART1
	rcc_periph_clock_enable(RCC_USART1);
}

static void usart_setup(void)
{
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
	// Setup GPIO pins for USART1 transmit and receive
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO10);

	// Setup USART1 TX pin and RX pin as alternate function
	gpio_set_af(GPIOA, GPIO_AF1, GPIO9 | GPIO10);
}

int main(void)
{
	clock_setup();
	gpio_setup();
	usart_setup();

	while (1)
	{
		// Wait forever and do nothing
		__asm__("nop");
	}

	return 0;
}