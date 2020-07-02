#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

static void clock_setup(void)
{
	// Enable GPIOC clock for USART
	rcc_periph_clock_enable(RCC_GPIOA);

	// Enable clocks for USART1
	rcc_periph_clock_enable(RCC_USART1);
}

static void usart_setup(void)
{
	// Setup USART1 parameters
	usart_set_baudrate(USART1, 115200);
	usart_set_databits(USART1, 8);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_stopbits(USART1, USART_CR2_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

	// Finally enable the USART
	usart_enable(USART1);
}

static void gpio_setup(void)
{
	// Setup GPIO pins for USART1 transmit
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);

	// Setup USART1 TX pin as alternate function
	gpio_set_af(GPIOA, GPIO_AF1, GPIO9);
}

int main(void)
{
	int i;

	clock_setup();
	gpio_setup();
	usart_setup();

	while (1) {
		usart_send_blocking(USART1, 0x01); // Start byte
		usart_send_blocking(USART1, 0x02); // Counter
		usart_send_blocking(USART1, 0x11); // Command
		usart_send_blocking(USART1, 0x02); // Length
		usart_send_blocking(USART1, 0xff); // Payload
		usart_send_blocking(USART1, 0xff); // Payload
		usart_send_blocking(USART1, 0x02); // Checksum
		usart_send_blocking(USART1, 0x13); // Checksum
		usart_send_blocking(USART1, 0x04); // End byte

		for (i = 0; i < 1000000; i++) {	// Wait a bit
			__asm__("NOP");
		}
	}

	return 0;
}