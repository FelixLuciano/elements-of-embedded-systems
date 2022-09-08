#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

// LED
#define LED_PIO PIOA
#define LED_PIO_ID ID_PIOA
#define LED_ID 0
#define LED_ID_MASK (1u << LED_ID)

// Buttons
#define BUT_1_PIO PIOD
#define BUT_1_PIO_ID ID_PIOD
#define BUT_1_ID 28
#define BUT_1_ID_MASK (1u << BUT_1_ID)

#define BUT_2_PIO PIOC
#define BUT_2_PIO_ID ID_PIOC
#define BUT_2_ID 31
#define BUT_2_ID_MASK (1u << BUT_2_ID)


void clear_led() {
	pio_set(LED_PIO, LED_ID_MASK);
}
void set_led() {
	pio_clear(LED_PIO, LED_ID_MASK);
}

void pisca_led(int freq) {
	clear_led();

	if (freq == 0) {
		return;
	}

	delay_us(1000000.0 / freq / 2.0);
	set_led();
	delay_us(1000000.0 / freq / 2.0);
}

void draw_progressbar_label(const char *str) {
	gfx_mono_draw_string(str, 0, 0, &sysfont);
}

void draw_progressbar_value(int value) {
	int width = (value * 125) / 100;

	gfx_mono_draw_filled_rect(2 + width, 17, 125 - width, 14, GFX_PIXEL_CLR);

	for (int x = 2; x <= width; x++) {
		for (int y = 18; y <= 29; y++) {
			if ((x + y) % 8 > 1) {
				gfx_mono_draw_pixel(x, y, GFX_PIXEL_SET);
			}
			else {
				gfx_mono_draw_pixel(x, y, GFX_PIXEL_CLR);
			}
		}
	}
}

int blink_frequency = 10;
void set_blink_frequency(int frequency) {
	char str[13];
	
	blink_frequency = frequency;

	sprintf(str, "%d Hz", frequency);
	draw_progressbar_label(str);
}

int get_but_1() {
	return pio_get(BUT_1_PIO, PIO_INPUT, BUT_1_ID_MASK);
}

int get_but_2() {
	return pio_get(BUT_2_PIO, PIO_INPUT, BUT_2_ID_MASK);
}

void draw_progressbar_frame() {
	gfx_mono_draw_rect(0, 16, 128, 16, GFX_PIXEL_SET);
	gfx_mono_draw_pixel(0, 16, GFX_PIXEL_CLR);
	gfx_mono_draw_pixel(0, 31, GFX_PIXEL_CLR);
	gfx_mono_draw_pixel(127, 16, GFX_PIXEL_CLR);
	gfx_mono_draw_pixel(127, 31, GFX_PIXEL_CLR);
}

int main(void) {
	// Init Board
	board_init();
	sysclk_init();
	delay_init();

	// Init LED
	pmc_enable_periph_clk(LED_PIO);
	pio_set_output(LED_PIO, LED_ID_MASK, PIO_OUTPUT_1, 0, 0);

	// Init Buttons
	pmc_enable_periph_clk(BUT_1_PIO);
	pio_set_input(BUT_1_PIO, BUT_1_ID_MASK, PIO_PULLUP | PIO_DEGLITCH);

	pmc_enable_periph_clk(BUT_2_PIO);
	pio_set_input(BUT_2_PIO, BUT_2_ID_MASK, PIO_PULLUP | PIO_DEGLITCH);

	// Init OLED
	gfx_mono_ssd1306_init();
	draw_progressbar_frame();

	set_blink_frequency(blink_frequency);
	long but_1_state = 0;
	long but_1_count = 0;
	signed long led_count = 1;

	while (1) {
		if (!get_but_1()) {
			but_1_state = 1;
			but_1_count++;
			led_count = 1;
		}
		else if (but_1_state) {
			if (but_1_count < 10) {
				set_blink_frequency(blink_frequency+10);
			}
			else if (blink_frequency > 0){
				set_blink_frequency(blink_frequency-10);
			}
			but_1_count = 0;
			but_1_state = 0;
		}

		if (!get_but_2()) {
			but_1_count = 0;
			but_1_state = 0;
			led_count = 0;
		}

		if (led_count > 0 && led_count <= 100) {
			pisca_led(blink_frequency);
			draw_progressbar_value(led_count);
			led_count++;
		}
		else {
			clear_led();
		}
	}
}
