/*
 * hmi.c
 *
 * Created: Apr 2021
 * Author: Arjan te Marvelde
 * 
 * This file contains the HMI driver, processing user inputs.
 * It will also do the logic behind these, and write feedback to the LCD.
 *
 * The 4 auxiliary buttons have the following functions:
 * GP6 - Enter, confirm : Used to select menu items or make choices from a list
 * GP7 - Escape, cancel : Used to exit a (sub)menu or cancel the current action
 * GP8 - Left           : Used to move left, e.g. to select a digit
 * GP9 - Right			: Used to move right, e.g. to select a digit
 *
 * The rotary encoder (GP2, GP3) controls an up/down counter connected to some field. 
 * It may be that the encoder has a bushbutton as well, this can be connected to GP4.
 *     ___     ___
 * ___|   |___|   |___  A
 *   ___     ___     _
 * _|   |___|   |___|   B
 *
 * Encoder channel A triggers on falling edge. 
 * Depending on B level, count is incremented or decremented.
 * 
 * The PTT is connected to GP15 and will be active, except when VOX is used.
 *
 */
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "lcd.h"
#include "hmi.h"
#include "dsp.h"
#include "si5351.h"

/*
 * GPIO assignments
 */
#define GP_ENC_A	2
#define GP_ENC_B	3
#define GP_AUX_0	6 	// select frequency step - used with encoder
#define GP_AUX_1	7	// select demodulation type 
#define GP_AUX_2	8	// TBD
#define GP_AUX_3	9	// TBD
#define GP_MASK_IN	((1<<GP_ENC_A)|(1<<GP_ENC_B)|(1<<GP_AUX_0)|(1<<GP_AUX_1)|(1<<GP_AUX_2)|(1<<GP_AUX_3))

/*
 * Event flags
 */
#define GPIO_IRQ_ALL		(GPIO_IRQ_LEVEL_LOW|GPIO_IRQ_LEVEL_HIGH|GPIO_IRQ_EDGE_FALL|GPIO_IRQ_EDGE_RISE)
#define GPIO_IRQ_EDGE_ALL	(GPIO_IRQ_EDGE_FALL|GPIO_IRQ_EDGE_RISE)

/*
 * Display layout:
 *   +----------------+
 *   |USB 14074.0 R920| --> mode=USB, freq=14074.0kHz, state=Rx,S9+20dB
 *   |      Fast -10dB| --> ..., AGC=Fast, Pre=-10dB
 *   +----------------+
 * In this HMI state only tuning is possible, 
 *   using Left/Right for digit and ENC for value, Enter to commit change.
 * Press ESC to enter the submenu states (there is only one sub menu level):
 *
 * Submenu	Values								ENC		Enter			Escape	Left	Right
 * -----------------------------------------------------------------------------------------------
 * Mode		USB, LSB, AM, CW					change	commit			exit	prev	next
 * AGC		Fast, Slow, Off						change	commit			exit	prev	next
 * Pre		+10dB, 0, -10dB, -20dB, -30dB		change	commit			exit	prev	next
 * Vox		NoVOX, Low, Medium, High			change	commit			exit	prev	next
 *
 * --will be extended--
 */

/* Event definitions */
#define HMI_E_NOEVENT				0
#define HMI_E_INCREMENT				1
#define HMI_E_DECREMENT				2
#define HMI_E_FREQ_STEP_INCREMENT	3
#define HMI_E_FREQ_STEP_DECREMENT	4
#define HMI_E_DEMOD_TYPE_INCREMENT	5
#define HMI_E_DEMOD_TYPE_DECREMENT	6
#define HMI_E_AUDIO_GAIN_INCREMENT  7
#define HMI_E_AUDIO_GAIN_DECREMENT  8
#define HMI_E_BAND_INCREMENT		9
#define HMI_E_BAND_DECREMENT		10

//frequency control
#define HMI_MAXFREQ		30000000
#define HMI_MINFREQ		     100
uint32_t hmi_freq;

//frequency step control
#define HMI_FREQ_STEP_MIN 	0
#define HMI_FREQ_STEP_MAX   5
uint8_t hmi_freq_step_index = HMI_FREQ_STEP_MIN;
uint32_t hmi_freq_step[6] = { 10, 100, 1000, 10000, 100000, 1000000};


//demodulation step control
#define HMI_DEMOD_TYPE_MIN 	0
#define HMI_DEMOD_TYPE_MAX  2
char hmi_demod_type_list[HMI_DEMOD_TYPE_MAX+1][4]= {"USB\0", "LSB\0", " AM\0"};
uint8_t hmi_demod_index = HMI_DEMOD_TYPE_MIN;

//audio gain control
#define HMI_AUDIO_GAIN_MIN (-20)
#define HMI_AUDIO_GAIN_MAX (20)
int16_t hmi_audio_gain=0;

//RF band control
#define HMI_BAND_SELECT_MIN	0
#define HMI_BAND_SELECT_MAX 8
uint8_t hmi_bandSelect = 0;
char hmi_band_text[HMI_BAND_SELECT_MAX+1][17]= { 
	"80m 3.50-4.00   \0", "40m 7.00-7.30   \0", "40m 10.10-10.15 \0", "20m 14.00-14.35 \0", 
	"17m 18.06-18.17 \0", "15m 21.00-21.45 \0", "12m 24.80-25.00 \0", "10m 28.00-29.70 \0"
};
uint32_t hmi_band_startFreq[HMI_BAND_SELECT_MAX+1] = { 
	 3500000UL,  7000000UL, 10100000UL, 14000000UL,
	18060000UL, 21000000UL, 24800000UL, 28000000UL
};

// set to true when it's time to refresh
bool hmi_update;

/*
 * Some macros
 */
#ifndef MIN
	#define MIN(x, y)        ((x)<(y)?(x):(y))  // Get min value
#endif
#ifndef MAX
	#define MAX(x, y)        ((x)>(y)?(x):(y))  // Get max value
#endif

//local functions
void hmi_bandUpdate(void);
void hmi_handler(uint8_t event);
void hmi_gpio_callback(uint gpio, uint32_t events);
void hmi_evaluate(void);
bool hmi_timer_callback(struct repeating_timer *t);
void hmi_init(void);


/*
 * HMI State Machine,
 * Handle event according to current state
 * Code needs to be optimized
 */
void hmi_handler(uint8_t event) {
	if(event==HMI_E_INCREMENT) {
		if (hmi_freq < (HMI_MAXFREQ - hmi_freq_step[hmi_freq_step_index])) {	// Boundary check
			hmi_freq += hmi_freq_step[hmi_freq_step_index];						// Increment selected digit
		}
	}
	if(event==HMI_E_DECREMENT) {
		if (hmi_freq > (hmi_freq_step[hmi_freq_step_index] + HMI_MINFREQ)) {	// Boundary check
			hmi_freq -= hmi_freq_step[hmi_freq_step_index];						// Decrement selected digit
		}
	}

	if(HMI_E_FREQ_STEP_INCREMENT==event) {
		if(HMI_FREQ_STEP_MAX>hmi_freq_step_index) {
			hmi_freq_step_index++;
		}
	}
	if(HMI_E_FREQ_STEP_DECREMENT==event) {
		if(HMI_FREQ_STEP_MIN<hmi_freq_step_index) {
			hmi_freq_step_index--;
		}
	}

	// demodulation type control
	if(HMI_E_DEMOD_TYPE_INCREMENT==event) {
		if(HMI_DEMOD_TYPE_MAX>hmi_demod_index) {
			hmi_demod_index ++;
		}
		
	}
	if(HMI_E_DEMOD_TYPE_DECREMENT==event) {
		if(HMI_DEMOD_TYPE_MIN<hmi_demod_index) {
			hmi_demod_index--;
		}
	}

	// audio gain control
	if(HMI_E_AUDIO_GAIN_INCREMENT==event) {
		if(HMI_AUDIO_GAIN_MAX>hmi_audio_gain) {
			hmi_audio_gain += 1;
		}
	}
	if(HMI_E_AUDIO_GAIN_DECREMENT==event) {
		if(hmi_audio_gain > (HMI_AUDIO_GAIN_MIN)) {
			hmi_audio_gain -= 1;
		}
	}


	//frequency band select
	if(HMI_E_BAND_INCREMENT==event) {
		if(HMI_BAND_SELECT_MAX>hmi_bandSelect) {
			hmi_bandSelect += 1;
		}
		hmi_bandUpdate();
	}
	if(HMI_E_BAND_DECREMENT==event) {
		if(hmi_bandSelect > HMI_BAND_SELECT_MIN) {
			hmi_bandSelect -= 1;
		}
		hmi_bandUpdate();
	}


	//trigger a screen update
	if(HMI_E_NOEVENT!=event) {
		hmi_update = true;
	}
}

void hmi_bandUpdate(void) {
	hmi_freq = hmi_band_startFreq[hmi_bandSelect];
}

/*
 * GPIO IRQ callback routine
 * Sets the detected event and invokes the HMI state machine
 * encoder increment / decrement => frequency change
 * AUX0 + encoder increment / decrement => freq step change
 * AUX1 + encoder increment / decrement => demodulator change 
 */
void hmi_gpio_callback(uint gpio, uint32_t events) {
	uint8_t evt=HMI_E_NOEVENT;

	// all these actions take place when the encoder is used
	if(GP_ENC_A == gpio) {
		// determine how the encoder is moving
		if (events&GPIO_IRQ_EDGE_FALL) {
			evt = gpio_get(GP_ENC_B)?HMI_E_INCREMENT:HMI_E_DECREMENT;
		}

		// if AUX0 is pressed
		if(false==gpio_get(GP_AUX_0)) {
			if(HMI_E_INCREMENT==evt) {
				evt = HMI_E_FREQ_STEP_INCREMENT;
			}
			if(HMI_E_DECREMENT==evt) {
				evt = HMI_E_FREQ_STEP_DECREMENT;
			}
		}

		// if AUX1 is pressed
		if(false==gpio_get(GP_AUX_1)) {
			if(HMI_E_INCREMENT==evt) {
				evt = HMI_E_DEMOD_TYPE_INCREMENT;
			}
			if(HMI_E_DECREMENT==evt) {
				evt = HMI_E_DEMOD_TYPE_DECREMENT;
			}
		}

		// if AUX2 is pressed
		if(false==gpio_get(GP_AUX_2)) {
			if(HMI_E_INCREMENT==evt) {
				evt = HMI_E_AUDIO_GAIN_INCREMENT;
			}
			if(HMI_E_DECREMENT==evt) {
				evt = HMI_E_AUDIO_GAIN_DECREMENT;
			}
		}

		// if AUX3 is pressed
		if(false==gpio_get(GP_AUX_3)) {
			if(HMI_E_INCREMENT==evt) {
				evt = HMI_E_BAND_INCREMENT;
			}
			if(HMI_E_DECREMENT==evt) {
				evt = HMI_E_BAND_DECREMENT;
			}
		}
	}

	// process HMI events
	hmi_handler(evt);
}

/*
 * Redraw the display, representing current state
 * This function is called regularly from the main loop.
 */
void hmi_evaluate(void) {
	char s[32];
	// main update routine
	if(true == hmi_update) { // update display only when required
		//line 0: current frequency
		sprintf(s, "F:%05u.%03uHz", (uint16_t)(hmi_freq/1000), (uint16_t)(hmi_freq%1000));
		lcd_writexy(0,0, (uint8_t *)s);

		//line 1: pointer to the curret frequency update step		
		switch(hmi_freq_step_index) {
			case 0:
				sprintf(s, "         ^ ");
				break;
			case 1:
				sprintf(s, "        ^  ");
				break;
			case 2:
				sprintf(s, "      ^    ");
				break;
			case 3:
				sprintf(s, "     ^     ");
				break;
			case 4:
				sprintf(s, "    ^      ");
				break;
			case 5:
				sprintf(s, "   ^       ");
				break;
			default:
				sprintf(s, "ERROR!!!");
				break;
		}
		lcd_writexy(0,1, (uint8_t *)s);

		//line 2 : demodulation mode
		sprintf(s, "Demod: %s", hmi_demod_type_list[hmi_demod_index]);
		lcd_writexy(0,2, (uint8_t *)s);

		//line 3 : audio gain
		sprintf(s,"Ag: %04d", hmi_audio_gain);
		lcd_writexy(0,3, (uint8_t *)s);

		// line 4: inputMag
		sprintf(s,"A:%04d D:%04d %c", dsp_getInputMag(),
			dsp_getOutputMag(), dsp_getClip()?'*':' ');
		lcd_writexy(0,4, (uint8_t *)s);

		// line 5: rf band
		sprintf(s, "%s", hmi_band_text[hmi_bandSelect]);
		lcd_writexy(0,5, (uint8_t *)s);


		// Set parameters corresponding to latest entered option value
		SI_SETFREQ(0, hmi_freq);
		dsp_setmode(hmi_demod_index);
		//dsp_setagc(2);
		set_audio_gain(hmi_audio_gain);

		
		// UI update done, clear the flag
		hmi_update = false;
	}
}

// this timer will trigger a periodic UI refresh
struct repeating_timer hmi_timer;
bool hmi_timer_callback(struct repeating_timer *t) {
	(void)t;
	hmi_update = true;
	return true;
}


/*
 * Initialize the User interface
 */
void hmi_init(void) {
	/*
	 * Notes on using GPIO interrupts: 
	 * The callback handles interrupts for all GPIOs with IRQ enabled.
	 * Level interrupts don't seem to work properly.
	 * For debouncing, the GPIO pins should be pulled-up and connected to gnd with 100nF.
	 * PTT has separate debouncing logic
	 */
	 
	// Init input GPIOs
	gpio_init_mask(GP_MASK_IN);
	
	// Enable pull-ups
	gpio_pull_up(GP_ENC_A);
	gpio_pull_up(GP_ENC_B);
	gpio_pull_up(GP_AUX_0);
	gpio_pull_up(GP_AUX_1);
	gpio_pull_up(GP_AUX_2);
	gpio_pull_up(GP_AUX_3);
	
	// Enable interrupt on level low
	gpio_set_irq_enabled(GP_ENC_A, GPIO_IRQ_EDGE_ALL, true);
	//gpio_set_irq_enabled(GP_AUX_0, GPIO_IRQ_EDGE_ALL, true);
	//gpio_set_irq_enabled(GP_AUX_1, GPIO_IRQ_EDGE_ALL, true);
	//gpio_set_irq_enabled(GP_AUX_2, GPIO_IRQ_EDGE_ALL, true);
	//gpio_set_irq_enabled(GP_AUX_3, GPIO_IRQ_EDGE_ALL, true);

	// Set callback, one for all GPIO, not sure about correctness!
	gpio_set_irq_enabled_with_callback(GP_ENC_A, GPIO_IRQ_EDGE_ALL, true, hmi_gpio_callback);
		
	// Initialize LCD and set VFO
	//hmi_freq = 14000000UL;							// Initial frequency
	hmi_bandSelect = 3;
	hmi_bandUpdate();
	hmi_freq_step_index = 1;						// initial control step
	hmi_demod_index = 0;							// start with USB demodulation
	hmi_audio_gain = 0;
	
	SI_SETFREQ(0, hmi_freq);
	SI_SETPHASE(0, 1);
	dsp_setmode(hmi_demod_index);	// initial USB demodulation
	set_audio_gain(hmi_audio_gain);
	//dsp_setagc(hmi_sub[HMI_S_AGC]);	TODO	
	//hmi_update = true;

	add_repeating_timer_us(-(1000000), hmi_timer_callback, NULL, &hmi_timer);
}

/*
Display organisation:
Line 0: RX frequency, with 3 decimals
Line 1: pointer to the current frequency change step '^'
Line 2: demodulation mode and AGC mode
Line 3: RX filter width: CW / SSB / AM
Line 4: TBD
Line 5: TBD
Line 6: TBD
Line 7: TBD
*/