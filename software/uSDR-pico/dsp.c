/*
 * dsp.c
 *
 * Created: Mar 2021
 * Author: Arjan te Marvelde
 * 
 * Signal processing of RX and TX branch, to be run on the second processor core.
 * Each branch has a dedicated routine that must run on set times.
 * The period is determined by reads from the inter-core fifo, by the dsp_loop() routine. 
 * This fifo is written from core0 from a 16us timer callback routine (i.e. 62.5kHz)
 *
 * The RX branch:
 * - Sample I and Q QSD channels, and shift into I and Q delay line (62.5 kHz per channel)
 * - Low pass filter: Fc=4kHz
 * - Quarter rate (15.625 kHz) to improve low freq behavior of Hilbert transform
 * - Calculate 15 tap Hilbert transform on Q
 * - Demodulate, taking proper delays into account
 * - Push to Audio output DAC
 *
 * Always perform audio sampling (62.5kHz) and level detections, in case of VOX active
 *
 * The TX branch (if VOX or PTT):
 * - Low pass filter: Fc=3kHz
 * - Eight rate (7.8125 kHz) to improve low F behavior of Hilbert transform
 * - Generate Q samples by doing a Hilbert transform
 * - Push I and Q to QSE output DACs
 *
 */

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "hardware/structs/bus_ctrl.h"
#include "dsp.h"
#include "hmi.h"


#define ADC0_IRQ_FIFO 		22		// FIFO IRQ number



/* 
 * DAC_RANGE defines PWM cycle, determining DAC resolution and PWM frequency.
 * DAC resolution = Vcc / DAC_RANGE
 * PWM frequency = Fsys / DAC_RANGE
 * A value of 250 means 125MHz/250=500kHz
 * ADC is 12 bit, so resolution is by definition 4096
 * To eliminate undefined behavior we clip off the upper 4 sample bits.
 */
#define DAC_RANGE	4096
#define DAC_BIAS	(DAC_RANGE/2)
#define ADC_RANGE	4096
#define ADC_BIAS	(ADC_RANGE/2)

/* 
 * Callback timeout and inter-core FIFO commands. 
 * The timer value in usec determines frequency of TX and RX loops
 * Exact time is obtained by passing the value as negative
 *	16us		62500Hz
 *	32us		31250Hz
 *	64us		15625Hz
 */
//#define DSP_USE_OUTPUT_FILTER
//#define DSP_USE_WEAVER		// demodulation mode: WEAVER or HILBERT
#ifdef DSP_USE_WEAVER
	// the weaver demodulation runs with a 31250Hz clock
	// the input lowpass filter runs with 31250Hz rate
	// the demodulation code is called with 6250Hz rate - decimation 5
	// the weaver multiplicatio runs in 4 steps -> 1562.5 center frequency
	// the weaver output filter runs at 6250Hz
	#define DSP_US		32
	volatile uint8_t sineIndex;

	//31250Hz sample rate
	int32_t in_filter[8]  = {-69, 112, 608, 1478, 2634, 3832, 4742, 5082};
	int32_t out_filter[8] = {1176, 1568, 1955, 2315, 2621, 2857, 3005, 3056};
	int16_t sine_table[20] = {0, 10125, 19260, 26509, 31163, 32767, 31163, 26509, 19260,
			10125, 0, -10126, -19261, -26510, -31164, -32768, -31164, -26510, -19261, -10126
		};
	void dsp_weaverDemodulator(int16_t i_accu, int16_t q_accu);
#else
	// the phase-shift demodulation is running with a 15625Hz clock
	#define DSP_US 		64
	//15625Hz sample rate
	int32_t in_filter[8]  = {-86, 18, 488, 1404, 2669, 3998, 5013, 5394 };
	int32_t out_filter[8] = {-42, 119, 619, 1498, 2648, 3818, 4696, 5022 };
	void dsp_hilbertDemodulation(int16_t i_accu, int16_t q_accu);
#endif

// Some macro's
// See Alpha Max plus Beta Min algorithm for MAG (vector length)
#define ABS(x)		((x)<0?-(x):(x))
#define MAG(i,q)	(ABS(i)>ABS(q) ? ABS(i)+((3*ABS(q))>>3) : ABS(q)+((3*ABS(i))>>3))

// reference for the audio output PWM
volatile uint16_t dac_audio;




// MODE is modulation/demodulation 
// This setting steers the signal processing branch chosen
volatile uint16_t dsp_mode;				// For values see hmi.c, assume {USB,LSB,AM,CW}
void dsp_setmode(int mode) {
	dsp_mode = (uint16_t)mode;
}

// CORE1: 
// ADC IRQ handler.
// Fills the results array in RR fashion, for 3 channels (~2usec per channel).
volatile uint16_t adc_result[3];
volatile int adc_next;								// Remember which ADC the result is from
void adcfifo_handler(void) {
	// Get result from fifo
	adc_result[adc_next] = adc_fifo_get();

	// Update adc_next with HW CS register
	// Shift and Mask: Only 0, 1 and 2 are valid numbers
	adc_next = adc_hw->cs;							
	adc_next = (adc_next >> ADC_CS_AINSEL_LSB) & 0b011;	
}
	

/* 
 * CORE1: 
 * Execute RX branch signal processing, max time to spend is <16us, i.e. rate is 62.5 kHz
 * No ADC sample interleaving, read both I and Q channels.
 * The delay is only 2us per conversion, which causes less distortion than interpolation of samples.
 */
volatile int16_t a_sample=0;
volatile int16_t i_s_raw[15], q_s_raw[15];			// Raw I/Q samples minus DC bias
volatile int16_t agc_gain=0;	       				// AGC gain (left-shift value)
volatile int16_t i_s[15], q_s[15];					// Filtered I/Q samples, hilbert transform
volatile int16_t i_dc, q_dc; 						// DC bias for I/Q channel
volatile int16_t audio_gain=0;
volatile int16_t audio_s[15];						// output low pass filter
volatile int16_t inputPk;
volatile int16_t outputPk;
volatile bool clip;

bool dsp_getClip(void) {
	bool oldClip = clip;
	clip = false;
	return oldClip;
}

int16_t dsp_getInputMag(void) {
	int16_t oldInputPk = inputPk;
	inputPk = 0;
	return oldInputPk;
}

int16_t dsp_getOutputMag(void) {
	int16_t oldOutputPk = outputPk;
	outputPk = 0;
	return oldOutputPk;
}

void dsp_set_audio_gain(uint16_t gain) {
	agc_gain = gain;
}
int16_t dsp_get_i_dc(void) {
	return i_dc;
}

int16_t dsp_get_q_dc(void) {
	return q_dc;
}

int16_t dsp_get_agc_gain(void) {
	return agc_gain;
}

// this functio is the RX loop
bool repeating_timer_callback_core_1_rx(struct repeating_timer *t) {
	uint16_t i;
	int16_t q_sample, i_sample;
	int32_t q_accu, i_accu, a_accu;
	(void)t;

	//gpio_put(15, true);

	/*** SAMPLING ***/
	q_sample = adc_result[0];						// Take last ADC 0 result, connected to Q input
	i_sample = adc_result[1];						// Take last ADC 1 result, connected to I input

	/*
	 * Remove DC and store new sample
	 * IIR filter: dc = a*sample + (1-a)*dc  where a = 1/128
	 * Amplitude of samples should fit inside [-2048, 2047]
	 */
	q_sample = (q_sample&0x0fff) - ADC_BIAS;		// Clip to 12 bits and subtract mid-range
	q_dc += q_sample/128 - q_dc/128;				//   then IIR running average
	q_sample -= q_dc;								//   and subtract DC
	i_sample = (i_sample&0x0fff) - ADC_BIAS;		// Same for I sample
	i_dc += i_sample/128 - i_dc/128;
	i_sample -= i_dc;

	// at this point the data is in signed int16
	// and it will be handled as s16

	// shift raw I and Q sample
	for (i=0; i<14; i++) {
		i_s_raw[i] = i_s_raw[i+1];
		q_s_raw[i] = q_s_raw[i+1];
	}
	i_s_raw[14] = i_sample;
	q_s_raw[14] = q_sample;

	//16 bit LPF implemetation
	i_accu  = (int32_t)in_filter[0]*(int32_t)(i_s_raw[ 0]+i_s_raw[14]);
	i_accu += (int32_t)in_filter[1]*(int32_t)(i_s_raw[ 1]+i_s_raw[13]);
	i_accu += (int32_t)in_filter[2]*(int32_t)(i_s_raw[ 2]+i_s_raw[12]);
	i_accu += (int32_t)in_filter[3]*(int32_t)(i_s_raw[ 3]+i_s_raw[11]);
	i_accu += (int32_t)in_filter[4]*(int32_t)(i_s_raw[ 4]+i_s_raw[10]);
	i_accu += (int32_t)in_filter[5]*(int32_t)(i_s_raw[ 5]+i_s_raw[ 9]);
	i_accu += (int32_t)in_filter[6]*(int32_t)(i_s_raw[ 6]+i_s_raw[ 8]);
	i_accu += (int32_t)in_filter[7]*(int32_t)(i_s_raw[ 7]);
	i_accu = i_accu>>16; // drop down to 16 bits
	if(ABS((int16_t)i_accu) > inputPk) {
		inputPk = ABS((int16_t)i_accu);
	}

	q_accu  = (int32_t)in_filter[0]*(int32_t)(q_s_raw[ 0]+q_s_raw[14]);
	q_accu += (int32_t)in_filter[1]*(int32_t)(q_s_raw[ 1]+q_s_raw[13]);
	q_accu += (int32_t)in_filter[2]*(int32_t)(q_s_raw[ 2]+q_s_raw[12]);
	q_accu += (int32_t)in_filter[3]*(int32_t)(q_s_raw[ 3]+q_s_raw[11]);
	q_accu += (int32_t)in_filter[4]*(int32_t)(q_s_raw[ 4]+q_s_raw[10]);
	q_accu += (int32_t)in_filter[5]*(int32_t)(q_s_raw[ 5]+q_s_raw[ 9]);
	q_accu += (int32_t)in_filter[6]*(int32_t)(q_s_raw[ 6]+q_s_raw[ 8]);
	q_accu += (int32_t)in_filter[7]*(int32_t)(q_s_raw[ 7]);
	q_accu = q_accu>>16;
	if(ABS((int16_t)q_accu) > inputPk) {
		inputPk = ABS((int16_t)q_accu);
	}

#ifdef DSP_USE_WEAVER
	dsp_weaverDemodulator((int16_t)i_accu, (int16_t)q_accu);
#else
	dsp_hilbertDemodulation((int16_t)i_accu, (int16_t)q_accu);
#endif

	int16_t audio_out;
#ifdef DSP_USE_OUTPUT_FILTER
	// output filter
	for (i=0; i<14; i++) {
		audio_s[i] = audio_s[i+1];
	}
	audio_s[14] = a_sample;
		
	a_accu  = (int32_t)out_filter[0]*(int32_t)(audio_s[ 0]+audio_s[14]);
	a_accu += (int32_t)out_filter[1]*(int32_t)(audio_s[ 1]+audio_s[13]);
	a_accu += (int32_t)out_filter[2]*(int32_t)(audio_s[ 2]+audio_s[12]);
	a_accu += (int32_t)out_filter[3]*(int32_t)(audio_s[ 3]+audio_s[11]);
	a_accu += (int32_t)out_filter[4]*(int32_t)(audio_s[ 4]+audio_s[10]);
	a_accu += (int32_t)out_filter[5]*(int32_t)(audio_s[ 5]+audio_s[ 9]);
	a_accu += (int32_t)out_filter[6]*(int32_t)(audio_s[ 6]+audio_s[ 8]);
	a_accu += (int32_t)out_filter[7]*(int32_t)(audio_s[ 7]);
	audio_out = (int16_t)(a_accu>>16); // drop down to 16 bits
#else
	audio_out = a_sample;
#endif

	// some kind of gain on the final audio data
	// something that works on int16 - signed fixed point multiplication insteand of the current way
	if (agc_gain > 0) {
		audio_out = audio_out * (agc_gain);
	} else if (agc_gain < 0) {
		audio_out = audio_out / (-agc_gain);
	}

	/*** AUDIO GENERATION ***/
	if(ABS(audio_out) > outputPk) {
		outputPk = ABS(audio_out);
	}

	// Scale and clip output 
	audio_out += DAC_BIAS;
	if (audio_out > DAC_RANGE) { // limit to the maximum value accepted by the PWM output 
		audio_out = DAC_RANGE;
		clip = true;
	} else if (audio_out<0) { // it can't go below 0 because it will wrap go 0xFFFF
		audio_out = 0;
		clip = true;
	}
	// Send to audio DAC output
	pwm_set_chan_level(dac_audio, PWM_CHAN_A, audio_out);

	// signal loop end
	gpio_put(15, false);
	return true;
}

#ifdef DSP_USE_WEAVER
void dsp_weaverDemodulator(int16_t i_accu, int16_t q_accu) {
	// use a sine-table with 20 steps => 1562.5Hz

	// in phase multiplier
	int16_t i = ((int32_t)i_accu*(int32_t)sine_table[sineIndex])>>16;

	// quadrature multiplier - 90 degrees ahead
	int16_t q = ((int32_t)q_accu*(int32_t)sine_table[(sineIndex+5)%20])>>16;

	switch (dsp_mode) {
		case 0: //USB
			a_sample = i+q;
			break;
		case 1: //LSB
			a_sample = i-q;
			break;
		case 2: //AM
			// AM demodulate: sqrt(sqr(i)+sqr(q))
			a_sample = MAG(i_accu, q_accu);
			break;
		default:
			a_sample = 0;
			break;
	}
	
	sineIndex ++;
	sineIndex = sineIndex%20;
}
#else
void dsp_hilbertDemodulation(int16_t i_accu, int16_t q_accu) {
	uint8_t i;
	int16_t qh;

	// shift the filtered samples into the demodulation buffer
	// this is used for the hilbert transformation / delay
	for (i=0; i<14; i++) {
		i_s[i] = i_s[i+1];
		q_s[i] = q_s[i+1];
	}
	i_s[14] = (int16_t)i_accu;
	q_s[14] = (int16_t)q_accu;

	// compute the Hilbert transform 
	q_accu  = (int32_t) 2980*(int32_t)(q_s[ 0]-q_s[14]);
	q_accu += (int32_t) 4172*(int32_t)(q_s[ 2]-q_s[12]);
	q_accu += (int32_t) 6953*(int32_t)(q_s[ 4]-q_s[10]);
	q_accu += (int32_t)20860*(int32_t)(q_s[ 6]-q_s[ 8]);
	qh = (int16_t)(q_accu>>16);

	switch (dsp_mode) {
		case 0: //USB
			// USB demodulate: I[15] - Qh,
			// Qh is Classic Hilbert transform 15 taps, 12 bits (see Iowa Hills calculator)
			a_sample = i_s[7] - qh;
			break;
		case 1: //LSB
			// LSB demodulate: I[7] + Qh,
			// Qh is Classic Hilbert transform 15 taps, 12 bits (see Iowa Hills calculator)
			a_sample = i_s[7] + qh;
			break;
		case 2: //AM
			// AM demodulate: sqrt(sqr(i)+sqr(q))
			// Approximated with MAG(i,q)
			a_sample = MAG(i_s[14], q_s[14]);
			break;
		default:
			a_sample = 0;
			break;
	}
}
#endif

/* 
 * CORE0: 
 * start up all DSP related data
 * 
 */
void dsp_init(void) {
	// set CORE1 bus access to high prio 
 	bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_PROC1_BITS; // Set Core 1 prio high
	
	// GP15 is used to monitor the DSP loop
	gpio_init(15);
	gpio_set_dir(15, GPIO_OUT);


	/* Initialize DACs, default mode is free running, A and B pins are output */
	gpio_set_function(22, GPIO_FUNC_PWM);			// GP22 is PWM for Audio DAC (Slice 3, Channel A)
	dac_audio = pwm_gpio_to_slice_num(22);			// Find PWM slice for GP22
	pwm_set_clkdiv_int_frac (dac_audio, 1, 0);		// clock divide by 1: 125MHz
	pwm_set_wrap(dac_audio, DAC_RANGE-1);			// Set cycle length; nr of counts until wrap, i.e. 125/DAC_RANGE MHz
	pwm_set_enabled(dac_audio, true); 				// Set the PWM running

	/* Initialize ADCs */
	// Initialize ADC to known state
	adc_init();
	// adc sample frequency is 48e6 / (1+adc_set_clkdiv) => 100KHz
	// we sample 2 chanels => actual sample rate is 50KHz
	adc_set_clkdiv(0);
	adc_gpio_init(26);								// GP26 is ADC 0 for Q channel
	adc_gpio_init(27);								// GP27 is ADC 1 for I channel
	//adc_gpio_init(28);								// GP28 is ADC 2 for Audio channel
	adc_select_input(0);							// Start with ADC0
	adc_next = 0;
	
	//adc_set_round_robin(0x01+0x02+0x04);			// Sequence ADC 0-1-2 (GP 26, 27, 28) free running
	adc_set_round_robin(0x01+0x02);
	adc_fifo_setup(true,false,1,false,false);		// IRQ for every result (fifo threshold = 1)
	adc_irq_set_enabled(true);

	// configure the ADC irq handler and set top priority
    irq_set_exclusive_handler(ADC0_IRQ_FIFO, adcfifo_handler);
	irq_set_priority(ADC0_IRQ_FIFO, 0xC0); 
	irq_set_enabled(ADC0_IRQ_FIFO, true);

	// start ADC
	adc_run(true);
	
	// create a timer pool for CORE1, alarm pool 2, max. 16 timers
	alarm_pool_t *core1pool ;
    core1pool = alarm_pool_create(1, 16) ;
	// add the rx() code to that timer pool
	struct repeating_timer timer_core_1;
	alarm_pool_add_repeating_timer_us(core1pool, -DSP_US, 
        repeating_timer_callback_core_1_rx, NULL, &timer_core_1);

	// keep core 1 busy over here
    while(1) { 		
 		sleep_ms(10000);
   }
}
