//
// Simple thermostat controller
//

#include "msp430.h"
#include "flash.h"
#include "temp.h"
#include <stdint.h>

#define BTN BIT3
#define LED1 BIT0
#define LED2 BIT6
#define HEATER LED1

#define P1T (BIT4|BIT5)
#define P2T (BIT0|BIT1|BIT2)

// Button events
typedef enum {
	btn_none,
	btn_long_pressed,
	btn_released,
} btn_evt_t;

// UI state
typedef enum {
	ui_idle,    // idle mode
	ui_monitor, // temperature monitoring
	ui_setting, // threshold level setting
} ui_state_t;

uint8_t    btn_hist;           // Button state history
btn_evt_t  btn_evt;            // Last button event
uint8_t    btn_epoch;          // Epoch updated on every new button event
uint16_t   btn_pressed_ts;     // Button pressed time stamp
uint16_t   wdt_clock;          // Watchdog clock 
uint8_t    t_level_set = 2;    // Temperature threshold level set by user
int8_t     t_level_current;    // Current temperature level
uint16_t   t_sum;              // Temperature readings accumulator
uint8_t    t_cnt;              // The number of accumulated temperature readings
int8_t     t_measured;         // Measured temperature in Celsius
uint8_t    t_epoch;            // Epoch updated on every t_measured update
ui_state_t ui_state = ui_idle; // User interface state
uint16_t   ui_state_ts;        // User interface state changed time stamp

// Temperature thresholds
#define T_LEVELS 5
static const int8_t t_levels[T_LEVELS] = {2, 4, 6, 8, 12};

// Returns the number of ticks elapsed since particular time stamp
static inline int elapsed(uint16_t since)
{
	return (int16_t)(wdt_clock - since);
}

// Routine handling button press
#define LONG_PRESS 200 // 1.6 sec
static void btn_process(void)
{
	int pressed = !(P1IN & BTN);
	int was_pressed = btn_hist;
	btn_hist <<= 1;
	btn_hist |= pressed;
	if (btn_hist) {
		if (!was_pressed) {
			btn_pressed_ts = wdt_clock;
			btn_evt = btn_none;
		} else if (btn_evt == btn_none && elapsed(btn_pressed_ts) > LONG_PRESS) {
			btn_evt = btn_long_pressed;
			++btn_epoch;
		}
	} else {
		if (was_pressed) {
			if (btn_evt == btn_none) {
				btn_evt = btn_released;
				++btn_epoch;
			}
		}
	}
}

// Routine driving temperature control LEDs
#define BLINK_SHIFT 4
static void led_process(void)
{
	P1OUT |= P1T;
	P2OUT |= P2T;
	if (ui_state == ui_idle) {
		return;
	}
	P1OUT |= HEATER;
	if (ui_state == ui_setting && !(wdt_clock & (1 << BLINK_SHIFT))) {
		return;
	}
	if (ui_state == ui_monitor && t_level_current < 0) {
		return;
	}
	uint8_t t_level = ui_state == ui_monitor ? t_level_current : t_level_set;
	uint8_t mask = (1 << (t_level + 1)) - 1;
	P1OUT &= ~((mask & 3) << 4);
	P2OUT &= ~(mask >> 2);
}

// Start temperature measurement
static inline void t_sample(void)
{
	// Sampling and conversion start
	ADC10CTL0 |= ENC + ADC10SC;
}

// ADC10 interrupt service routine
#define T_SAMPLES 64
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR (void)
{
	t_sum += ADC10MEM;
	if (++t_cnt == T_SAMPLES) {
		t_measured = adc2Temp(t_sum/T_SAMPLES);
		t_sum = 0;
		t_cnt = 0;
		++t_epoch;
	}
}

// Routine called in case temperature measurement result updated
static void t_updated(void)
{
	int8_t i;
	for (i = 0; i < T_LEVELS; ++i) {
		if (t_measured < t_levels[i])
			break;
	}
	t_level_current = i - 1;
	if (ui_state != ui_idle) {
		return;
	}
	if (t_level_current < t_level_set) {
		P1OUT |= HEATER;
	} else {
		P1OUT &= ~HEATER;
	}
}

// Stop execution and signal by heater output flushing
static void stop(void)
{
	for (;;) {
		P1OUT |= HEATER;
		__delay_cycles(200000);
		P1OUT &= ~HEATER;
		__delay_cycles(200000);
	}
}

//
// Configuration routines
//

// Information segment where we store current configuration
#define CFG_SEG_ADDR 0x1000
#define CFG_SEG_SZ 64
__no_init volatile uint8_t cfg_seq[CFG_SEG_SZ] @CFG_SEG_ADDR;

static inline uint8_t cfg_level(void)
{
	return cfg_seq[0];
}

static inline int cfg_valid(void)
{
	return cfg_level() < T_LEVELS;
}

static inline void cfg_erase(void)
{
	flash_erase(CFG_SEG_ADDR, 1);
}

static inline void cfg_save(void)
{
	cfg_erase();
	flash_write(CFG_SEG_ADDR, &t_level_set, sizeof(uint8_t));
}

//
// User interface routines
//

static inline void ui_set_state(ui_state_t st)
{
	ui_state = st;
	ui_state_ts = wdt_clock;
}

// Idle mode button events handler
static void ui_idle_handler(void)
{
	switch (btn_evt) {
	case btn_released:
		ui_set_state(ui_monitor);
		break;
	case btn_long_pressed:
		ui_set_state(ui_setting);
		break;
	}
}

// Temperature monitoring mode button events handler
static void ui_monitor_handler(void)
{
	switch (btn_evt) {
	case btn_released:
		ui_set_state(ui_idle);
		break;
	case btn_long_pressed:
		ui_set_state(ui_setting);
		break;
	}
}

// Temperature threshold setting mode button events handler
static void ui_setting_handler(void)
{
	switch (btn_evt) {
	case btn_released:
		t_level_set = (t_level_set + 1) % T_LEVELS;
		ui_set_state(ui_setting);
		break;
	case btn_long_pressed:
		cfg_save();
		ui_set_state(ui_idle);
		break;
	}
}

// Button events handler
static void handle_btn_evt(void)
{
	switch (ui_state) {
	case ui_idle:    // idle mode
		ui_idle_handler();
		break;
	case ui_monitor: // temperature monitoring
		ui_monitor_handler();
		break;
	case ui_setting: // threshold level setting
		ui_setting_handler();
		break;
	}
}

#define UI_TIMEOUT 2000 // 16 sec
static inline void ui_chk_timeout(void)
{
	if (ui_state != ui_idle && elapsed(ui_state_ts) > UI_TIMEOUT) {
		ui_set_state(ui_idle);
	}
}

//
// Periodic watchdog timer routine called every 8 msec
//
#pragma vector=WDT_VECTOR
__interrupt void watchdog_timer(void)
{
	++wdt_clock;
	btn_process();
	led_process();
	t_sample();
	ui_chk_timeout();
	__low_power_mode_off_on_exit();
}

//
// The entry point
//
void main( void )
{
	// Configure watchdog timer
	WDTCTL = WDT_MDLY_8;
	IE1 |= WDTIE; // Enable WDT interrupt

	P1DIR = LED1 | LED2 | P1T;
	P1OUT = BTN | P1T;
	P1REN = BTN;

	P2DIR = P2T;
	P2OUT = P2T;

	if (cfg_valid()) {
		t_level_set = cfg_level();
	}

	if (!adcTempCalibValid()) {
		// Ensure we have ADC calibration data
		stop();
	}

	// Configure ADC: SMCLK, 64 periods sample time
	ADC10CTL0 = SREF_1 + ADC10SHT_3 + REFON + ADC10ON + ADC10IE;
	ADC10CTL1 = INCH_10 + ADC10SSEL_3;	// Temp sensor

	uint8_t last_btn_epoch = btn_epoch;
	uint8_t last_t_epoch = t_epoch;

	for (;;) {
		__low_power_mode_0();
		if (last_btn_epoch != btn_epoch) {
			last_btn_epoch = btn_epoch;
			handle_btn_evt();
		}
		if (last_t_epoch != t_epoch) {
			last_t_epoch = t_epoch;
			t_updated();
		}
	}
}
