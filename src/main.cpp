#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>

// void initialize() {
//   init_clock();
//   // init_ADC0();
//   init_timers();
//   init_millis();
//   // init();
//   //  RTC_init(); /* Initialize the RTC timer */
//   pinMode(0, INPUT_PULLUP);
//   pinMode(1, INPUT_PULLUP);
//   pinMode(2, INPUT_PULLUP);
//   pinMode(3, INPUT_PULLUP);
//   pinMode(4, INPUT_PULLUP);
//   pinMode(5, INPUT_PULLUP);
//   pinMode(6, INPUT_PULLUP);
//   pinMode(7, INPUT_PULLUP);
//   pinMode(8, INPUT_PULLUP);
//   pinMode(9, INPUT_PULLUP);
//   pinMode(10, INPUT);
//   pinMode(11, INPUT_PULLUP);
//   pinMode(12, INPUT_PULLUP);
//   pinMode(13, INPUT_PULLUP);
//   pinMode(14, INPUT_PULLUP);
//   pinMode(15, INPUT_PULLUP);
//   pinMode(16, INPUT_PULLUP);
//   set_sleep_mode(SLEEP_MODE_PWR_DOWN); /* Set sleep mode to POWER DOWN mode
//   */ sleep_enable(); /* Enable sleep mode, but not going to sleep yet */
// }

// int main() {
//   initialize();
//   while (1) {
//     sleep_cpu(); /* Sleep the device and wait for an interrupt to continue */
//     // digitalWrite(PIN_PA6, CHANGE); /* Device woke up and toggle LED on
//     pin#7
//     // */
//   }
//   return (0);
// }

// #include <Arduino.h>
// #include <avr/sleep.h>
uint8_t state = 0;

void setup_timer_b0() {
  TCB0.CCMP = 5000;                                    // CCMP/1MHz.
  TCB0.CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm;  //| TCB_RUNSTDBY_bm;
  TCB0.INTFLAGS = TCB_CAPT_bm;  // Clear any pending interrupt flags
  TCB0.INTCTRL = TCB_CAPT_bm;   // Enable TCB0 timeout interrupt
};

void setup_timer_rtc() {
  RTC.CLKSEL = RTC_CLKSEL_INT1K_gc;  // 1khz internal
  RTC.CTRLA = CLKCTRL_RUNSTDBY_bm | RTC_RTCEN_bm |
              RTC_PRESCALER0_bm;  // run in standby sleep mode and enable rtc no
                                  // prescaler
  RTC.PITCTRLA = RTC_PERIOD_CYC1024_gc | RTC_PITEN_bm;  // every 2 sec interrupt
  RTC.PITINTCTRL = RTC_PI_bm;                           // enable interrupt
}
void button5() {
  if (digitalRead(10) == HIGH) {
    // state = !state;
    // digitalWrite(3, state);
  }
}

void setup() {
  pinMode(3, OUTPUT);
  digitalWrite(3, LOW);
  // _PROTECTED_WRITE(CLKCTRL_MCLKCTRLA, CLKCTRL_CLKSEL1_bm);
  // _PROTECTED_WRITE(CLKCTRL_MCLKCTRLB, CLKCTRL_PEN_bp);
  // _PROTECTED_WRITE(CLKCTRL.OSC32KCTRLA, CLKCTRL_RUNSTDBY_bm);
  // while (!(CLKCTRL_MCLKSTATUS & CLKCTRL_OSC32KS_bm));
  // _PROTECTED_WRITE()
  pinMode(0, INPUT_PULLUP);
  pinMode(1, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  pinMode(10, INPUT);
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
  pinMode(15, INPUT_PULLUP);
  pinMode(16, INPUT_PULLUP);
  ADC0.CTRLA &= ~ADC_ENABLE_bm;
  //   set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  // set_sleep_mode(SLEEP_MODE_IDLE); //380UA
  set_sleep_mode(SLEEP_MODE_STANDBY);
  sleep_enable();
  attachInterrupt(10, button5, CHANGE);

  setup_timer_b0();
  setup_timer_rtc();
  sei();  // Enable global interrupts
  // set_sleep_mode(SLEEP_MODE_STANDBY);
  // sleep_mode();
}

void loop() {
  // put your main code here, to run repeatedly:
  // ADC0.CTRLA &= ~ADC_ENABLE_bm;
  sleep_cpu();
  // delay(1000);
  // digitalWrite(3, 1);
  // delay(1000);
  // digitalWrite(3, 0);
}

ISR(TCB0_INT_vect) {  /// 10ms timer  // active only when awake
  static unsigned int timercount = 50, xx = 0;
  timercount--;
  if (timercount == 0) {
    timercount = 50;
    xx = !xx;
    // digitalWrite(3, xx);
    PORTA_OUT = xx << 7;
    // PORTA_OUTTGL |= PIN7_bm;
  }
  TCB0.INTFLAGS = TCB_CAPT_bm; /* Clear the interrupt flag */
  // sleep_cpu();
}

ISR(RTC_PIT_vect) {  /// 1sec timer  /// 5 sleep and 5 sec awake
  static unsigned int sec = 5, timercount = sec, xx = 0;
  timercount--;
  if (timercount == 0) {
    timercount = sec;
    xx = !xx;
    if (xx) {
      sleep_disable();
    } else {
      sleep_enable();
    }
    // digitalWrite(3, xx);
  }
  RTC.PITINTFLAGS = RTC_PI_bm; /* Clear the interrupt flag */
}