#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
// ATTINY3224/6 has 3 timers  (TCA, TCB0 & TCB1), TCB1 is used by millis, so
// only TCB0 & TCA available TCA has three Compare and 1 period, so 3 PWM and 1
// Overflow ISR and 3 compare ISR can be generated this code demonstrate TCA0
// CMP0 ISR, and TCB0 CMP ISR. this code also demonstrate RTC 1KHz sleep mode
// running in sleep mode as ULPM, Sleeps for 5seconds then activate CPU for 5sec
// in which TCA or TCB0 will toggle LED on PA7(3) pin at 1Hz.
// Also demonstrate wakeup from sleep with PC0(10) button input and Toggle LED
// on PA7(3).
uint8_t state = 0;
void setup_timer_a() {
  takeOverTCA0();
  TCA0_SINGLE_PER = 10000;  // PER/F_CPU, 10ms at 1MHz
  TCA0_SINGLE_CMP0 = 2500;  // intterupt at CMP0<TOP, TOP=PER;
  TCA0_SINGLE_CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc |  // F_CPU * DIV1
                      TCA_SINGLE_ENABLE_bm;        // TCA_SINGLE_RUNSTDBY_bm;
  TCA0_SINGLE_INTFLAGS =
      TCA_SINGLE_CMP0_bm;  // Clear Any pending interrupt flags
  TCA0_SINGLE_INTCTRL = TCA_SINGLE_CMP0_bm;  // Enable TCA0 timeout interrupt
};

void setup_timer_b0() {
  TCB0.CCMP = 10000;                                   // CCMP/1MHz, 10ms
  TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm;  //| TCB_RUNSTDBY_bm;
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
    state = !state;
    digitalWrite(3, state);
  }
}

void setup() {
  pinMode(3, OUTPUT);
  digitalWrite(3, LOW);
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
  setup_timer_a();
  // setup_timer_b0(); // use either a0 or b0
  setup_timer_rtc();
  sei();  // Enable global interrupts
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

ISR(TCA0_CMP0_vect) {  /// 10ms timer  // active only when awake
  static unsigned int t0 = 50, timercount = t0, xx = 0;
  timercount--;
  if (timercount == 0) {
    timercount = t0;
    xx = !xx;
    digitalWrite(3, xx);
    // PORTA_OUT = xx << 7;
    // PORTA_OUTTGL |= PIN7_bm;
  }
  TCA0_SINGLE_INTFLAGS = TCA_SINGLE_CMP0_bm; /* Clear the interrupt flag */
  // sleep_cpu();
}

ISR(TCB0_INT_vect) {  ///  // active only when awake
  static unsigned int t0 = 50, timercount = t0, xx = 0;
  timercount--;
  if (timercount == 0) {
    timercount = t0;
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