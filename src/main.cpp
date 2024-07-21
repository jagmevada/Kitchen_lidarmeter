#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>
// #include <avr/iotn3226.h>
#include <avr/sleep.h>
#include <function.h>
typedef ADC_MUXPOS_t adc_0_channel_t;
// ATTINY3224/6 has 3 timers  (TCA, TCB0 & TCB1), TCB1 is used by millis, so
// only TCB0 & TCA available TCA has three Compare and 1 period, so 3 PWM and 1
// Overflow ISR and 3 compare ISR can be generated this code demonstrate TCA0
// CMP0 ISR, and TCB0 CMP ISR. this code also demonstrate RTC 1KHz sleep mode
// running in sleep mode as ULPM, Sleeps for 5seconds then activate CPU for 5sec
// in which TCA or TCB0 will toggle LED on PA7(3) pin at 1Hz.
// Also demonstrate wakeup from sleep with PC0(10) button input and Toggle LED
// on PA7(3).
// ######################### defination ########################################
#define SWINPUT 2
#define ADCPIN 0
#define LDOEN 1
#define LDOENABLE() ditialWrite(LDOEN, 1);
#define LDODISABLE() digitalWrite(LDOEN, 0);
#define SETUPLDO() \
  pinMode(         \
      1,           \
      OUTPUT);  // LDOEN OUT external pulled down and LDO UP when goes HIGH.
#define INTERRUPT_EN 0x3
#define INTERRUPT_DIS 0x0
#define SW_INTCTRL PORTA.PIN6CTRL
#define OUTPUT_PINMASK PIN5_bm
#define DEBOUNCE_PERIOD 5     /// Multiple of 128ms
#define AUTOOFF_TIMEOUT 2445  // 3130  Multiple of 128ms or 100ms
#define ADC_PERIOD 156        // 20s at multiple of 128ms
#define BAT_LOW_TIMEOUT 7
#define UTH 3200
#define LTH 2800
#define LOW 0
#define HIGH 1
/*
    Main application
*/
// ######################### variables ########################################
bool new_input, batstate = 1, ldostate = 0, ldostateprev = 0;
unsigned int vbat = 4095, vbatx = 4095;
unsigned char t0 = DEBOUNCE_PERIOD, t2 = 0;
uint16_t t1 = ADC_PERIOD;
uint16_t timeout = 0;

// ######################### function defs ####################################
void Timer_A0();
void button_input();
void button5();
void setup_timer_a();
void setup_timer_b0();
void setup_timer_rtc();
void ADC0_StartConversion(adc_0_channel_t channel);
void ADC0_Enable(void);
void ADC0_Disable(void);
int8_t ADC0_Initialize(void);

// // ######################### adc_startconv function #########################
void ADC0_StartConversion(adc_0_channel_t channel) {
  ADC0.MUXPOS &= ADC_VIA_gm;
  ADC0.MUXPOS |= channel;
  ADC0.COMMAND &= ~ADC_DIFF_bm;
  ADC0.COMMAND |= ADC_START_IMMEDIATE_gc;
}

// ####################### adc_enable function  ######################
void ADC0_Enable(void) { ADC0.CTRLA |= ADC_ENABLE_bm; }
// ####################### adc_disable function  ######################
void ADC0_Disable(void) { ADC0.CTRLA &= ~ADC_ENABLE_bm; }

// ####################### adc_initializaion function  ######################

int8_t ADC0_Initialize(void) {
  // PRESC System clock divided by 64;
  ADC0.CTRLB = 0xF;

  // FREERUN disabled; LEFTADJ disabled; SAMPNUM No accumulation;
  ADC0.CTRLF = 0x0;

  // REFSEL Internal 2.048V Reference; TIMEBASE 1;
  ADC0.CTRLC = 0xD;

  // WINCM No Window Comparison; WINSRC RESULT;
  ADC0.CTRLD = 0x0;

  // SAMPDUR 255;
  ADC0.CTRLE = 0xFF;

  // ADCPGASAMPDUR 6 ADC cycles; GAIN 1X Gain; PGABIASSEL 1x BIAS current; PGAEN
  // disabled;
  ADC0.PGACTRL = 0x0;

  // DBGRUN disabled;
  ADC0.DBGCTRL = 0x0;

  // DIFF disabled; MODE SINGLE_12BIT; START Stop an ongoing conversion;
  ADC0.COMMAND = 0x10;

  // RESOVR disabled; RESRDY enabled; SAMPOVR disabled; SAMPRDY disabled;
  // TRIGOVR disabled; WCMP disabled;
  ADC0.INTCTRL = 0x1;

  // MUXPOS ADC input pin 4; VIA Via ADC;
  ADC0.MUXPOS = 0x4;

  // MUXNEG Ground; VIA Via ADC;
  ADC0.MUXNEG = 0x30;

  // Window comparator high threshold
  ADC0.WINHT = 0x0;

  // Window comparator low threshold
  ADC0.WINLT = 0x0;

  // ENABLE enabled; LOWLAT disabled; RUNSTDBY disabled;
  ADC0.CTRLA = 0x1;

  return 0;
}

// ######################### timer_a0 isr function #############################
void Timer_A0(void) {  // 100ms

  static unsigned char tx = 5;
  if (tx == 0) tx = 5;
  tx--;

  if (tx == 1) {  /// output blink before permanently low
    if (t2 >
        0) {  // bat low timeout count to zero to blink and then LDO set to LOW
      t2--;
      if (t2 == 0) {
        PORTA.OUT &= ~OUTPUT_PINMASK;  // turn off LDO
      }
    }
  }

  //        SLPCTRL.CTRLA = 0x3;
}
// ######################### button_input #############################
void button_input(void) {
  // if(new_input==0){
  t0 = DEBOUNCE_PERIOD;
  SW_INTCTRL = INTERRUPT_DIS;
  // ldostateprev = ldostate;
  ldostate = !ldostate;
  new_input = 1;
  if (ldostate) {                 // LDO to be turned on
    PORTA.OUT |= OUTPUT_PINMASK;  // turn on again but
    ADC0_Enable();
    timeout = AUTOOFF_TIMEOUT;
  } else {                         // LDO to be turned off
    PORTA.OUT &= ~OUTPUT_PINMASK;  // turn off LDO bat low
    ADC0_Disable();
    timeout = 0;
  }
  //}
  PORTA.INTFLAGS = 0x40;  // PA6 // clear INTFLAG
  //    SLPCTRL.CTRLA = 0x3; // enable sleep and go to standby mode
}
// ######################### button5 #############################
uint8_t state = 0;
void button5() {
  if (digitalRead(10) == HIGH) {
    state = !state;
    digitalWrite(3, state);
  }
}
// ######################### setup_timer_a0 #############################
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
// ######################### setup_timer_b0 #############################
void setup_timer_b0() {
  TCB0.CCMP = 10000;                                   // CCMP/1MHz, 10ms
  TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm;  //| TCB_RUNSTDBY_bm;
  TCB0.INTFLAGS = TCB_CAPT_bm;  // Clear any pending interrupt flags
  TCB0.INTCTRL = TCB_CAPT_bm;   // Enable TCB0 timeout interrupt
};
// ######################### setup_timer_rtc #############################
void setup_timer_rtc() {
  RTC.CLKSEL = RTC_CLKSEL_INT1K_gc;  // 1khz internal
  RTC.CTRLA = CLKCTRL_RUNSTDBY_bm | RTC_RTCEN_bm |
              RTC_PRESCALER0_bm;  // run in standby sleep mode and enable rtc no
                                  // prescaler
  RTC.PITCTRLA = RTC_PERIOD_CYC128_gc | RTC_PITEN_bm;  // every 2 sec interrupt
  RTC.PITINTCTRL = RTC_PI_bm;                          // enable interrupt
}

//@@@@@@@@@@@@@@@@@@@@@@@@@  SETUP  @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
void setup() {
  pinMode(0, INPUT);  // ADC
  SETUPLDO();
  LDODISABLE();
  pinMode(2, INPUT);  // SW IN external pulled up
  pinMode(3, OUTPUT);
  digitalWrite(3, LOW);
  // it is external pulled down pinMode(5, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);  // CHARGE IN OPTIONAL, NEED EX. PULLDOWN AND IT
                             // WILL RISE UP WHEN USB CONNECTED
  pinMode(10, INPUT_PULLUP);
  // pinMode(11, INPUT_PULLUP);
  // pinMode(12, INPUT_PULLUP);
  // pinMode(13, INPUT_PULLUP);
  // pinMode(14, INPUT_PULLUP);
  // pinMode(15, INPUT_PULLUP);
  // pinMode(16, INPUT_PULLUP);
  // ADC0.CTRLA &= ~ADC_ENABLE_bm;
  //   set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  // set_sleep_mode(SLEEP_MODE_IDLE); //380UA
  batstate = HIGH;
  new_input = 0;
  set_sleep_mode(SLEEP_MODE_STANDBY);
  sleep_enable();
  attachInterrupt(SWINPUT, button_input, FALLING);
  // setup_timer_a();
  ADC0_Initialize();
  ADC0_Disable();
  // setup_timer_b0(); // use either a0 or b0
  setup_timer_rtc();
  sei();  // Enable global interrupts
}

//@@@@@@@@@@@@@@@@@@@@@@@@@  LOOP  @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
void loop() {
  // put your main code here, to run repeatedly:
  // ADC0.CTRLA &= ~ADC_ENABLE_bm;
  sleep_cpu();
}

// ************************* ISR Timer RTC ***************************
ISR(RTC_PIT_vect) {  /// 128 msec timer

  // static unsigned int sec = 5, timercount = sec, xx = 0;
  // timercount--;
  // if (timercount == 0) {
  //   timercount = sec;
  //   xx = !xx;
  //   if (xx) {
  //     sleep_disable();
  //   } else {
  //     sleep_enable();
  //   }
  //   // digitalWrite(3, xx);
  // }
  if (new_input) {
    t0--;           // debounce timer countdown
    if (t0 == 0) {  // button debounce period over
      new_input = 0;
      SW_INTCTRL = INTERRUPT_EN;  // enable interrupt
      PORTA.INTFLAGS = PIN6_bm;   // clear previous flags
      t1 = ADC_PERIOD;
    }
  }

  if (timeout > 0) {  // Normal autoff timeout countdown
    timeout--;
    if (timeout == 0) {              // Normal autoff timeout event
      PORTA.OUT &= ~OUTPUT_PINMASK;  // turn off LDO bat low
      ldostateprev = 0;
      ldostate = 0;
      ADC0_Disable();
    }
  }
  if (ldostate == 1) {  // Vbat monitoring and auto cutoff
    if (t1 == 0) {      // adc timing
      t1 = ADC_PERIOD;
      vbatx = vbat;  // Copy previous conversoin result
      ADC0_StartConversion(ADC_MUXPOS_AIN4_gc);
    }
    t1--;

    //    static unsigned char batstate=LOW;
    if (batstate == HIGH) {  // discharging to cutoff
      if (vbatx < LTH) {     // reached at cutoff
        batstate = LOW;
        timeout = 1;
        // if ((PORTA.OUT & OUTPUT_PINMASK) == OUTPUT_PINMASK) {  /// if LDO
        // ON??
      }
    } else {  // charging to reach full charge voltage 4.2V
      if (vbatx > UTH) {
        batstate = HIGH;
      }
    }
  }

  RTC.PITINTFLAGS = RTC_PI_bm; /* Clear the interrupt flag */
}

// ************************* ISR ADC Result ready  ***************************
ISR(ADC0_RESRDY_vect) {
  /* Insert your ADC result ready interrupt handling code here */
  vbat = (uint8_t)ADC0.RESULT;
  /* The interrupt flag has to be cleared manually */
  ADC0.INTFLAGS = ADC_RESRDY_bm;
}

// ************************* ISR Timer A0 ***************************
ISR(TCA0_CMP0_vect) {  /// 10ms timer  // active only when awake
  TCA0_SINGLE_INTFLAGS = TCA_SINGLE_CMP0_bm; /* Clear the interrupt flag */
}
// ************************* ISR Timer B0 ***************************
ISR(TCB0_INT_vect) {           ///  // active only when awake
  TCB0.INTFLAGS = TCB_CAPT_bm; /* Clear the interrupt flag */
}