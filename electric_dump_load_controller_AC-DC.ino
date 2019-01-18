#include <PID_v1.h>
#include <FastPID.h>
// Chester Lowrey 2018-12-28

#include <Arduino.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST77XX
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include "Adafruit_ILI9341.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

#define TFT_CS         9
#define TFT_RST        -1 // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC         10

#define debug

#ifdef ARDUINO_ARCH_AVR
#define REGTYPE uint8_t   // AVR uses 8-bit registers
#else
#define REGTYPE uint32_t
#endif

REGTYPE pin11;
volatile REGTYPE *mode11;
volatile REGTYPE *out11;

REGTYPE pin12;
volatile REGTYPE *mode12;
volatile REGTYPE *out12;

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

#define PINREFV A1
#define PINLVDC A2
#define PINHVDC A3


#define acVcal 225

#define SDA_PIN 21
#define SCL_PIN 22

float rawHVDC = 0;
float rawREFV = 0;
float rawLVDC = 0;

unsigned long loopCounter;
unsigned long TCCintTemp;
unsigned long TCCintTime;
unsigned long TCCintCounter;
long lastpost;
float offsetLVDC;
float filteredLVDC;

unsigned int loop_count = 0;
unsigned int irq_ovf_count = 0;

unsigned int pwm;

float error, out;


//Define Variables we'll be connecting to
//double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
//double Kp = 200, Ki = 50, Kd = 1;
//PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

float Kp = 5, Ki = 20, Kd = 3, Hz = 20000;
int output_bits = 12;
bool output_signed = false;
float setpoint;
float input;
float output;

float shuntvoltage = 0;
float busvoltage = 0;

FastPID myPID(Kp, Ki, Kd, Hz, output_bits, output_signed);

void setup()
{


  setpoint = 1240.9;

  //Wire.setClock(2560000);
  ina219.begin();
  ina219.setCalibration_16V_400mA();
  //Wire.setClock(2560000);

  //turn the PID on
  //myPID.SetMode(AUTOMATIC);

  /*
    //************************* Fast Analog Stuff ****************************
    GCLK->PCHCTRL[ADC0_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1_Val | (1 << GCLK_PCHCTRL_CHEN_Pos); //use clock generator 1 (48Mhz)
    ADC0->CTRLA.bit.PRESCALER = ADC_CTRLA_PRESCALER_DIV16_Val;
    ADC0->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_12BIT_Val;
    while ( ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_CTRLB ); //wait for sync
    ADC0->SAMPCTRL.reg = 0x00;                        // Set max Sampling Time Length to half divided ADC clock pulse (5.33us)
    //ADC0->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 |    // 1 sample only (no oversampling nor averaging)
    //                   ADC_AVGCTRL_ADJRES(0x0ul);   // Adjusting result by 0

    GCLK->PCHCTRL[ADC1_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1_Val | (1 << GCLK_PCHCTRL_CHEN_Pos); //use clock generator 1 (48Mhz)
    ADC1->CTRLA.bit.PRESCALER = ADC_CTRLA_PRESCALER_DIV16_Val;
    ADC1->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_12BIT_Val;
    while ( ADC1->SYNCBUSY.reg & ADC_SYNCBUSY_CTRLB ); //wait for sync
    ADC1->SAMPCTRL.reg = 0x00;                        // Set max Sampling Time Length to half divided ADC clock pulse (5.33us)
    //ADC1->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 |    // 1 sample only (no oversampling nor averaging)
    //                   ADC_AVGCTRL_ADJRES(0x0ul);   // Adjusting result by 0
  */

  ADC0->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV16_Val | // Divide Clock ADC GCLK by 512 (48MHz/512 = 93.7kHz)
                    ADC_CTRLB_RESSEL_12BIT;         // Set ADC resolution to 12 bits
  while (ADC0->STATUS.bit.SYNCBUSY);                // Wait for synchronization
  ADC0->SAMPCTRL.reg = 0x00;                        // Set max Sampling Time Length to half divided ADC clock pulse (5.33us)

  //************************* Fast Pin Toggle Stuff ****************************

  pin11 = digitalPinToBitMask(11); // direct pin hardware access setup
  mode11 = portModeRegister(digitalPinToPort(11));
  out11 = portOutputRegister(digitalPinToPort(11));
  // set pin 11 port as ouput
  *mode11 |= pin11;
  pin12 = digitalPinToBitMask(12); // direct pin hardware access setup
  mode12 = portModeRegister(digitalPinToPort(12));
  out12 = portOutputRegister(digitalPinToPort(12));
  // set pin 12 port as ouput
  *mode12 |= pin12;


  Serial.begin(115200);
  analogReadResolution(12);
  analogWriteResolution(12);

  //************************* LCD Screen Setup ****************************
  tft.initR(INITR_BLACKTAB);      // Init ST77XXS chip, black tab
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);


  //************************* PWM Setup ****************************

  TCC0->CTRLA.bit.ENABLE = 0; // Some PWM registers can only be updated when TCCn.CTRLA.ENABLE = 0

  MCLK->APBBMASK.reg |= MCLK_APBBMASK_TCC0; // at 15.8.9 unlock TCC0

  // Set up the generic clock (GCLK7) used to clock timers
  //GCLK->GENCTRL[7].reg = GCLK_GENCTRL_DIV(2) |       // Divide the 120MHz clock source by divisor 1: 120MHz/1 = 120MHz
  //                       GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
  //                       GCLK_GENCTRL_GENEN |        // Enable GCLK7
  //                       GCLK_GENCTRL_SRC_DPLL0;     // Generate from 120MHz DPLL clock source
  //while (GCLK->SYNCBUSY.bit.GENCTRL7);               // Wait for synchronization

  //GCLK->GENCTRL[5].reg = GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_DFLL) |
  //                       GCLK_GENCTRL_GENEN;
  //while (GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_GENCTRL5);

  // Connect generic clock 2 to TCC0 - this clock is setup by Arduino at 100mhz
  GCLK->PCHCTRL[TCC0_GCLK_ID].reg = GCLK_PCHCTRL_CHEN | // Enable perhipheral channel
                                    GCLK_PCHCTRL_GEN_GCLK2;

  // Enable the peripheral multiplexer on pin 7 (A18)
  PORT->Group[g_APinDescription[7].ulPort].PINCFG[g_APinDescription[7].ulPin].bit.PMUXEN = 1;

  // Set ARduino pin 7 (A18) to the peripheral multiplexer to peripheral G(6): TCC0, Channel 6
  // Pin A18 is an even pin which means we must use the PORT_PMUX_PMUXE for Even not PORT_PMUX_PMUXO for Odd
  // This is determined by the chart at section 6.1 of the datasheet. TCC0 is in the G column which comes to 6 (A-N = 0-13)
  PORT->Group[g_APinDescription[7].ulPort].PMUX[g_APinDescription[7].ulPin >> 1].reg |= PORT_PMUX_PMUXE(6);

  TCC0->CTRLA.reg |= TCC_CTRLA_PRESCALER_DIV1 |  // Set prescaler to 1, 120MHz/1 = 120MHz
                     TCC_CTRLA_PRESCSYNC_PRESC | // Set the reset/reload to trigger on prescaler clock
                     TCC_CTRLA_RESOLUTION(0);

  // Select Waveform Generation operation in the WAVE register WAVE.WAVEGEN
  TCC0->WAVE.reg = TCC_WAVE_WAVEGEN(TCC_WAVE_WAVEGEN_NPWM_Val); //
  while (TCC1->SYNCBUSY.bit.WAVE);               // Wait for synchronization

  //We want OTMX - Output Matrix Channel Pin Routing Configuration - at 0x0

  /*
    TCC0->WEXCTRL.reg = TCC_WEXCTRL_DTHS(1) | // Dead time high
                        TCC_WEXCTRL_DTLS(1) | // Dead time low
                        TCC_WEXCTRL_DTIEN1 |  // Dead time insertion enable
                        TCC_WEXCTRL_DTIEN2 |
                        TCC_WEXCTRL_DTIEN3 |
                        TCC_WEXCTRL_DTIEN0 |
                        TCC_WEXCTRL_OTMX(0);
  */

  REG_TCC0_PER = 4999; // equals 20khz when starting from 100mhz clock - 4999 makes 5000 = 100% duty
  while (TCC0->SYNCBUSY.bit.PER);                // Wait for synchronization

  TCC0->COUNT.reg = 0;
  REG_TCC0_CC0 = 5000;
  while (TCC0->SYNCBUSY.bit.CC0);                // Wait for synchronization

  //enable interrupts
  REG_TCC0_INTENSET = TCC_INTENSET_OVF; //Set up interrupt at TOP of each PWM cycle

  //This function sets the interrupts priority to highest and then enables the PWM interrupt
  NVIC_SetPriority(TCC0_0_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority
  NVIC_EnableIRQ(TCC0_0_IRQn);

  TCC0->CTRLA.bit.ENABLE = 1; // Enable TCC0 with CTRLA.ENABLE
  while (TCC0->SYNCBUSY.bit.ENABLE);

}

void loop() {

  //busvoltage = ina219.getBusVoltage_raw();

  //rawLVDC = analogRead(PINLVDC);
  //rawHVDC = analogRead(PINHVDC) ;
  //rawREFV = analogRead(PINREFV);


  // TCC0->CTRLA.reg |= TCC_CTRLA_ENABLE;

  //analogWrite(A1, rawLVDC);


  //TC0->COUNT16.CCBUF[1].reg = map(rawLVDC, 0, 4095, 0, 6000);                  // Set the duty cycle by loading buffered CC1 register CCBUF1

  //offsetLVDC = offsetLVDC + ((rawLVDC - offsetLVDC) / 1024); // these equations remove all DC offset - and by design only pass AC value
  //filteredLVDC = rawLVDC - offsetLVDC;


#ifdef debug
  if ((millis() - lastpost) >= 100) // A simple timer to slow down serial data send / read
  {

    tft.setCursor(0, 0);
    tft.print("TCCintCntr ");
    tft.println(TCCintCounter);

    filteredLVDC = rawLVDC / 1240.9;

    tft.print("LoopCntr: ");
    tft.print(loopCounter);
    tft.println("  ");
    tft.print("rawBusV: ");
    tft.print(busvoltage);
    tft.println("V");
    tft.print("RAW LVDC: ");
    tft.println(rawLVDC);
    tft.print("Filtered LVDC: ");
    tft.print((float)filteredLVDC);

    tft.println(" V");
    tft.print("PWM: ");
    tft.print(pwm);
    tft.println(" Count");
    tft.print("rawBusV: ");
    tft.print(busvoltage);
    tft.println("V");

    tft.print("Print T:");
    tft.print(((float)millis() - lastpost));
    tft.println(" ms");




    Serial.print("Loop Time:  ");
    Serial.print((float)500 / loopCounter, 6); // must cast millis() as float or result will not contain milliseconds
    Serial.println(" ms");
    Serial.print("Raw LVDC: ");
    Serial.println(rawLVDC);
    Serial.print("Filtered LVDC: ");
    Serial.print((float)filteredLVDC);
    Serial.println(" V");
    Serial.println(" ");
    Serial.print("PWM: ");
    Serial.print(pwm);
    Serial.println(" Count");
    Serial.print("Print Time: ");
    Serial.print(((float)millis() - lastpost)); // must cast millis() as float or result will not contain milliseconds
    Serial.println(" ms");





    lastpost = millis();
    loopCounter = 0;
    TCCintCounter = 0;
  }
  loopCounter++;
#endif
}

void TCC0_0_Handler()
{
  Tcc* TC = (Tcc*) TCC0;       // get timer struct
  if (TC->INTFLAG.bit.OVF == 1) {  // A overflow caused the interrupt

    TCCintCounter ++;
    //*out11 |= pin11; // fast toggle pin 11
    //*out11 &= ~pin11;

    //busvoltage = ina219.getBusVoltage_V();

    rawLVDC = analogRead(PINLVDC);
    rawHVDC = analogRead(PINHVDC); // need to speed up somehow
    rawREFV = analogRead(PINREFV);

    pwm = myPID.step(setpoint, rawLVDC);

    pwm = map(pwm, 0, 4095, 0, 5000); // 0 = 0% duty cycle, 0 = 6000

    pwm = 5000 - pwm;

    if (pwm < 100) { // make sure our pulse width is not too short.
      pwm = 0;
    }
    else if (pwm > 4900) { // make sure our pulse width is not too short. // duty cycle 0% = 0 , 100% = 6001
      pwm = 5000;
    }

    TCC0->CCBUF[0].reg = pwm;
    //REG_TCC0_CC0 = pwm;
    //TCC0->CCBUF[0].reg = 1;
    //while (TCC0->SYNCBUSY.bit.CC0);                // Wait for synchronization

    //TCCintTime = micros() - TCCintTemp;

    TC->INTFLAG.bit.OVF = 1;    // writing a one clears the flag ovf flag
  }

  //if (TC->INTFLAG.bit.MC0 == 1) {  // A compare to cc0 caused the interrupt

  //  *out12 |= pin12; // fast toggle pin 12
  // *out12 &= ~pin12;

  // TC->INTFLAG.bit.MC0 = 1;    // writing a one clears the flag ovf flag
  //}
}

/*
  void AdcBooster()
  {
  ADC->CTRLA.bit.ENABLE = 0;                     // Disable ADC
  while ( ADC->STATUS.bit.SYNCBUSY == 1 );       // Wait for synchronization
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV64 |   // Divide Clock by 64.
                   ADC_CTRLB_RESSEL_12BIT;       // Result on 12 bits
  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 |   // 1 sample
                     ADC_AVGCTRL_ADJRES(0x00ul); // Adjusting result by 0
  ADC->SAMPCTRL.reg = 0x00;                      // Sampling Time Length = 0
  ADC->CTRLA.bit.ENABLE = 1;                     // Enable ADC
  while ( ADC->STATUS.bit.SYNCBUSY == 1 );       // Wait for synchronization
  } // AdcBooster
*/
