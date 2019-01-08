// Chester Lowrey 2018-12-28

#include <Arduino.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST77XX
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include "Adafruit_ILI9341.h"
#include <SPI.h>

#define TFT_CS         9
#define TFT_RST        -1 // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC         10

#define debug


const int16_t dSize = 1024; //used to set number of samples
const byte chipSelect = 38; //used for SPI chip select pin
const byte gClk = 3; //used to define which generic clock we will use for ADC
const byte intPri = 0; //used to set interrupt priority for ADC
const int cDiv = 1; //divide factor for generic clock
const float period = 3.3334; //period of 300k sample rate
volatile int aDCVal[dSize]; //array to hold ADC samples
volatile int count = 0; //tracks how many samples we have collected
bool done = false; //tracks when done writing data to SD card

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

#define PINLVDC A2
#define PINHVDC A3
#define PINVREF A4

#define acVcal 225

#define SDA_PIN 21
#define SCL_PIN 22

float rawHVDC = 0;
float rawREFv = 0;
float rawLVDC = 0;

long loopCounter;
long lastpost;
float offsetLVDC;
float filteredLVDC;

unsigned int loop_count = 0;
unsigned int irq_ovf_count = 0;

void setup()
{
  GCLK->PCHCTRL[ADC0_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1_Val | (1 << GCLK_PCHCTRL_CHEN_Pos); //use clock generator 1 (48Mhz)
  ADC0->CTRLA.bit.PRESCALER = ADC_CTRLA_PRESCALER_DIV16_Val;
  ADC0->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_12BIT_Val;
  while( ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_CTRLB );  //wait for sync
  ADC0->SAMPCTRL.reg = 5;                        // Set max Sampling Time Length to half divided ADC clock pulse (5.33us)



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

  tft.initR(INITR_BLACKTAB);      // Init ST77XXS chip, black tab
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);

  MCLK->APBAMASK.reg |= MCLK_APBAMASK_TC0;           // Activate timer TC0

  // Set up the generic clock (GCLK7) used to clock timers
  GCLK->GENCTRL[7].reg = GCLK_GENCTRL_DIV(1) |       // Divide the 120MHz clock source by divisor 1: 120MHz/1 = 120MHz
                         GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                         GCLK_GENCTRL_GENEN |        // Enable GCLK7
                         GCLK_GENCTRL_SRC_DPLL0;     // Generate from 120MHz DPLL clock source
  while (GCLK->SYNCBUSY.bit.GENCTRL7);               // Wait for synchronization

  GCLK->PCHCTRL[9].reg = GCLK_PCHCTRL_CHEN |         // Enable perhipheral channel
                         GCLK_PCHCTRL_GEN_GCLK7;     // Connect generic clock 7 to TC0

  // Enable the peripheral multiplexer on pin A1
  PORT->Group[g_APinDescription[A1].ulPort].PINCFG[g_APinDescription[A1].ulPin].bit.PMUXEN = 1;

  // Set A1 the peripheral multiplexer to peripheral E(4): TC0, Channel 1
  PORT->Group[g_APinDescription[A1].ulPort].PMUX[g_APinDescription[A1].ulPin >> 1].reg |= PORT_PMUX_PMUXO(4);

  TC0->COUNT16.CTRLA.reg = TC_CTRLA_PRESCALER_DIV1 |        // Set prescaler to 1, 120MHz/1 = 120MHz
                           TC_CTRLA_PRESCSYNC_PRESC |        // Set the reset/reload to trigger on prescaler clock
                           TC_CTRLA_MODE_COUNT16;            // Set the counter to 16-bit mode

  TC0->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MPWM;      // Set-up TC0 timer for Match PWM mode (MPWM)

  TC0->COUNT16.CC[0].reg = 6000;                    // Use CC0 register as TOP value, set for 20kHz PWM
  while (TC0->COUNT16.SYNCBUSY.bit.CC0);             // Wait for synchronization

  TC0->COUNT16.CC[1].reg = 3000;                     // Set the duty cycle to 50% (CC1 half of CC0)
  while (TC0->COUNT16.SYNCBUSY.bit.CC1);             // Wait for synchronization



  NVIC_SetPriority(TC0_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority for TC0 to 0 (highest)
  NVIC_EnableIRQ(TC0_IRQn);         // Connect TC0 to Nested Vector Interrupt Controller (NVIC)
  REG_TC0_INTFLAG |= TC_INTFLAG_MC1 | TC_INTFLAG_MC0 | TC_INTFLAG_OVF;        // Clear the interrupt flags
  REG_TC0_INTENSET =  TC_INTENSET_MC1;    // Enable TC0 interrupts TC_INTENSET_MC1 || TC_INTENSET_OVF;

  TC0->COUNT16.CTRLA.bit.ENABLE = 1;                 // Enable timer TC0
  while (TC0->COUNT16.SYNCBUSY.bit.ENABLE);          // Wait for synchronization

}

void loop() {

 // rawLVDC = analogRead(PINLVDC);
  //rawHVDC = analogRead(PINHVDC) ;
  //rawREFv = analogRead(PINREFV);




  //TC0->COUNT16.CCBUF[1].reg = map(rawLVDC, 0, 4100, 0, 6000);                  // Set the duty cycle by loading buffered CC1 register CCBUF1

  //offsetLVDC = offsetLVDC + ((rawLVDC - offsetLVDC) / 1024); // these equations remove all DC offset - and by design only pass AC value
  //filteredLVDC = rawLVDC - offsetLVDC;

  //filteredLVDC = rawLVDC / 1240.9;


#ifdef debug
  if ((millis() - lastpost) >= 500) // A simple timer to slow down serial data send / read
  {


    tft.setCursor(0, 0);
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.print("Print T:");
    tft.print(((float)millis() - lastpost));
    tft.println(" ms");
    tft.print("Loop T: ");
    tft.print((float)500 / loopCounter, 6);
    tft.println(" ms");
    tft.print("RAW LVDC: ");
    tft.println((float)rawLVDC);
    tft.print("Filtered LVDC: ");
    tft.print((float)filteredLVDC);
    tft.println(" V");



    Serial.print("Print Time: ");
    Serial.print(((float)millis() - lastpost)); // must cast millis() as float or result will not contain milliseconds
    Serial.println(" ms");
    Serial.print("Loop Time:  ");
    Serial.print((float)500 / loopCounter, 6); // must cast millis() as float or result will not contain milliseconds
    Serial.println(" ms");
    Serial.print("Raw LVDC: ");
    Serial.println((float)rawLVDC);
    Serial.print("Filtered LVDC: ");
    Serial.print((float)filteredLVDC);
    Serial.println(" V");
    Serial.println(" ");




    lastpost = millis();
    loopCounter = 0;
  }
  loopCounter++;
#endif
}

void TC0_Handler()
{
  TcCount16* TC = (TcCount16*) TC0; // get timer struct
  if (TC->INTFLAG.bit.OVF == 1) {  // A overflow caused the interrupt
    *out11 |= pin11; // fast toggle pin 11
    *out11 &= ~pin11;



    TC->INTFLAG.bit.OVF = 1;    // writing a one clears the flag ovf flag
  }

  if (TC->INTFLAG.bit.MC1 == 1) {  // A compare to cc0 caused the interrupt
    *out12 |= pin12; // fast toggle pin 12
    *out12 &= ~pin12;

  rawLVDC = analogRead(PINLVDC);
  TC0->COUNT16.CCBUF[1].reg = map(rawLVDC, 0, 4100, 0, 6000);                  // Set the duty cycle by loading buffered CC1 register CCBUF1
    
    TC->INTFLAG.bit.MC1 = 1;    // writing a one clears the flag ovf flag
  }
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
