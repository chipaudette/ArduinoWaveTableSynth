/* 
 Created: Chip Audette, Feb 2012
 Purpose: For use with keyboard...record gate and CV and loop out
 */

// include the library code:
#include <LiquidCrystal.h>
#include <SPI.h>
#include <Streaming.h>
#include <String.h>
#include "Types.h"
#include "pitchPeriodTable.h"

//pitchStruct readAndQuantizeCV(pitchStruct,int,int);

/* This function places the current value of the heap and stack pointers in the
 * variables. You can call it from any place in your code and save the data for
 * outputting or displaying later. This allows you to check at different parts of
 * your program flow.
 * The stack pointer starts at the top of RAM and grows downwards. The heap pointer
 * starts just above the static variables etc. and grows upwards. SP should always
 * be larger than HP or you'll be in big trouble! The smaller the gap, the more
 * careful you need to be. Julian Gall 6-Feb-2009.
 */
uint8_t * heapptr, * stackptr;
void check_mem() {
  stackptr = (uint8_t *)malloc(4);          // use stackptr temporarily
  heapptr = stackptr;                     // save value of heap pointer
  free(stackptr);      // free up the memory again (sets stackptr to 0)
  stackptr =  (uint8_t *)(SP);           // save value of stack pointer
}


// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(4,5,6,7,8,9);  //rs, enable, d0, d1, d2, d3
int currentDisplayMode = 0;  //what type of info is the LCD displaying?
int prevDisplayMode = 0;     //what was the LCD displaying the last time?

// initialize variables for analogIn
const int sensorPin = A0;    // select the input pin for the potentiometer
const int gateInPin = A1;
const int FS1_pin = A2; //footswitch 1
const int FS2_pin = A3; //footswitch 2
const int gateOutPin = 12;
const int pushButton_pin=A4;     // Push button on rotary encoder
const int slaveSelectPin = 10; //slave select for the ADC
const int encoderPinA = 3;
const int encoderPinB = 2;
const int ribbonPin = A5;


//Define bit depths
int nBitsFS = 14;
int fullScale = ((int)pow(2,nBitsFS));
int nBitsADC = 10;  //Arduino's built-in ADC has 10 bits
int nBitsDAC = 12;  //MCP4922 DAC has 12 bits
float floatConstrainValue = ((float)fullScale)-1.0;
//int ribbonBitShift = nBitsFS-nBitsADC;
int ribbonBitShift = 4; //nBitsFS-nBitsADC = 14-10 = 4

// Initialize the measurement and smoothing variables
//int FS1_val = HIGH;
int FS2_val = HIGH; //initialize the two foot switches
debounceStruct FS2_state;  //set the FS2 states
//pitchStruct pitchInfo;
//recordAndLoopStateInfo loopStateInfo;

  
//define user set variables
//#define nOctaveOffset 9
//float octaveOffsets[nOctaveOffset] = {-4.0,-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0,4.0};
//int curOctaveOffsetIndex = 4;
////int octaveOffsetCounts[nOctaveOffset];
//int halfStepOffsets[nOctaveOffset];
//int shiftOneOctave = 0;
float pitchQuantizeFactor = 0;
//int curInp2OctaveOffsetIndex = 4;

//define operating speed
static int sampleRateHz=3000; //guess...will be updated in the code itself
int outputValueDAC, outputValueDAC2;
boolean computeNextValue = true;

void enableFootswitchesOnSecondInputs() {
  pinMode(FS1_pin,INPUT);
  pinMode(FS2_pin,INPUT);
  digitalWrite(FS1_pin,HIGH);  //enable pull-up
  digitalWrite(FS2_pin,HIGH);  //enable pull-up
} 

//setup routine
void setup() {
  pinMode(gateOutPin,OUTPUT);
  digitalWrite(gateOutPin,LOW);
  pinMode(ribbonPin,INPUT);
  enableFootswitchesOnSecondInputs();

  // initialize SPI:
  SPI.begin(); 
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);

  // initialize the serial communications:
  Serial.begin(115200);

  // set up the LCD and put text on the display
  lcd.begin(16, 2);  //columns and rows
  lcd.clear();
  lcd.print("Initializing");
   
  //Initialize pitch quantize factor
  pitchQuantizeFactor = ((float)fullScale) / 5.0 / 12.0;  //divide full scale into 5 octaves, then chromatically
 
  //setup the timer
  setupTimer();

  //final update to LCD
  lcd.clear();
  //lcd.print(getCurrentStateString(loopStateInfo) + "...");
  lcd.print("Running...");
}

//Code from: http://letsmakerobots.com/node/28278
void setupTimer(void) {
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  // compare match register 16MHz/256/2Hz
  int prescaler = 1;        //1 = CS10, 8 = CS11, 256 = CS12...others available too
  int match_value = ((16000000L)/((long int)sampleRateHz))/((long int)prescaler);
  //Serial << "match_value: " << match_value << endl;
//  OCR1A = 0x7a12;           // compare match register...16MHz/prescaler/sample_rate_hz
  OCR1A = match_value;
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS10);    // prescaler  (CS10=1)
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts
}

ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine
{
   dacWriteOneChannel(slaveSelectPin,outputValueDAC);
   computeNextValue = true;
}


int firstTime=1;
int printMessageToSerial = 0;
void loop() {
  static int gateValue1,gateValue2,loopGateValue,gateValue;
  static int calibratedValue1,calibratedValue2;
  static int calibratedValue;
  static int prevPitchValue = 0, prevGateValue = LOW;
  static float smoothedValue = 0;
  int inpPitch_halfSteps;
  static boolean redraw_lcd = false;
  int curChangeMode=0;
  static boolean modeWasChanged=false;
  int foo;
  static int outputValue = 0;
  static boolean newestIsRibbon = true;
  static int prev_FS2_val;
  static long int curMillis;
  static boolean useLoopOutput = false;
  static int prevOutputValue = 0;
  static unsigned long startMicros=0;
  static unsigned long curMicros=0;
  static unsigned int sampleCount=0;
  static boolean resetWAVE = false;
  static float curPhase_frac = 0;
  static float period_usec = 10000.0;
  
  //if (computeNextValue) {
    // what is the sample rate and memory usage
    sampleCount++;
    //checkSampleRate(sampleCount,sampleRateHz);
  
    //service the rotary encoder
    modeWasChanged=false;
    //curChangeMode = servicePushbutton(pushButton_pin,modeWasChanged,redraw_lcd);
    //serviceEncoder(curChangeMode,redraw_lcd);
    
    //service the footswitch to see what mode we're in
    //FS1_val = digitalRead(FS1_pin);
    //prev_FS2_val = FS2_val;  //needed for calibration of ribbon only
    //serviceFootswitchBasic(FS2_pin, redraw_lcd, FS2_state);
    //FS2_val = FS2_state.curState;
    //if (FS2_val == HIGH) { FS2_val = LOW; } else { FS2_val = HIGH; }
    
    //read input pair 1...from keyboard
    int prevValue = calibratedValue1;
    int prevGateValue1 = gateValue1;
    serviceInput1(gateInPin,sensorPin,gateValue1,calibratedValue1);
    int dPitch = calibratedValue1 - prevValue;
   
    if (((prevGateValue1 == LOW) && (gateValue1 == HIGH)) ||
       (dPitch > 100) || (dPitch < -100)) {
      //Serial << "resetWAVE: dPitch = " << dPitch << ", prevVal = " << prevValue << ", curVal = " << calibratedValue1 << endl;
      resetWAVE = true;
    } else {
      resetWAVE = false;
    }
      
    //compute the pitch...(2^15)/(5*12) = C = 273.0667 counts
    //HALF_STEP_SCALE = 2^(1/12)
    #define HALF_STEP_SCALE (1.059463094)
    //FREQ_LOW_C_HZ = 440 / pow(HALF_STEP_SCALE,-24-9);  //A=440, Asume Low C is -24-9 half steps down
    #define FREQ_LOW_C_HZ (32.7032)
    if (resetWAVE) {
      float half_steps_above_lowC = ((float)calibratedValue1)/pitchQuantizeFactor;
      //half_steps_above_lowC = (float)(int)(half_steps_above_lowC+0.5); //quantize
      //float freq_Hz = FREQ_LOW_C_HZ*pow(HALF_STEP_SCALE,half_steps_above_lowC);
      //float period_usec = 1.0E6/freq_Hz;
      int index = (int)(half_steps_above_lowC+0.5);
      period_usec = all_period_usec[index];
      //Serial << "reset: half_steps: " << index << endl;
    }
  
      
      //compute position in WAVE
      curMicros = micros();
      if (resetWAVE) startMicros = curMicros;
      curPhase_frac = ((float)(curMicros - startMicros)) / period_usec;
      unsigned int period_count = (unsigned int)curPhase_frac; //floor
      curPhase_frac = curPhase_frac - (float)period_count; //remove whole number
      //byte curPhase_samp = (byte)(128.0 * curPhase_frac);
      
  //    if (resetWAVE) {
  //       Serial << "pitch: inp: " << calibratedValue1 << ", half steps: " << half_steps_above_lowC << endl;
  //       //Serial << "    : freq Hz: " << freq_Hz << end;
  //       Serial << "    : period usec: " << period_usec << endl;
  //    }
      
      //compute output value
      #define OUT_SCALE (fullScale >> 1)
      prevOutputValue = outputValue;
      if ((period_count % 1) > 0) {
        //pulse wave
        if (curPhase_frac < 0.5) {
          outputValue = OUT_SCALE;
        } else {
          outputValue = 0;
        }
      } else {
        if (1) {
          //saw
          outputValue = OUT_SCALE -(int)(curPhase_frac * OUT_SCALE + 0.5); //round
        } else {
          //triangle
          outputValue = (int)(curPhase_frac * 2 * OUT_SCALE + 0.5); //round
          if (curPhase_frac < 0.5) {
            //first half of tri: ramp up...no change
          } else {
            //second half of tri: ramp down
            outputValue = (2*OUT_SCALE) - outputValue;
          }    
        }     
      }
      if (gateValue1 == LOW) outputValue = 0;
      //outputValue = (outputValue/4 + (7*(prevOutputValue/4))) / (8/4); //LP IIR Filter
      //outputValue = (outputValue + (2*(prevOutputValue)) + 1) / 3; //LP IIR Filter, the +1 is to round
      //outputValue = (outputValue + prevOutputValue)/ 2; //LP IIR Filter
  
      
    //outputValue = smoothedValue;  //trial by WEA
    outputValueDAC = constrain(outputValue,0,fullScale-1) >> (nBitsFS - nBitsDAC);
    //outputValueDAC2 = constrain(filterOutput,0,fullScale-1) >> (nBitsFS - nBitsDAC); //assume set low
    //Serial << "outputValue, filterOutput = " << outputValueDAC << " " << outputValueDAC2 << endl;
   
    //outputValueDAC2 = outputValueDAC2 >> (nBitsFS - nBitsDAC);
    //dacWrite(slaveSelectPin,outputValueDAC2,outputValueDAC);
    //dacWriteOneChannel(slaveSelectPin,outputValueDAC);
    //dacWrite(slaveSelectPin,outputValueDAC,outputValueDAC);
    //digitalWrite(gateOutPin,gateValue);
//    computeNextValue = false;
//  } else {
//    delayMicroseconds(25);
//  }
  
}

#define SAMPLE_RATE_COUNT_LIM (4000)
void checkSampleRate(unsigned int &sampleCount,int &sampleRateHz) {
  static unsigned long origMillis=0;
  static unsigned long newMillis=0;
  //static unsigned long SCALED_COUNT_LIM = (unsigned long)((long)(COUNT_LIM-1)*1000L);
  static float fSCALED_COUNT_LIM = (float)((long)(SAMPLE_RATE_COUNT_LIM -1)*1000L);
  
  if (sampleCount==SAMPLE_RATE_COUNT_LIM) {
    newMillis = millis();
    //sampleRateHz = 1000.0*((float)(COUNT_LIM-1)) / ((float)(newMillis-origMillis));
    sampleRateHz = (int)((fSCALED_COUNT_LIM) / ((float)(newMillis-origMillis)));
    //sampleRateHz = SCALED_COUNT_LIM / (newMillis-origMillis);
    sampleCount=0;
    origMillis = newMillis;

    check_mem();
 
    //Display sample rate and memory status
    Serial << "%% Mem: Heap: " << (int)heapptr << ", Stack: " << (int)stackptr << endl;
    Serial << "%% fs, Hz: " << sampleRateHz << endl;
    //Serial << "%% Sample Count = " << sampleCount-1 << ", Sample Rate = " << sampleRateHz << endl;
  }
}

void printDigitsToLCD(LiquidCrystal lcd, int value, int dispDigits,int col, int row) {
  int digits = 0;
  int i=0;
  int foo = 0;

  foo = abs(value);

  if (foo >= 10000) {
    digits = 5;
  } 
  else if (foo >= 1000) {
    digits = 4;
  } 
  else if (foo >= 100) {
    digits = 3;
  } 
  else if (foo >= 10) {
    digits = 2;
  } 
  else if (foo >= 0) {
    digits = 1;
  }
  if (value < 0) digits = digits+1;
  lcd.setCursor(col,row);
  for (i=0;i<dispDigits-digits;i++) {
    lcd.print(' ');
  }
  lcd.print(value);
}

#define gateThresh (11500)
void serviceInput1(int gateInPin,int sensorPin,int &gateValue1, int &calibratedValue1) {
  gateValue1 = analogRead(gateInPin) << (nBitsFS - nBitsADC);  //shift to fullscale 
  calibratedValue1 = analogRead(sensorPin) << (nBitsFS - nBitsADC); //shift to fullscale
 
  //process the gate signals and calibrate the reading, if necessary
  if (gateValue1 > gateThresh) { 
    gateValue1 = HIGH;
  } else {
    gateValue1 = LOW;
  }
}


