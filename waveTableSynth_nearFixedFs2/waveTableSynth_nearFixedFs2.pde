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
//debounceStruct FS2_state;  //set the FS2 states
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
volatile boolean computeNextValue = true;
volatile unsigned long samplesSinceStartNote=0;
volatile int missed_samples = 0;

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

  // compare match register 16MHz/256/2Hz
  int prescaler = 1;        //1 = CS10, 8 = CS11, 256 = CS12...others available too
  int match_value = ((16000000L)/((long int)sampleRateHz))/((long int)prescaler);
  setupTimer(match_value);
}

void setupTimer(const int &match_value_counts) {
   noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  // compare match register
  //  OCR1A = 0x7a12;           // compare match register...16MHz/prescaler/sample_rate_hz
  OCR1A = match_value_counts;
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS10);    // prescaler  (CS10=1)
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts
}

volatile int gateValue1 = LOW;
float curPhase_frac = 0;
float curPhase_frac2 =0;
//float period_usec = 10000.0;
int period_samps = 100;
float half_steps_above_lowC;
unsigned int period_count = 0;
int prevOutputValue = 0;
int outputValue = 0;
int outVal[7];
int weight;
int fooVal1,fooVal2,fooWeightScale;
int LFO_Block=0, block_ind1,block_ind2;
ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine
{
   digitalWrite(gateOutPin,HIGH);
   //computeNextValue = true;
   samplesSinceStartNote++;
   
  //compute position in WAVE     
  curPhase_frac = ((float)(samplesSinceStartNote % period_samps))/ ((float)period_samps);
  //curPhase_frac2 = ((float)((2*samplesSinceStartNote) % period_samps))/ ((float)period_samps);
   
  //compute wave table values
  #define OUT_SCALE (fullScale >> 1)
  outVal[0] = (OUT_SCALE -(int)(curPhase_frac * OUT_SCALE + 0.5))-(OUT_SCALE>>1); //round
  if (curPhase_frac < 0.15) { outVal[1] = OUT_SCALE>>1; } else { outVal[1] = -(OUT_SCALE>>1); }
  //if ((curPhase_frac >= 0.5) & (curPhase_frac < 0.7)) { outVal[2] = OUT_SCALE>>1; } else { outVal[2] = -(OUT_SCALE>>1); }
  outVal[2]=outVal[1];
  if (curPhase_frac >= 0.5) {
    if (curPhase_frac < 0.65) {
      outVal[2] = OUT_SCALE>>1; 
    } else { 
      outVal[2] = -(OUT_SCALE>>1); 
    }
  }
  outVal[3]=outVal[2];
  if (curPhase_frac < 0.3) {
    outVal[4] = OUT_SCALE >> 1;
  } else if (curPhase_frac < 0.5) {
    outVal[4] = -OUT_SCALE >> 1; 
  } else if (curPhase_frac < 0.8) {
    outVal[4] = OUT_SCALE >> 4;
  } else {
    outVal[4] = -OUT_SCALE >> 4; 
  }
  if (curPhase_frac < 0.4) { outVal[5] = OUT_SCALE>>2; } else { outVal[5] = -(OUT_SCALE>>2); }
  //if (curPhase_frac < 0.2) { outVal[3] = OUT_SCALE>>1; } else { outVal[3] = -(OUT_SCALE>>1);}
  //outVal[3] = OUT_SCALE -(int)(curPhase_frac2 * OUT_SCALE + 0.5); //round
  //outVal[3]=outVal[0];
 
   //outVal[0]=outVal[0]>>1; //attenuate
   //outVal[1]=outVal[1]>>1; //attenuate
   //outVal[2]=outVal[2]>>1;//attenuate
   outVal[3]=outVal[3]>>1;//attenuate
   outVal[4]=outVal[4]>>2;////attenuate a little more
   outVal[5]=outVal[5]>>1;//attenuate
   outVal[6]=outVal[6]>>1;//attenuate
 
  //outputValue = (constrain((4000-samplesSinceStartNote)/1000,0,4)*outVal2
  //   + constrain(samplesSinceStartNote/1000,0,4)*outVal1)/4;
  #define MAX_LFO_WAVE_STEP (6)
  #define LFO_ONE_SHOT (false)
  #define LFO_PERIOD_SAMPS (4096)
  prevOutputValue = outputValue;
  LFO_Block = (int)(samplesSinceStartNote / LFO_PERIOD_SAMPS);
  if (LFO_ONE_SHOT) {
    block_ind1 = constrain(LFO_Block,0,MAX_LFO_WAVE_STEP-1);
    block_ind2 = constrain(LFO_Block+1,0,MAX_LFO_WAVE_STEP-1);
  } else {
    if (LFO_Block < (MAX_LFO_WAVE_STEP-1)) {
      block_ind1 = LFO_Block;
      block_ind2 = LFO_Block+1;
    } else {
      //loop last two
      //block_ind1 = ((LFO_Block-1) % (MAX_LFO_WAVE_STEP-1))+2;
      //block_ind2 = ((LFO_Block-1+1) % (MAX_LFO_WAVE_STEP-1))+2;
      block_ind1 = ((LFO_Block-((MAX_LFO_WAVE_STEP) % 2)) % 2) + (MAX_LFO_WAVE_STEP-2);
      block_ind2 = ((LFO_Block+1-((MAX_LFO_WAVE_STEP) % 2)) % 2) + (MAX_LFO_WAVE_STEP-2);
    }
  }
  fooVal1 = outVal[block_ind1] >> 4; //divide by 16
  fooVal2 = outVal[block_ind2] >> 4;
  fooWeightScale = (int)(samplesSinceStartNote - LFO_Block*LFO_PERIOD_SAMPS);
  weight = (fooWeightScale) >> 8; //this value is log2(LFO_PEROD/16).  So, for LFO=4096, andser is 256 =>  bit shift by 8.  2048 => 7.  8192 =>9
  weight = constrain(weight,0,16);
  //weight = 0;
  outputValue = fooVal2 * weight + fooVal1*(16 - weight); //the "16" corresponds to the bit shift by "4" done for fooVal1 and fooVal2
  if (gateValue1 == LOW) outputValue = 0;
  
  //filtering
  //outputValue = (outputValue/4 + (7*(prevOutputValue/4))) / (8/4); //LP IIR Filter
  //outputValue = (outputValue + (2*(prevOutputValue)) + 1) / 3; //LP IIR Filter, the +1 is to round
  //outputValue = (outputValue + prevOutputValue)/ 2; //LP IIR Filter
  
  outputValue = outputValue + (fullScale >> 2);  //shift the baseline from zero to the middle of fullscale
  outputValueDAC = constrain(outputValue,0,fullScale-1) >> (nBitsFS - nBitsDAC);
  dacWriteOneChannel(slaveSelectPin,outputValueDAC);
  //computeNextValue = true;
  digitalWrite(gateOutPin,LOW);
}


int printMessageToSerial = 0;
void loop() {
  static int gateValue2;
  static int calibratedValue1;
  static int calibratedValue;
  static int prevPitchValue = 0, prevGateValue = LOW;
  static int prevValue, prevGateValue1;
  static int dPitch;
  static float smoothedValue = 0;
  //int inpPitch_halfSteps;
  //static boolean redraw_lcd = false;
  //int curChangeMode=0;
  //static boolean modeWasChanged=false;
  int foo;
  //static boolean newestIsRibbon = true;
  static int prev_FS2_val;
  static long int curMillis;
  //static boolean useLoopOutput = false;
  //static unsigned long startMicros=0;
  //static unsigned long curMicros=0;
  static unsigned int sampleCount=0;
  static boolean resetWAVE = false;

  
  //if (computeNextValue) {
    // what is the sample rate and memory usage
    sampleCount++;
    //checkSampleRate(sampleCount,sampleRateHz);
  
    //service the rotary encoder
    //modeWasChanged=false;
    //curChangeMode = servicePushbutton(pushButton_pin,modeWasChanged,redraw_lcd);
    //serviceEncoder(curChangeMode,redraw_lcd);
    
    //service the footswitch to see what mode we're in
    //FS1_val = digitalRead(FS1_pin);
    //prev_FS2_val = FS2_val;  //needed for calibration of ribbon only
    //serviceFootswitchBasic(FS2_pin, redraw_lcd, FS2_state);
    //FS2_val = FS2_state.curState;
    //if (FS2_val == HIGH) { FS2_val = LOW; } else { FS2_val = HIGH; }
    
    //read input pair 1...from keyboard
    resetWAVE = false;
    if ((samplesSinceStartNote % 1UL) == 0) {
      prevValue = calibratedValue1;
      prevGateValue1 = gateValue1;
      int fooGateIn=LOW;
      serviceInput1(gateInPin,sensorPin,fooGateIn,calibratedValue1);
      gateValue1 = fooGateIn;  //gateInPin is a volatile, so can't be passed in
      dPitch = calibratedValue1 - prevValue;
   
      if (((prevGateValue1 == LOW) && (gateValue1 == HIGH)) ||
         (dPitch > 100) || (dPitch < -100)) {
          //Serial << "resetWAVE: dPitch = " << dPitch << ", prevVal = " << prevValue << ", curVal = " << calibratedValue1 << endl;
          resetWAVE = true;
          samplesSinceStartNote = 0;
      }
      
      //compute the pitch...(2^15)/(5*12) = C = 273.0667 counts
      //HALF_STEP_SCALE = 2^(1/12)
      #define HALF_STEP_SCALE (1.059463094)
      //FREQ_LOW_C_HZ = 440 / pow(HALF_STEP_SCALE,-24-9);  //A=440, Asume Low C is -24-9 half steps down
      #define FREQ_LOW_C_HZ (32.7032)
      if (resetWAVE) {
        half_steps_above_lowC = ((float)calibratedValue1)/pitchQuantizeFactor;
        //half_steps_above_lowC = (float)(int)(half_steps_above_lowC+0.5); //quantize
        //float freq_Hz = FREQ_LOW_C_HZ*pow(HALF_STEP_SCALE,half_steps_above_lowC);
        //float period_usec = 1.0E6/freq_Hz;
        int index = (int)(half_steps_above_lowC+0.5);
        index = constrain(index,0,N_PITCH_TABLE-1);
        //period_usec = all_period_usec[index];
        //Serial << "reset: half_steps: " << index << endl;
        
        //update screen
        
        //lcd.setCursor(10,0);
        //lcd.print(missed_samples);
//        lcd.setCursor(0,1); //col,row
//        lcd.print("I=              "); //clear row
//        lcd.setCursor(2,1);//col,row
//        if (index < 10) lcd.print(" ");
//        lcd.print(index);
//        lcd.print(", fs=");
//        lcd.print((16000000/1/all_timer_count[index]));
//        lcd.print("Hz");
               
        //set timer and set period
        setupTimer(all_timer_count[index]);
        period_samps = all_period_samp[index];
        
      }
    }
    delayMicroseconds(25);
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


