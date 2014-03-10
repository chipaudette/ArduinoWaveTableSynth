/* 
 Created: Chip Audette, Feb 2012
 Purpose: For use with keyboard...record gate and CV and loop out
 
 Re-Run Again: Sept 2013.
 Physical Setup:
     Input from Mon/Poly: Top-Right 1/8th inch Stereo Jack
         White (Left Chan) is the CV out
         Red (Right Chan) is Trigger Out
     Input from Footswitches: Bottom 1/8th inch stereo jack
         Black (tip) increases the WAVETABLE sweep speed (envelope)
         Red (ring) adds a little portamento
     Output to KB Amp: Top-Left 1/8th inch 4 conductor jack
         Yellow (3rd Chan) is audio out
 
 Wave Select:
    Select the waveform either from the ribbon or from the rotary encoder. 
 
 */

// include the library code:
#include <LiquidCrystal.h>
#include <SPI.h>
#include <Streaming.h>
#include <String.h>
#include <avr/pgmspace.h>
#include "Types.h"
//#include "rateTable64.h"
//#include "wT64_Synth.h"
//#include "wT64_allVow.h"
//#include "wT64_all.h"

//#include "rateTable128.h"
#include "rateTable128_noDiv.h"
#include "wT128_allVow.h";

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
const int encoderPinA = 3;  //this also happens to be PWM pin for timer2
const int encoderPinB = 2;
const int ribbonPin = A5;


//Define bit depths
const int nBitsFS = 14;
const int fullScale = ((int)pow(2,nBitsFS));
const int nBitsADC = 10;  //Arduino's built-in ADC has 10 bits
const int nBitsDAC = 12;  //MCP4922 DAC has 12 bits
const float floatConstrainValue = ((float)fullScale)-1.0;
const int ribbonBitShift = 4; //nBitsFS-nBitsADC = 14-10 = 4

// Initialize the measurement and smoothing variables
int FS1_val = HIGH;
int FS2_val = HIGH; //initialize the two foot switches
debounceStruct FS1_state,FS2_state;  //set the FS2 states
//pitchStruct pitchInfo;
//recordAndLoopStateInfo loopStateInfo;

volatile boolean useRibbonVal = false;
int ribbonBlockSpan;
int waveMixValue = 0;
boolean rememberRibbonVal = true;
#define RIBBON_SPAN (1024)

//define user set variables
float pitchQuantizeFactor = 0;
volatile int gateValue1 = LOW;
int curPhase_ind = 0;
int targWaveSkipFacBits = 0;
volatile int curWaveSkipFacBits=0;
int targSampleRateCounter=0,curSampleRateCounter=0;
float targPitch_Hz=100.0,curPitch_Hz=100.0;
float half_steps_above_lowC;
unsigned int period_count = 0;

//define LFO variables
unsigned long int startLFO_millis = 0;
#define LFO_ONE_SHOT (false)
#define ENV_PERIOD_MILLIS (100)     //was 80
#define LFO_PERIOD_MILLIS (1500)   //was 2000
int envPeriod_msec=LFO_PERIOD_MILLIS;
int all_envPeriod_msec[] = {LFO_PERIOD_MILLIS,ENV_PERIOD_MILLIS};
int LFOPeriod_msec=LFO_PERIOD_MILLIS;

#define N_WAVES_IN_SEQ (5)
#define LFO_END_LOOP_LEN (2)
//volatile int wave_start_ind = 0;
//int wave_seq[] = {0,1,3,2,4};
int wave_seq[] = {0,1,2,3,4}; //A
//int wave_seq[] = {12,13,14,15,0,1,2,3}; //8: O,A
//int wave_seq[] = {8,9,10,11,11,12,13,14,15,15,16,17,18,19}; //14: I,O,U
//int wave_seq[] = {4,5,6,7,7,8,9,10,11,11,4,5,6,7,7,8,9,10,11,11,12,13,14,15}; //24: E,I,E,I,O
//int wave_seq[] = {4,5,6,7}; //E
//int wave_seq[] = {8,9,10,11}; //I
//int wave_seq[] = {12,13,14,15}; //O
//int wave_seq[] = {16,17,18,19}; //U

int LFO_Block=0, wave_ind1,wave_ind2;

//define operating speed
int maxSampleRate_Hz = 7000; //was 4200
int outputValueDAC, outputValueDAC2;
//volatile boolean computeNextValue = true;
volatile unsigned long samplesSinceStartNote=0;
//volatile int missed_samples = 0;
boolean add_subOsc=false;

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
   
  //Initialize some constants
  pitchQuantizeFactor = ((float)fullScale) / 5.0 / 12.0;  //divide full scale into 5 octaves, then chromatically
  ribbonBlockSpan = RIBBON_SPAN / (N_WAVES-1);
 
  //setup the rotary encoder
  initEncoder(pushButton_pin,encoderPinA,encoderPinB);

  //setup the timer
  setupTimer(10000);
  updateSampleRate(false,1500);

  //final update to LCD
  lcd.clear();
  //lcd.print(getCurrentStateString(loopStateInfo) + "...");
  lcd.print("WaveSynCV RibSel");
  //lcd.setCursor(0,1);
  //lcd.print("Start Wave: ");
  printWaveSeq();
}

////Code from: http://letsmakerobots.com/node/28278
//void setupTimer(void) {
//
//  // compare match register 16MHz/256/2Hz
//  int prescaler = 1;        //1 = CS10, 8 = CS11, 256 = CS12...others available too
//  int match_value = ((16000000L)/((long int)sampleRateHz))/((long int)prescaler);
//  setupTimer(match_value);
//}

void setupTimer(const int &match_value_counts) {
   noInterrupts();           // disable all interrupts
   //we're going to use timer1
  TCCR1A = 0;  //enable normal operation (mode 0)
  TCCR1B = 0;  
  TCNT1  = 0; //reset the counter

  // compare match register
  //  OCR1A = 0x7a12;           // compare match register...16MHz/prescaler/sample_rate_hz
  OCR1A = match_value_counts;
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS10);    // prescaler  (CS10=1)
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts
}

void adjustTimer(const int &match_value_counts) {
  // noInterrupts();           // disable all interrupts
  //TCCR1A = 0;
  //TCCR1B = 0;
  //TCNT1  = 0;

  // compare match register
  //  OCR1A = 0x7a12;           // compare match register...16MHz/prescaler/sample_rate_hz
  OCR1A = match_value_counts;
  //TCCR1B |= (1 << WGM12);   // CTC mode
  //TCCR1B |= (1 << CS10);    // prescaler  (CS10=1)
  //TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  //interrupts();             // enable all interrupts
}

int prevOutputValue = 0;
int outputValue = 0;
volatile int weight1_frac16,weight2_frac16;
int fooValA,fooValB,fooVal1,fooVal2,fooWeightScale;
volatile int rib_wave_ind1=0;
volatile int rib_weight=0;
ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine
{
  static unsigned long fooSamples;
  static int foo,fooS;
  
   //digitalWrite(gateOutPin,HIGH);
   //computeNextValue = true;
   samplesSinceStartNote++;
   fooSamples = samplesSinceStartNote;
   
   //compute sample for the wave
   curPhase_ind = (int)(fooSamples % (N_POINTS_WAVE >> curWaveSkipFacBits));
   curPhase_ind = curPhase_ind << curWaveSkipFacBits;
   if (curPhase_ind < (N_POINTS_WAVE>>1)) {
     digitalWrite(gateOutPin,HIGH);
   } else {
     digitalWrite(gateOutPin,LOW);
   }
   
   //get wave values from wave table
   //wave_ind1 = constrain(wave_ind1,0,N_WAVES-1);
   while (wave_ind1 < 0) wave_ind1 += N_WAVES;  while(wave_ind1 >= N_WAVES) wave_ind1 -= N_WAVES;
   //wave_ind2 = constrain(wave_ind2,0,N_WAVES-1);
   while (wave_ind2 < 0) wave_ind2 += N_WAVES;  while(wave_ind2 >= N_WAVES) wave_ind2 -= N_WAVES;
   fooVal1=0;fooVal2=0;
   fooVal1 = ((int)pgm_read_word_near(&(waveTable[wave_ind1][curPhase_ind]))) >> 4; //get waveTable value from PROGMEM, then divide by 16/2
   fooVal2 = ((int)pgm_read_word_near(&(waveTable[wave_ind2][curPhase_ind]))) >> 4; //get waveTable value from PROGMEM, then divide by 16/2

   
   
   //compute output value
   outputValue = ((fooVal1*weight1_frac16 + fooVal2 * weight2_frac16));  //the weights go from 0->16, that's why I pre-divided by 16 (shift by 4 bits) in the previous code
   if (gateValue1 == LOW) outputValue = 0;
  
  //filtering
  //outputValue = (outputValue/4 + (7*(prevOutputValue/4))) / (8/4); //LP IIR Filter
  //outputValue = (outputValue + (2*(prevOutputValue)) + 1) / 3; //LP IIR Filter, the +1 is to round
  //outputValue = (outputValue + prevOutputValue)/ 2; //LP IIR Filter
  
  //send output to DAC
  outputValue = outputValue + (fullScale >> 1);  //shift the baseline from zero to the middle of fullscale
  //if ((outputValue < 0) | (outputValue > fullScale)) Serial << "out: " << outputValue << endl;
  outputValueDAC = constrain(outputValue,0,fullScale-1) >> (nBitsFS - nBitsDAC);
  dacWriteOneChannel(slaveSelectPin,outputValueDAC);
  //digitalWrite(gateOutPin,LOW);
}
  
 
void printStartWaveIndex(void) {
  lcd.setCursor(12,1);
  if (wave_seq[0] < 10) lcd.print(" ");
  lcd.print(wave_seq[0]);
}; 
 
void printWaveSeq(void) {
  lcd.setCursor(0,1);
  lcd.print("                ");
  lcd.setCursor(0,1);
  for (int i=0;i<N_WAVES_IN_SEQ;i++) {
    if (i > 0) lcd.print(",");
    if (useRibbonVal) {
      lcd.print((wave_seq[i]+rib_wave_ind1) % N_WAVES);
    } else {
      lcd.print(wave_seq[i]);
    }
  }
}
    
void incrementWaveIndices(const int &encoderChange)
{
  for (int i=0; i < N_WAVES_IN_SEQ; i++) {
      wave_seq[i] += encoderChange;
      while (wave_seq[i] < 0) wave_seq[i] += N_WAVES;
      while (wave_seq[i] >= N_WAVES) wave_seq[i] -= N_WAVES;
  }
}

void setWaveIndices(const int &startVal)
{
  int removeVal = wave_seq[0];
  incrementWaveIndices(startVal - removeVal);
}
    
int printMessageToSerial = 0;
void loop() {
  static int prevKybdValue,kybdValue;
  static int  prevGateValue = LOW,gateValue;
  static int dPitch;
  static int ribbonVal=0;
  static int ribbonGate=LOW;
  static float smoothedValue = 0;
  //int inpPitch_halfSteps;
  static boolean redraw_lcd = false;
  //int curChangeMode=0;
  //static boolean modeWasChanged=false;
  int foo;
  static int prev_FS2_val;
  static unsigned long int curMillis;
  static unsigned long int loopCount=0;
  static boolean resetWAVE = false;
  static boolean glideOn = true;
  static int loopRate_Hz = 1000;
  static int countForDisplayUpdate = 0;
  static boolean updateDisplay = true;
  
  // what is the sample rate and memory usage
  loopCount++;
  curMillis = millis();
  //checkLoopRate(loopCount,loopRate_Hz);
  
  //service the rotary encoder
  //modeWasChanged=false;
  //curChangeMode = servicePushbutton(pushButton_pin,modeWasChanged,redraw_lcd);
  //serviceEncoder();
  int encoderChange = getEncoderChange();
  if (encoderChange != 0) {
    updateDisplay = true;
    incrementWaveIndices(encoderChange);
  }
  
  //service the footswitch to see what mode we're in
  FS1_val = digitalRead(FS1_pin);
  //serviceFootswitchBasic(FS1_pin,redraw_lcd,FS1_state);
  prev_FS2_val = FS2_val;  //needed for calibration of ribbon only
  serviceFootswitchBasic(FS2_pin, redraw_lcd, FS2_state);
  //FS1_val = FS1_state.curState;
  if (FS1_val == HIGH) {envPeriod_msec = all_envPeriod_msec[0];} else { envPeriod_msec = all_envPeriod_msec[1];}
  //if (FS1_val == HIGH) { maxSampleRate_Hz=9000;} else {  maxSampleRate_Hz =2400;}
  FS2_val = FS2_state.curState;
  if (FS2_val == HIGH) { FS2_val = LOW; } else { FS2_val = HIGH; }
  glideOn=false;
  if (FS2_val == HIGH) glideOn=true;  
  
  //read input from keyboard
  resetWAVE = false;
  
  prevKybdValue = kybdValue;
  prevGateValue = gateValue1;
  int fooGateIn=LOW;
  serviceInput1(gateInPin,sensorPin,fooGateIn,kybdValue);
  gateValue1 = fooGateIn;  //gateInPin is a volatile, so can't be passed in
  dPitch = kybdValue - prevKybdValue;
   
  if ((prevGateValue == LOW) && (gateValue1 == HIGH)) {
    //retrigger note and the LFO
    samplesSinceStartNote = 0;
    resetWAVE = true;
    startLFO_millis = curMillis;
  } else if ((dPitch > 100) || (dPitch < -100)) {
    //retrigger just the note
    resetWAVE = true;
  }
  
  //compute the pitch...(2^15)/(5*12) = C = 273.0667 counts
  //HALF_STEP_SCALE = 2^(1/12)
  #define HALF_STEP_SCALE (1.059463094)
  //FREQ_LOW_C_HZ = 440 / pow(HALF_STEP_SCALE,-24-9);  //A=440, Asume Low C is -24-9 half steps down
  #define FREQ_LOW_C_HZ (32.7032)
  if (resetWAVE) {
    
    half_steps_above_lowC = ((float)kybdValue)/pitchQuantizeFactor;
    //half_steps_above_lowC = (float)(int)(half_steps_above_lowC+0.5); //quantize
    //float freq_Hz = FREQ_LOW_C_HZ*pow(HALF_STEP_SCALE,half_steps_above_lowC);
    //float period_usec = 1.0E6/freq_Hz;
    int index = (int)(half_steps_above_lowC+0.5);
    index = constrain(index,0,N_PITCH_TABLE-1);
    
    //set timer and set period
    //targSampleRateCounter = all_timer_count[index];
    //targWaveSkipFacBits = all_wave_skip_fac_bits[index];
    //curSampleRateCounter = all_timer_count[index];
    //curWaveSkipFacBits = all_wave_skip_fac_bits[index];
    //setupTimer(all_timer_count[index]);
    targPitch_Hz = all_pitch_Hz[index];
    //Serial << "targPitch_Hz = " << targPitch_Hz << endl;
   
    //reset LFO
    //startLFO_millis = curMillis;
  }
   
  //update sample rate
  updateSampleRate(glideOn,loopRate_Hz);
   
  //read input from ribbon
  static int prevRibVal = 0;
  serviceRibbon_span(ribbonPin,RIBBON_SPAN-1,ribbonVal,ribbonGate);
  interpretRibbon(ribbonVal,ribbonGate);
  if (ribbonGate==HIGH) {
    //ribbon is pressed
    startLFO_millis = curMillis;
    
    if (ribbonVal != prevRibVal) {
      setWaveIndices(ribbonVal);
      prevRibVal = ribbonVal;
      updateDisplay = true;
    }
  }
  
  //update the LFO/Envelope
  updateLFO(curMillis);

  //update the LCD
  countForDisplayUpdate++;
  if (countForDisplayUpdate > 500) { //update every X times around
    if (gateValue1 == LOW) { //if no key is pressed
      if (updateDisplay) { //only update if the values have changed
        printWaveSeq();  //print the wave sequency to the screen
        updateDisplay = false;
      }
      countForDisplayUpdate = 0; //reset the counter
    }
  }

}

#define SAMPLE_RATE_COUNT_LIM (2000)
void checkLoopRate(unsigned long &loopCount,int &loopRate_Hz) {
  static unsigned long origMillis=0;
  static unsigned long newMillis=0;
  static float fSCALED_COUNT_LIM = (float)((long)(SAMPLE_RATE_COUNT_LIM -1)*1000L);
  //static int loopRate_Hz = 1000;
  
  if (loopCount==SAMPLE_RATE_COUNT_LIM) {
    newMillis = millis();
    loopRate_Hz = (int)((fSCALED_COUNT_LIM) / ((float)(newMillis-origMillis)));
    loopCount=0;
    origMillis = newMillis;

    check_mem();
 
    //Display sample rate and memory status
    //Serial << "%% Mem: Heap: " << (int)heapptr << ", Stack: " << (int)stackptr << endl;
    //Serial << "%% loop rate, Hz: " << loopRate_Hz << endl;
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

void interpretRibbon(const int &ribbonVal,const int &ribbonGate) {
  #define nPrevRibbonVals 6
  static int prevRibbonVals[nPrevRibbonVals],prevRibbonValsCounter=0;
  static int prevRibbonVal=0, prevRibbonGate=LOW;
  static boolean everPressedRibbon = false;
 
  waveMixValue = ribbonVal;
  if (ribbonGate==HIGH) {
    useRibbonVal=true;
    if (prevRibbonGate == LOW) {
      for (int i=0;i<nPrevRibbonVals;i++) {
        prevRibbonVals[i]=ribbonVal;  //initialize the whole buffer
        prevRibbonValsCounter=-1;
      }
    }
    prevRibbonValsCounter = (prevRibbonValsCounter+1) % nPrevRibbonVals;
    prevRibbonVals[prevRibbonValsCounter]=ribbonVal; //add current ribbon value to buffer
    everPressedRibbon = true;
  } else {
    if (rememberRibbonVal & everPressedRibbon) {
      //Serial << "Gate LOW: prevRibbon " << prevRibbonVal << ", cur ribbon " << ribbonVal << endl;
      int getIndex = prevRibbonValsCounter - (nPrevRibbonVals-1);
      if (getIndex < 0) getIndex += nPrevRibbonVals;
      waveMixValue = prevRibbonVals[getIndex]; //get historical value from buffer
      useRibbonVal = true; 
    } else {
      //Serial << "Gate LOW: useRibbonVal will be false" << endl;
      useRibbonVal = false; 
    }
  }

  if (useRibbonVal) {
     rib_wave_ind1 = waveMixValue / ribbonBlockSpan;
     rib_weight = ((waveMixValue - rib_wave_ind1* ribbonBlockSpan)*16)/ribbonBlockSpan;  //should span zero to 16
  }
  
  prevRibbonGate = ribbonGate; //save for next time
}



void updateLFO(const unsigned long int &curMillis) {
  static unsigned long int sinceStart_millis;
  static int rib_offset;
  static int foo_wave_ind1,foo_wave_ind2;
  
  //compute time since start of LFO
  sinceStart_millis = curMillis - startLFO_millis;
  
  //adjust start to account for ribbon setting
  //if (useRibbonVal) {
  //  rib_offset = rib_weight*(LFOPeriod_msec / 16);
  //  sinceStart_millis += (unsigned long int)rib_offset;
    //Serial << "rib: w, off: " << rib_weight << " " << rib_offset << endl;
  //}
   
  //compute position and phase of the LFO
  int nPeriods = sinceStart_millis / envPeriod_msec;
  int withinPeriod_frac16;
  if (nPeriods < (N_WAVES_IN_SEQ-1)) {
    //within the envelope period
    withinPeriod_frac16 = (sinceStart_millis - nPeriods*envPeriod_msec) / (envPeriod_msec/16);  //should be zero to 16
  } else {
    sinceStart_millis -= (envPeriod_msec * (N_WAVES_IN_SEQ-1));
    nPeriods = sinceStart_millis / LFOPeriod_msec;
    withinPeriod_frac16 = (sinceStart_millis - nPeriods*LFOPeriod_msec) / (LFOPeriod_msec/16);  //should be zero to 16
    nPeriods += (N_WAVES_IN_SEQ-1);
  }
  withinPeriod_frac16 = constrain(withinPeriod_frac16,0,16);

  //find the position within the WAV sequence
  if (nPeriods < (N_WAVES_IN_SEQ -1)) {
    foo_wave_ind1 = wave_seq[nPeriods];
    foo_wave_ind2 = wave_seq[nPeriods+1];
  } else {
    if (LFO_ONE_SHOT) {
      foo_wave_ind1 = wave_seq[N_WAVES_IN_SEQ-1];
      foo_wave_ind2 = foo_wave_ind1;
    } else {
      //oscillate the last two
      if ((N_WAVES_IN_SEQ % 2) == 0) {
        //even length of sequence...note nPeriods counts from zero
        if ((nPeriods % 2) == 1) {
          //we're in an even numbered periods
          foo_wave_ind1 = wave_seq[N_WAVES_IN_SEQ-1];
          foo_wave_ind2 = wave_seq[N_WAVES_IN_SEQ-2];
        } else { 
          //we're in an odd numbered period...note nPeriods counts from zero
          foo_wave_ind1 = wave_seq[N_WAVES_IN_SEQ-2];
          foo_wave_ind2 = wave_seq[N_WAVES_IN_SEQ-1];
        }
      } else {
        //odd length of sequency...note nPeriods counts from zero
         if ((nPeriods % 2) == 1) {
          //we're in an even numbered periods
          foo_wave_ind1 = wave_seq[N_WAVES_IN_SEQ-2];
          foo_wave_ind2 = wave_seq[N_WAVES_IN_SEQ-1];
        } else { 
          //we're in an odd numbered period...note nPeriods counts from zero
          foo_wave_ind1 = wave_seq[N_WAVES_IN_SEQ-1];
          foo_wave_ind2 = wave_seq[N_WAVES_IN_SEQ-2];
        }
      }
    }
  }
  
  //adjust wave block for the ribbon
  if (useRibbonVal) {
    foo_wave_ind1 += rib_wave_ind1;
    foo_wave_ind1 = foo_wave_ind1 % N_WAVES;
    foo_wave_ind2 += rib_wave_ind1;
    foo_wave_ind2 = foo_wave_ind2 % N_WAVES;
  }
  
  //set the volatile variables
  wave_ind1 = foo_wave_ind1;
  weight1_frac16 = 16 - withinPeriod_frac16;
  wave_ind2 = foo_wave_ind2;
  weight2_frac16 = withinPeriod_frac16;
  //Serial << "b1,b2,w: " << wave_ind1 << ", " << wave_ind2 << ", " << weight2_frac16 << endl;
}
    
void updateSampleRate(const boolean &glideOn,const int &loopRate_Hz) {
  static int dCounter=0,counterIncrement=0;;
  static float reqSampleRate_Hz=0;
  static int preScaler = 1;
  int fooWaveSkipFacBits=0;  
  static int dPitch_10=0,pitchIncr_10=0;
  
  dPitch_10 = (int)(10.0*(targPitch_Hz - curPitch_Hz));
  if (abs(dPitch_10) > 0) {
    if (glideOn) {
      pitchIncr_10 = dPitch_10 / (loopRate_Hz / 32);
      if (pitchIncr_10 == 0) {
        if (dPitch_10 > 0) { pitchIncr_10 += 1; } else { pitchIncr_10 -= 1; }; //might have rounded off completely...so force it up
      }
      curPitch_Hz += ((float)pitchIncr_10)/10.0;
      //Serial << "Tp, dP, Pi, Cp: " << targPitch_Hz << " " << dPitch_10 << " " << pitchIncr_10 << " " << curPitch_Hz << endl;
    } else {
      curPitch_Hz = targPitch_Hz;
    }

    //find the best counter and skip factor
    fooWaveSkipFacBits = 1;  //I decided that I don't like the full 128 sample waveforms at fs=4kHz
    //reqSampleRate_Hz = (int)((16000000L) / ((long)preScaler) / ((long)curSampleRateCounter));
    reqSampleRate_Hz = curPitch_Hz * (float)((N_POINTS_WAVE >> fooWaveSkipFacBits));
    //Serial << "cP, init fs = " << curPitch_Hz << " " << reqSampleRate_Hz << endl;
    while (reqSampleRate_Hz > ((float)maxSampleRate_Hz)) {
      fooWaveSkipFacBits++;
      reqSampleRate_Hz = curPitch_Hz * (float)((N_POINTS_WAVE  >> fooWaveSkipFacBits));
      //Serial << "bits, fs = " << fooWaveSkipFacBits << " " << reqSampleRate_Hz << endl;
    }
    curSampleRateCounter = (int)((16000000.0) / ((long)reqSampleRate_Hz));
    //Serial << "curSampleRateCounter = " << curSampleRateCounter << endl;

    //set the values
    adjustTimer(curSampleRateCounter);
    curWaveSkipFacBits = fooWaveSkipFacBits; //set the volatile value
    //Serial << "dP, curCount, curSkip, fs: " << dPitch_10 << " " << curSampleRateCounter << " " << curWaveSkipFacBits << " " << reqSampleRate_Hz << endl;
  } 

};
    
  
