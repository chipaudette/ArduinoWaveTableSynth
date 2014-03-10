
#define modeNoChange 0
//#define modeSustainPitchChange 1
#define modeOffsetChange 1
#define modeChangeRibbonMode 2
#define modeQuantizedRibbonChange 3
#define modeChangeInp2Octave 4
#define modeChangeBothControllerMethod 5
#define modeChangeFilterKeyTracking 6
//#define modeScaleFacChange 3
//#define modeSmoothChange 4
#define nModes 7

//encoder variables
volatile int encoderPos = 0;
boolean A_set = false;
boolean B_set = false;

void initEncoder(const int pushButton_pin,const int encoderPinA,const int encoderPinB) {
  pinMode(pushButton_pin,INPUT);
  digitalWrite(pushButton_pin, HIGH);
  pinMode(encoderPinA, INPUT); 
  digitalWrite(encoderPinA, HIGH);       // turn on pullup resistor
  pinMode(encoderPinB, INPUT); 
  digitalWrite(encoderPinB, HIGH);       // turn on pullup resistor

  //initialize interrupt variables
  A_set = digitalRead(encoderPinA) == HIGH;
  B_set = digitalRead(encoderPinB) == HIGH;

  // setup the interrupts for the encoder services
  attachInterrupt(0, doEncoderA, CHANGE);
  //attachInterrupt(1, doEncoderB, CHANGE);
}

// Interrupt on A changing state
void doEncoderA(){
  static boolean prev_B_set = HIGH;

  // Test transition
  A_set = digitalRead(encoderPinA) == HIGH;
  prev_B_set = B_set;
  B_set = digitalRead(encoderPinB) == HIGH; 

  if (prev_B_set != B_set) {
    // and adjust counter + if A leads B
    encoderPos -= (A_set != B_set) ? +1 : -1;  //reversed sign because of my particular wiring
    //Serial << "doEncoderA: " << encoderPos << endl;
  }
}
void doEncoderA_orig(){
  // Test transition
  A_set = digitalRead(encoderPinA) == HIGH;

  // and adjust counter + if A leads B
  encoderPos += (A_set != B_set) ? +1 : -1;  //reversed sign because of my particular wiring
}

// Interrupt on B changing state
void doEncoderB(){
  // Test transition
  B_set = digitalRead(encoderPinB) == HIGH;
  // and adjust counter + if B follows A
  encoderPos += (A_set == B_set) ? +1 : -1;  //reversed sign because of my particular wiring
  //Serial << "doEncoderB: " << encoderPos << endl;
}

#define WAIT_FOR_PRESS (0)
#define WAIT_FOR_RELEASE (1)
int changeModeCounter=0;
int changeModeCounter_resetValue = (4*1000);  //Use 4 x sampleRate
int servicePushbutton(const int pin,boolean &modeWasChange,boolean &endChangeMode) {
  static int state = WAIT_FOR_PRESS;
  static int pushButton_accumVal = 0;
  const int pushButton_thresh(20);  // was 10 for sample rate of 360
  static int curChangeMode = modeNoChange;

  changeModeCounter = constrain(changeModeCounter-1,0,10000);
  if (changeModeCounter <= 0) {
    if (curChangeMode != 0) endChangeMode=true;
    curChangeMode=0;
  }

  //read value and accumulate
  if (digitalRead(pushButton_pin) == HIGH) {
    //button is not pressed
    pushButton_accumVal--;
  } 
  else {
    //button is pressed
    pushButton_accumVal++;
  }
  pushButton_accumVal = constrain(pushButton_accumVal,0,pushButton_thresh);

  switch (state) {
  case WAIT_FOR_PRESS:
    if (pushButton_accumVal >= pushButton_thresh) {
      //changeKey(1,changed_state);
      curChangeMode = (curChangeMode+1) % nModes;
      modeWasChange = true;
      changeModeCounter=changeModeCounter_resetValue;
      if (curChangeMode == modeNoChange) {
        changeModeCounter=0;
        endChangeMode=true;
      }
      Serial << "SwitchAndKnob: changedMode: curChangeMode = " << curChangeMode << endl;
      state = WAIT_FOR_RELEASE; //change state...now wait for the release
    }
    break;
  case WAIT_FOR_RELEASE:
    if (pushButton_accumVal <= 0) {
      state = WAIT_FOR_PRESS;
    }
    break;
  }
  return curChangeMode;
}

int getChangeModeCounter() {
  return changeModeCounter;
}


void serviceEncoder(const int changeModeFlag, boolean &redraw_lcd) {
  static int encoderDelayCount=0;
  encoderDelayCount=constrain(encoderDelayCount-1,0,200);
  if (encoderPos != 0) {

    if (encoderDelayCount <= 0) {
      //Serial << "ChangingKey: encoderPos: " << encoderPos << endl;
      int change = constrain(encoderPos,-1,1);
      switch (changeModeFlag) {
        case modeNoChange:
          //no action
          break;
        case modeOffsetChange:
          changeOctaveOffset(change);
          break;
        case modeQuantizedRibbonChange:
          changeQuantizedRibbon(change);
          break; 
        case modeChangeRibbonMode:
          changeRibbonMode(change);
          break;
        case modeChangeInp2Octave:
          changeInp2OctaveOffset(change);
          break;
        case modeChangeBothControllerMethod:
          changeBothControllerMethod(change);
          break;
        case modeChangeFilterKeyTracking:
          changeFilterKeyTrackingCoeff(change);
          break;
      } 
      if (changeModeFlag > 0) changeModeCounter=changeModeCounter_resetValue;
      encoderDelayCount=130;   //this works as my de-bounce...assume 40 at fs=350Hz
    }
    encoderPos=0;
  }
}

//in this debounce routine, have a counter that blocks out any change in state for XX milliseconds
//after the state changes.
long savedMillis;
void serviceFootswitchBasic(const int pin, boolean &changedState, debounceStruct &debounceState) {
  const int debounceSamps = 140; //~100 ms at fs = 1300 Hz
  
  //service the counter
  int curCounter = debounceState.debounceCounter;
  curCounter = constrain(curCounter-1,0,debounceSamps);
  debounceState.debounceCounter = curCounter;
  
  //read value
  int curVal = digitalRead(pin);
  //Serial << "serviceFootswitchBasic: curCounter, = " << curCounter << ", curVal = " << curVal << endl;
  
  //check the debounce state
  //changedState = false;
  if (curCounter == 0) {
    //OK, we can react to the button
    int curState = debounceState.curState;
    if (curVal != curState) {
      //change the state
      debounceState.curState = curVal;  //set the state
      debounceState.debounceCounter = debounceSamps; //reset the counter
      changedState = true;  //confirm to the world that the state has changed
      //long foo = millis();
      //Serial << "serviceFootswitchBasic: changed.  curState = " << curState << ", d(millis) = " << foo - savedMillis << endl;
      //savedMillis = foo;
    } else {
      //no change, just update
    }
  }
}
 
  
int serviceFootswitch(const int pin, boolean &changed_state) {
  static int state = WAIT_FOR_PRESS;
  static int pushButton_accumVal = 0;
  static int max_val = 0;
  const int pushButton_thresh1=15;  //5 for fs = 360
  const int pushButton_thresh2=100; //100 for fs = 360

  //read value and accumulate
  int cur_val = digitalRead(pin);
  if (cur_val == HIGH) {
    //button is not pressed
    pushButton_accumVal--;    
    if (pushButton_accumVal < (pushButton_thresh1-pushButton_thresh2)) { 
      pushButton_accumVal = min(pushButton_accumVal,pushButton_thresh1);  
    }
  } 
  else {
    //button is pressed
    pushButton_accumVal++;
    max_val = max(max_val,pushButton_accumVal);
  }
  pushButton_accumVal = constrain(pushButton_accumVal,0,pushButton_thresh2);

  switch (state) {
  case WAIT_FOR_PRESS:
    if (pushButton_accumVal >= pushButton_thresh1) {
      state = WAIT_FOR_RELEASE; //change state...now wait for the release
    }
    break;
  case WAIT_FOR_RELEASE:
    if (pushButton_accumVal <= (max_val - pushButton_thresh1)) {
      state = WAIT_FOR_PRESS;
      if (max_val >= pushButton_thresh2) {
        //do nothing here
      } 
      else if (max_val >= pushButton_thresh1) {
        //key_scale_index = (key_scale_index+1) % 3;  //the "3" is for the number of scales the user can toggle between with the footswitch
        //changed_state = true;
      }
      max_val = 0;
    }
    break;
  }

  return cur_val;
}



void drawChangeModeInterfaceRibbon2Line(LiquidCrystal &lcd, const float &octaveOffset,const int &ribbonMode,const boolean &useQuantizedRibbon,int inp2OctaveOffset,int bothControllerMethod,const float &filterKeyTrackCoeff,int curChangeMode) {
  String fooStr;
  
  if (curChangeMode > modeNoChange) {
    lcd.clear();
    lcd.setCursor(0,0);
    switch (curChangeMode) {
      case modeOffsetChange:
        makeFloatString(octaveOffset,0,fooStr);
        lcd.print("Loop Octave: " + fooStr);
        break;
      case modeQuantizedRibbonChange:
        lcd.print("Ribbon Quantize:");
        lcd.setCursor(2,1);
        if (useQuantizedRibbon) {
          lcd.print("Yes");
        } else {
          lcd.print("No");
        } 
        break;
     case modeChangeRibbonMode:
       lcd.print("Ribbon Mode:");
       lcd.setCursor(2,1);
       lcd.print(getChangeRibbonModeText(ribbonMode));
       break;
     case modeChangeInp2Octave:
        makeFloatString(inp2OctaveOffset,0,fooStr);
        lcd.print("Inp2 Octave: " + fooStr);
        break;
     case modeChangeBothControllerMethod:
        //makeFloatString(bothControllerMethod,0,fooStr);
        lcd.print("When Inp1+Inp2:");
        lcd.setCursor(2,1);
        lcd.print(getBothControllerMethodString(bothControllerMethod));
        break;
     case modeChangeFilterKeyTracking:
        makeFloatString(filterKeyTrackCoeff,2,fooStr);
        lcd.print("Filter Tracking:");
        lcd.setCursor(2,1);
        lcd.print(fooStr);
        break;
      }
  }
}


void makeFloatString(float in, int n_decimal, String &str) {
  int val;
  val = floor(in);
  str=String(val);
  if (n_decimal > 0) {
    str = str + String(".");
    in -= val;
  
    for (int i=0;i<n_decimal;i++) {
      in *= 10;
      val = floor(in);
      str = str + String(val);
      in -= val;
    }
  }
}
