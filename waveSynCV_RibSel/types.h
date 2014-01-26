typedef struct {
  int unsmoothedSensorValue;
  int sensorValue;
  int calibratedValue;
  int rawNoteVal;
  int prevRawNoteVal;
  String setPitchName;
  String outPitchName;
} pitchStruct;

typedef struct {
  int curState;
  int debounceCounter;
} debounceStruct;

typedef struct {
  int start_msec;
  int pitch_halfSteps;
  int gateVal;
} loopStep;

#define maxLoopSteps 64
typedef struct {
  int curState;  //passing through, recording, or looping?
  unsigned long refMillis;  //time when we switched to this record or play state
  boolean changedState;     //have we changed from passing-through to recording. or to looping, or back?
  boolean changedLoopStep;  //have we changed what loop step we're on?

  //these are the values as read from the inputs
  //int inputPitch_halfSteps;  //stored halfSteps relative to bottom C (Volt = 0)
  //boolean inputGateVal;  // true is on, false is off
  
  //these are the values to be output
  int outputPitch_halfSteps;  //if looping, this is the current pitch to play
  int outputGateVal;          //if looping, this is the current gate to play
  
  //here is the info about the looping
  int curLoopStep;            //if looping, here is the step that we're on
  int nLoopSteps;             //here are the total number of steps
  int loopDuration_msec;      //here's the duratio nof the whole loop
  loopStep allLoopSteps[maxLoopSteps];  //here is the data for each step in the whole loop
} recordAndLoopStateInfo;


//defines for recordAndLoopStates
#define loopState_passThrough 0
#define loopState_record 1
#define loopState_playLoop 2


