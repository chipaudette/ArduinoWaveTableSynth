



//define passThroughState 0
//define recordState 1
//define loopState 2

//void resetLoopData(recordAndLoopStateInfo &);

void initializeRecordAndLoopStateInfo(recordAndLoopStateInfo &state, unsigned long curMillis,int inpPitch,int inpGate) {
  state.curState = loopState_passThrough;
  state.refMillis = curMillis;
  state.changedState = true;
  resetLoopData(state);
  
  //these are the values to be output
  state.outputPitch_halfSteps = inpPitch;
  state.outputGateVal=inpGate;
  
  //return state;
}

String getCurrentStateString(recordAndLoopStateInfo &inputStateInfo) {
  switch (inputStateInfo.curState) {
  case loopState_passThrough:
    return String("Normal Play...");
    break;
  case loopState_record:
    return String("Record Step...");
    break;
  case loopState_playLoop:
    return String("Playing ") + String(inputStateInfo.curLoopStep) + String(" of ") + String(inputStateInfo.nLoopSteps);
    break;
  }
  return String("Unknown");
}
    

void chooseRecordOrLoopState(recordAndLoopStateInfo &state,unsigned long curMillis,int switchVal) {
  state.changedState = false;
  switch (state.curState) {
    case loopState_passThrough:
      if (switchVal == HIGH) {
        //switch to record mode
        state.curState = loopState_record;
        state.changedState = true;
        resetLoopData(state); //clear the loop data in preparation for recording
        //Serial << "chooseState: in passThroughState, state = " << state.curState << endl;
      } else { 
        //Serial << "chooseState: in passThroughState, keeping state, switchVal = " << switchVal << endl;
      }
      break;
    case loopState_record:
      if (switchVal == LOW) {
        //check to see if we can switch to loop mode
        if (state.nLoopSteps >= 2) {
          //switch to loop playback mode
          state.loopDuration_msec = (int)(curMillis - state.refMillis);
          state.curState = loopState_playLoop;
          state.curLoopStep = 0;
          state.changedState = true;
        } else {
          //switch back to pass-through mode
          state.curState = loopState_passThrough;
          state.changedState = true;
        }
        //Serial << "chooseState: in loopState_record, state = " << state.curState << endl;
      } else { 
        //Serial << "chooseState: in loopState_record, keeping state, switchVal = " << switchVal << endl;
      }
        
      break;
    case loopState_playLoop:
      if (switchVal == HIGH) {
        //switch to passThroughState
        state.curState = loopState_passThrough;
        state.changedState = true;
      }
      break;
  }
  if (state.changedState) {
    state.refMillis = curMillis;
    //Serial << "chooseR|L: changedState: " << state.curState << ", curMillis: " << curMillis << ", FS2: " << switchVal << endl;
  }
  //return state;
}

void resetLoopData(recordAndLoopStateInfo &state) {
  state.curLoopStep = -1;
  state.nLoopSteps = 0;
  state.loopDuration_msec = 0;
  //return state;
}
  
void restartLoop(recordAndLoopStateInfo &state, unsigned long curMillis) {
  //only accept this command if we are looping
  if (state.curState == loopState_playLoop) {
    state.refMillis = curMillis;
    state.curLoopStep = 0;
  } else {
    //do nothing!
  }
}

  
void serviceRecordAndLoopState(recordAndLoopStateInfo &state,unsigned long curMillis,int inpPitch_halfSteps,int inpGate,int switchVal) {
 
  boolean recordNewElement=false;
  //decide what state we're in
  chooseRecordOrLoopState(state,curMillis,switchVal);
  //Serial << "serviceRecordAndLoopState: state = " << state.curState << endl;
  
  //act on the current state
  switch (state.curState) {
    case loopState_passThrough:
      //just pass the inputs to the outputs
      state.outputPitch_halfSteps = inpPitch_halfSteps;
      state.outputGateVal = inpGate;
      break;
    case loopState_record:
      //first, pass the inputs to the outputs so that we can hear them
      state.outputPitch_halfSteps = inpPitch_halfSteps;
      state.outputGateVal = inpGate;
      
      //what are our criteria for recording a new loop element?
      if (state.curLoopStep >= 0) {
        //regular rules...any change in pitch or gate
        int prevPitch = state.allLoopSteps[state.curLoopStep].pitch_halfSteps;
        int prevGate = state.allLoopSteps[state.curLoopStep].gateVal;
        if ((prevPitch != inpPitch_halfSteps) | (prevGate != inpGate)) recordNewElement = true;
      } else {
        //special rules for first step...simply wait for inpGate = true
        if (inpGate == HIGH) {
          state.refMillis = curMillis;  //first gate triggers the start of the loop
          recordNewElement = true;
        }
      }
        
      //add the element, if required
      if (recordNewElement) {
        state.curLoopStep = min((state.curLoopStep + 1),maxLoopSteps);
        state.allLoopSteps[state.curLoopStep].start_msec = (int)(curMillis - state.refMillis);
        state.allLoopSteps[state.curLoopStep].pitch_halfSteps = inpPitch_halfSteps;
        state.allLoopSteps[state.curLoopStep].gateVal = inpGate;
        state.nLoopSteps = state.curLoopStep+1;
        /*
        Serial << "service: added loop element " << state.nLoopSteps << ", t = " << state.allLoopSteps[state.curLoopStep].start_msec 
          << ", pitch = " <<  state.allLoopSteps[state.curLoopStep].pitch_halfSteps 
          << ", gate = " << state.allLoopSteps[state.curLoopStep].gateVal
          << endl;
        */
      }
      break;
    case loopState_playLoop:
      int nextLoopStep;
      int curBlockStart_msec,curBlockEnd_msec;
      int curTime_msec;
      curTime_msec = (int)((curMillis - state.refMillis) % ((unsigned long)state.loopDuration_msec));  //get the time relative to the reference time
      boolean keepLooping = true;
      while (keepLooping) {
        curBlockStart_msec = state.allLoopSteps[state.curLoopStep].start_msec;
        nextLoopStep = (state.curLoopStep+1) % state.nLoopSteps;
        if (nextLoopStep == 0) {
          if (curTime_msec < curBlockStart_msec) {
            //time has wrapped...
            curBlockEnd_msec = state.allLoopSteps[nextLoopStep].start_msec;
          } else {
            //time has not yet wrapped
            curBlockEnd_msec = state.loopDuration_msec;
          }
        } else {
          //the normal branch of logic
          curBlockEnd_msec = state.allLoopSteps[nextLoopStep].start_msec;
        }
        
        //Serial << "curStep = " << state.curLoopStep  << ", nextStep " << nextLoopStep << ", curTime = " << curTime_msec  << ", end = " << curBlockEnd_msec << endl;
        
        //check to see whether it is time to switch to the next loop element
        if (curTime_msec > curBlockEnd_msec) {
          state.curLoopStep = nextLoopStep;
          keepLooping = true;
        } else {
          keepLooping = false;
        }
      }
      
      //decide output
      if (inpGate == LOW) {
        //copy out the current loop element's pitch and gate
        state.outputPitch_halfSteps = state.allLoopSteps[state.curLoopStep].pitch_halfSteps;
        state.outputGateVal = state.allLoopSteps[state.curLoopStep].gateVal;
      } else {
        //user pressed a key. override with current pitch
        state.outputPitch_halfSteps = inpPitch_halfSteps;
        state.outputGateVal = inpGate;
      }
        
      break;
  }
  //return state;
}

  
