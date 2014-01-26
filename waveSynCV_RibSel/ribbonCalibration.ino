
#define N_RIBBON_TABLE (37)
int ribbon_cal[N_RIBBON_TABLE] = {
3145   ,
3496   ,
3822   ,
4163   ,
4456   ,
4812   ,
5162   ,
5471   ,
5790   ,
6150   ,
6463   ,
6802   ,
7166   ,
7500   ,
7860   ,
8230   ,
8578   ,
8976   ,
9337   ,
9683   ,
10047   ,
10462   ,
10838   ,
11201   ,
11605   ,
11986   ,
12385   ,
12801   ,
13179   ,
13636   ,
14015   ,
14417   ,
14844   ,
15246   ,
15631   ,
16006   ,
16376
};

int getCalibratedRibbonValue(const int &input,const boolean &useQuantizedRibbon) {
  float fac = 0.0;
  
  //search through table...end with index of table just smaller than given value
  int count=0;
  while ((count < (N_RIBBON_TABLE-1)) && (ribbon_cal[count] < input)) count++;  //this ends with count being just above
  count--;  //this pus it just below
  count = max(0,count);
  
  //interpolate...scaling is 2^14 spans 5 octaves. count==0 is (1/5*2^14) = 3276.8.  One half step is 273.0667
  int output;
  if (input < ribbon_cal[0]-100) {
    if (useQuantizedRibbon) {
      output = ribbon_cal[0]; //quanitze to lowest value in table
    } else {
      output = ribbon_cal[0]-100;  //allow some bending for vibrato
    }
  } else {
    fac = ((float)(input - ribbon_cal[count]))/((float)(ribbon_cal[count+1]-ribbon_cal[count]));
    if (useQuantizedRibbon) {
      //put dividing line halfway between steps
      if (fac < -0.5) { fac = -1.0; } else if (fac > 0.5) { fac = 1.0; } else { fac = 0.0; } 
    }
    fac = fac + ((float)(count + 12)); //the +12 is to add the octave that's off the bottom of the ribbon
    output  = (int)(fac*273.0667+0.5);  //the +0.5 is so that it rounds instead of truncates
  }
  return output;
}

#define ribbonThresh 2000
void serviceRibbon(const int &ribbonPin, const int &bitShift, const boolean &useQuantizedRibbon, int &pitchOutput, int &gateOutput) {
  int sensorValue = analogRead(ribbonPin) << bitShift;
    
  //Serial << "serviceRibbon: val = " << sensorValue << endl;
  gateOutput = LOW;
  if (sensorValue > ribbonThresh) {
    gateOutput = HIGH;
  }
  pitchOutput = getCalibratedRibbonValue(sensorValue,useQuantizedRibbon);  
  //pitchOutput = sensorValue; //enable this line to defeat the calibration so that we can display the raw value in the main program
  //Serial << "serviceRibbon: " << sensorValue << ", " << gateOutput << ", " << pitchOutput << endl;
}

void serviceRibbon_span(const int &ribbonPin, const int &span,int &pitchOutput, int &gateOutput) {
  int sensorValue = analogRead(ribbonPin) << 4;
    
  //Serial << "serviceRibbon: val = " << sensorValue << endl;
  gateOutput = LOW;
  if (sensorValue > ribbonThresh) {
    gateOutput = HIGH;
  }
  pitchOutput = constrain(map(sensorValue,ribbon_cal[0],ribbon_cal[N_RIBBON_TABLE-1],0,span),0,span);  
  //pitchOutput = sensorValue; //enable this line to defeat the calibration so that we can display the raw value in the main program
  //Serial << "serviceRibbon: " << sensorValue << ", " << gateOutput << ", " << pitchOutput << endl;
}
