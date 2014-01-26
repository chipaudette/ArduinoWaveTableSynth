/*
   Created: Chip Audette, Feb 2011
   Purpose: Routines to send data via the MCP4922 12-bit, 2-channel DAC
   Approach: The MCP4922 is a SPI device.  Push out 2-byte codes per sample per channel.
   
   Pin3 = (NOT)CS   = Arduino Pin 10 (SS)
   Pin4 = SCK       = Arduino Pin 13 (SCK)
   Pin5 = SDI       = Arduino Pin 11 (MOSI)
   Pin8 = (NOT)LDAC = LOW
   Pin9 = (NOT)SHDN = HIGH
   Pin11 = Pin12 = Vref = HIGH
   
   Pin14 = Vout A
   Pin10 = Vout B
*/

/*
   //Main sketch must include the following code
   #include <SPI.h>

   void setup() {
  	// set the slaveSelectPin as an output:
  	pinMode(slaveSelectPin, OUTPUT);
  
	// initialize SPI:
  	SPI.begin(); 
  	SPI.setDataMode(SPI_MODE0); //MCP4922 can be either Mode 0 or Mode 3 (supposedly)
  	SPI.setBitOrder(MSBFIRST);
   }
*/


void dacWrite(const int &slavePin,const int &value1,const int &value2) {
  int value = 0;
  byte configByte = 0;
  byte data=0;
  int channel=0;
  
  for (channel = 0; channel < 2; channel++) {
    digitalWrite(slavePin,LOW);  //set DAC ready to accept commands
    if (channel == 0) {
      configByte = B01110000; //channel 0, Vref buffered, Gain of 1x, Active Mode
      value = value1;
    } else {
      configByte = B11110000; //channel 1, Vref buffered, Gain of 1x, Active Mode
      value = value2;
    }
    
    //write first byte
    data = highByte(value);
    data = B00001111 & data;  //clear out the 4 command bits
    data = configByte | data;  //set the first four command bits
    SPI.transfer(data);
    
    //write second byte
    data = lowByte(value);
    SPI.transfer(data);
    
    //close the transfer
    digitalWrite(slavePin,HIGH);  //set DAC ready to accept commands
  }
}

void dacWriteOneChannel(const int &slavePin,const int &value) {
  //int value = 0;
  //byte configByte = 0;
  static byte configByte =  B01110000; //channel 0, Vref buffered, Gain of 1x, Active Mode
  byte data=0;
  //static int channel=0;
  
  //for (channel = 0; channel < 2; channel++) {
    digitalWrite(slavePin,LOW);  //set DAC ready to accept commands
  //  if (channel == 0) {
  //    configByte = B01110000; //channel 0, Vref buffered, Gain of 1x, Active Mode
  //    value = value1;
  //  } else {
  //    configByte = B11110000; //channel 1, Vref buffered, Gain of 1x, Active Mode
  //    value = value2;
  //  }
    
    //write first byte
    data = highByte(value);
    data &= B00001111;  //clear out the 4 command bits
    data |= configByte;  //set the first four command bits
    SPI.transfer(data);
    
    //write second byte
    data = lowByte(value);
    SPI.transfer(data);
    
    //close the transfer
    digitalWrite(slavePin,HIGH);  //set DAC ready to accept commands
  //}
}

