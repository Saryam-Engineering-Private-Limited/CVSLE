#include "Arduino.h"

#include "CVSLE.h"

boolean flag=false;

//The setup function is called once at startup of the sketch
void setup()
{
// Add your initialization code here
  Serial.begin(115200);
  cvsLE.begin(18, 5, 6, false);

  cvsLE.attachRoutineForCompare(compareIR);
  cvsLE.attachRoutineForOverflow(overflowIR);


  Serial.println("Setup Completed");

}

// The loop function is called in an endless loop
void loop()
{
//Add your repeated code here

  if(cvsLE.tempFlag){

    Serial.println("Zero detect timer running");

  }

  cvsLE.startLoadSoft();

}


void compareIR(void){

  cvsLE.tempFlag=true;
  //Serial.println("Zero detect timer running");
}


void overflowIR(void){

  cvsLE.tempFlag=false;
  //Serial.println("Zero detect timer NOT running");
}
