/*
 * CVSLE.cpp
 *
 *
 * This is a library for enabling control of any load through a Triac/Triac driver
 * combination. It provides various features such as load soft-start, Max load
 * operating speed, frequency monitoring and direct load control.
 *
 * WORKS ONLY IN AVR Architecture boards. Needs two 16 bit timers for better efficiency.
 *
 * Designed specifically to work with any triac/triac-driver circuit with in built
 * zero-detect. The original purpose of this library is for Saryam's "Centralized
 * Vacuum System".
 * ----> https://saryamstores.com/saryam-store/ols/products/cvs-ts2-unit
 *
 * The library makes use of in-built timers and interrupts to provide necessary
 * functionality.
 *
 * Saryam invests time and resources providing this open source code,
 * please support Saryam and open-source hardware by purchasing
 * products from Saryam!
 *
 * Written by Ajay Sarathy/Arunmani G/Abdhulla Sheik for Saryam Eng Pvt Ltd.
 * BSD license, all text above must be included in any redistribution
 *
 *  Created on: 25-Jul-2021
 *      Author: Saryam Engineering Private Limited
 */

#include "CVSLE.h"


//CVSLE Object
CVSLE cvsLE;


//Begin function
byte CVSLE::begin(byte interruptPin, byte triacDriverPin, byte loadRelayPin, bool inputPullupINT){

	//Variables
	byte result=0;


	/*
	 * The following tasks will be performed:
	 * 1) Init all variables, both public and private
	 * 2) Disable interrupts
	 * 3) Init process timer variable based on selected timer
	 * 4) Init zero-detect timer variable based on selected timer
	 * 5) AttachInterrupt for zero detect

	 *
	 */


	//Step 1=> init all variables

	//Args
	_interruptPin=interruptPin;
	_triacDriverPin=triacDriverPin;
	_inputPullupINT=inputPullupINT;
	_loadRelayPin=loadRelayPin;

	//process data members
	_softStartInterval=CVSLE_softStartInterval;
	_motorMax=CVSLE_loadMaxPercent;
	_currentLoadStart=0;
	_previousLoadStart=0;
	_softStartIntervalCount=0;
	_ZDCounter=0;
	_ZDTD=0;
	motorMaxFlag=false;
	motorStatus=false;
	_absMotorFlag=false;
	tempFlag=false;

	//process timer init
#if (CVSLE_ProcessTimer == 1)

	_timerCounter_P=&TCNT1;
	_outputCompare_P=&OCR1A;
	_timerMode_P=&TCCR1A;
	_prescaler_P=&TCCR1B;
	_interruptMask_P=&TIMSK1;
	_interruptflag_P=&TIFR1;

	result=1;


#elif (CVSLE_ProcessTimer == 3)

	_timerCounter_P=&TCNT3;
	_outputCompare_P=&OCR3A;
	_timerMode_P=&TCCR3A;
	_prescaler_P=&TCCR3B;
	_interruptMask_P=&TIMSK3;
	_interruptflag_P=&TIFR3;

	result=1;


#elif (CVSLE_ProcessTimer == 4)

	_timerCounter_P=&TCNT4;
	_outputCompare_P=&OCR4A;
	_timerMode_P=&TCCR4A;
	_prescaler_P=&TCCR4B;
	_interruptMask_P=&TIMSK4;
	_interruptflag_P=&TIFR4;

	result=1;


#elif (CVSLE_ProcessTimer == 5)

	_timerCounter_P=&TCNT5;
	_outputCompare_P=&OCR5A;
	_timerMode_P=&TCCR5A;
	_prescaler_P=&TCCR5B;
	_interruptMask_P=&TIMSK5;
	_interruptflag_P=&TIFR5;

	result=1;


#else
	result=0;


#endif


	//ZD timer init
#if (CVSLE_ZDTimer == 1)

	_timerCounter_ZD=&TCNT1;
	_outputCompare_ZD=&OCR1A;
	_timerMode_ZD=&TCCR1A;
	_prescaler_ZD=&TCCR1B;
	_interruptMask_ZD=&TIMSK1;
	_interruptflag_ZD=&TIFR1;

	result=1;


#elif (CVSLE_ZDTimer == 3)

	_timerCounter_ZD=&TCNT3;
	_outputCompare_ZD=&OCR3A;
	_timerMode_ZD=&TCCR3A;
	_prescaler_ZD=&TCCR3B;
	_interruptMask_ZD=&TIMSK3;
	_interruptflag_ZD=&TIFR3;

	result=1;


#elif (CVSLE_ZDTimer == 4)

	_timerCounter_ZD=&TCNT4;
	_outputCompare_ZD=&OCR4A;
	_timerMode_ZD=&TCCR4A;
	_prescaler_ZD=&TCCR4B;
	_interruptMask_ZD=&TIMSK4;
	_interruptflag_ZD=&TIFR4;

	result=1;


#elif (CVSLE_ZDTimer == 5)

	_timerCounter_ZD=&TCNT5;
	_outputCompare_ZD=&OCR5A;
	_timerMode_ZD=&TCCR5A;
	_prescaler_ZD=&TCCR5B;
	_interruptMask_ZD=&TIMSK5;
	_interruptflag_ZD=&TIFR5;

	result=1;


#else
	result=0;

#endif

	//Check result
	if(result!=0){

		//Step 2 => Disable interrupts
		noInterrupts();

		//Step 3 => Setup process timer
		*_timerCounter_P=0;
		*_outputCompare_P=0;
		*_timerMode_P=0;
		*_prescaler_P=0;
		*_interruptMask_P=0;
		*_interruptflag_P=0;

		*_outputCompare_P=CVSLE_PTMAXTC;    // Compare match register for little less than 10ms
		*_timerMode_P |= (1 << WGM12);   // CTC mode
		*_interruptMask_P |= (1 << OCIE1A) | (1 << TOIE1);  // enable timer compare interrupt



		//Step 4 => Setup ZD timer
		*_timerCounter_ZD=0;
		*_outputCompare_ZD=0;
		*_timerMode_ZD=0;
		*_prescaler_ZD=0;
		*_interruptMask_ZD=0;
		*_interruptflag_ZD=0;

		*_timerMode_ZD |= (1 << WGM12);   // CTC mode
		*_interruptMask_ZD |= (1 << OCIE1A) | (1 << TOIE1);  // enable timer compare and overflow interrupt
		*_prescaler_ZD |= (1 << CS12);    // 256 prescaler

		//Check pullup for interrupt
		if(_inputPullupINT){

			//pinMode
			pinMode(_interruptPin, INPUT_PULLUP);

		}//EOP check flag

		//pinModes

		//Triac driver pin
		pinMode(_triacDriverPin,OUTPUT);

		//Load relay pin
		pinMode(_loadRelayPin,OUTPUT);


		//Attach zero detect interrupt
		attachInterrupt(digitalPinToInterrupt(_interruptPin), _ZDRoutine, RISING);


		//Enable interrupts
		interrupts();

		//Set result
		result=1;

	}//EOP result NOK

	//Return statement
	return result;

}//EOP begin function


//get soft start interval
byte CVSLE::getSoftStartInterval(){

	//Variables
	byte result=0;

	result=_softStartInterval;

	//Return
	return result;

}//EOP getsoftstartinterval


//set soft interval
void CVSLE::setSoftStartInterval(byte softStartInterval){

	//Check motorStatus
	if(!motorStatus){

		//Check softStartInterval
		if(softStartInterval>CVSLE_softStartIntervalMax){

			_softStartInterval=CVSLE_softStartIntervalMax;

		}//EOP beyond max limit
		else if(softStartInterval<CVSLE_softStartIntervalMin){

			_softStartInterval=CVSLE_softStartIntervalMin;

		}//EOP below min limit
		else{

			_softStartInterval=softStartInterval;

		}//EOP given input ok


	}//EOP motor Status is OFF


}//EOP setSoftStartInterval


//get load max %
byte CVSLE::getLoadMax(){

	//Variables
	byte result=0;

	result=_motorMax;

	//Return
	return result;

}//EOP getLoadMax


//set load max %
void CVSLE::setLoadMax(byte motorMax){

	//Check motorStatus
	if(!motorStatus){

		//Check softStartInterval
		if(motorMax>CVSLE_loadMaxPercent){

			_motorMax=CVSLE_loadMaxPercent;

		}//EOP beyond max limit
		else if(motorMax<CVSLE_loadMinPercent){

			_motorMax=CVSLE_loadMinPercent;

		}//EOP below min limit
		else{

			_motorMax=motorMax;

		}//EOP given input ok


	}//EOP motor Status is OFF


}//EOP setLoadMax


//startLoadSoft
void CVSLE::startLoadSoft(){

	/*
	 * The following steps are undertaken:
	 * 1) Calculate TC value based on maxLoad value
	 * 2) Set min and max TC value based on above point
	 * 3) Calculate decrement period based on soft start interval
	 * 4) Calculate TCIMin based on range
	 * 5) Start interval polling
	 * 6) Set motor Status
	 * 7) Set triac driver pin
	 * 8) set load relay pin
	 *
	 */

	//Step 1 => Calculate TC value based on maxLoad value
	long TCDiff=(long)CVSLE_PTMAXTC-(long)CVSLE_PTMINTC;
	long TCRange=((TCDiff)*_motorMax)/100;

	//Step 2 => Calculate Min and Max range
	long TCMin=CVSLE_PTMAXTC-TCRange;
	long TCMax=CVSLE_PTMAXTC;

	//Step 3 => Calculate decrement period based on soft start interval
	int TCP=(_softStartInterval*1000)/CVSLE_PTTCDIV;

	//Step 4 => Calculate TCIMin based on range
	long TCIMin=TCRange/CVSLE_PTTCDIV;

	//Step 5 => Start interval polling
	_currentLoadStart=millis();

	if( (_currentLoadStart-_previousLoadStart) > TCP){

		//Check interval counter
		if(_softStartIntervalCount<CVSLE_PTTCDIV){


			//Reset interval polling
			_previousLoadStart=_currentLoadStart;

			//Change output compare register
			*_outputCompare_P=*_outputCompare_P-TCIMin;


			//Increment interval counter
			_softStartIntervalCount++;


		}//EOP Interval divisions count not reached
		else{

			//Check if output compare is close to CVSLE_PTMINTC
			int outputC=*_outputCompare_P-CVSLE_PTMINTC;

			if( (outputC < CVSLE_PTABSMAXT) || (_motorMax==100) ){

				//Set flag
				_absMotorFlag=true;


			}//EOP Absolute max load % reached
			else{

				//Set flag
				_absMotorFlag=false;


			}//EOP absolute max load % NOT reached


			//Set maxFlag
			motorMaxFlag=true;


		}//EOP interval divisions count reached



	}//EOP check interval


	//Set motor status
	motorStatus=true;


	//Set triac driver trigger
	if(_absMotorFlag){
	
		digitalWrite(_triacDriverPin, HIGH);
	
	}//EOP trigger triac-driver only when ABSLoadMax reached


	//Set load relay enable
	digitalWrite(_loadRelayPin,HIGH);


}//EOP startLoadSoft


//Stop load
void CVSLE::stopLoad(){


	//Reset all variables
	_currentLoadStart=millis();
	_previousLoadStart=_currentLoadStart;
	_softStartIntervalCount=0;
	motorMaxFlag=false;
	motorStatus=false;
	_absMotorFlag=false;


	//Reset all output pins

	//Reset triac driver trigger
	digitalWrite(_triacDriverPin, LOW);


	//Reset load relay enable
	digitalWrite(_loadRelayPin,LOW);


}//EOP stopLoad



//startLoadSoft
void CVSLE::startLoadHard(){

	/*
	 * The following steps are undertaken:
	 * 1) Calculate TC value based on maxLoad value
	 * 2) Set min and max TC value based on above point
	 * 3) Calculate decrement period based on soft start interval
	 * 4) Calculate TCIMin based on range
	 * 5) Start interval polling
	 * 6) Set motor Status
	 * 7) Set triac driver pin
	 * 8) set load relay pin
	 *
	 */

	//Step 1 => Calculate TC value based on maxLoad value
	long TCDiff=(long)CVSLE_PTMAXTC-(long)CVSLE_PTMINTC;
	long TCRange=((TCDiff)*_motorMax)/100;

	//Step 2 => Calculate Min and Max range
	long TCMin=CVSLE_PTMAXTC-TCRange;
	long TCMax=CVSLE_PTMAXTC;

	//Step 3 => Calculate decrement period based on soft start interval
	int TCP=(CVSLE_hardStartInterval*1000)/CVSLE_PTTCDIV;

	//Step 4 => Calculate TCIMin based on range
	long TCIMin=TCRange/CVSLE_PTTCDIV;

	//Step 5 => Start interval polling
	_currentLoadStart=millis();

	if( (_currentLoadStart-_previousLoadStart) > TCP){

		//Check interval counter
		if(_softStartIntervalCount<CVSLE_PTTCDIV){


			//Reset interval polling
			_previousLoadStart=_currentLoadStart;

			//Change output compare register
			*_outputCompare_P=*_outputCompare_P-TCIMin;


			//Increment interval counter
			_softStartIntervalCount++;


		}//EOP Interval divisions count not reached
		else{

			//Check if output compare is close to CVSLE_PTMINTC
			int outputC=*_outputCompare_P-CVSLE_PTMINTC;

			if( (outputC < CVSLE_PTABSMAXT) || (_motorMax==100) ){

				//Set flag
				_absMotorFlag=true;


			}//EOP Absolute max load % reached
			else{

				//Set flag
				_absMotorFlag=false;


			}//EOP absolute max load % NOT reached


			//Set maxFlag
			motorMaxFlag=true;


		}//EOP interval divisions count reached



	}//EOP check interval


	//Set motor status
	motorStatus=true;


	//Set triac driver trigger
	if(_absMotorFlag){
	
		digitalWrite(_triacDriverPin, HIGH);
	
	}//EOP trigger triac-driver only when ABSLoadMax reached


	//Set load relay enable
	digitalWrite(_loadRelayPin,HIGH);


}//EOP startLoadHard


//getInputTimePeriod
int CVSLE::getInputTimePeriod(){

	//Variables
	int result=0;

	int TCDiff=abs(_ZDCounter-624);

	//Check TCDiff
	if(TCDiff<50){

		result=(_ZDCounter*10)/624;

	}//EOP calC TP
	else{

		result=-1;

	}//EOP error

	//return
	return result;


}//EOP getInputTimePeriod



//getInputFrequency
float CVSLE::getInputFrequency(){

	//Variables
	float result=0;

	int TCDiff=abs(_ZDCounter-624);

	//Check TCDiff
	if(TCDiff<50){

		int TCP=(_ZDCounter*10)/624;

		result=100/(TCP*2);

	}//EOP calC TP
	else{

		result=-1;

	}//EOP error

	//return
	return result;


}//EOP getInputFrequency




//****************************
//  Interrupt Function
//****************************

//process timer init
#if (CVSLE_ProcessTimer == 1)

//Compare ISR
ISR(TIMER1_COMPA_vect){

	//Compare routine
	cvsLE.compareInterruptRoutine();


}//EOP compare ISR


//Overflow ISR
ISR(TIMER1_OVF_vect){

	//Overflow routine
	cvsLE.overflowInterruptRoutine();

}//EOP overflow ISR

#elif (CVSLE_ProcessTimer == 3)

//Compare ISR
ISR(TIMER3_COMPA_vect){

	//Compare routine
	cvsLE.compareInterruptRoutine();

}//EOP compare ISR


//Overflow ISR
ISR(TIMER3_OVF_vect){

	//Overflow routine
	cvsLE.overflowInterruptRoutine();

}//EOP overflow ISR



#elif (CVSLE_ProcessTimer == 4)

//Compare ISR
ISR(TIMER4_COMPA_vect){

	//Compare routine
	cvsLE.compareInterruptRoutine();

}//EOP compare ISR


//Overflow ISR
ISR(TIMER4_OVF_vect){

	//Overflow routine
	cvsLE.overflowInterruptRoutine();

}//EOP overflow ISR



#elif (CVSLE_ProcessTimer == 5)

//Compare ISR
ISR(TIMER5_COMPA_vect){

	//Compare routine
	cvsLE.compareInterruptRoutine();

}//EOP compare ISR


//Overflow ISR
ISR(TIMER5_OVF_vect){

	//Overflow routine
	cvsLE.overflowInterruptRoutine();

}//EOP overflow ISR


#endif


//ZD timer init
#if (CVSLE_ZDTimer == 1)

//Compare ISR
ISR(TIMER1_COMPA_vect){


}//EOP compare ISR


//Overflow ISR
ISR(TIMER1_OVF_vect){



}//EOP overflow ISR



#elif (CVSLE_ZDTimer == 3)

//Compare ISR
ISR(TIMER3_COMPA_vect){


}//EOP compare ISR


//Overflow ISR
ISR(TIMER3_OVF_vect){



}//EOP overflow ISR



#elif (CVSLE_ZDTimer == 4)

//Compare ISR
ISR(TIMER4_COMPA_vect){


}//EOP compare ISR


//Overflow ISR
ISR(TIMER4_OVF_vect){



}//EOP overflow ISR



#elif (CVSLE_ZDTimer == 5)

//Compare ISR
ISR(TIMER5_COMPA_vect){


}//EOP compare ISR


//Overflow ISR
ISR(TIMER5_OVF_vect){



}//EOP overflow ISR


#endif



//Set compare attach routine to default
void (*CVSLE::isrCompare)()= CVSLE::isrDefaultUnused;

//Set overflow attach routine to default
void (*CVSLE::isrOverflow)()= CVSLE::isrDefaultUnused;

//Default function
void CVSLE::isrDefaultUnused()
{
}


//Static wrapper for ZD
void CVSLE::_ZDRoutine()
{

	//check motorStatus flag
	if(cvsLE.motorStatus){

		//Process ISR
		cvsLE.zeroDetectISR();

	}//EOP Load in ON
	else{

		//Reset prescaler and stop timer
		*cvsLE._prescaler_P=0;

		//Reset counter value
		*cvsLE._timerCounter_P=0;

	}//EOP load is OFF


	//ZD ISR
	cvsLE.ZDTimerCalC();

}

//Zero detect interrupt routine
void CVSLE::zeroDetectISR()
{

	//Set prescaler and start timer
	*_prescaler_P=0;
	*_prescaler_P=(1 << CS12);

	//Reset counter value
	*_timerCounter_P=0;


}//EOP zeroDetectISR



//Routine for ZD Timer
void CVSLE::ZDTimerCalC()
{

	//ZD Timer
	_ZDCounter=*_timerCounter_ZD;

	//Check zd-Counter
	if(_ZDCounter>CVSLE_ZDMTC){

		//Set ZDcounter to zero
		_ZDCounter=0;

	}//EOP greater than required count

	//Reset ZD Counter
	*_timerCounter_ZD=0;


}//EOP zeroDetectISR




//compareInterruptRoutine
void CVSLE::compareInterruptRoutine()
{

	//Set triacDriver High
	digitalWrite(_triacDriverPin, HIGH);


	//Call user defined routine
	isrCompare();


	//Set counter value close to overflow to switch off triacDriver pulse
	*_timerCounter_P=CVSLE_PTimerMax-CVSLE_triacDriverDelay;


}//EOP compareInterruptRoutine

//overflowInterruptRoutine
void CVSLE::overflowInterruptRoutine()
{

	//Check absolute load max flag

	if(!_absMotorFlag){

		//Set triacDriver High
		digitalWrite(_triacDriverPin, LOW);


	}//EOP motor Max not reached

	//Call user defined routine
	isrOverflow();

	//Reset prescaler and stop timer
	*_prescaler_P=0;


}//EOP overflowInterruptRoutine

