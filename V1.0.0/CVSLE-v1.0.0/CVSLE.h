/*
 * CVSLE.h
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

#ifndef CVSLE_H_
#define CVSLE_H_

#include <Arduino.h>

#include <avr/io.h>
#include <avr/interrupt.h>

#define CVSLE_interrupt 18 //Zero-detect Interrupt pin
#define CVSLE_triacDriver 5 //TriacDriver enable pin
#define CVSLE_loadRelay 6 //load relay pin
#define CVSLE_softStartInterval 10 //Soft start interval in seconds
#define CVSLE_hardStartInterval 5 //Hard start interval in seconds
#define CVSLE_softStartIntervalMax 60 //Soft start interval maximum value in seconds
#define CVSLE_softStartIntervalMin 5 //Soft start interval minimum value in seconds
#define CVSLE_ProcessTimer 1 //process timer
#define CVSLE_loadMaxPercent 100 //maximum motor speed %
#define CVSLE_loadMinPercent 20 //minimum motor speed %
#define CVSLE_triacDriverDelay 5 //TriacDriver Delay
#define CVSLE_ZDTimer 3 //Zero-detect timer
#define CVSLE_PTMAXTC 600 //Max counter value for PT
#define CVSLE_PTMINTC 100 //Min counter value for PT
#define CVSLE_PTABSMAXT 50 //Min counter value for PT
#define CVSLE_PTTCDIV 100 //Divisions for soft start time period
#define CVSLE_ZDMTC 1250 //Max counter value for ZD
#define CVSLE_ZDTP 20 //Input AC time period in milliSecs
#define CVSLE_ZDF 50 //Input AC frequency in Hz

#define CVSLE_ZDMode RISING //Mode for interrupt attach of zero-detect

#if (CVSLE_ProcessTimer==2)
#define CVSLE_PTimerMax 255 //Timer max
#else
#define CVSLE_PTimerMax 65535 //Timer max
#endif


#if (CVSLE_ZDTimer==2)
#define CVSLE_ZDTimerMax 255 //Timer max
#else
#define CVSLE_ZDTimerMax 65535 //Timer max
#endif

class CVSLE {


public:

	//Motor Status
	volatile bool motorStatus;

	//Motor Max
	volatile bool motorMaxFlag;

	//Motor Max
	volatile bool tempFlag;

	byte begin(byte interruptPin = CVSLE_interrupt, byte triacDriverPin = CVSLE_triacDriver, byte loadRelayPin = CVSLE_loadRelay, bool inputPullupINT = true );
	/*!
	 * @brief Inits all necessary variables. Needs to be called for the
	 * class functionality to work
	 * @return Returns "1" for success and "0" for failure
	 */


	byte getSoftStartInterval();
	/*!
	 * @brief Get the current soft start interval
	 * @return Returns the value of the interval in byte
	 */


	void setSoftStartInterval(byte softStartInterval=CVSLE_softStartInterval);
	/*!
	 * @brief Set the current soft start interval
	 * @return void
	 */

	byte getLoadMax();
	/*!
	 * @brief Get the Load Max Value %
	 * @return Returns the value of the load max %
	 */


	void setLoadMax(byte loadMax=CVSLE_loadMaxPercent);
	/*!
	 * @brief Set the Load Max Value %
	 * @return void
	 */


	void startLoadSoft();
	/*!
	 * @brief Initiate soft start for the given load
	 * @return void
	 */

	void stopLoad();
	/*!
	 * @brief Initiate STOP for the given load
	 * @return void
	 */


	void startLoadHard();
	/*!
	 * @brief Initiate hard start for the given load
	 * @return void
	 */


	int getInputTimePeriod();
	/*!
	 * @brief Get input signals's time period
	 * @return Time period as int or -1 in case of error
	 */


	float getInputFrequency();
	/*!
	 * @brief Get input signals's frequency
	 * @return frequency as float or -1 in case of error
	 */


	//****************************
	//  Interrupt Function
	//****************************
	void attachRoutineForCompare(void (*isr)()) __attribute__((always_inline)) {

		isrCompare = isr;

	}
	/*!
	 * @brief Attach Interrupt for when Compare ISR is triggered.
	 * @return void
	 */

	void attachRoutineForOverflow(void (*isr)()) __attribute__((always_inline)) {

		isrOverflow = isr;

	}
	/*!
	 * @brief Attach Interrupt for when Overflow ISR is triggered.
	 * @return void
	 */

	static void (*isrCompare)();
	/*
	 * @brief Stored custom function for compare ISR
	 */

	static void (*isrOverflow)();
	/*
	 * @brief Stored custom function for compare ISR
	 */

	static void isrDefaultUnused();
	/*
	 * @brief Default function for interrupts
	 */

	void compareInterruptRoutine();
	/*
	 * @brief Custom function for compareISR
	 */

	void overflowInterruptRoutine();
	/*
	 * @brief Custom function for overflowISR
	 */

	void zeroDetectISR();
	/*
	 * @brief CISR for zero detect
	 */

	void ZDTimerCalC();
	/*
	 * @brief Routine for ZD Time for calculation time period of input signal
	 */

private:

	byte _interruptPin;
	byte _triacDriverPin;
	byte _softStartInterval;
	byte _loadRelayPin;
	byte _motorMax;
	uint16_t volatile _ZDCounter;
	byte _ZDTD;
	bool _inputPullupINT;
	unsigned long _currentLoadStart;
	unsigned long _previousLoadStart;
	byte _softStartIntervalCount;
	boolean _absMotorFlag;


	uint16_t volatile *  _timerCounter_P;
	uint16_t volatile *  _outputCompare_P;
	uint16_t volatile *  _inputCompare_P;
	uint8_t volatile *  _timerMode_P;
	uint8_t volatile *  _prescaler_P;
	uint8_t volatile *  _interruptMask_P;
	uint8_t volatile *  _interruptflag_P;


	uint16_t volatile *  _timerCounter_ZD;
	uint16_t volatile *  _outputCompare_ZD;
	uint16_t volatile *  _inputCompare_ZD;
	uint8_t volatile *  _timerMode_ZD;
	uint8_t volatile *  _prescaler_ZD;
	uint8_t volatile *  _interruptMask_ZD;
	uint8_t volatile *  _interruptflag_ZD;


	static void _ZDRoutine();
	/*
	 * @brief ZD Static wrapper
	 */


};//EOP class


extern CVSLE cvsLE;

#endif /* CVSLE_H_ */
