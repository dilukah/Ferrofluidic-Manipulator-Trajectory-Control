/*
controller.cpp - Ferrofludic manipulator controller interface
Date: 2021-07-07
Author: P. A. Diluka Harischandra
*/

#include "Vision.h"

#include "Controller.h"
#include <stdio.h>
#include <iostream>
#include <vector>
#include <numeric>      
#include <algorithm>  
#include "utility.h"


#define DAQmxErrChk(functionCall) if( DAQmxFailed(error=(functionCall)) ) DAQ_ErrorHandling(); else



using namespace std;

//Constructor
Controller::Controller()
{
	taskHandle = 0;
}

/**====================================================
* Function to select the coils to be activated using Linear Programming
* Input: COG of the particle, Target, Positions of the coil tips
* Output: unsigned 8 bit int
*======================================================*/
uInt8 Controller::selectCoilsLP(vpImagePoint particlePos, vpImagePoint target, vpImagePoint coilTip[])
{
	uInt8 activationCoils = 0b00000000;
	activationCoils = lpModel(particlePos, target, coilTip);
	return activationCoils;
}

/**====================================================
* Function to initialize DAQ
* Input: NULL
* Output: NULL
*======================================================*/
void Controller::initDAQ()
{
		DAQmxErrChk(DAQmxCreateTask("", &taskHandle));
		DAQmxErrChk(DAQmxCreateDOChan(taskHandle, "Dev1/port0", "", DAQmx_Val_ChanForAllLines));
		// DAQmx Start Code
		DAQmxErrChk(DAQmxStartTask(taskHandle));
}

/**====================================================
* Function to write to DAQ
* Input: unsigned 8 bit int for the digital output
* Output: NULL
*======================================================*/
void Controller::writeToDAQ(uInt8 data)
{
	DAQmxErrChk(DAQmxWriteDigitalU8(taskHandle, 1, 1, 10.0, DAQmx_Val_GroupByChannel, &data, NULL, NULL));
}

/**====================================================
* Function to stop and clear task
* Input: NULL
* Output: NULL
*======================================================*/
void Controller::stopDAQ()
{
	if (taskHandle != 0) {

		DAQmxStopTask(taskHandle);
		DAQmxClearTask(taskHandle);
	}
}


/**====================================================
* Function to handle DAQ Errors
* Input: NULL
* Output: NULL
*======================================================*/
void Controller::DAQ_ErrorHandling()
{
	if (DAQmxFailed(error))
		DAQmxGetExtendedErrorInfo(errBuff, 2048);

	if (DAQmxFailed(error))
		printf("DAQmx Error: %s\n", errBuff);
}


/**====================================================
* Function for manual coil actuation with keyboard input
* Input: NULL
* Output: unsigned 8 bit int
*======================================================*/
uInt8 Controller::ManualCoilControl()

{
	uInt8 manualCoilAct = 0b00000000;
	//numpad buttons to the coils map
	for (int i = 0; i < 9; i++)
		num[i] = (bool)(GetKeyState(i + 97) & 0x8000);

	if (num[coil1_key])	manualCoilAct |= (1 << 0);
	if (num[coil2_key]) manualCoilAct |= (1 << 1);
	if (num[coil3_key]) manualCoilAct |= (1 << 2);
	if (num[coil4_key]) manualCoilAct |= (1 << 3);
	if (num[coil5_key]) manualCoilAct |= (1 << 4);
	if (num[coil6_key]) manualCoilAct |= (1 << 5);
	if (num[coil7_key]) manualCoilAct |= (1 << 6);
	if (num[coil8_key]) manualCoilAct |= (1 << 7);

	return manualCoilAct;

}


