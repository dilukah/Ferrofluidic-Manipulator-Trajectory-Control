#pragma once

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <NIDAQmx.h>

#include <ilcplex/ilocplex.h>

ILOSTLBEGIN

/*	Note: Keyboard to coil map
*	Actual keyboard number is coilx_key + 1, Real keyboard number 5 is not used.
*/
#define coil1_key 8 
#define coil2_key 5
#define coil3_key 2
#define coil4_key 1
#define coil5_key 0
#define coil6_key 3
#define coil7_key 6
#define coil8_key 7

extern bool keyboardInputEnabled;
void lpkeyboardInput();
extern bool PrintToConsole;

uInt8 lpModel(vpImagePoint particlePos, vpImagePoint target, vpImagePoint coilTip[]);

class Controller
{
public:
	//Constructor
	Controller();

	void initDAQ();
	void writeToDAQ(uInt8 data);
	void stopDAQ();
	void DAQ_ErrorHandling();

	uInt8 selectCoilsLP(vpImagePoint particlePos, vpImagePoint target, vpImagePoint coilTip[]);

	uInt8 ManualCoilControl();

private:

	TaskHandle  taskHandle;
	int32       error = 0;
	char        errBuff[2048] = { '\0' };
	
	bool num[9] = { 0,0,0,0,0,0,0,0,0 };

	bool incrementVal = 0;
	bool decrementVal = 0;
	int editingVariable = 0;

};


#endif //CONTROLLER_H