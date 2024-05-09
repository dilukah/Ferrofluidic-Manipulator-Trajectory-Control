/*
VisualServo.cpp - Ferrofludic manipulator visual servoing algorithms
Date: 2021-07-07
Author: P. A. Diluka Harischandra
*/

#include "VisualServo.h"

// State variables
bool mode = 0; //1 auto 0 manual
bool userReqStop = 0;
bool stopCondition = 0;
bool stopAllOperations = 0;
bool recording = 0;
bool trajectoryMode = 0;
bool p2pMode = 0;
bool oneCoilMode = 0;
bool stepMode = 0;
bool stepX = 1; //step direction: 1 for X, 0 for Y
bool stepModeStarted = 0;
bool trajectoryStarted = 0;
bool p2pStarted = 0;
bool programmableManipulationStarted = 0;
bool offTime = 0;
bool openLoopMode = 0;
int skipMode = 0;
bool keyboardInputEnabled = 1;
bool DisplayVariables = 0;
int trajectory_id = 0;
bool firstRun = 0;
bool startRecording = 0;
bool stopRecording = 0;

//Other variables
double stepsize = 6.0;
int nRepeats = 0;
double positionErrorTolerance = 3.0;
const int numberOfCoils = 8;
const double degToRad = M_PI / 180.0;

double MX = 470;
double MY = 532;

//Set the desired fps here.
float fps = 10.0;
float frameLength = 1000 / fps;

//Construct Vision Object
Vision MyVision = Vision(true, false, fps);

//Construct Controller Object
Controller MyControl = Controller();

//Log file
ofstream outfile;

using namespace std;

void VisionServoing()
{

	vpImagePoint clickedTarget;
	vpImagePoint cmdPosition;
	vpImagePoint cog, prevCog;

	std::cout << "Initializing camera" << endl;
	MyVision.Initialize(1024, 1024);
	std::cout << "Initialized camera" << endl;

	uInt8 activationCoil;

	//Coil position configuration
	vpImagePoint coilTip[numberOfCoils];

	coilTip[0].set_u(621);
	coilTip[0].set_v(355);
	coilTip[1].set_u(705);
	coilTip[1].set_v(513);
	coilTip[2].set_u(625);
	coilTip[2].set_v(693);
	coilTip[3].set_u(441);
	coilTip[3].set_v(763);
	coilTip[4].set_u(293);
	coilTip[4].set_v(682);
	coilTip[5].set_u(222);
	coilTip[5].set_v(507);
	coilTip[6].set_u(297);
	coilTip[6].set_v(360);
	coilTip[7].set_u(485);
	coilTip[7].set_v(281);

	std::cout << "Initializing DAQ" << endl;
	MyControl.initDAQ();
	std::cout << "Initialized DAQ" << endl;

	chrono::high_resolution_clock::time_point startTime = chrono::high_resolution_clock::now();
	chrono::high_resolution_clock::time_point currentTime = chrono::high_resolution_clock::now();
	MyVision.AcquireImage();

#ifdef BinaryDebugDisplay
	MyVision.InitializeBinary(128);
#endif

	while (true) 
	{
		chrono::high_resolution_clock::time_point t1 = chrono::high_resolution_clock::now();
		auto duration = chrono::duration_cast<chrono::microseconds>(t1 - startTime).count();

		MyVision.AcquireImage();
		MyVision.ConvertToBinary(128);

		if (recording)
			MyVision.AddFrameToVideo(); //Add the frame to video

		MyVision.DisplayImage();

#ifdef BinaryDebugDisplay
		MyVision.DisplayBinary();
#endif
		if (mode == Automatic)
		{
			MyVision.DisplayText("Automatic Mode", 15, 40, vpColor::darkRed);
			//Track the blob
			if (MyVision.TrackBlob())
			{
				MyVision.GetBlobTrackerCoG(cog);
			}
			else
			{
				cout << "Could not track. Switching to manual mode." << endl;
				stopAllOperations = 1;
				mode = !Automatic;
			}

			//Write to log file.
			if (recording)
				outfile << duration << "," << cmdPosition.get_u() << "," << cmdPosition.get_v() << "," << cog.get_u() << "," << cog.get_v() << "," << (int)activationCoil << endl;

			displayParticleMotionVector(prevCog, cog, 100.0);
			MyVision.DisplayBlobTracker();

			if (trajectoryMode)
			{
				if (trajectory_id == ID_CIRCLE)
					cmdPosition = circularTrajectory(cog);
				else if (trajectory_id == ID_SPIRAL)
					cmdPosition = spiralTrajectory(cog);
				else
					cmdPosition = trajectory(cog);
			}
			if (p2pMode)
			{
				cmdPosition = p2p(cog);
			}

			else if (stepMode)
			{
				cmdPosition = stepping(cog);
			}
			else if (MyVision.getClickedPosition(&clickedTarget) && !trajectoryMode && !p2pMode) //Get clicked position from mouse click
			{
				cmdPosition = clickedTarget;
			}

			//Display target position
			std::stringstream ss2;
			ss2 << "Tx" << cmdPosition.get_u() << " " << "Ty" << cmdPosition.get_v() << endl;
			MyVision.DisplayText(ss2.str());
			ss2.str("");

			//Draw target
			MyVision.drawCross(cmdPosition, vpColor::yellow);
			//Display target vector
			MyVision.DisplayArrow(cog, cmdPosition, vpColor::lightGreen);
			if (!openLoopMode)
			{
				activationCoil = MyControl.selectCoilsLP(cog, cmdPosition, coilTip);
			}
			else
			{
				//Run experiments
				openLoopInfinityExp(cog, activationCoil, coilTip, cmdPosition);
				//openLoopDifferentDistancesExp(cog, activationCoil, coilTip, cmdPosition);
				//openLoopDifferentVoltageExp(cog, activationCoil, coilTip, cmdPosition);
				//openLoopDifferentCombinationsExp(cog, activationCoil, coilTip, cmdPosition);
				//closedLoopPosititioningExp(cog, activationCoil, coilTip, cmdPosition);
			}
		}
		else
		{
			MyVision.DisplayText("Manual Mode", 15, 40, vpColor::lightRed);
			activationCoil = MyControl.ManualCoilControl();
			if (oneCoilMode)
				activationCoil = 0b10000000;
		}

		MyControl.writeToDAQ(activationCoil);
		//Display coil status
		displayCoilStatus(activationCoil, coilTip);

		prevCog = cog;

		MyVision.Flush();

#ifdef BinaryDebugDisplay
		MyVision.FlushBinary();
#endif

		//Keyboard inputs

		if ((GetKeyState(VK_CONTROL) & 0x8000) && (GetKeyState('K') & 0x8000))
		{
			while (GetKeyState('K') & 0x8000);//wait for unpress
			keyboardInputEnabled = !keyboardInputEnabled;
			if (keyboardInputEnabled)
				std::cout << "Keyboard Input Enabled" << endl;
			else
				std::cout << "Keyboard Input Disabled" << endl;
		}

		if (keyboardInputEnabled)
		{
			//Switch  Mode
			if ((GetKeyState('M') & 0x8000))
			{
				while (GetKeyState('M') & 0x8000); //wait for unpress
				mode = !mode;
				if (mode == Automatic)
				{
					std::cout << "Initializing Tracking" << endl;
					/* Initialize vpDot blob tracker*/
					if (MyVision.InitializeBlobTracking())
					{
						std::cout << "Initialized Tracking" << endl;
						//Set command position to center
						cmdPosition.set_u(MX);
						cmdPosition.set_v(MY);
					}
					else
					{
						std::cout << "Could not initialize tracking. Switching to manual mode." << endl;
						mode = 0;
					}
				}
			}
			// Quit the program
			if ((GetKeyState('Q') & 0x8000)) // Detect if a key was pressed
			{
				while (GetKeyState('Q') & 0x8000);//wait for unpress
				userReqStop = 1; // Quit the while()
			}
			//Switch Trajectory Mode
			if ((GetKeyState('T') & 0x8000)) // Detect if a key was pressed
			{
				while (GetKeyState('T') & 0x8000);//wait for unpress

				if (!trajectoryMode)
				{
					nRepeats = 0;
					startRecording = 1;
					recording = 1;
					cout << "Started Trajectory and Recording" << endl;
					trajectoryMode = 1; //trajectory mode
					trajectoryStarted = 0;
				}
				else
				{
					stopRecording = 1;
					cout << "Stopped Trajectory and Recording" << endl;
					trajectoryMode = 0;
				}
			}

			//Switch Open Loop mode
			if ((GetKeyState('Y') & 0x8000)) // Detect if a key was pressed
			{
				while (GetKeyState('Y') & 0x8000);//wait for unpress

				if (!openLoopMode)
				{
					nRepeats = 0;
					cout << "Started Open Loop Control" << endl;
					openLoopMode = 1; //open loop mode
				}
				else
				{
					if (recording)
					{
						stopRecording = 1; // check if this causes any issues 11 26 2020
					}
					cout << "Stopped  Open Loop Control" << endl;
					openLoopMode = 0;
				}
			}

			//Increase step size
			if ((GetKeyState('H') & 0x8000)) // Detect if a key was pressed
			{
				while (GetKeyState('H') & 0x8000);//wait for unpress

				stepsize = stepsize + 5.5;
				if (stepsize == 55.0)
					stepsize = 5.5;
				cout << "StepSize = " << stepsize / 55 << "mm" << endl;
			}

			//Switch One coil mode
			if ((GetKeyState('W') & 0x8000)) // Detect if a key was pressed
			{
				while (GetKeyState('W') & 0x8000);//wait for unpress

				if (!oneCoilMode)
				{
					Sleep(10); // Wait 10ms
					startRecording = 1;
					cout << "Started One coil mode" << endl;
					oneCoilMode = 1; //trajectory mode
				}
				else
				{
					stopRecording = 1;
					cout << "Stopped One coil mode" << endl;
					oneCoilMode = 0;
				}
			}

			//Switch Stepping Mode
			if ((GetKeyState('S') & 0x8000)) // Detect if a key was pressed
			{
				while (GetKeyState('S') & 0x8000);//wait for unpress

				if (!stepMode)
				{
					Sleep(10);
					startRecording = 1;
					cout << "Started Stepping mode and  Recording" << endl;
					stepMode = 1;
					stepModeStarted = 0;
				}
				else
				{
					stopRecording = 1;
					cout << "Stopped Stepping mode" << endl;
					stepMode = 0;
				}
			}
			//Switch Recording Mode
			if ((GetKeyState('R') & 0x8000)) // Detect if a key was pressed
			{
				while (GetKeyState('R') & 0x8000);//wait for unpress
				if (!recording)
				{
					startRecording = 1;
				}
				else
				{
					stopRecording = 1;
				}
			}
			//Skip mode
			if ((GetKeyState('Z') & 0x8000)) // Detect if a key was pressed
			{
				while (GetKeyState('Z') & 0x8000);//wait for unpress
				skipMode++;
				cout << "Skip Mode On, Skips: " << skipMode << endl;
				if (skipMode == 3)
				{
					skipMode = 0;
					cout << "Skip Mode Off" << endl;
				}
			}

			//Change Trajectory
			if ((GetKeyState('V') & 0x8000)) // Detect if a key was pressed
			{
				while (GetKeyState('V') & 0x8000);//wait for unpress
				trajectory_id = trajectory_id + 1;
				if (trajectory_id == nID)
					trajectory_id = 0;

				PrintTrajectoryID();

				trajectoryStarted = 0; //reset trajectory
			}
		}
		if (startRecording)
		{
			MyVision.StartRecordingVideo();
			startTime = chrono::high_resolution_clock::now();
			startLogging();
			recording = 1;
			cout << "Started Recording" << endl;
			startRecording = 0;
		}
		if (stopRecording)
		{
			MyVision.StopRecordingVideo();
			stopLogging();
			recording = 0;
			cout << "Stopped Recording" << endl;
			stopRecording = 0;
		}

		chrono::high_resolution_clock::time_point t2 = chrono::high_resolution_clock::now();

		auto frameDuration = chrono::duration_cast<chrono::milliseconds>(t2 - t1).count();


		while (frameDuration < frameLength)
		{
			chrono::high_resolution_clock::time_point t2 = chrono::high_resolution_clock::now();
			frameDuration = chrono::duration_cast<chrono::milliseconds>(t2 - t1).count();
		}

		if (frameDuration > frameLength)
		{
			cout << "Unable to record at " << fps << " FPS" << endl;
			cout << "FPS: " << 1000.0 / ((float)frameDuration) << endl;
			cout << "Check USB port version: 3.0 or 2.0? /n Try Grayscale" << endl;
		}

		stopCondition = userReqStop;

		if (stopCondition)
		{
			cout << "Exiting the program" << endl;
			MyControl.writeToDAQ(0b00000000);
			Sleep(500);
			cout << "All outputs Low" << endl;
			MyControl.stopDAQ();
			cout << "DAQ Shutdown" << endl;
			Sleep(500);
			break;
		}

		if (stopAllOperations)
		{
			if (recording)
			{
				stopRecording = 1;
			}
			recording = 0;
			trajectoryMode = 0;
			oneCoilMode = 0;
			stepMode = 0;
			stopAllOperations = 0;
		}


	}
}


/**====================================================
* Function to print trajectory id to console.
* Input: COG of the object
* Output: Null
*======================================================*/
void PrintTrajectoryID()
{
	if (trajectory_id == ID_V)
	{
		cout << "Draw Verticle Lines" << endl;
	}
	else if (trajectory_id == ID_VO)
	{
		cout << "Draw Opposite Verticle Lines" << endl;
	}
	else if (trajectory_id == ID_H)
	{
		cout << "Draw Horizontal Lines" << endl;
	}
	else if (trajectory_id == ID_HO)
	{
		cout << "Draw Opposite Horizontal Lines" << endl;
	}
	else if (trajectory_id == ID_A)
	{
		cout << "Draw Letter A" << endl;
	}
	else if (trajectory_id == ID_U)
	{
		cout << "Draw Letter U" << endl;
	}
	else if (trajectory_id == ID_S)
	{
		cout << "Draw Letter S" << endl;
	}
	else if (trajectory_id == ID_e)
	{
		cout << "Draw Letter e" << endl;
	}
	else if (trajectory_id == ID_d)
	{
		cout << "Draw Letter d" << endl;
	}
	else if (trajectory_id == ID_O)
	{
		cout << "Draw Letter O" << endl;
	}
	else if (trajectory_id == ID_I)
	{
		cout << "Draw Letter I" << endl;
	}

	else if (trajectory_id == ID_L)
	{
		cout << "Draw Letter L" << endl;
	}
	else if (trajectory_id == ID_T)
	{
		cout << "Draw Letter T" << endl;
	}
	else if (trajectory_id == ID_SQUARE)
	{
		cout << "Draw Square" << endl;
	}
	else if (trajectory_id == ID_CIRCLE)
	{
		cout << "Draw Circle" << endl;
	}
	else if (trajectory_id == ID_SPIRAL)
	{
		cout << "Draw Spiral" << endl;
	}

}


/**====================================================
* Function to control trajectory.
* Input: COG of the object
* Output: Null
*======================================================*/
vpImagePoint trajectory(vpImagePoint cog)
{
	vpImagePoint positionCommand;
	static int k;
	static int j;
	static std::vector<double> x;
	static std::vector<double> y;

	std::vector<double> vec_x;
	std::vector<double> vec_y;
	int xdiff = 0;
	int ydiff = 0;
	int steps;
	float lineMagnitude = 0.0;

	// 7-segmentish style coordinates
	double	     MX = 470;
	double       MY = 532;
	double	     LX = MX - 60;
	double       RX = MX + 60;

	double HY = MY - 90;
	double LY = MY + 90;


	// Square coordinates
	double S_LX = MX - 93;
	double S_RX = MX + 93;
	double S_HY = MY - 93;
	double S_LY = MY + 93;

	//Reset variables
	if (!trajectoryStarted)
	{
		k = 0;
		j = 0;
		trajectoryStarted = 1;
	}

	//Initialization
	if (!firstRun)
	{
		//By default draw letter A
		x.assign({ LX,   LX,   RX,   RX,   RX,  LX }); //Ax 
		y.assign({ LY,   HY,   HY,   LY,   MY,  MY }); //Ay

		firstRun = 1;
	}


	if (trajectory_id == ID_H)
	{
		x.assign({ S_LX,   S_RX });
		y.assign({ MY,   MY });
	}

	else if (trajectory_id == ID_HO)
	{
		x.assign({ S_RX, S_LX });
		y.assign({ MY,   MY });
	}
	else if (trajectory_id == ID_A)
	{
		x.assign({ LX,   LX,   RX,   RX,   RX, LX }); //Ax 
		y.assign({ LY,   HY,   HY,   LY,   MY, MY }); //Ay
	}
	else if (trajectory_id == ID_U)
	{
		x.assign({ LX,   LX,   RX,   RX });
		y.assign({ HY,   LY,   LY,   HY });
	}
	else if (trajectory_id == ID_S)
	{
		x.assign({ RX,   LX,   LX,   RX,   RX, LX }); //Sx 
		y.assign({ HY,   HY,   MY,   MY,   LY, LY }); //Sy
	}
	else if (trajectory_id == ID_O)
	{
		x.assign({ LX,   LX,   RX,   RX,   LX });
		y.assign({ HY,   LY,   LY,   HY,   HY });
	}
	else if (trajectory_id == ID_e)
	{
		x.assign({ LX, RX, RX, LX, LX, RX });
		y.assign({ MY, MY, HY, HY, LY, LY });
	}
	else if (trajectory_id == ID_d)
	{
		x.assign({ RX,   RX,   LX,   LX,   RX });
		y.assign({ HY,   LY,   LY,   MY,   MY });
	}
	else if (trajectory_id == ID_I)
	{
		x.assign({ RX,   RX });
		y.assign({ HY,   LY });
	}
	else if (trajectory_id == ID_V)
	{
		x.assign({ MX,   MX });
		y.assign({ LY,   HY });

	}
	else if (trajectory_id == ID_VO)
	{
		x.assign({ MX,   MX });
		y.assign({ HY,   LY });
	}

	else if (trajectory_id == ID_L)
	{
		x.assign({ LX,   LX,   RX });
		y.assign({ HY,   LY,   LY });
	}
	else if (trajectory_id == ID_T)
	{
		x.assign({ LX,   RX,   MX,   MX });
		y.assign({ HY,   HY,   HY,   LY });
	}
	else if (trajectory_id == ID_SQUARE)
	{
		x.assign({ S_LX,   S_RX,   S_RX,   S_LX, S_LX });
		y.assign({ S_HY,	S_HY,	S_LY,	S_LY, S_HY });
	}


	//Go to starting point of the trajectory
	if (k == 0)
	{
		positionCommand.set_u(x[k]);
		//cout << "Xcmd" << x[k] << endl;
		positionCommand.set_v(y[k]);
		if ((abs(x[k] - cog.get_u()) < positionErrorTolerance) && ((abs(y[k] - cog.get_v()) < positionErrorTolerance)))
			k++;

	}
	//Generate trajectory points and update the position command
	if (k > 0 && k < x.size())
	{
		xdiff = (x[(long long)k - 1] - x[k]);
		ydiff = (y[(long long)k - 1] - y[k]);
		lineMagnitude = sqrt(xdiff * xdiff + ydiff * ydiff);
		steps = int(lineMagnitude / stepsize);

		vec_x = linspace(x[(long long)k - 1], x[k], steps);
		vec_y = linspace(y[(long long)k - 1], y[k], steps);
		positionCommand.set_u(vec_x.at(j));
		positionCommand.set_v(vec_y.at(j));
		//Increment the trajectory point if the minimum tolerance is met.
		if ((abs(vec_x.at(j) - cog.get_u()) < positionErrorTolerance) && ((abs(vec_y.at(j) - cog.get_v()) < positionErrorTolerance)))
			j++;
		else if (((j + 2) <= steps) && (skipMode == 1 || skipMode == 2))
		{
			if ((abs(vec_x.at((long long)j + 1) - cog.get_u()) < positionErrorTolerance) && ((abs(vec_y.at((long long)j + 1) - cog.get_v()) < positionErrorTolerance)))
			{
				j = j + 2;
				cout << "skipped 1" << endl;
			}
		}
		else if (((j + 3) <= steps) && skipMode == 2)
		{
			if ((abs(vec_x.at((long long)j + 2) - cog.get_u()) < positionErrorTolerance) && ((abs(vec_y.at((long long)j + 2) - cog.get_v()) < positionErrorTolerance)))
			{
				j = j + 3;
				cout << "skipped 2" << endl;
			}
		}
		if (j == steps)
		{
			j = 0;
			k++;
		}

	}

	if (k == x.size())
	{
		k = 0;

		nRepeats = nRepeats + 1;
		cout << "Iteration No: " << nRepeats + 1 << endl;

		trajectoryStarted = 0;
	}

	return positionCommand;

}


/**====================================================
* Function to perform circular trajectory.
* Input: COG of the object
* Output: Null
*======================================================*/
vpImagePoint  circularTrajectory(vpImagePoint cog)
{

	vpImagePoint positionCommand;
	static int k;

	static std::vector<double> circtheta;

	static std::vector<double> vec_x;
	static std::vector<double> vec_y;

	//center coordinates
	double	     MX = 470;
	double       MY = 532;

	// circle spec
	double startAngle = 0.0;
	double endAngle = 2 * M_PI;
	double radius = 93;
	int steps = 94;

	//Reset variables
	if (!trajectoryStarted)
	{
		k = 0;
		circtheta = linspace(startAngle, endAngle, steps);
		vec_x = circtheta;//initialize
		vec_y = circtheta;//initialize
		for (int i = 0; i < circtheta.size(); i++)
		{
			vec_x.at(i) = MX + radius * sin(circtheta[i]);
			vec_y.at(i) = MY + radius * cos(circtheta[i]);
		}
		cout << "Theta" << circtheta.at(59) << endl;

		trajectoryStarted = 1;
	}

	//Initialization
	if (!firstRun)
	{
		firstRun = 1;
	}



	//Go to starting point of the trajectory
	if (k == 0)
	{
		positionCommand.set_u(vec_x[k]);
		positionCommand.set_v(vec_y[k]);
		if ((abs(vec_x[k] - cog.get_u()) < positionErrorTolerance) && ((abs(vec_y[k] - cog.get_v()) < positionErrorTolerance)))
			k++;

	}

	//Follow the rest of the tracjectory
	if (k > 0 && k < vec_x.size())
	{
		positionCommand.set_u(vec_x.at(k));
		positionCommand.set_v(vec_y.at(k));
		//Increment the trajectory point if the minimum tolerance is met.
		if ((abs(vec_x.at(k) - cog.get_u()) < positionErrorTolerance) && ((abs(vec_y.at(k) - cog.get_v()) < positionErrorTolerance)))
			k++;

		else if ((((long long)k + 2) <= vec_x.size()) && (skipMode == 1 || skipMode == 2))
		{
			if ((abs(vec_x.at((long long)k + 1) - cog.get_u()) < positionErrorTolerance) && ((abs(vec_y.at((long long)k + 1) - cog.get_v()) < positionErrorTolerance)))
			{
				k = k + 2;
				cout << "skipped 1" << endl;
			}
		}
		else if ((((long long)k + 3) <= vec_x.size()) && skipMode == 2)
		{
			if ((abs(vec_x.at((long long)k + 2) - cog.get_u()) < positionErrorTolerance) && ((abs(vec_y.at((long long)k + 2) - cog.get_v()) < positionErrorTolerance)))
			{
				k = k + 3;
				cout << "skipped 2" << endl;
			}
		}

	}

	if (k == vec_x.size())
	{
		k = 0;

		nRepeats = nRepeats + 1;
		cout << "Iteration No: " << nRepeats + 1 << endl;

		trajectoryStarted = 0;
	}

	return positionCommand;

}

/**====================================================
* Function to perform spiral trajectory.
* Input: COG of the object
* Output: Null
*======================================================*/
vpImagePoint  spiralTrajectory(vpImagePoint cog)
{
	vpImagePoint positionCommand;
	static int k;

	static std::vector<double> spiraltheta;

	static std::vector<double> vec_x;
	static std::vector<double> vec_y;

	//center coordinates
	double	     MX = 470;
	double       MY = 532;

	// circle spec
	double r = 185;// outer radius
	double a = 5;// inner radius
	double b = 30;// incerement per rev
	double n = (r - a) / (b);// number  of revolutions
	double th = 2 * n * M_PI;// angle
	int  steps = 200;
	double startAngle = 0.0;
	double endAngle = 2 * M_PI;
	double radius = 90.0; //pixels

	//Reset variables
	if (!trajectoryStarted)
	{
		k = 0;
		spiraltheta = linspace(startAngle, th, steps);
		vec_x = spiraltheta;//initialize
		vec_y = spiraltheta;//initialize
		for (int i = 0; i < spiraltheta.size(); i++)
		{
			vec_x.at(i) = MX + (a + b * spiraltheta[i] / (2 * M_PI)) * cos(spiraltheta[i]);
			vec_y.at(i) = MY + (a + b * spiraltheta[i] / (2 * M_PI)) * sin(spiraltheta[i]);
		}

		trajectoryStarted = 1;
	}
	//Initialization
	if (!firstRun)
	{
		firstRun = 1;
	}
	//Go to starting point of the trajectory
	if (k == 0)
	{
		positionCommand.set_u(vec_x[k]);
		positionCommand.set_v(vec_y[k]);
		if ((abs(vec_x[k] - cog.get_u()) < positionErrorTolerance) && ((abs(vec_y[k] - cog.get_v()) < positionErrorTolerance)))
			k++;


	}

	for (int i = 0; i < steps; i++)
		MyVision.drawCross(vpImagePoint(vec_y[i], vec_x[i]), vpColor::yellow);

	//Follow the rest of the tracjectory
	if (k > 0 && k < vec_x.size())
	{
		positionCommand.set_u(vec_x.at(k));
		positionCommand.set_v(vec_y.at(k));
		//Increment the trajectory point if the minimum tolerance is met.
		if ((abs(vec_x.at(k) - cog.get_u()) < positionErrorTolerance) && ((abs(vec_y.at(k) - cog.get_v()) < positionErrorTolerance)))
			k++;
		else if ((((long long)k + 2) <= vec_x.size()) && (skipMode == 1 || skipMode == 2))
		{
			if ((abs(vec_x.at((long long)k + 1) - cog.get_u()) < positionErrorTolerance) && ((abs(vec_y.at((long long)k + 1) - cog.get_v()) < positionErrorTolerance)))
			{
				k = k + 2;
				cout << "Skipped 1" << endl;
			}
		}
		else if ((((long long)k + 3) <= vec_x.size()) && skipMode == 2)
		{
			if ((abs(vec_x.at((long long)k + 2) - cog.get_u()) < positionErrorTolerance) && ((abs(vec_y.at((long long)k + 2) - cog.get_v()) < positionErrorTolerance)))
			{
				k = k + 3;
				cout << "Skipped 2" << endl;
			}
		}
	}

	if (k == vec_x.size())
	{
		k = 0;

		nRepeats = nRepeats + 1;
		cout << "Iteration No: " << nRepeats + 1 << endl;

		trajectoryStarted = 0;
	}
	return positionCommand;
}

/**====================================================
* Function to perform point to point control.
* Input: COG of the object
* Output: Null
*======================================================*/
vpImagePoint  p2p(vpImagePoint cog)
{

	vpImagePoint positionCommand;
	static int k;
	static int p;
	static std::vector<double> x;
	static std::vector<double> y;
	static bool start_delay_loop = false;

	// 7-segment style coordinates
	double	     MX = 470;
	double       MY = 532;
	double	     LX = MX - 60;
	double       RX = MX + 60;

	double HY = MY - 93;
	double LY = MY + 93;



	//Reset variables
	if (!p2pStarted)
	{
		k = 0;
		p = 0;
		p2pStarted = 1;
	}

	//Initialization
	if (!firstRun)
	{
		//Draw letter A by default
		x.assign({ LX,   LX,   RX,   RX,   RX, LX }); //Ax 
		y.assign({ LY,   HY,   HY,   LY,   MY, MY }); //Ay

		firstRun = 1;
	}


	if (trajectory_id == ID_V)
	{
		x.assign({ LX, LX, MX, MX, RX, RX,LX, LX, MX, MX, RX, RX, });
		y.assign({ HY, LY, HY, LY, HY, LY,LY, HY, LY, HY, LY, HY, });
	}



	else if (trajectory_id == ID_H)
	{
		x.assign({ LX, RX, LX, RX, LX, RX, RX, LX, RX, LX, RX, LX });
		y.assign({ LY, LY, MY, MY, HY, HY, LY, LY, MY, MY, HY, HY });
	}



	//set target point
	positionCommand.set_u(x[k]);
	positionCommand.set_v(y[k]);


	if ((abs(x[k] - cog.get_u()) < positionErrorTolerance) && ((abs(y[k] - cog.get_v()) < positionErrorTolerance)) && !start_delay_loop)
	{
		start_delay_loop = true;

	}
	if (start_delay_loop)
	{
		if (p < 20)
			p++;
		else
		{
			k++;
			start_delay_loop = false;
			p = 0;
		}
	}

	if (k == x.size())
	{
		k = 0;
		nRepeats = nRepeats + 1;
		cout << "Iteration No: " << nRepeats + 1 << endl;
		p2pStarted = 0;
	}

	return positionCommand;

}

/**====================================================
* Function to provide step input to the controller.
* Input: COG of the object
* Output: Null
*======================================================*/
vpImagePoint  stepping(vpImagePoint cog)
{
	vpImagePoint positionCommand;
	static double x;
	static double y;

	if ((GetKeyState('V') & 0x8000)) // Press V to change variable
	{
		while (GetKeyState('V') & 0x8000);//wait for unpress

		stepX = !stepX;
	}

	if ((GetKeyState('E') & 0x8000)) // Press E to give step
	{
		while (GetKeyState('E') & 0x8000);//wait for unpress

		if (stepX)
			x = x + 50;
		else
			y = y + 50;

	}
	if (!stepModeStarted)
	{
		x = cog.get_u();
		y = cog.get_v();
		stepModeStarted = 1;
	}

	positionCommand.set_u(x);
	positionCommand.set_v(y);

	return positionCommand;

}

/**====================================================
* Function to start logging.
* Input: NULL
* Output: NULL
*======================================================*/
void startLogging()
{
	auto now = std::chrono::system_clock::now();
	auto in_time_t = std::chrono::system_clock::to_time_t(now);
	std::stringstream ss;
	ss << std::put_time(std::localtime(&in_time_t), "%Y_%m_%d_%H_%M_%S");
	string filename = ss.str() + ".txt";

	outfile.open(filename, ios::out);

}

/**====================================================
* Function to stop logging
* Input: NULL
* Output: NULL
*======================================================*/
void stopLogging()
{
	outfile.close();
}

/**====================================================
* Function to display particle motion vector
* Input: NULL
* Output: NULL
*======================================================*/
void displayParticleMotionVector(vpImagePoint realArrowBegin, vpImagePoint realArrowEnd, float scalingFactor)
{
	vpImagePoint displayArrowEndCoord;
	double u;
	double v;
	double theta;
	double magnitude;
	double arrowDisplayMagnitude;
	double uArrow;
	double vArrow;

	u = realArrowEnd.get_u() - realArrowBegin.get_u();
	v = realArrowEnd.get_v() - realArrowBegin.get_v();
	theta = atan2(u, v);
	magnitude = sqrt(u * u + v * v);
	arrowDisplayMagnitude = magnitude * scalingFactor;
	uArrow = realArrowEnd.get_u() + arrowDisplayMagnitude * sin(theta);
	vArrow = realArrowEnd.get_v() + arrowDisplayMagnitude * cos(theta);
	displayArrowEndCoord.set_u(uArrow);
	displayArrowEndCoord.set_v(vArrow);

	//Display velocity vector
	MyVision.DisplayArrow(realArrowEnd, displayArrowEndCoord, vpColor::lightBlue);
}

/**====================================================
* Function to display coil status
* Input: Coil status, Coil positions
* Output: NULL
*======================================================*/
void displayCoilStatus(uInt8 coilData, vpImagePoint coilPositions[])
{
	for (int i = 0; i < numberOfCoils; i++)
	{
		if (coilData & (1 << i))
			MyVision.drawCircle(coilPositions[i], vpColor::lightGreen, true);
		else
			MyVision.drawCircle(coilPositions[i], vpColor::darkRed, true);
		//Mark the first coil with yellow cross
		if (i == 0)
		{
			MyVision.drawCross(coilPositions[i], vpColor::yellow);
		}
		//Mark the last coil with red cross
		if (i == 7)
		{
			MyVision.drawCross(coilPositions[i], vpColor::red);
		}
	}

}

/**=============================================================================
* Function to perform open loop different distance vs velocity experiments
* Input: COG of the object, Activation coils, Coil positions, Command Position
* Output: Null
*==============================================================================*/
void openLoopDifferentDistancesExp(vpImagePoint cog, uInt8& coilActivation, vpImagePoint coilTip[], vpImagePoint& cmdPosition)
{
	static int state = 0;

	const uInt8 initialCoilActuation = 0b00001000;
	static uInt8 coils;

	vpImagePoint positionCommand;
	static int k;

	static int nIterations = 0;
	const int maxIterations = 6;

	static std::vector<double> vec_x;
	static std::vector<double> vec_y;

	//center coordinates
	double	     MX = 470;
	double       MY = 532;
	vpImagePoint center;
	center.set_u(MX);
	center.set_v(MY);
	// circle spec
	double radius = 120.0;
	double noEntryRadius = 140.0;
	double tlr = 20;
	int steps = 5;//360;

	chrono::high_resolution_clock::time_point currentTime = chrono::high_resolution_clock::now();
	static chrono::high_resolution_clock::time_point startTime;
	long long duration = 0;

	MyVision.drawCircleWithRadius(center, (int)radius, vpColor::darkGreen, 0);
	MyVision.drawCircleWithRadius(center, (int)noEntryRadius, vpColor::darkRed, 0);
	//Reset variables
	if (state == 0)
	{
		k = 0;//4;
		vec_x = linspace(coilTip[3].get_u(), MX, steps);
		vec_y = linspace(MY + radius - tlr, MY - radius + tlr, steps);
		state = 1;
		coils = initialCoilActuation; //First coil to activate
		cout << "Initialized" << endl;
	}

	if (state == 1)
	{
		cmdPosition.set_u(vec_x[k]);
		cmdPosition.set_v(vec_y[k]);
		coilActivation = MyControl.selectCoilsLP(cog, cmdPosition, coilTip);

		for (int i = 0; i < steps; i++)
			MyVision.drawCross(vpImagePoint(vec_y[i], vec_x[i]), vpColor::yellow);
	}

	if (state == 1 && ((abs(vec_x[k] - cog.get_u()) < positionErrorTolerance) && (abs(vec_y[k] - cog.get_v()) < positionErrorTolerance)))
	{
		cmdPosition.set_u(vec_x[k]);
		cmdPosition.set_v(vec_y[k]);
		coilActivation = MyControl.selectCoilsLP(cog, cmdPosition, coilTip);
		startTime = chrono::high_resolution_clock::now();
		state = 2;
	}
	if (state == 2)
	{
		cmdPosition.set_u(vec_x[k]);
		cmdPosition.set_v(vec_y[k]);
		coilActivation = MyControl.selectCoilsLP(cog, cmdPosition, coilTip);
		duration = chrono::duration_cast<chrono::milliseconds>(currentTime - startTime).count();
	}

	if (state == 2 && duration > 1000)
	{
		coilActivation = 0b00000000;
		startTime = chrono::high_resolution_clock::now();
		state = 3;
	}

	if (state == 3)
	{
		coilActivation = 0b00000000;
		duration = chrono::duration_cast<chrono::milliseconds>(currentTime - startTime).count();

		for (int i = 0; i < steps; i++)
			MyVision.drawCross(vpImagePoint(vec_y[i], vec_x[i]), vpColor::yellow);
	}

	if (state == 3 && duration > 1000)
	{

		startRecording = 1;
		startTime = chrono::high_resolution_clock::now();
		state = 4;
	}
	if (state == 4)
	{
		coilActivation = coils;
		duration = chrono::duration_cast<chrono::milliseconds>(currentTime - startTime).count();

		for (int i = 0; i < steps; i++)
			MyVision.drawCross(vpImagePoint(vec_y[i], vec_x[i]), vpColor::yellow);
	}

	if (state == 4 && ((duration > 10000) || ((abs(MX - cog.get_u()) > noEntryRadius) || (abs(MY - cog.get_v()) > noEntryRadius))))
	{
		stopRecording = 1;

		if (nIterations < maxIterations - 1)
		{
			nIterations++;
			state = 1;
		}
		else
		{
			k++;
			if (k == steps)
				k = 0;
			nIterations = 0;
			state = 1;
		}

	}
}

/**=============================================================================
* Function to perform open loop voltage vs velocity experiments
* Input: COG of the object, Activation coils, Coil positions, Command Position
* Output: Null
*==============================================================================*/
void openLoopDifferentVoltageExp(vpImagePoint cog, uInt8& coilActivation, vpImagePoint coilTip[], vpImagePoint& cmdPosition)
{
	static int state = 0;

	const uInt8 initialCoilActuation = 0b00001000;
	static uInt8 coils;

	vpImagePoint positionCommand;
	static int k;

	static int nIterations = 0;
	const int maxIterations = 6;
	//center coordinates
	double	     MX = 450;
	double       MY = 532;
	vpImagePoint center;
	center.set_u(MX);
	center.set_v(MY);
	// circle spec
	double radius = 120.0; //pixels
	double tlr = 40;

	chrono::high_resolution_clock::time_point currentTime = chrono::high_resolution_clock::now();
	static chrono::high_resolution_clock::time_point startTime;
	long long duration = 0;

	MyVision.drawCircleWithRadius(center, (int)radius, vpColor::darkRed, 0);
	//Reset variables
	if (state == 0)
	{
		state = 1;
		coils = initialCoilActuation; //First coil to activate
		cout << "Initialized" << endl;
	}

	if (state == 1)
	{
		cmdPosition.set_u(MX);
		cmdPosition.set_v(MY + radius - tlr);
		coilActivation = MyControl.selectCoilsLP(cog, cmdPosition, coilTip);
	}

	if (state == 1 && ((abs(MX - cog.get_u()) < positionErrorTolerance) && (abs(MY + radius - tlr - cog.get_v()) < positionErrorTolerance)))
	{
		coilActivation = 0b00000000;
		startTime = chrono::high_resolution_clock::now();
		state = 2;
	}
	if (state == 2)
	{
		duration = chrono::duration_cast<chrono::milliseconds>(currentTime - startTime).count();
	}

	if (state == 2 && duration > 1000)
	{

		startRecording = 1;

		startTime = chrono::high_resolution_clock::now();
		state = 3;
	}
	if (state == 3)
	{
		coilActivation = coils;
		duration = chrono::duration_cast<chrono::milliseconds>(currentTime - startTime).count();
	}

	if (state == 3 && ((duration > 10000) || ((abs(MX - cog.get_u()) > radius) || (abs(MY - cog.get_v()) > radius))))
	{
		stopRecording = 1;

		if (nIterations < maxIterations - 1)
		{
			nIterations++;
			cout << "nIterations" << nIterations + 1 << endl;
			state = 1;
		}
		else
		{
			nIterations = 0;
			state = 1;
		}
	}

}

/**=============================================================================
* Function to perform open loop infinity test
* Input: COG of the object, Activation coils, Coil positions, Command Position
* Output: Null
*==============================================================================*/
void openLoopInfinityExp(vpImagePoint cog, uInt8& coilActivation, vpImagePoint coilTip[], vpImagePoint& cmdPosition)
{
	static int state = 0;

	const uInt8 initialCoilActuation = 0b00001000;
	static uInt8 coils;

	vpImagePoint positionCommand;
	static int k;

	static int nIterations = 0;
	const int maxIterations = 6;

	//center coordinates
	double	     MX = 470;
	double       MY = 532;
	vpImagePoint center;
	center.set_u(MX);
	center.set_v(MY);
	// circle spec
	double radius = 120.0; //pixels
	double tlr = 40;
	chrono::high_resolution_clock::time_point currentTime = chrono::high_resolution_clock::now();
	static chrono::high_resolution_clock::time_point startTime;

	long long duration = 0;

	MyVision.drawCircleWithRadius(center, (int)radius, vpColor::darkRed, 0);
	//Reset variables
	if (state == 0)
	{
		state = 1;
		coils = initialCoilActuation; //First coil to activate
		cout << "Initialized" << endl;
	}

	if (state == 1)
	{
		cmdPosition.set_u(MX);
		cmdPosition.set_v(MY + radius - 40);
		coilActivation = MyControl.selectCoilsLP(cog, cmdPosition, coilTip);
	}

	if (state == 1 && ((abs(MX - cog.get_u()) < positionErrorTolerance) && (abs(MY + radius - tlr - cog.get_v()) < positionErrorTolerance)))
	{
		startTime = chrono::high_resolution_clock::now();
		state = 2;
	}
	if (state == 2)
	{
		duration = chrono::duration_cast<chrono::milliseconds>(currentTime - startTime).count();
	}

	if (state == 2 && duration > 1000)
	{

		startRecording = 1;
		startTime = chrono::high_resolution_clock::now();
		state = 3;
	}
	if (state == 3)
	{
		coilActivation = coils;
		duration = chrono::duration_cast<chrono::milliseconds>(currentTime - startTime).count();
	}

	if (state == 3 && ((duration > 10000)))
	{
		stopRecording = 1;

		if (nIterations < maxIterations - 1)
		{
			nIterations++;
			cout << "nIterations" << nIterations + 1 << endl;
			state = 1;
		}
		else
		{
			nIterations = 0;
			state = 1;
		}


	}

}

/**=============================================================================
* Function to perform open loop different combination vs velocity experiments
* Input: COG of the object, Activation coils, Coil positions, Command Position
* Output: Null
*==============================================================================*/
void openLoopDifferentCombinationsExp(vpImagePoint cog, uInt8& coilActivation, vpImagePoint coilTip[], vpImagePoint& cmdPosition)
{
	static int state = 0; // 0 Initialize, 1 Move to shooting point, 

	const uInt8 initialCoilActuation = 0b00001000;
	static uInt8 coils;

	vpImagePoint positionCommand;
	static int k;

	static int nIterations = 0;
	const int maxIterations = 7;

	//center coordinates
	double	     MX = 470;
	double       MY = 532;
	vpImagePoint center;
	center.set_u(MX);
	center.set_v(MY);
	// circle spec
	double radius = 120.0; //pixels

	chrono::high_resolution_clock::time_point currentTime = chrono::high_resolution_clock::now();
	static chrono::high_resolution_clock::time_point startTime;
	long long duration = 0;

	MyVision.drawCircleWithRadius(center, (int)radius, vpColor::darkRed, 0);
	//Reset variables
	if (state == 0)
	{
		k = 0;
		state = 1;
		coils = initialCoilActuation; //First coil to activate
		cout << "Initialized" << endl;
	}

	if (state == 1 || state == 2)
	{
		cmdPosition.set_u(MX);
		cmdPosition.set_v(MY);
		coilActivation = MyControl.selectCoilsLP(cog, cmdPosition, coilTip);
	}

	if (state == 1 && ((abs(cmdPosition.get_u() - cog.get_u()) < positionErrorTolerance) && (abs(cmdPosition.get_v() - cog.get_v()) < positionErrorTolerance)))
	{
		startTime = chrono::high_resolution_clock::now();
		state = 2;
	}
	if (state == 2 || state == 3)
	{
		duration = chrono::duration_cast<chrono::milliseconds>(currentTime - startTime).count();
	}

	if (state == 3 && duration > 1000)
	{
		startRecording = 1;
		coilActivation = coils;
		startTime = chrono::high_resolution_clock::now();
		state = 4;
	}
	if (state == 2 && duration > 1000)
	{
		startTime = chrono::high_resolution_clock::now();
		coilActivation = 0b00000000;
		state = 3;
	}

	if (state == 4)
	{
		coilActivation = coils;
		duration = chrono::duration_cast<chrono::milliseconds>(currentTime - startTime).count();
	}

	if (state == 4 && ((duration > 10000) || ((abs(MX - cog.get_u()) > radius) || (abs(MY - cog.get_v()) > radius))))
	{
		stopRecording = 1;

		if (nIterations < maxIterations - 1)
		{
			nIterations++;
			state = 1;
		}
		else
		{
			k++;
			if (k > 7)
			{
				k = 0;
				coils = 0b00001000;
			}
			if (k == 1)
				coils = 0b00000100;
			if (k == 2)
				coils = 0b00001100;
			if (k == 3)
				coils = 0b00011100;
			if (k == 4)
				coils = 0b00001110;

			if (k > 4)
			{
				k = 0;
				coils = 0b00001000;
			}

			nIterations = 0;
			state = 1;
		}

	}
}

/**=============================================================================
* Function to perform closed loop positioning experiment
* Input: COG of the object, Activation coils, Coil positions, Command Position
* Output: Null
*==============================================================================*/
void closedLoopPosititioningExp(vpImagePoint cog, uInt8& coilActivation, vpImagePoint coilTip[], vpImagePoint& cmdPosition)
{
	static int state = 0;

	const uInt8 initialCoilActuation = 0b00001000;
	static uInt8 coils;

	vpImagePoint positionCommand;
	static int k;

	static int nIterations = 0;
	const int maxIterations = 1;

	static std::vector<double> vec_x;
	static std::vector<double> vec_y;

	//center coordinates
	double	     MX = 470;
	double       MY = 532;
	vpImagePoint center;
	center.set_u(MX);
	center.set_v(MY);
	// circle spec
	double radius = 10.0;
	double tlr = 9;
	int steps = 3;

	chrono::high_resolution_clock::time_point currentTime = chrono::high_resolution_clock::now();
	static chrono::high_resolution_clock::time_point startTime;
	long long duration = 0;

	MyVision.drawCircleWithRadius(center, (int)radius, vpColor::darkGreen, 0);
	//Reset variables
	if (state == 0)
	{
		k = 0;
		vec_x = linspace(MX - tlr, MX + tlr, steps);
		vec_y = linspace(MY + radius, MY - radius, steps);

		state = 1;

		coils = initialCoilActuation; //First coil to activate
		cout << "Initialized Closed loop positioning Test" << endl;
	}

	if (state == 1)
	{
		cmdPosition.set_u(vec_x[k]);
		cmdPosition.set_v(vec_y[k]);
		coilActivation = MyControl.selectCoilsLP(cog, cmdPosition, coilTip);

		for (int i = 0; i < steps; i++)
			MyVision.drawCross(vpImagePoint(vec_y[i], vec_x[i]), vpColor::yellow);
	}

	if (state == 1 && ((abs(vec_x[k] - cog.get_u()) < positionErrorTolerance) && (abs(vec_y[k] - cog.get_v()) < positionErrorTolerance)))
	{
		startTime = chrono::high_resolution_clock::now();
		state = 2;
	}
	if (state == 2)
	{
		cmdPosition.set_u(vec_x[k]);
		cmdPosition.set_v(vec_y[k]);
		coilActivation = MyControl.selectCoilsLP(cog, cmdPosition, coilTip);
		duration = chrono::duration_cast<chrono::milliseconds>(currentTime - startTime).count();
		for (int i = 0; i < steps; i++)
			MyVision.drawCross(vpImagePoint(vec_y[i], vec_x[i]), vpColor::yellow);

	}

	if (state == 2 && duration > 2000)
	{
		startRecording = 1;
		cmdPosition.set_u(vec_x[k]);
		cmdPosition.set_v(vec_y[k]);
		coilActivation = MyControl.selectCoilsLP(cog, cmdPosition, coilTip);
		startTime = chrono::high_resolution_clock::now();
		state = 3;
	}

	if (state == 3)
	{
		duration = chrono::duration_cast<chrono::milliseconds>(currentTime - startTime).count();
		cmdPosition.set_u(vec_x[k]);
		cmdPosition.set_v(vec_y[k]);
		coilActivation = MyControl.selectCoilsLP(cog, cmdPosition, coilTip);

		for (int i = 0; i < steps; i++)
			MyVision.drawCross(vpImagePoint(vec_y[i], vec_x[i]), vpColor::yellow);
	}

	if (state == 3 && (duration > 60000))
	{
		stopRecording = 1;
		k++;
		if (k >= steps)
			k = 0;

		cmdPosition.set_u(vec_x[k]);
		cmdPosition.set_v(vec_y[k]);
		coilActivation = MyControl.selectCoilsLP(cog, cmdPosition, coilTip);
		nIterations = 0;
		state = 1;
	}
}

