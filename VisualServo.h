#pragma once
#ifndef VISUALSERVO_H
#define VISUALSERVO_H

#include <chrono>
#include <ctime>

#include "Vision.h"

#include "Controller.h"

//#include "FlyCapture2.h"
#include <thread>
#include <visp3/core/vpConfig.h>
#include <bitset>
#include "utility.h"



//Defvions
#define Automatic 1

//Trajectory IDs
#define ID_V 0 //Verticle Lines
#define ID_A 1 //letter A
#define ID_L 2 //letter L
#define ID_T 3 //letter T
#define ID_S 4 //letter S
#define ID_e 5 //letter e
#define ID_O 6 //letter D or O
#define ID_I 7 //letter I
#define ID_L 8 //letter L
#define ID_U 9 //letter U
#define ID_d 10 //letter d
#define ID_H 11 //Horizontal Lines
#define ID_SQUARE 12// Square
#define ID_CIRCLE 13//Circle
#define ID_SPIRAL 14//Spiral
#define ID_HO 15 //Horizontal Lines Opposite
#define ID_VO 16 //Verticle Lines Opposite

#define nID 17 //number of IDs



#define nExp 6
#define DC 0
#define DD 1
#define MA 3
#define DV 4
#define IT 5
#define MA2 6


//Function prototypes
void displayParticleMotionVector(vpImagePoint realArrowBegin, vpImagePoint realArrowEnd, float scalingFactor);
void displayCoilStatus(uInt8 activatedCoil, vpImagePoint coilPositions[]);
void PrintTrajectoryID();

vpImagePoint  trajectory(vpImagePoint cog);
vpImagePoint  stepping(vpImagePoint cog);
vpImagePoint  p2p(vpImagePoint cog);
vpImagePoint  circularTrajectory(vpImagePoint cog);
vpImagePoint  spiralTrajectory(vpImagePoint cog);

void openLoopInfinityExp(vpImagePoint cog, uInt8& coilActivation, vpImagePoint coilTip[], vpImagePoint& cmdPosition);
void openLoopDifferentDistancesExp(vpImagePoint cog, uInt8& coilActivation, vpImagePoint coilTip[], vpImagePoint& cmdPosition);
void openLoopDifferentVoltageExp(vpImagePoint cog, uInt8& coilActivation, vpImagePoint coilTip[], vpImagePoint& cmdPosition);
void openLoopDifferentCombinationsExp(vpImagePoint cog, uInt8& coilActivation, vpImagePoint coilTip[], vpImagePoint& cmdPosition);

void closedLoopPosititioningExp(vpImagePoint cog, uInt8& coilActivation, vpImagePoint coilTip[], vpImagePoint& cmdPosition);

void VisionServoing();

void startLogging();
void stopLogging();





#endif