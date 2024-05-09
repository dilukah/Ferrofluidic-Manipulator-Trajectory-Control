/*
model.cpp - Ferrofludic manipulator linear programming model
Date: 2021-07-07
Author: P. A. Diluka Harischandra
*/


#include <ilcplex/ilocplex.h>
#include <stdio.h>
#include <iostream>
#include "Vision.h"
#include <NIDAQmx.h>

#include "Controller.h"
using namespace std;


bool incrementVal = 0;
bool decrementVal = 0;
int editingVariable = 0;

//initialize objective function weights
double alpha = 0.4145;
double beta = 0.2685;
double gamma = 0.0001;
double delta = 0.5;
double changingValue = 0.005;
double scalingFactorPower = 0;
double mm2pix = 46.5;

bool PrintToConsole = 0; 

/***********************************************************
	Linear Programming model
***********************************************************/
uInt8 lpModel(vpImagePoint particlePos, vpImagePoint target, vpImagePoint coilTip[])
{
	uInt8 activationCoils = 0b00000000;
	const int numberOfCoils = 8;
	double scalingFactor;
	double MPx = 0.0;
	double MPy = 0.0;
	double MP_norm = 0.0;
	double PT_norm_mm = 0.0;
	double PT_norm = 0.0;
	double PTO_norm = 0.0;
	double indicatorCoeficient = 20.0;

	scalingFactor = pow(10.0, scalingFactorPower);

	double Vx[numberOfCoils] = {};
	double Vy[numberOfCoils] = {};


	vpColVector PT(2); // Particle Target Vector (PT)
	vpColVector PTO(2); //PT orthogonal
	vpColVector PT_unitVec(2);
	vpColVector PTO_unitVec(2);
	vpColVector MP(2);
	vpColVector MF(2);
	vpColVector FO(2);

	PT[0] = target.get_u() - particlePos.get_u();
	PT[1] = target.get_v() - particlePos.get_v();
	PTO[0] = -PT[1];
	PTO[1] = PT[0];
	PT_norm = PT.euclideanNorm();
	PTO_norm = PTO.euclideanNorm();

	PT_norm_mm = PT_norm / mm2pix;

	PT_unitVec = PT / PT_norm;
	PTO_unitVec = PTO / PTO_norm;

	for (int i = 0; i < numberOfCoils; i++)
	{

		MP[0] = particlePos.get_u() - coilTip[i].get_u();
		MP[1] = particlePos.get_v() - coilTip[i].get_v();
		MP_norm = MP.euclideanNorm();
		
		MF = scalingFactor * (2.4675 * pow(MP_norm / mm2pix, -0.8652)) * (MP / (MP_norm));
		
		Vx[i] = vpColVector::dotProd(MF, PT_unitVec);
		Vy[i] = vpColVector::dotProd(MF, PTO_unitVec);
		Vy[i] = abs(Vy[i]); 
		
		if (PrintToConsole)
		{
			cout << "Vx " << i << ": " << Vx[i] << endl;
			cout << "Vy " << i << ": " << Vy[i] << endl;
		}
	}
	
	IloEnv env;
	try {
		IloModel model(env);
		IloExpr exprVx(env);
		IloExpr exprVy(env);
		IloExpr exprPT(env);

		IloExpr consExpr(env);
		IloNumVarArray weight(env);

		env = model.getEnv();

		for (int i = 0; i < numberOfCoils; i++)
		{
			weight.add(IloNumVar(env, 0, 1, ILOINT));
		}

		//Projection
		for (int i = 0; i < numberOfCoils; i++)
		{
			exprVx += Vx[i] * weight[i];
		}
		//Orthogonal Projection
		for (int i = 0; i < numberOfCoils; i++)
		{
			exprVy += Vy[i] * weight[i];
		}

		for (int i = 0; i < numberOfCoils; i++)
		{
			exprPT += 1 / (PT_norm_mm * PT_norm_mm) * weight[i];
		}
		
		model.add(IloMaximize(env, alpha * exprVx - beta * exprVy - gamma * exprPT));
							
		IloCplex cplex(model);
		if (!PrintToConsole)
		{
			cplex.setOut(env.getNullStream()); //Disables output to console.
		}
		cplex.solve();
		// Output solutions
		if (PrintToConsole)
		{
			env.out() << "Solution status = " << cplex.getStatus() << endl;
			env.out() << "Solution value = " << cplex.getObjValue() << endl;
		}

		IloNumArray vals(env);
		cplex.getValues(vals, weight);
		if (PrintToConsole)
		{
			env.out() << "Values = " << vals << endl;
		}
		for (int i = 0; i < numberOfCoils; i++)
		{
			if ((int)vals.operator[]((IloInt)i))
			{
				activationCoils |= (1 << i);
			}
		}
	}
	catch (IloException e) {
		cerr << "Concert exception caught: " << e << endl;
	}
	catch (...) {
		cerr << "Unknown exception caught." << endl;
	}
	
	env.end();

	return activationCoils;
}

void lpkeyboardInput()
{

if ((GetKeyState('P') & 0x8000)) // Detect if a key was pressed
{
	while (GetKeyState('P') & 0x8000);//wait for unpress
	cout << "Alpha " << alpha << "Beta " << beta << "Gamma " << gamma << "Delta " << delta << "Scaling Factor Power" << scalingFactorPower << "mm2pix " << mm2pix << "Changing Value " << changingValue << endl;
}

if ((GetKeyState('I') & 0x8000)) // Detect if a key was pressed
{
	if (editingVariable == 0)
	{
		alpha = alpha + changingValue;
		cout << "Alpha " << alpha << endl;
	}
	if (editingVariable == 1)
	{
		beta = beta + changingValue;
		cout << "Beta " << beta << endl;
	}
	if (editingVariable == 2)
	{
		gamma = gamma + changingValue;

		cout << "Gamma " << gamma << endl;
	}
	if (editingVariable == 3)
	{
		delta = delta + changingValue;

		cout << "Delta " << delta << endl;
	}
	if (editingVariable == 4)
	{
		changingValue = changingValue * 10;

		cout << "changingValue " << changingValue << endl;
	}


}
if ((GetKeyState('D') & 0x8000)) // Detect if a key was pressed
{
	if (editingVariable == 0)
	{
		alpha = alpha - changingValue;
		cout << "Alpha " << alpha << endl;
	}
	if (editingVariable == 1)
	{
		beta = beta - changingValue;

		cout << "Beta " << beta << endl;
	}
	if (editingVariable == 2)
	{
		gamma = gamma - changingValue;

		cout << "Gamma " << gamma << endl;
	}
	if (editingVariable == 3)
	{
		delta = delta - changingValue;

		cout << "Delta " << delta << endl;
	}
	if (editingVariable == 4)
	{
		changingValue = changingValue /10;
		cout << "Changing Value " << changingValue << endl;
	}
}

//Change variable
if ((GetKeyState('C') & 0x8000))
{
	while (GetKeyState('C') & 0x8000);
	editingVariable++;
	if (editingVariable == 5)
		editingVariable = 0;
	if (editingVariable == 0) cout << "Varying Alpha" << endl;
	if (editingVariable == 1) cout << "Varying Beta" << endl;
	if (editingVariable == 2) cout << "Varying Gamma" << endl;
	if (editingVariable == 3) cout << "Varying Delta" << endl;
	if (editingVariable == 4) cout << "Varying Changing Value" << endl;
}

}