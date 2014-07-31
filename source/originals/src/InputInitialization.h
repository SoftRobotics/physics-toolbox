/*
 * InputInitialization.h
 *
 *  Created on: Oct 26, 2013
 *      Author: Marium Zeeshan
 */

#ifndef INPUTINITIALIZATION_H_
#define INPUTINITIALIZATION_H_

#include "Ground.h"
#include <math.h>
#include <tr1/random>
#include <ctime>
#include <random>
#include <iostream>

struct posMass
{
	double mass;
	double x;
	double y;
	double z;
	double radius;
};


class InputInitialization {
	//for the normal distribution generation
	typedef std::tr1::ranlux64_base_01 Engine;
	//typedef std::tr1::mt19937 Engine;
	typedef std::tr1::normal_distribution<double> Distribution;

public:
	Ground* ground = new Ground();

	InputInitialization();
	//Overloaded Constructor
	InputInitialization(double min,double max);

	virtual ~InputInitialization();

	void initialize();
	void reset();
	void assignValues();

	double getDistMax() const;
	void setDistMax(double distMax);

	double getDistMin() const;
	void setDistMin(double distMin);

private:
	double _distMin, //min value for distribution of mass and radius
		   _distMax; //max value for the distribution of mass and radius


	Distribution myDist;

	Distribution CoX;
	Distribution CoY;
	Distribution CoZ;

	Engine myEngine;

	posMass _posMass;
};

#endif /* INPUTINITIALIZATION_H_ */
