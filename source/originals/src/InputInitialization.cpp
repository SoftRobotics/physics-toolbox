/*
 * InputInitialization.cpp
 *
 *  Created on: Oct 26, 2013
 *      Author: marium
 */
#include "InputInitialization.h"


//for the normal distribution generation
typedef std::tr1::ranlux64_base_01 Engine;
//typedef std::tr1::mt19937 Engine;
typedef std::tr1::normal_distribution<double> Distribution;


InputInitialization::InputInitialization() {
	this->_distMin = 1.0;
	this->_distMax = 5.0;

}

InputInitialization::InputInitialization(double min = 1.0,double max = 5.0){
	this->_distMin = min;
	this->_distMax = max;
}

InputInitialization::~InputInitialization() {
}

double InputInitialization::getDistMax() const {
	return _distMax;
}

void InputInitialization::setDistMax(double distMax) {
	_distMax = distMax;
}

double InputInitialization::getDistMin() const {
	return _distMin;
}

void InputInitialization::setDistMin(double distMin) {
	_distMin = distMin;
}

void InputInitialization::initialize(){
	myDist(btScalar(getDistMin()),btScalar(getDistMax()));

	//To place them on plane
	//Taking it half because box is placed on the origin symmetrically
	CoX(ground->getGroundOrigin().getX()/2.0,ground->getGroundDimensions().getX()/2.0);
	CoY(ground->getGroundOrigin().getY()/2.0,ground->getGroundDimensions().getY()/2.0);
	CoZ(ground->getGroundOrigin().getZ()/2.0,ground->getGroundDimensions().getZ()/2.0);
}

void InputInitialization::reset(){
	myDist.reset(); //cleans cache values
	CoX.reset();
	CoY.reset();
	CoZ.reset();
}

void InputInitialization::assignValues(){
	_posMass.mass = fabs(random_normal(myEngine,myDist));

}
