/*
 *
 *  Created on: Oct 26, 2013
 *      Author: Marium Zeeshan
 */

#include "SliderConnection.h"

SliderConnection::SliderConnection() {
	this->_lowerAngularLimit = 0.0F;
	this->_upperAngularLimit = 0.0F;

}

SliderConnection::~SliderConnection() {
}

btScalar SliderConnection::getLowerAngularLimit() const {
	return _lowerAngularLimit;
}

void SliderConnection::setLowerAngularLimit(btScalar lowerAngularLimit) {
	_lowerAngularLimit = lowerAngularLimit;
}

btScalar SliderConnection::getUpperAngularLimit() const {
	return _upperAngularLimit;
}

void SliderConnection::setUpperAngularLimit(btScalar upperAngularLimit) {
	_upperAngularLimit = upperAngularLimit;
}

btSliderConstraint* SliderConnection::addSliderConnection(btRigidBody* body1,btTransform t1,btRigidBody* body2,btTransform t2){
	btSliderConstraint* pSlider = new btSliderConstraint(*body1,*body2,t1,t2,false);

	//Locking axes
	pSlider->setLowerAngLimit(getLowerAngularLimit());
	pSlider->setUpperAngLimit(getUpperAngularLimit());

	pSlider->setDbgDrawSize(btScalar(5.f));
	return pSlider;
}

