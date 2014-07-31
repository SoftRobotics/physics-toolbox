/*
 * ConeTwistConnection.cpp
 *
 *  Created on: Oct 26, 2013
 *      Author: marium
 */

#include "ConeTwistConnection.h"

ConeTwistConnection::ConeTwistConnection() {
	this->_swingSpan1 = btScalar(SIMD_PI*0.5*0.6f);
	this->_swingSpan2 = btScalar(SIMD_PI*0.5f);
	this->_twistSpan = btScalar(SIMD_PI) * 0.8f;
	this->_softness = 1.0f;
}

ConeTwistConnection::~ConeTwistConnection() {
}

btConeTwistConstraint* ConeTwistConnection::addConeTwistConnection(btRigidBody* body1,btTransform t1,btRigidBody* body2,btTransform t2){
	btConeTwistConstraint* coneTwist = new btConeTwistConstraint(*body1, *body2,t1,t2);
	coneTwist->setLimit(getSwingSpan1(),getSwingSpan2(),getTwistSpan(),getSoftness());

	coneTwist->setDbgDrawSize(btScalar(5.f));
	return coneTwist;
}


btScalar ConeTwistConnection::getSoftness() const {
	return _softness;
}

void ConeTwistConnection::setSoftness(btScalar softness) {
	_softness = softness;
}

btScalar ConeTwistConnection::getSwingSpan1() const {
	return _swingSpan1;
}

void ConeTwistConnection::setSwingSpan1(btScalar swingSpan1) {
	_swingSpan1 = swingSpan1;
}

btScalar ConeTwistConnection::getSwingSpan2() const {
	return _swingSpan2;
}

void ConeTwistConnection::setSwingSpan2(btScalar swingSpan2) {
	_swingSpan2 = swingSpan2;
}

btScalar ConeTwistConnection::getTwistSpan() const {
	return _twistSpan;
}

void ConeTwistConnection::setTwistSpan(btScalar twistSpan) {
	_twistSpan = twistSpan;
}
