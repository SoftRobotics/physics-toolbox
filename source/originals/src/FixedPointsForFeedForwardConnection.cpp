/*
 * FixedPointsForFeedForwardConnection.cpp
 *
 *  Created on: Oct 26, 2013
 *      Author: marium
 */

#include "FixedPointsForFeedForwardConnection.h"


FixedPointsForFeedForwardConnection::FixedPointsForFeedForwardConnection() {
	//this->_fixedJointPosition = fixedJointPosition;

}

FixedPointsForFeedForwardConnection::~FixedPointsForFeedForwardConnection() {
}


void FixedPointsForFeedForwardConnection::addJointPosition(btVector3 addJoint){
	fixedJointPoints.push_back(addJoint);
}

double FixedPointsForFeedForwardConnection::getFixedJointPosition(int iter){
	_fixedJointPosition = WINDOWS_DIMENSION - (5 * iter);
	return _fixedJointPosition;
}

void FixedPointsForFeedForwardConnection::setFixedJointPosition(double fixedJointPosition) {
	_fixedJointPosition = fixedJointPosition;
}

RigidBodyandTransformPair FixedPointsForFeedForwardConnection::getJointRigidBody(){
	return _rtPair;

}
/*
 *
 */
btBoxShape* FixedPointsForFeedForwardConnection::addJoint(double fixedJointPosition){

	btBoxShape* box = new btBoxShape(btVector3(btScalar(0.5),btScalar(0.5),btScalar(0.5)));
	btVector3 inertia(0,0,0);

	btScalar mass(0.);

	//Adding a box on the screen
	//m_collisionShapes.push_back(box);

	btTransform t;
	t.setIdentity();
	t.setOrigin(btVector3(10.,btScalar(fixedJointPosition),0.));
	t.getBasis().setEulerZYX(0,0,0);

	addJointPosition(btVector3(10.,btScalar(fixedJointPosition),0.));

	btDefaultMotionState* myMotionState = new btDefaultMotionState(t);
	btRigidBody::btRigidBodyConstructionInfo info(mass,myMotionState,box,inertia); //motion state would actually be non-null in most real usages
	info.m_restitution = 1.3f;
	info.m_friction = 1.5f;
	btRigidBody* body = new btRigidBody(info);

	//btRigidBody* body = localCreateRigidBody(mass,t,box);
	body->setActivationState(DISABLE_DEACTIVATION);

	_rtPair.setRigidBodyandTransformPair(std::make_pair(body,t));

	return box;
}
