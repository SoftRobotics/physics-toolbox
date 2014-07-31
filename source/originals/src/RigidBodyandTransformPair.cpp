/*
 * RigidBodyandTransformPair.cpp
 *
 *  Created on: Nov 9, 2013
 *      Author: marium
 */

#include "RigidBodyandTransformPair.h"


RigidBodyandTransformPair::RigidBodyandTransformPair() {
}

RigidBodyandTransformPair::~RigidBodyandTransformPair() {
}

std::pair < btRigidBody*,btTransform> RigidBodyandTransformPair::getRigidBodyandTransformPair(){
	return std::make_pair(_rigidBody,_transform);
}

void RigidBodyandTransformPair::setRigidBodyandTransformPair(std::pair < btRigidBody*,btTransform> rtPair){
	_rigidBody = rtPair.first;
	_transform = rtPair.second;
}
