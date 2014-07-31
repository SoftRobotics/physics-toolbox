/*
 * RNNMassSpringModel.cpp
 *
 *  Created on: Oct 26, 2013
 *      Author: marium
 */

#include "RNNMassSpringModel.h"

std::vector<RigidBodyandTransformPair> RNNMassSpringModel::_masses;
std::vector<btVector3> RNNMassSpringModel::_massRestingPositions;
std::vector<double> RNNMassSpringModel::_massValues;

RNNMassSpringModel::RNNMassSpringModel(int numberOfMasses) {
	this->_numberOfMasses = numberOfMasses;

}

RNNMassSpringModel::~RNNMassSpringModel() {
}

double RNNMassSpringModel::getMass() const {
	return _mass;
}

void RNNMassSpringModel::setMass(double mass) {
	_mass = mass;
}

void RNNMassSpringModel::updateNetwork(){
	//Adding in vectors
	RNNMassSpringModel::_masses.push_back(mass->getMassRigidBody());
	RNNMassSpringModel::_massRestingPositions.push_back(btVector3(getLength(),getHeight(),0.));
	RNNMassSpringModel::_massValues.push_back(getMass());

}

std::pair < std::pair <btBoxShape*,btSphereShape*>, btGeneric6DofSpringConstraint*> startModel(int i){
//initialization

	for(int j=0;j<i;j++){
		if(RNNMassSpringModel::_masses.size()>1){



		}

	}
}
