/*
 * FeedForwardMassSpringModel.cpp
 *
 *  Created on: Oct 26, 2013
 *      Author: marium
 */

#include "FeedForwardMassSpringModel.h"

std::vector<RigidBodyandTransformPair> FeedForwardMassSpringModel::_masses;
std::vector<btVector3> FeedForwardMassSpringModel::_massRestingPositions;
std::vector<double> FeedForwardMassSpringModel::_massValues;

FeedForwardMassSpringModel::FeedForwardMassSpringModel(int numberOfMasses) {
	this->_numberOfMasses = numberOfMasses;
}

FeedForwardMassSpringModel::~FeedForwardMassSpringModel() {
}

int FeedForwardMassSpringModel::getLength() const {
	return _length;
}

void FeedForwardMassSpringModel::setLength(int length) {
	_length = length;
}

double FeedForwardMassSpringModel::getHeight() const {
	return _height;
}

double FeedForwardMassSpringModel::getMass() const {
	return _mass;
}

void FeedForwardMassSpringModel::setMass(double mass) {
	_mass = mass;
}

void FeedForwardMassSpringModel::setHeight(double height) {
	_height = height;
}

double FeedForwardMassSpringModel::randomFloat(float a, float b) {
    float random = ((float) rand()) / (float) RAND_MAX;
    float diff = b - a;
    float r = random * diff;
    return a + r;
}


double FeedForwardMassSpringModel::getLengthofSpring(float min,float max){
	return randomFloat(min , max);

}
std::pair< RigidBodyandTransformPair,RigidBodyandTransformPair> FeedForwardMassSpringModel::getMassAndFixedPoint(){
	RigidBodyandTransformPair massRT = mass->getMassRigidBody();
	RigidBodyandTransformPair jointRT = fpFFM->getJointRigidBody();
	return std::make_pair(massRT,jointRT);
}

void FeedForwardMassSpringModel::updateNetwork(){
	//Adding in vectors
	FeedForwardMassSpringModel::_masses.push_back(mass->getMassRigidBody());
	FeedForwardMassSpringModel::_massRestingPositions.push_back(btVector3(getLength(),getHeight(),0.));
	FeedForwardMassSpringModel::_massValues.push_back(getMass());

}

std::pair < std::pair <btBoxShape*,btSphereShape*>, btGeneric6DofSpringConstraint*> FeedForwardMassSpringModel::startModel(int i){
		// find length of spring through random function, distribution is from 1-10 as default
		double length = getLengthofSpring();
		setLength(length);

		//find height for the fixed points
		double height = fpFFM->getFixedJointPosition(i);
		setHeight(height);

		//mass values
		double massValue = 1.0;//TODO = from normal distribution
		setMass(massValue);


		//add mass
		btSphereShape* mass1 = mass->addMass(length,height,0.,2.0,false,true,massValue);
		//add fixed point
		btBoxShape* mass2 = fpFFM->addJoint(height);

		//add spring connection
		btGeneric6DofSpringConstraint* springConstraint = spring->addSpringConstraint(mass->getMassRigidBody().getRigidBodyandTransformPair().first,mass->getMassRigidBody().getRigidBodyandTransformPair().second,fpFFM->getJointRigidBody().getRigidBodyandTransformPair().first,fpFFM->getJointRigidBody().getRigidBodyandTransformPair().second);

		//Fixed all the axes except the driving axis
		spring->setLowerAngularLimit(btVector3(0,0,0));
		spring->setUpperAngularLimit(btVector3(0,0,0));
		spring->setLowerLimitofSpring(btVector3(0.,0.5,0.));
		spring->setUpperLimitofSpring(btVector3(0.,-0.5,0.));

		return std::make_pair(std::make_pair(mass2,mass1),springConstraint);
}


