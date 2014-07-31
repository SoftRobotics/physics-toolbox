/*
 * FeedForwardMassSpringModel.h
 *
 *  Created on: Oct 26, 2013
 *      Author: marium
 */

#ifndef FEEDFORWARDMASSSPRINGMODEL_H_
#define FEEDFORWARDMASSSPRINGMODEL_H_

#include <tr1/random>
#include <ctime>
#include <cmath>
#include <iostream>

#include "Mass.h"
#include "FixedPointsForFeedForwardConnection.h"
#include "SpringConnection.h"
#include "RigidBodyandTransformPair.h"

class FeedForwardMassSpringModel {
	Mass* mass  = new Mass();
	FixedPointsForFeedForwardConnection* fpFFM = new FixedPointsForFeedForwardConnection();
	SpringConnection* spring = new SpringConnection();


public:
	static std::vector<RigidBodyandTransformPair> _masses;
	static std::vector<btVector3> _massRestingPositions; //Rest points of Mass
	static std::vector<double> _massValues; //Mass values

	FeedForwardMassSpringModel(int numberOfMasses);
	virtual ~FeedForwardMassSpringModel();

	std::pair < std::pair <btBoxShape*,btSphereShape*>, btGeneric6DofSpringConstraint*> startModel(int i); //@ param i = iteration index of number of masses
	std::pair< RigidBodyandTransformPair,RigidBodyandTransformPair> getMassAndFixedPoint();

	double getLengthofSpring(float min = 1.0,float max = 10.0);
	double randomFloat(float a, float b);

	void updateNetwork();

	int getLength() const;
	void setLength(int length);

	double getHeight() const;
	void setHeight(double height);

	double getMass() const;
	void setMass(double mass);

private:
	int _numberOfMasses;
	int _length;
	double _height;
	double _mass;


};

#endif /* FEEDFORWARDMASSSPRINGMODEL_H_ */
