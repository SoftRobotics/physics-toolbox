/*
 * RNNMassSpringModel.h
 *
 *  Created on: Oct 26, 2013
 *      Author: marium
 */

#ifndef RNNMASSSPRINGMODEL_H_
#define RNNMASSSPRINGMODEL_H_

#include <ctime>
#include <cmath>
#include <iostream>

#include "Mass.h"
#include "FixedPointsForFeedForwardConnection.h"
#include "SpringConnection.h"
#include "RigidBodyandTransformPair.h"
#include "InputInitialization.h"

class RNNMassSpringModel {
	Mass* mass  = new Mass();
	SpringConnection* spring = new SpringConnection();

public:
	static std::vector<RigidBodyandTransformPair> _masses;
	static std::vector<btVector3> _massRestingPositions; //Rest points of Mass
	static std::vector<double> _massValues; //Mass values

	RNNMassSpringModel(int numberOfMasses);
	virtual ~RNNMassSpringModel();

	std::pair < std::pair <btBoxShape*,btSphereShape*>, btGeneric6DofSpringConstraint*> startModel(int i);
	void updateNetwork();

	double getMass() const;
	void setMass(double mass);

private:
	int _numberOfMasses;
	double _mass;
};

#endif /* RNNMASSSPRINGMODEL_H_ */
