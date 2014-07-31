/*
 * FixedPointsForFeedForwardConnection.h
 *
 *  Created on: Oct 26, 2013
 *      Author: marium
 */

#ifndef FIXEDPOINTSFORFEEDFORWARDCONNECTION_H_
#define FIXEDPOINTSFORFEEDFORWARDCONNECTION_H_

#define WINDOWS_DIMENSION 30

#include "RigidBodyandTransformPair.h"
#include "GlutStuff.h"
#include "BulletDynamics/Dynamics/btDynamicsWorld.h"
#include "btBulletDynamicsCommon.h"
#include <vector>


class FixedPointsForFeedForwardConnection {
public:
	FixedPointsForFeedForwardConnection();
	//FixedPointsForFeedForwardConnection(double fixedJointPosition);
	virtual ~FixedPointsForFeedForwardConnection();

	void addJointPosition(btVector3 addJoint);

	btBoxShape* addJoint(double fixedJointPosition);
	RigidBodyandTransformPair getJointRigidBody();

	double getFixedJointPosition(int iter);
	void setFixedJointPosition(double fixedJointPosition);


private:
	double _fixedJointPosition;
	std::vector<btVector3> fixedJointPoints; //Position of fixed points in feed-forward network

	RigidBodyandTransformPair _rtPair;
};

#endif /* FIXEDPOINTSFORFEEDFORWARDCONNECTION_H_ */
