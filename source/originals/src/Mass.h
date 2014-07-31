/*
 * Mass.h
 *
 *  Created on: Oct 26, 2013
 *      Author: marium
 */

#ifndef MASS_H_
#define MASS_H_

#include<vector>
#include "GlutStuff.h"
#include "btBulletDynamicsCommon.h"
#include "RigidBodyandTransformPair.h"

class Mass {
public:
	Mass();
	virtual ~Mass();

	btSphereShape* addMass(double x,double y,double z,double radius,bool fixed,bool activate,double massValue);
	RigidBodyandTransformPair getMassRigidBody();



private:
	RigidBodyandTransformPair _rtPair;
};

#endif /* MASS_H_ */
