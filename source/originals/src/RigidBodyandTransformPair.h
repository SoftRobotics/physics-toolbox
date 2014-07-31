/*
 * RigidBodyandTransformPair.h
 *
 *  Created on: Nov 9, 2013
 *      Author: marium
 */

#ifndef RIGIDBODYANDTRANSFORMPAIR_H_
#define RIGIDBODYANDTRANSFORMPAIR_H_

#include "GlutStuff.h"
#include "BulletDynamics/Dynamics/btDynamicsWorld.h"
#include "btBulletDynamicsCommon.h"
#include <stdlib.h>
#include <iostream>
#include <vector>

class RigidBodyandTransformPair {
public:
	RigidBodyandTransformPair();
	virtual ~RigidBodyandTransformPair();

	std::pair < btRigidBody*,btTransform> getRigidBodyandTransformPair();
	void setRigidBodyandTransformPair(std::pair < btRigidBody*,btTransform> rtPair);


private:
	btRigidBody* _rigidBody;
	btTransform _transform;

};

#endif /* RIGIDBODYANDTRANSFORMPAIR_H_ */
