/*
 * PointToPointConnection.h
 *
 *  Created on: Oct 26, 2013
 *      Author: marium
 */

#ifndef POINTTOPOINTCONNECTION_H_
#define POINTTOPOINTCONNECTION_H_

#include "GlutStuff.h"
#include "btBulletDynamicsCommon.h"

class PointToPointConnection {
public:
	PointToPointConnection();
	virtual ~PointToPointConnection();

	btTypedConstraint* addPointToPointConnection(btRigidBody* body1,btTransform t1,btRigidBody* body2,btTransform t2);
private:

};

#endif /* POINTTOPOINTCONNECTION_H_ */
