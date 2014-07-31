/*
 * PointToPointConnection.cpp
 *
 *  Created on: Oct 26, 2013
 *      Author: marium
 */

#include "PointToPointConnection.h"

PointToPointConnection::PointToPointConnection() {


}

PointToPointConnection::~PointToPointConnection() {

}

btTypedConstraint* PointToPointConnection::addPointToPointConnection(btRigidBody* body1,btTransform t1,btRigidBody* body2,btTransform t2){
	btVector3 pivotInA = body1->getCenterOfMassTransform().inverse().getOrigin(); //Finding center of body1
	btVector3 pivotInB = body2->getCenterOfMassTransform().inverse().getOrigin(); //Finding center of body2

	btTypedConstraint* p2p = new btPoint2PointConstraint(*body1,*body2,pivotInA,pivotInB);

	p2p->setDbgDrawSize(btScalar(5.f));
	return p2p;

}
