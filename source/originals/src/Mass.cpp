/*
 * Mass.cpp
 *
 *  Created on: Oct 26, 2013
 *      Author: marium
 */

#include "Mass.h"

Mass::Mass() {

}

Mass::~Mass() {

}
RigidBodyandTransformPair Mass::getMassRigidBody(){
	return _rtPair;


}

btSphereShape* Mass::addMass(double x,double y,double z,double radius,bool fixed,bool activate,double massValue){
	//Adding sphere as mass
	btSphereShape* sphere = new btSphereShape(radius);
	//m_collisionShapes.push_back(sphere);

	btScalar mass(massValue);
	if(fixed)
		mass = 0.;

	bool isDynamic = (mass != 0.f);
	btVector3 localInertia(0,0,0);

	if(isDynamic)
		sphere->calculateLocalInertia(mass,localInertia);


	btTransform t;
	t.setIdentity();

	t.setOrigin(btVector3(btScalar(x),btScalar(y),btScalar(z)));
	t.getBasis().setEulerZYX(0,0,0); //Avoid rotation

	btDefaultMotionState* myMotionState = new btDefaultMotionState(t);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,sphere,localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);
	//btRigidBody* body = localCreateRigidBody(btScalar(mass),t,sphere);

	//_masses.push_back(std::make_pair(body,t));
	//_massValues.push_back(mass);
	//_massRestingPositions.push_back(t.getOrigin());

	//body->setRestitution(btScalar(1));//set as elastic
	if (activate)
		activate = DISABLE_DEACTIVATION;
		//activate = ACTIVE_TAG;

	body->setActivationState(activate);
	//body->setLinearVelocity(btVector3(0.,0.,0.));
	_rtPair.setRigidBodyandTransformPair(std::make_pair(body,t));

	return sphere;

	}
