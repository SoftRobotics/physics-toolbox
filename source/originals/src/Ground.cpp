/*
 * Ground.cpp
 *
 *  Created on: Oct 26, 2013
 *      Author: Marium Zeeshan
 */

#include "Ground.h"

Ground::Ground() {
	_groundDimensions = btVector3(200.,20.,80.); 	//default 200 X 20 X 80 initialization
	_groundOrigin = btVector3(0.,-40.,0.);	//default 0 X -50 X 0 initialization
	_gravity = btVector3(0.,-10.,0.); //default gravity is set for 10m/s in downward direction

}

Ground::~Ground() {
}

const btVector3& Ground::getGroundDimensions() const {
	return _groundDimensions;
}

void Ground::setGroundDimensions(const btVector3& groundDimensions) {
	_groundDimensions = groundDimensions;
}

const btVector3& Ground::getGroundOrigin() const {
	return _groundOrigin;
}

void Ground::setGroundOrigin(const btVector3& groundOrigin) {
	_groundOrigin = groundOrigin;
}

const btVector3& Ground::getGravity() const {
	return _gravity;
}

void Ground::setGravity(const btVector3& gravity) {
	_gravity = gravity;
}

std::pair <btBoxShape*,btRigidBody*> Ground::addGround(){
	btBoxShape* groundShape = new btBoxShape(getGroundDimensions());
	groundShape->initializePolyhedralFeatures();
	//	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);

	//m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(getGroundOrigin());

	btScalar mass(0.);

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0,0,0);
	if (isDynamic)
		groundShape->calculateLocalInertia(mass,localInertia);


	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

	btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);

	body->setGravity(btVector3(0,-9.81,0));
	body->setFriction(btScalar(0.7)); // coefficient of friction = 0.7 (Almost equivalent to Aluminium)
	body->setRestitution(btScalar(0.)); //non-elastic, coefficient of restitution = 0
	body->setDamping(btScalar(0.9),btScalar(0.9)); //linear and angular damping

	//TODO
	//place body on the ground always...play with ERP values
	//http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=9&t=5393&view=next
	return std::make_pair(groundShape,body);

}
