/*
 * SpringConnection.cpp
 *
 *  Created on: Oct 26, 2013
 *      Author: marium
 */

#include "SpringConnection.h"

SpringConnection::SpringConnection() {
	this->_upperLimitofSpring = btVector3(5., 0., 0.);
	this->_lowerLimitofSpring = btVector3(-5., 0., 0.);
	this->_lowerAngularLimit = btVector3(0.f, 0.f, -1.5f);
	this->_upperAngularLimit = btVector3(0.f, 0.f, 1.5f);
	this->_springStiffness = 39.478f;
	this->_springDamping = 0.5f;
	this->_calledFromActiveSpring = false;

}

SpringConnection::~SpringConnection() {
}

bool SpringConnection::isCalledFromActiveSpring() const {
	return _calledFromActiveSpring;
}

void SpringConnection::setCalledFromActiveSpring(bool calledFromActiveSpring) {
	_calledFromActiveSpring = calledFromActiveSpring;
}

const btVector3& SpringConnection::getLowerAngularLimit() const {
	return _lowerAngularLimit;
}

void SpringConnection::setLowerAngularLimit(
		const btVector3& lowerAngularLimit) {
	_lowerAngularLimit = lowerAngularLimit;
}

const btVector3& SpringConnection::getLowerLimitofSpring() const {
	return _lowerLimitofSpring;
}

void SpringConnection::setLowerLimitofSpring(
		const btVector3& lowerLimitofSpring) {
	_lowerLimitofSpring = lowerLimitofSpring;
}

btScalar SpringConnection::getSpringDamping() const {
	return _springDamping;
}

void SpringConnection::setSpringDamping(btScalar springDamping) {
	_springDamping = springDamping;
}

btScalar SpringConnection::getSpringStiffness() const {
	return _springStiffness;
}

void SpringConnection::setSpringStiffness(btScalar springStiffness) {
	_springStiffness = springStiffness;
}

const btVector3& SpringConnection::getUpperAngularLimit() const {
	return _upperAngularLimit;
}

void SpringConnection::setUpperAngularLimit(
		const btVector3& upperAngularLimit) {
	_upperAngularLimit = upperAngularLimit;
}

const btVector3& SpringConnection::getUpperLimitofSpring() const {
	return _upperLimitofSpring;
}

void SpringConnection::setUpperLimitofSpring(const btVector3& upperLimitofSpring) {
	_upperLimitofSpring = upperLimitofSpring;
}

//========================================================================================================================
/*
 * This is used to add Spring constraint
 * @param : body1 = one side of the spring
 * @param : body2 = other side of the spring
 * @param : t1 = transform of body1
 * @param : t2 = transform of body2
 */
//========================================================================================================================
btGeneric6DofSpringConstraint* SpringConnection::addSpringConstraint(btRigidBody* body1,btTransform t1,btRigidBody* body2,btTransform t2){

	btGeneric6DofSpringConstraint* pGen6DOFSpring = new btGeneric6DofSpringConstraint(*body1, *body2, t1, t2, true);

	pGen6DOFSpring->setLinearUpperLimit(getUpperLimitofSpring());
	pGen6DOFSpring->setLinearLowerLimit(getLowerLimitofSpring());

	pGen6DOFSpring->setAngularLowerLimit(getLowerAngularLimit());
	pGen6DOFSpring->setAngularUpperLimit(getUpperAngularLimit());

	//m_dynamicsWorld->addConstraint(pGen6DOFSpring, true);

	pGen6DOFSpring->setDbgDrawSize(btScalar(5.f));

	pGen6DOFSpring->enableSpring(0, true);
	pGen6DOFSpring->setStiffness(0, getSpringStiffness());
	pGen6DOFSpring->setDamping(0, getSpringDamping());

	pGen6DOFSpring->enableSpring(5, true);
	pGen6DOFSpring->setStiffness(5, getSpringStiffness());
	pGen6DOFSpring->setDamping(0, getSpringDamping());
	pGen6DOFSpring->setEquilibriumPoint();

	return pGen6DOFSpring;


}
