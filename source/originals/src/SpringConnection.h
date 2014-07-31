/*
* SpringConnection.h
 *
 *  Created on: Oct 26, 2013
 *      Author: Marium Zeeshan
 */

#ifndef SPRINGCONNECTION_H_
#define SPRINGCONNECTION_H_

#include "GlutStuff.h"
#include "btBulletDynamicsCommon.h"
#include <vector>

struct SpringConnectionAtEquilibrium
{
	btVector3 connectionFrom;
	btVector3 connectionTo;
	double stiffness;

	btScalar getNeutralLength(){
		btScalar length = btDistance(connectionFrom,connectionTo);
		return length;
	}
};

class SpringConnection {
public:
	SpringConnection();
	virtual ~SpringConnection();

	bool isCalledFromActiveSpring() const;
	void setCalledFromActiveSpring(bool calledFromActiveSpring);

	const btVector3& getLowerAngularLimit() const;
	void setLowerAngularLimit(const btVector3& lowerAngularLimit);

	const btVector3& getLowerLimitofSpring() const;
	void setLowerLimitofSpring(const btVector3& lowerLimitofSpring);

	const btVector3& getUpperAngularLimit() const;
	void setUpperAngularLimit(const btVector3& upperAngularLimit);

	const btVector3& getUpperLimitofSpring() const;
	void setUpperLimitofSpring(const btVector3& upperLimitofSpring);

	btScalar getSpringDamping() const;
	void setSpringDamping(btScalar springDamping);

	btScalar getSpringStiffness() const;
	void setSpringStiffness(btScalar springStiffness);

	btGeneric6DofSpringConstraint* addSpringConstraint(btRigidBody* body1,btTransform t1,btRigidBody* body2,btTransform t2);


private:
	//Spring features
	btVector3 _upperLimitofSpring,
			  _lowerLimitofSpring,
			  _lowerAngularLimit,
			  _upperAngularLimit;

	btScalar _springStiffness,
			 _springDamping;

	//For active spring
	bool _calledFromActiveSpring;

	//Connection Map for Spring connection
	std::vector<SpringConnectionAtEquilibrium> springAtEquilibrium; //Used for setting stiffness at the time of active spring
};

#endif /* SPRINGCONNECTION_H_ */
