/*
 * ConeTwistConnection.h
 *
 *  Created on: Oct 26, 2013
 *      Author: marium
 */

#ifndef CONETWISTCONNECTION_H_
#define CONETWISTCONNECTION_H_

#include "GlutStuff.h"
#include "btBulletDynamicsCommon.h"

class ConeTwistConnection {
public:
	ConeTwistConnection();
	virtual ~ConeTwistConnection();

	btConeTwistConstraint* addConeTwistConnection(btRigidBody* body1,btTransform t1,btRigidBody* body2,btTransform t2);
	btScalar getSoftness() const;
	void setSoftness(btScalar softness);

	btScalar getSwingSpan1() const;
	void setSwingSpan1(btScalar swingSpan1);

	btScalar getSwingSpan2() const;
	void setSwingSpan2(btScalar swingSpan2);

	btScalar getTwistSpan() const;
	void setTwistSpan(btScalar twistSpan);

private:
	btScalar _swingSpan1,
			 _swingSpan2,
			 _twistSpan,
			 _softness;

};

#endif /* CONETWISTCONNECTION_H_ */
