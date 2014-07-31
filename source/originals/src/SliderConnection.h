/*
 * SliderConnection.h
 *
 *  Created on: Oct 26, 2013
 *      Author: marium
 */

#ifndef SLIDERCONNECTION_H_
#define SLIDERCONNECTION_H_

#include "GlutStuff.h"
#include "btBulletDynamicsCommon.h"

class SliderConnection {
public:
	SliderConnection();
	virtual ~SliderConnection();


	btScalar getLowerAngularLimit() const;
	void setLowerAngularLimit(btScalar lowerAngularLimit);


	btScalar getUpperAngularLimit() const;
	void setUpperAngularLimit(btScalar upperAngularLimit);

	btSliderConstraint* addSliderConnection(btRigidBody* body1,btTransform t1,btRigidBody* body2,btTransform t2);

private:

	btScalar _lowerAngularLimit,
			 _upperAngularLimit;
};

#endif /* SLIDERCONNECTION_H_ */
