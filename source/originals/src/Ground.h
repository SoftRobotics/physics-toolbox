/*
 * Ground.h
 *
 *  Created on: Oct 26, 2013
 *      Author: Marium Zeeshan
 */

#ifndef GROUND_H_
#define GROUND_H_

#include "GlutStuff.h"
#include "btBulletDynamicsCommon.h"

#include <utility>

class Ground {
public:
	Ground();
	virtual ~Ground();

	const btVector3& getGroundDimensions() const;
	void setGroundDimensions(const btVector3& groundDimensions);

	const btVector3& getGroundOrigin() const;
	void setGroundOrigin(const btVector3& groundOrigin);

	//Set gravity of the ground
	const btVector3& getGravity() const;
	void setGravity(const btVector3& gravity);

	//Add ground into simulation
	std::pair <btBoxShape*,btRigidBody*> addGround();

private:
	btVector3 _groundDimensions,
			  _groundOrigin,
			  _gravity;

};

#endif /* GROUND_H_ */
