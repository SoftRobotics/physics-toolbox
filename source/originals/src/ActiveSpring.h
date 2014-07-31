/*
 * ActiveSpring.h
 *
 *  Created on: Oct 26, 2013
 *      Author: Marium Zeeshan
 */

#ifndef ACTIVESPRING_H_
#define ACTIVESPRING_H_

#include "GlutStuff.h"
#include "btBulletDynamicsCommon.h"
#include <vector>

struct ConnectionMap
{
	double connectionFrom;
	double connectionTo;
	int connectionType;
	btScalar currentlength;
};


class ActiveSpring {
public:
	ActiveSpring();
	virtual ~ActiveSpring();

private:
	double springStiffnessForActiveSpring;
	btVector3 restingLengthForActiveSpring;
	btVector3 currentLengthofActiveSpring;

	int codeForFeatureSelectedInActiveSpring;
	bool calledFromActiveSpring;

	std::pair<int,int> indexOfActiveSpringForStiffness;
	std::vector<std::pair <int,ConnectionMap> > mapOfActiveSpring; //feature selected for the active spring and the connection map between two masses
	std::vector<int> listoftimeStepforActiveSpring; //maintaining list of time-step in which active spring is active

};

#endif /* ACTIVESPRING_H_ */
