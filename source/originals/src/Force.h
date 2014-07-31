/*
 * Force.h
 *
 *  Created on: Oct 26, 2013
 *      Author: marium
 */

#ifndef FORCE_H_
#define FORCE_H_

#include <vector>

class Force {
public:
	Force();
	virtual ~Force();

private:
	std::vector<int> listoftimeStepforApplyingForceonMass; //maintaining list of time-step on which force is applied on masses
	std::vector<int> listofMassesOntoWhichForceIsApplied; //holds list of masses on which force is applied

};

#endif /* FORCE_H_ */
