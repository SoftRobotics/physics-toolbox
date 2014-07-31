/*
 * FreeConnectionMassSpringModel.h
 *
 *  Created on: Oct 26, 2013
 *      Author: marium
 */

#ifndef FREECONNECTIONMASSSPRINGMODEL_H_
#define FREECONNECTIONMASSSPRINGMODEL_H_

#include <vector>
#include "GlutStuff.h"
#include "btBulletDynamicsCommon.h"

struct ConnectionMap
{
	double connectionFrom;
	double connectionTo;
	int connectionType;
	btScalar currentlength;
};


class FreeConnectionMassSpringModel {
public:
	FreeConnectionMassSpringModel(int numberOfMasses);
	virtual ~FreeConnectionMassSpringModel();

private:
	std::vector<ConnectionMap> massMappingForFreeConnection;

};

#endif /* FREECONNECTIONMASSSPRINGMODEL_H_ */
