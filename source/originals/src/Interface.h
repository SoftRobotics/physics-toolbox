/*
 * Interface.h
 *
 *  Created on: Oct 26, 2013
 *      Author: marium
 */

#ifndef INTERFACE_H_
#define INTERFACE_H_

#ifdef _WINDOWS
#include "Win32DemoApplication.h"
#define PlatformDemoApplication Win32DemoApplication
#else
#include "GlutDemoApplication.h"
#define PlatformDemoApplication GlutDemoApplication
#endif

#include "BulletDynamics/Dynamics/btDynamicsWorld.h"
#include "GlutDemoApplication.h"
#include "LinearMath/btAlignedObjectArray.h"


#include "GlutStuff.h"
#include "btBulletDynamicsCommon.h"
#include "GLDebugDrawer.h"

#include <stdio.h>
#include <iostream>
//#include <stdlib.h>
#include "FeedForwardMassSpringModel.h"
#include "RNNMassSpringModel.h"
#include "FreeConnectionMassSpringModel.h"

#define _USE_MATH_DEFINES


class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;


class Interface : public PlatformDemoApplication {

	//Initialization of world
	btBroadphaseInterface*	m_broadphase;
	btCollisionDispatcher*	m_dispatcher;
	btConstraintSolver*	m_solver;
	btDefaultCollisionConfiguration* m_collisionConfiguration;

public:
	Interface();
	virtual ~Interface();

	void initPhysics(void);
	void exitPhysics(void);

	virtual void clientMoveAndDisplay(void);
	virtual void displayCallback(void);
	virtual void	clientResetScene(void);

	int getModeSelected() const;
	void setModeSelected(int modeSelected);

private:
	int _modeSelected;
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;
};

#endif /* INTERFACE_H_ */
