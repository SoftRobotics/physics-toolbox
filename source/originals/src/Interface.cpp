/*
 * Interface.cpp
 *
 *  Created on: Oct 26, 2013
 *      Author: marium
 */
///scaling of the objects (0.1 = 20 centimeter boxes )
#define SCALING 1.

#include "Interface.h"

#include "GlutStuff.h"
#include "btBulletDynamicsCommon.h"
#include "GLDebugDrawer.h"
#include "Ground.h"

#include <stdio.h>
#include <iostream>
#include <stdlib.h>

static GLDebugDrawer gDebugDrawer;
static double timeStep = 0.0;

//for the normal distribution generation
typedef std::tr1::ranlux64_base_01 Engine;
Engine myEngine;


Interface::Interface() {

}

Interface::~Interface() {
	exitPhysics();
}

void Interface::initPhysics(){
	setTexturing(true);
	setShadows(true);
	setCameraDistance(btScalar(SCALING*50.));

	/////Initialize world

	//collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	//m_collisionConfiguration->setConvexConvexMultipointIterations();

	//use the default collision dispatcher. For parallel processing you can use a different dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
	m_broadphase = new btDbvtBroadphase();

	//the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
	m_solver = sol;

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
	m_dynamicsWorld->setDebugDrawer(&gDebugDrawer);

	std::cout<<"**************************************************************************"<<std::endl;
	std::cout<<"\t\t TOOLBOX FOR MORPHOLOGICAL COMPUTATION"<<std::endl;
	std::cout<<"**************************************************************************\n"<<std::endl;

	std::string groundAddition;
	std::cout<<"Do you want Ground in simulation(y/n)?"<<std::endl;
	std::cin>>groundAddition;

	Ground* ground = new Ground();
	if(groundAddition == "y"){
		m_collisionShapes.push_back(ground->addGround().first);
		m_dynamicsWorld->addRigidBody(ground->addGround().second);
	}

	// set gravity
	ground->setGravity(btVector3(0,-10,0));
	m_dynamicsWorld->setGravity(ground->getGravity());
	//

	int NumberofMasses;
	std::cout<< "Enter number of masses for the simulation"<<std::endl;
	std::cin>>NumberofMasses;

	int connectionType;
	if(NumberofMasses>1){
		while(true)
			{
				std::cout<< "Which type of connection do you want?\n"
						"Select the number:\n"
						"=====================================\n"
						"1.FeedForward Mass Spring Connection \n"
						"2. Recurrent Mass Spring Connection  \n"
						"3. Connection of your own \n"
						<<std::endl;
				std::cin>>connectionType;

				if(connectionType == 1)
					{
					if(NumberofMasses>0){
						setModeSelected(1);

						srand((unsigned)time(0)); // initialize random engine
						for (int i=0;i<NumberofMasses;i++){
							FeedForwardMassSpringModel* ffMSM = new FeedForwardMassSpringModel(NumberofMasses);

							std::pair < std::pair <btBoxShape*,btSphereShape*>, btGeneric6DofSpringConstraint*> objects = ffMSM->startModel(i);
							m_collisionShapes.push_back(objects.first.first);
							m_collisionShapes.push_back(objects.first.second);

							std::pair <RigidBodyandTransformPair,RigidBodyandTransformPair>  massFixedrigidBody = ffMSM->getMassAndFixedPoint();

							m_dynamicsWorld->addRigidBody(massFixedrigidBody.first.getRigidBodyandTransformPair().first);
							m_dynamicsWorld->addRigidBody(massFixedrigidBody.second.getRigidBodyandTransformPair().first);

							m_dynamicsWorld->addConstraint(objects.second,true);
							ffMSM->updateNetwork();
							}

						break;
					}}
					else if(connectionType == 2)
						{
						if(NumberofMasses>1){
							setModeSelected(2);

							myEngine.seed((unsigned int) time(NULL)); //initializing generator to January 1, 1970
							for(int i=0;i<NumberofMasses;i++){
								RNNMassSpringModel* rrnMSM = new RNNMassSpringModel(NumberofMasses);
							}

							break;
						}}
					else if (connectionType == 3)
						{if(NumberofMasses>1){
							setModeSelected(3);
							FreeConnectionMassSpringModel* fcMSM = new FreeConnectionMassSpringModel(NumberofMasses);
							break;
						}}
					else
						std::cout<<"Try Again!!"<<std::endl;
					}
	}
		else{
			//for one mass
			//FileReading();
			//ConsoleOutputforLengthSetting();
			//std::pair<btRigidBody*,btTransform> Latest = Initialization(getDistributionRange().first,getDistributionRange().second); //min and max of distribution
			//myfile<<"Mass Number 0"<<","<<","<<",";
			//m_dynamicsWorld->addRigidBody(Latest.first);
			//myfile<<std::endl;
		}

}










int Interface::getModeSelected() const {
	return _modeSelected;
}

void Interface::setModeSelected(int modeSelected) {
	_modeSelected = modeSelected;
}

void Interface::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	float dt = float(getDeltaTimeMicroseconds()) * 0.000001f;

	{
		static bool once = true;
		if ( m_dynamicsWorld->getDebugDrawer() && once)
		{
			m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawConstraints+btIDebugDraw::DBG_DrawConstraintLimits);
			once=false;
		}
	}

	{
		//during idle mode, just run 1 simulation step maximum
		int maxSimSubSteps = m_idle ? 1 : 1;
		if (m_idle)
			dt = 1.0f/1000.f;

		int numSimSteps = m_dynamicsWorld->stepSimulation(dt,maxSimSubSteps);

		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();

		bool verbose = false;
		if (verbose)
		{
			if (!numSimSteps)
				printf("Interpolated transforms\n");
			else
			{
				if (numSimSteps > maxSimSubSteps)
				{
					//detect dropping frames
					printf("Dropped (%i) simulation steps out of %i\n",numSimSteps - maxSimSubSteps,numSimSteps);
				} else
				{
					printf("Simulated (%i) steps\n",numSimSteps);
				}
			}
		}
	}


	renderme();
	glFlush();
	swapBuffers();
}

void Interface::clientResetScene()
{
	exitPhysics();
	initPhysics();
}

void Interface::displayCallback(void) {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	renderme();

	//optional but useful: debug drawing to detect problems
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glFlush();
	swapBuffers();
}

void Interface::exitPhysics(){
	//cleanup in the reverse order of creation/initialization
		for (int i=m_dynamicsWorld->getNumConstraints()-1; i>=0 ;i--)
			{
				btTypedConstraint* constraint = m_dynamicsWorld->getConstraint(i);
				m_dynamicsWorld->removeConstraint(constraint);
				delete constraint;
			}


		//remove the rigidbodies from the dynamics world and delete them
		int i;
		for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
		{
			btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
			btRigidBody* body = btRigidBody::upcast(obj);
			if (body && body->getMotionState())
			{
				delete body->getMotionState();
			}
			m_dynamicsWorld->removeCollisionObject( obj );
			delete obj;
		}

		//delete collision shapes
		for (int j=0;j<m_collisionShapes.size();j++)
		{
			btCollisionShape* shape = m_collisionShapes[j];
			delete shape;
		}

		m_collisionShapes.clear();

		delete m_dynamicsWorld;
		delete m_solver;
		delete m_broadphase;
		delete m_dispatcher;
		delete m_collisionConfiguration;

}
