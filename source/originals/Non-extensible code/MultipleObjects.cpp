/*
 * MultipleObjects.cpp
 *
 *  Created on: Aug 5, 2013
 *      Author: Marium Zeeshan
 */

///create 125 (5x5x5) dynamic object
/*
#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_Z 5
*/

//maximum number of objects (and allow user to shoot additional boxes)
//#define MAX_PROXIES (ARRAY_SIZE_X*ARRAY_SIZE_Y*ARRAY_SIZE_Z + 1024)

//========================================================================================================================
///scaling of the objects (0.1 = 20 centimeter boxes )
#define SCALING 1.
#define START_POS_X -5
#define START_POS_Y -5
#define START_POS_Z -3

#define WindowsDimensions 30

#include "MultipleObjects.h"
#include "GlutStuff.h"

#include "btBulletDynamicsCommon.h"

#include <stdio.h>
#include "GLDebugDrawer.h"

#include <iostream>
//#include <stdlib.h>

static GLDebugDrawer gDebugDrawer;

static double timeStep = 0.0; //do it inside the constructor

//for the normal distribution generation
typedef std::tr1::ranlux64_base_01 Engine;
//typedef std::tr1::mt19937 Engine;
typedef std::tr1::normal_distribution<double> Distribution;

Engine myEngine;



//========================================================================================================================
/*
 * For displaying anything on console
 */
//========================================================================================================================
void MultipleObjects::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//simple dynamics world doesn't handle fixed-time-stepping
	float dt = float(getDeltaTimeMicroseconds()) * 0.000001f;

	//Information of the simulation
		std::cout<<"===================================================="<<std::endl;
		std::cout<<"INFORMATION OF SIMULATION"<<std::endl;
		std::cout<<"===================================================="<<std::endl;
		std::cout<<" Time Step "<< ++timeStep <<std::endl;
		std::cout<<" Gravity  ("<< m_dynamicsWorld->getGravity().getX()<<","<<m_dynamicsWorld->getGravity().getY()<<","<<m_dynamicsWorld->getGravity().getZ()<<")"<<std::endl;

		//std::vector<Triplet> vector1,vector2;

		//For the ACTIVE SPRING
		if(listoftimeStepforActiveSpring.size()>=1){

			for(int i=0;i<listoftimeStepforActiveSpring.size();i++){
				//std::cout<<"TIME"<<listoftimeStepforActiveSpring.size();
				if(timeStep == listoftimeStepforActiveSpring[i] ){
					UpdateFeaturesofAllActiveSpring();
					}
				if(listoftimeStepforActiveSpring[i]>timeStep){
					break;
					}
				}
			}
		/////////////////////////*******************************

		//for disabling objects
		/*
		if(m_dynamicsWorld->getGravity() == btVector3(0,-0,0)){
			std::cout<<"Disble all objects"<<std::endl;
			for(int m = 0;m<masses.size();m++){
				masses[m].first->setGravity(btVector3(0,-0,0));
				//masses[m].first->setActivationState(0);
				//m_collisionShapes[m]->setMargin(0.2);
				//stop objects
				//masses[m].first->setLinearVelocity(btVector3(0,0,0));
				//masses[m].first->setAngularFactor(btVector3(0,0,0));
			}
		}
		*/

		/////////////////////////////////////////////////////////////

		//For applying FORCES
		if(listoftimeStepforApplyingForceonMass.size()>=1){
			for(int f=0;f<listoftimeStepforApplyingForceonMass.size();f++){

				if(timeStep == listoftimeStepforApplyingForceonMass[f]){
					//Apply force on the listed masses
					// For now: Linear forces are applied
					for(int m = 0;m<listofMassesOntoWhichForceIsApplied.size();m++){
						ApplyLinearForce(listofMassesOntoWhichForceIsApplied[m],timeStep);

					}

				}
				if(listoftimeStepforApplyingForceonMass[f]>timeStep){
					break;
				}
			}
		}

		btTransform trans;
		for(int i=0;i<masses.size();i++){
			//Disable linear velocity at certain timestep
					/*if(timeStep == 100){
						//Disabling Velocities
						std::cout<<"Diabling Velocities"<<std::endl;
						masses[i].first->setLinearVelocity(btVector3(0,0,0));
						masses[i].first->setAngularFactor(btVector3(0,0,0));

					}*/

			//masses[i].first->getMotionState()->getWorldTransform(masses[i].second);
			masses[i].first->getMotionState()->getWorldTransform(trans);
			std::cout<<"Mass "<<i<<"Forces"<<masses[i].first->getTotalForce().getX()<<" "<<masses[i].first->getTotalForce().getY()<<" "<<masses[i].first->getTotalForce().getZ()<<std::endl;
			std::cout<<"Mass "<<i<<"Torque"<<masses[i].first->getTotalTorque().getX()<<" "<<masses[i].first->getTotalTorque().getY()<<" "<<masses[i].first->getTotalTorque().getZ()<<std::endl;

			if(masses[i].first->isActive()){
				std::cout<< "Mass" << i <<std::endl;
				std::cout<< "("
					<<trans.getOrigin().getX()
					<<","
					<< trans.getOrigin().getY()
					<< ","
					<< trans.getOrigin().getZ()
					<< ")"
					<< std::endl;

				double lengthX = masses[i].second.getOrigin().getX();
				double lengthY = masses[i].second.getOrigin().getY();
				double lengthZ = masses[i].second.getOrigin().getZ();

				if (getMode() == 1) //FeedForward Network
				{	btScalar ChangeofLength = btDistance(btVector3(lengthX,lengthY,lengthZ),massRestingLength[i]);

					//std::cout<<"original length @@"<<springEquilibriumPointandStiffness[i].first.getX()<<","<<springEquilibriumPointandStiffness[i].first.getY()<<","<<springEquilibriumPointandStiffness[i].first.getZ()<<std::endl;
					std::cout << "Change of Length of Spring of Mass " << i <<" = "<< ChangeofLength << std::endl;

					btScalar LengthofSpring = btDistance(btVector3(lengthX,lengthY,lengthZ),fixedPoints[i]);
					std::cout<<"Current Length of spring of Mass "<< i << " = " <<LengthofSpring<<std::endl;
					std::cout<<"Angular velocity of mass"<< i<<" = "<< " ("<<masses[i].first->getAngularVelocity().getX() << " , "<<masses[i].first->getAngularVelocity().getY()<<" , "<<masses[i].first->getAngularVelocity().getZ()<<" ) rad/s"<<std::endl;
					std::cout<<"Speed of mass "<<i<<" = "<<masses[i].first->getLinearVelocity().length()<<std::endl;

				}

				if(getMode() == 2){ // For Recurrent Network
					std::cout<<"Angular velocity of mass"<< i<<" = "<< " ("<<masses[i].first->getAngularVelocity().getX() << " , "<<masses[i].first->getAngularVelocity().getY()<<" , "<<masses[i].first->getAngularVelocity().getZ()<<" ) rad/s"<<std::endl;
					for(int j=0;j<i;j++){
						//Finding the length of spring each of the connection
						double lengthX_new = masses[j].second.getOrigin().getX();
						double lengthY_new = masses[j].second.getOrigin().getY();
						double lengthZ_new = masses[j].second.getOrigin().getZ();

						// I think change of length can be possible but it wont give any useful info..

						btScalar LengthofSpring = btDistance(btVector3(lengthX,lengthY,lengthZ),btVector3(lengthX_new,lengthY_new,lengthZ_new));
						std::cout<<"Current Length of spring between Mass "<< j << " and " << i <<" = " << LengthofSpring<<std::endl;

					}
				}

				if(getMode() == 3){ //Connection of any-type (of your choice)
					std::cout<<"Angular velocity of mass"<< i<<" = "<< " ("<<masses[i].first->getAngularVelocity().getX() << " , "<<masses[i].first->getAngularVelocity().getY()<<" , "<<masses[i].first->getAngularVelocity().getZ()<<" ) rad/s"<<std::endl;
					if(massMappingForFreeConnection.size()>=1){

						//sort the vector
						/*std::sort(massMappingForFreeConnection.begin(), massMappingForFreeConnection.end(),
						          boost::bind(&ConnectionMap::second, _1) <
						          boost::bind(&ConnectionMap::second, _2));
						*/

						for(int j=0;j<massMappingForFreeConnection.size();j++){
							if(i == massMappingForFreeConnection[j].connectionFrom){
								//calculate length
								double lengthX_new = masses[massMappingForFreeConnection[j].connectionTo].second.getOrigin().getX();
								double lengthY_new = masses[massMappingForFreeConnection[j].connectionTo].second.getOrigin().getY();
								double lengthZ_new = masses[massMappingForFreeConnection[j].connectionTo].second.getOrigin().getZ();

								btScalar LengthofSpring = btDistance(btVector3(lengthX,lengthY,lengthZ),btVector3(lengthX_new,lengthY_new,lengthZ_new));



								//Update Length of Spring
								massMappingForFreeConnection[j] = {massMappingForFreeConnection[j].connectionFrom,massMappingForFreeConnection[j].connectionTo,getTypeofconnection(),LengthofSpring};


								std::cout<<"Current Length of spring between Mass "<< massMappingForFreeConnection[j].connectionFrom << " and " << massMappingForFreeConnection[j].connectionTo <<" = " << massMappingForFreeConnection[j].currentlength<<std::endl;

							}
						}


					}
			}

				// Writing to a file
				//myfile<<masses[i].second.getOrigin().getX()<<","<<masses[i].second.getOrigin().getY()<<","<<masses[i].second.getOrigin().getZ()<<",";
				myfile<<trans.getOrigin().getX()<<","<<trans.getOrigin().getY()<<","<<trans.getOrigin().getZ()<<",";
				//vector1.push_back(Triplet());
				//vector1.push_back({masses[i].second.getOrigin().getX(),masses[i].second.getOrigin().getY(),masses[i].second.getOrigin().getZ()});
				}
			else{ //for non-active masses

				//vector1.push_back(Triplet());
				//vector1.push_back({0,0,0});
				myfile<<"NaN,NaN,NaN,";
			}
			//vector2.insert( vector2.end(),vector1.begin(),vector1.end() );
			//vector1.clear();
			//std::cout<<"@@"<< vector2.front().one_<<vector2.front().two_<<vector2.front().three_<<std::endl;

			//writing in CSV
			//if(myfile.is_open()){
				//myfile<<masses[i].second.getOrigin().getX()<<","<<masses[i].second.getOrigin().getY()<<","<<masses[i].second.getOrigin().getZ()<<",";
			//}

		}
		myfile<<std::endl;

		//optional but useful: debug drawing
		//m_dynamicsWorld->debugDrawWorld();
		std::cout<<"===================================================="<<std::endl;

{
		static bool once = true;
		if ( m_dynamicsWorld->getDebugDrawer() && once)
		{
		 m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawConstraints+btIDebugDraw::DBG_DrawConstraintLimits);
		 once=false;
		}
}

// Step simulation
{
	 	//during idle mode, just run 1 simulation step maximum
		int maxSimSubSteps = m_idle ? 1 : 1;
		if (m_idle)
			dt = 1.0f/420.f;

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
//========================================================================================================================

//========================================================================================================================
void MultipleObjects::setGravity(double y){
	m_dynamicsWorld->setGravity(btVector3(0,-y,0));

}
//========================================================================================================================
//========================================================================================================================
void MultipleObjects::getResults(std::vector<std::pair<btRigidBody*,btTransform> > _massInformation){
	/*for (int i=0 ; i<300 ; i++) {
		m_dynamicsWorld->stepSimulation(1/60.f,10);

	    _massInformation->getMotionState()->getWorldTransform(trans);

	                std::cout << "sphere height: " << trans.getOrigin().getY() << std::endl;
	        }

*/

}
//========================================================================================================================
/*
 * This is used to set Simulation time
 */
//========================================================================================================================
void MultipleObjects::setSimulationTime(double _time){
	_timeStep = _time;
	timeStep = _timeStep;

}
//========================================================================================================================
/*
 * This is used to get Simulation time
 */
//========================================================================================================================
double MultipleObjects::getSimulationTime(void){
	_timeStep = timeStep;
	return _timeStep;
}
//========================================================================================================================
/*
 * For displaying anything on graphical window
 */
//========================================================================================================================
void MultipleObjects::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	renderme();

	//optional but useful: debug drawing to detect problems
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glFlush();
	swapBuffers();
}
//========================================================================================================================
/*
 * This is used to get origin of Ground
 */
//========================================================================================================================
Triplet MultipleObjects::getGroundDimensions(){
	return _groundDimensions;

}
//========================================================================================================================
/*
 * This is used to set origin of Ground
 */
//========================================================================================================================
Triplet MultipleObjects::getGroundOrigin(){
	return _groundOrigin;

}
//========================================================================================================================
/*
 * This is used to get the range of normal distribution
 */
//========================================================================================================================
std::pair<double,double> MultipleObjects::getDistributionRange(){
	return std::make_pair(distMin_,distMax_);
}
//========================================================================================================================
/*
 * This is used to set the range of normal distribution
 */
//========================================================================================================================
void MultipleObjects::setDistributionRange(double min_,double max_){
	distMin_ = min_;
	distMax_ = max_;

}
//========================================================================================================================
/*
 * Helping function used by initialization
 */
//========================================================================================================================
double random_normal(Engine &eng, Distribution dist) {
    return dist(eng);
}
//========================================================================================================================
/*
 * This function is used to initialize masses and their positions on the basis of gaussian normal distribution
 * @param : min = min value of the distribution
 * @param : max = max value of the distribution
 */
//========================================================================================================================
std::pair<btRigidBody*,btTransform> MultipleObjects::Initialization(double min,double max){

/*
	Engine myEngine;
	myEngine.seed((unsigned int) time(NULL)); //initializing generator to January 1, 1970
*/
	Distribution myDist(min,max);

	//To place them on plane
	//Taking it half because box is placed on the origin symmetrically
	Distribution CoX(getGroundOrigin().x_/2.0,getGroundDimensions().x_/2.0);
	Distribution CoY(getGroundOrigin().y_/2.0,getGroundDimensions().y_/2.0);
	Distribution CoZ(getGroundOrigin().z_/2.0,getGroundDimensions().z_/2.0);

	double mass,x,y,z,radius;
	std::string fixed,activate,connection;


	//std::cout<<getLengthofConstraint().x_<<","<<getLengthofConstraint().y_<<","<<getLengthofConstraint().z_<<")"<<std::endl;
	myDist.reset(); //cleans cache values
	CoX.reset();
	CoY.reset();
	CoZ.reset();

	// For true false random assignment
	//TODO add t as well for static
	//static std::string charset = "f";

	//mass = fabs(random_normal(myEngine,myDist));
	mass = 2.0;
	//btScalar mass2 = 2.0;

	//radius = fabs(random_normal(myEngine,myDist));
	radius = 0.3;

	//insert into mass values
	massValues.push_back(mass);

	//for specifying position coordinates
	if (getMode()==2)
	{
		x = random_normal(myEngine,CoX);
		y = random_normal(myEngine,CoY);
		z = random_normal(myEngine,CoZ);
		setLengthofConstraint({x,y,z});

	}

	//fixed = charset[rand() % charset.length()];
	//activate = charset[rand() % charset.length()];

	bool fixedMass,activateMass;
	//std::cout<<"Mass is fixed? true or false!!"<<std::endl;
	//std::cin>>fixed;

	fixed = std::string(readFileString, 0,1);
	//std::cout<<"fixed"<<std::string(readFileString, 0,1)<<std::endl;

			{
			if(fixed == "t")
				fixedMass = true;
			else
				fixedMass = false;

			/*
				activateMass = true;
			else
				activateMass = false;
				*/
			}
			activateMass = true;

			std::pair<btRigidBody*,btTransform> newMass;


			if(getMode() == 1) //For Feedforward to make them 1D
				{
				setLengthofConstraint({lengthofSpringforfeedforward.x_,lengthofSpringforfeedforward.y_,lengthofSpringforfeedforward.z_});
				newMass = addMass(getLengthofConstraint().x_,getLengthofConstraint().y_,getLengthofConstraint().z_,radius,fixedMass,activateMass,mass);

				}
			else
			{
				newMass = addMass(getLengthofConstraint().x_,getLengthofConstraint().y_,getLengthofConstraint().z_,radius,fixedMass,activateMass,mass);
			}

			//set equilibrium point of constraint
			//springEquilibriumPointandStiffness.push_back(std::make_pair(btVector3(getLengthofConstraint().x_,getLengthofConstraint().y_,getLengthofConstraint().z_),getSpringStiffness()));

			massRestingLength.push_back(btVector3(getLengthofConstraint().x_,getLengthofConstraint().y_,getLengthofConstraint().z_));


	//std::cout << "a random value == " << fabs(random_normal(myEngine,myDist)) << std::endl;
	//std::cout << myDist(myEngine) << std::endl;

	std::cout<< "Mass No = "<< masses.size()-1 << ": \t Mass = " << mass <<"\t Position = ("<<getLengthofConstraint().x_<<","<<getLengthofConstraint().y_<<","<<getLengthofConstraint().z_<<")"<<std::endl;
	return newMass;

}

//========================================================================================================================
/*
 * This function is used to set ground origin
 * @param : x = x coordinate
 * @param : y = y coordinate
 * @param : z = z coordinate
 */
//========================================================================================================================
void MultipleObjects::setGroundOrigin(double _x,double _y,double _z){
	_groundOrigin.x_ = _x;
	_groundOrigin.y_ = _y;
	_groundOrigin.z_ = _z;

}

//========================================================================================================================
/*
 * This function is used to get ground origin
 * @param : x = x coordinate
 * @param : y = y coordinate
 * @param : z = z coordinate
 */
//========================================================================================================================
void MultipleObjects::setGroundDimensions(double _x,double _y,double _z){
	_groundDimensions.x_ = _x;
	_groundDimensions.y_ = _y;
	_groundDimensions.z_ = _z;

}
//========================================================================================================================
/*
 * This function is used to add ground into the simulation
 */
//========================================================================================================================
void MultipleObjects::addGround(){
	btBoxShape* groundShape = new btBoxShape(btVector3(btScalar(_groundDimensions.x_),btScalar(_groundDimensions.y_),btScalar(_groundDimensions.z_)));
	groundShape->initializePolyhedralFeatures();
	//	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);

	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(btScalar(_groundOrigin.x_),btScalar(_groundOrigin.y_),btScalar(_groundOrigin.z_)));


	{
		btScalar mass(0.);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass,localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		body->setGravity(btVector3(0,-9.81,0));
		body->setFriction(btScalar(0.7)); // coefficient of friction = 0.7 (Almost equivalent to Aluminium)
		body->setRestitution(btScalar(0.)); //non-elastic, coefficient of restitution = 0
		body->setDamping(btScalar(0.9),btScalar(0.9)); //linear and angular damping

		//TODO
		//place body on the ground always...play with ERP values
		//http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=9&t=5393&view=next

		//add the body to the dynamics world
		m_dynamicsWorld->addRigidBody(body);

	}

}
//========================================================================================================================
/*
 * This function is REDUNDANT
 */
//========================================================================================================================
void	MultipleObjects::initPhysics()
{


	///create a few basic rigid bodies
	btBoxShape* groundShape = new btBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.)));
	groundShape->initializePolyhedralFeatures();
//	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);

	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-50,0));


	{
		btScalar mass(0.);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass,localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		//add the body to the dynamics world
		m_dynamicsWorld->addRigidBody(body);
	}

	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			//SPHERES
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		btSphereShape* sphere = new btSphereShape(4);
		//btCollisionShape* colShape = new btSphereShape(btScalar(1.));
		m_collisionShapes.push_back(sphere);

		/// Create Dynamic Objects
		//btTransform startTransform;
		//startTransform.setIdentity();

		btScalar	mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(10,10,0);
		if (isDynamic)
			sphere->calculateLocalInertia(mass,localInertia);

/*
		float start_x = START_POS_X - ARRAY_SIZE_X/2;
		float start_y = START_POS_Y;
		float start_z = START_POS_Z - ARRAY_SIZE_Z/2;

		for (int k=0;k<ARRAY_SIZE_Y;k++)
		{
			for (int i=0;i<ARRAY_SIZE_X;i++)
			{
				for(int j = 0;j<ARRAY_SIZE_Z;j++)
				{
					startTransform.setOrigin(SCALING*btVector3(
										btScalar(2.0*i + start_x),
										btScalar(20+2.0*k + start_y),
										btScalar(2.0*j + start_z)));
*/

		btTransform t;
		t.setIdentity(); //no rotation at all
		//t.getBasis().setEulerZYX(0,0,0);
		t.setOrigin(btVector3(0,-3,0));


		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

		btDefaultMotionState* myMotionState = new btDefaultMotionState(t);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,sphere,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		body->setRestitution(btScalar(1));
		m_dynamicsWorld->addRigidBody(body);


		/////////////////////////////////////////////////
		////TEST
		////////////////////////////////////////////////
		// Add another sphere
		/*
		btSphereShape* SphereShape = new btSphereShape(3);

		btTransform capsuleTransform;
		capsuleTransform.setIdentity();
		capsuleTransform.setOrigin(btVector3(0,5,6));

		btRigidBody *capsuleBody = localCreateRigidBody(btScalar(1),capsuleTransform,SphereShape);
		btTransform fixedpoint;
		fixedpoint.setIdentity();
		btGeneric6DofConstraint *joint0 = new btGeneric6DofConstraint( *capsuleBody, fixedpoint, false );

		m_dynamicsWorld->addConstraint( joint0, true );
		//m_dynamicsWorld->addRigidBody(SphereShape);

*/



		//Add another Sphere


		btSphereShape* sphere2 = new btSphereShape(1);
		//btCollisionShape* colShape = new btSphereShape(btScalar(1.));
		m_collisionShapes.push_back(sphere2);

		btScalar	mass2(0.f);

		bool isDynamictwo = (mass2 != 0.f);

		btVector3 localInertia2(0,0,0);
		if (isDynamictwo)
			sphere->calculateLocalInertia(mass2,localInertia2);

		btTransform t2;
		t2.setIdentity(); //no rotation at all
		//t2.getBasis().setEulerZYX(0,0,0);
		t2.setOrigin(btVector3(10,5,10));


		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

		btDefaultMotionState* myMotionState2 = new btDefaultMotionState(t2);
		btRigidBody::btRigidBodyConstructionInfo rbInfotwo(mass2,myMotionState2,sphere2,localInertia);
		btRigidBody* bodytwo = new btRigidBody(rbInfotwo);
		m_dynamicsWorld->addRigidBody(bodytwo);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			//CONSTRAINT
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		/*btVector3 sliderWorldPos(0,10,0);
        btVector3 sliderAxis(1,0,0);

        btScalar angle=0.f;//SIMD_RADS_PER_DEG * 10.f;
        btMatrix3x3 sliderOrientation(btQuaternion(sliderAxis ,angle));
        t.setOrigin(sliderWorldPos);
        //trans.setBasis(sliderOrientation);
        btTransform sliderTransform = t;
*/
		btGeneric6DofSpringConstraint* pGen6DOFSpring = new btGeneric6DofSpringConstraint(*bodytwo, *body, t2, t,true);
		pGen6DOFSpring->setLinearUpperLimit(btVector3(5., 0., 0.));
		pGen6DOFSpring->setLinearLowerLimit(btVector3(-5., 0., 0.));

		pGen6DOFSpring->setAngularLowerLimit(btVector3(0.f, 0.f, -1.5f));
		pGen6DOFSpring->setAngularUpperLimit(btVector3(0.f, 0.f, 1.5f));


		pGen6DOFSpring->setDbgDrawSize(btScalar(5.f));

		pGen6DOFSpring->enableSpring(0, true);
		pGen6DOFSpring->setStiffness(0, 39.478f);
		pGen6DOFSpring->setDamping(0, 0.5f);
		pGen6DOFSpring->enableSpring(5, true);
		pGen6DOFSpring->setStiffness(5, 39.478f);
		pGen6DOFSpring->setDamping(0, 0.3f);
		pGen6DOFSpring->setEquilibriumPoint();

		m_dynamicsWorld->addConstraint(pGen6DOFSpring, true);

			//	}
		//	}
	//}

	}


}
//========================================================================================================================
/*
 * This function is used to add masses
 * @param : x = position of x coordinate
 * @param : y = position of y coordinate
 * @param : z = position of z coordinate
 * @param : radius = radius of mass
 * @param : fixed = mass is fixed or not
 * @param : activate = mass is activate or not into the simulation
 */
//========================================================================================================================
std::pair <btRigidBody*,btTransform> MultipleObjects::addMass(double x,double y,double z,double radius,bool fixed,bool activate,double massValue){
	//Adding sphere as mass
	btSphereShape* sphere = new btSphereShape(radius);
	m_collisionShapes.push_back(sphere);

	btScalar mass(massValue);

	if(fixed)
		mass = 0.;


	bool isDynamic = (mass != 0.f);
	//btVector3 localInertia(0,0,0);

	/*
	if(isDynamic)
		sphere->calculateLocalInertia(mass,localInertia);
	*/

	btTransform t;
	t.setIdentity();

	t.setOrigin(btVector3(btScalar(x),btScalar(y),btScalar(z)));
	t.getBasis().setEulerZYX(0,0,0); //Avoid rotation

	//btDefaultMotionState* myMotionState = new btDefaultMotionState(t);
	//btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,sphere,localInertia);
	//btRigidBody* body = new btRigidBody(rbInfo);
	btRigidBody* body = localCreateRigidBody(btScalar(mass),t,sphere);
	masses.push_back(std::make_pair(body,t));

	//body->setRestitution(btScalar(1));//set as elastic
	if (activate)
		activate = DISABLE_DEACTIVATION;
		//activate = ACTIVE_TAG;

	body->setActivationState(activate);
	//body->setLinearVelocity(btVector3(0.,0.,0.));



	return std::make_pair(body,t);

}

//========================================================================================================================
/*
 * This is used to add Slider constraint
 * @param : body1 = one side of the spring
 * @param : body2 = other side of the spring
 * @param : t1 = transform of body1
 * @param : t2 = transform of body2
 */
//========================================================================================================================
void MultipleObjects::addSliderConstraint(btRigidBody* body1,btTransform t1,btRigidBody* body2,btTransform t2){
	btSliderConstraint* pSlider = new btSliderConstraint(*body1,*body2,t1,t2,false);

	//Locking axes
	pSlider->setLowerAngLimit(0.0F);
	pSlider->setUpperAngLimit(0.0F);

	pSlider->setDbgDrawSize(btScalar(5.f));
	m_dynamicsWorld->addConstraint(pSlider, true);


}
//========================================================================================================================
/*
 * This is used to add Point to Point constraint a.k.a Ball-socket
 * @param : body1 = one side of the spring
 * @param : body2 = other side of the spring
 * @param : t1 = transform of body1
 * @param : t2 = transform of body2
 */
//========================================================================================================================
void MultipleObjects::addConeTwistConstraint(btRigidBody* body1,btTransform t1,btRigidBody* body2,btTransform t2){
	btConeTwistConstraint* coneTwist = new btConeTwistConstraint(*body1, *body2,t1,t2);
	coneTwist->setLimit(btScalar(SIMD_PI*0.5*0.6f), btScalar(SIMD_PI*0.5f), btScalar(SIMD_PI) * 0.8f,1.0f);

	coneTwist->setDbgDrawSize(btScalar(5.f));
	m_dynamicsWorld->addConstraint(coneTwist, true);
}
//========================================================================================================================
/*
 * This is used to add Point to Point constraint a.k.a Ball-socket
 * @param : body1 = one side of the spring
 * @param : body2 = other side of the spring
 * @param : t1 = transform of body1
 * @param : t2 = transform of body2
 */
//========================================================================================================================
void MultipleObjects::addPointToPointConstraint(btRigidBody* body1,btTransform t1,btRigidBody* body2,btTransform t2){
	btVector3 pivotInA = body1->getCenterOfMassTransform().inverse().getOrigin(); //Finding center of body1
	btVector3 pivotInB = body2->getCenterOfMassTransform().inverse().getOrigin(); //Finding center of body2

	btTypedConstraint* p2p = new btPoint2PointConstraint(*body1,*body2,pivotInA,pivotInB);

	p2p->setDbgDrawSize(btScalar(5.f));
	m_dynamicsWorld->addConstraint(p2p);

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
void MultipleObjects::addSpringConstraint(btRigidBody* body1,btTransform t1,btRigidBody* body2,btTransform t2){
	btGeneric6DofSpringConstraint* pGen6DOFSpring = new btGeneric6DofSpringConstraint(*body1, *body2, t1, t2,true);

	pGen6DOFSpring->setLinearUpperLimit(upperLimitofSpring);
	pGen6DOFSpring->setLinearLowerLimit(lowerLimitofSpring);

	pGen6DOFSpring->setAngularLowerLimit(lowerAngularLimit);
	pGen6DOFSpring->setAngularUpperLimit(upperAngularLimit);

	m_dynamicsWorld->addConstraint(pGen6DOFSpring, true);

	pGen6DOFSpring->setDbgDrawSize(btScalar(5.f));

	//Setting different values of Spring coefficients on different points of the Spring
	pGen6DOFSpring->enableSpring(0, true);
	pGen6DOFSpring->setStiffness(0, getSpringStiffness());
	pGen6DOFSpring->setDamping(0, getSpringDamping());
	pGen6DOFSpring->enableSpring(5, true);
	pGen6DOFSpring->setStiffness(5, getSpringStiffness());
	pGen6DOFSpring->setDamping(0, getSpringDamping());
	//pGen6DOFSpring->setLimit(0,0,0);
	pGen6DOFSpring->setEquilibriumPoint();
	//pGen6DOFSpring->isEnabled();

	//to get the position of the spring
	//btVector3 currentLinearDiff = pGen6DOFSpring->getTranslationalLimitMotor()->m_currentLinearDiff;

	/* Setting ERP and CFM
	 * >>INFO: index 0-2 are for linear constraints, 3-5 for angular constraints
	 *If ERP=0 then no correcting force is applied and the bodies will eventually drift apart as the simulation proceeds.
	 *If ERP=1 then the simulation will attempt to fix all joint error during the next time step.
	 *
	 *If CFM is set to zero, the constraint will be hard.
	 *If CFM is set to a positive value, it will be possible to violate the constraint by "pushing on it"

	 * >>SYNTAX: constraint->setParam(BT_CONSTRAINT_STOP_CFM, myCFMvalue, index)
	 */
	//Insert into Spring Map

	//pGen6DOFSpring->setParam(BT_CONSTRAINT_STOP_CFM, 0, 1);
	//pGen6DOFSpring->setParam(BT_CONSTRAINT_STOP_ERP, 0.3,1);

	if(isCalledFromActiveSpring() && springAtEquilibrium.size()>=1){
			//search for the connection as it is called by Active Spring
			//and update stiffness as well as Resting Length.
			for(int act=0;act<springAtEquilibrium.size();act++){

				if((massRestingLength[getIndexOfActiveSpringForStiffness().first] == springAtEquilibrium[act].connectionFrom && massRestingLength[getIndexOfActiveSpringForStiffness().second] == springAtEquilibrium[act].connectionTo)
					||
					(massRestingLength[getIndexOfActiveSpringForStiffness().second] == springAtEquilibrium[act].connectionFrom && massRestingLength[getIndexOfActiveSpringForStiffness().first] == springAtEquilibrium[act].connectionTo))
				{
					//unset the function variable and just update stiffness at this position
					setCalledFromActiveSpring(false);
					springAtEquilibrium[act].stiffness = getSpringStiffness();
				}
			}
		}

	else // add new instance for new spring
	{
		springAtEquilibrium.push_back({t1.getOrigin(),t2.getOrigin(),getSpringStiffness()});

	}

	//m_dynamicsWorld->addConstraint(pGen6DOFSpring, true);

}

//========================================================================================================================
// Set the upper limit of the linear spring
//========================================================================================================================
void MultipleObjects::setUpperLimitofSpring(btVector3 _uplimspring){
	upperLimitofSpring = _uplimspring;
}
//========================================================================================================================
// Set the lower limit of the linear spring
//========================================================================================================================
void MultipleObjects::setLowerAngularLimitofSpring(btVector3 _lowlimspring){
	lowerLimitofSpring = _lowlimspring;

}
//========================================================================================================================
// Set the upper angular limit of the linear spring
//========================================================================================================================
void MultipleObjects::setUpperAngularLimitofSpring(btVector3 _uplimspring){
	upperAngularLimit = _uplimspring;
}
//========================================================================================================================
// Set the lower angular limit of the linear spring
//========================================================================================================================
void MultipleObjects::setLowerLimitofSpring(btVector3 _lowlimspring){
	lowerAngularLimit = _lowlimspring;
}
//========================================================================================================================
/*
 * This is the main interface from where all the processing starts
 */
//========================================================================================================================
void MultipleObjects::Interface(){
	setTexturing(true);
	setShadows(true);

	setCameraDistance(btScalar(SCALING*50.));

	///collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	//m_collisionConfiguration->setConvexConvexMultipointIterations();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

	m_broadphase = new btDbvtBroadphase();

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
	m_solver = sol;

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);

	m_dynamicsWorld->setDebugDrawer(&gDebugDrawer);


	//m_dynamicsWorld->setGravity(btVector3(0,-9.81,0));

	//Opening file
	myfile.open("/home/marium/Desktop/Experiments/Experiment 2/MassPositions_Experiment2_WithoutGravity.csv");

	//Open file to read
	readFile.open("/home/marium/Desktop/Experiments/Experiment 2/Experiment2.csv");
	connectionMatrixFile.open("/home/marium/Desktop/Experiments/Experiment 2/Experiment2_ConnectionMapping.csv");

	std::string groundAddition;

	std::cout<<"Do you want Ground in simulation(t/f)?"<<std::endl;
	std::cin >> groundAddition;

	if(groundAddition == "t")
		addGround();


	int NumberofMasses;
	std::cout<< "Enter number of masses"<<std::endl;
	std::cin>>NumberofMasses;

	// Setting the flag for String random selection!!
	srand(time(NULL));
	myEngine.seed((unsigned int) time(NULL)); //initializing generator to January 1, 1970

	int connectionType;

	//m_dynamicsWorld->setDebugDrawer(&gDebugDrawer);

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
						setMode(1);
						FeedForwardMassSpringConnection(NumberofMasses);
						break;

					}}
				else if(connectionType == 2)
					{
					if(NumberofMasses>1){
						setMode(2);
						RNNConnection(NumberofMasses);
						break;
					}}
				else if (connectionType == 3)
					{if(NumberofMasses>1){
						setMode(3);
						FreeMassSpringConnection(NumberofMasses);
						break;
					}}
				else
					std::cout<<"Try Again!!"<<std::endl;
				}

/*
	double mass,x,y,z;
	std::string fixed,activate,connection;

	for(int i = 0;i<NumberofMasses;i++){

		/*std::cout<<"Enter mass value of mass "<<i<<std::endl;
		std::cin>>mass;
		//std::cout<<<<masses.size()<<std::endl;
		std::cout<<"Enter x coordinate value of mass "<<i<<std::endl;
		std::cin>>x;
		std::cout<<"Enter y coordinate value of mass "<<i<<std::endl;
		std::cin>>y;
		std::cout<<"Enter z coordinate value of mass "<<i<<std::endl;
		std::cin>>z;
		std::cout<<"Mass is fixed? true or false!!"<<std::endl;
		std::cin>>fixed;
		std::cout<<"Do you want to activate this mass in the simulation? t or f"<<std::endl;
		std::cin>>activate;

		bool fixedMass,activateMass,connectionSpring;

		{
		if(fixed == "t")
			fixedMass = true;
		else
			fixedMass = false;

		if(activate == "t")
			activateMass = true;
		else
			activateMass = false;
		}

		std::pair<btRigidBody*,btTransform> Latest = addMass(x,y,z,mass,fixedMass,activateMass);
		*/
/*		std::pair<btRigidBody*,btTransform> Latest = Initialization(getDistributionRange().first,getDistributionRange().second); //min and max of distribution

		bool connectionSpring;

		myfile<<"Mass Number "<<i<<","<<","<<",";

		m_dynamicsWorld->addRigidBody(Latest.first);

		int connectionType;

		if(masses.size()>1){
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
					FeedForwardConnection(NumberofMasses);

					break;
					}
				else if(connectionType == 2)
					{
					RNNConnection();
					break;
					}
				else if (connectionType == 3)
					{
					FreeConnection(i,Latest);
					break;
					}
				else
					std::cout<<"Try Again!!"<<std::endl;
				}
*/
/*			std::cout<<"Do you want to connect mass "<<i<<" to any other previous mass(true/false)"<<std::endl;
			std::cin>>connection;

			if(connection == "t")
				connectionSpring = true;
			else
				connectionSpring = false;

			if(connectionSpring){
				addConnection(Latest);
			}
			*/
//		}
	}
	else{
		//for one mass
		FileReading();
		ConsoleOutputforLengthSetting();
		std::pair<btRigidBody*,btTransform> Latest = Initialization(getDistributionRange().first,getDistributionRange().second); //min and max of distribution
		myfile<<"Mass Number 0"<<","<<","<<",";
		m_dynamicsWorld->addRigidBody(Latest.first);
		myfile<<std::endl;
	}
	//myfile<<std::endl;



}

//========================================================================================================================
/*
 * This function is used to provide RNN connection among masses
 */
//========================================================================================================================
void MultipleObjects::RNNConnection(int NumberofMasses){

	for(int i=0;i<NumberofMasses;i++){
		std::pair<btRigidBody*,btTransform> NewMass = Initialization(getDistributionRange().first,getDistributionRange().second); //min and max of distribution
		myfile<<"Mass Number "<<i<<","<<","<<",";
		//m_dynamicsWorld->addRigidBody(NewMass.first);

		for(int j=0;j<i;j++){
			if(masses.size()>1){
				//Do connection for all masses that were before

				addSpringConstraint(NewMass.first,NewMass.second,masses[j].first,masses[j].second);
				}
			}
		}

	myfile<<std::endl;

	// Now apply force
	ApplyForce();
}
//========================================================================================================================
/*
 * This function is used to read connection map
 */
//========================================================================================================================
void MultipleObjects::ConnectionMapFileReading(void){
	//std::string token;
	connectionMapFileTuple.clear();

	if(connectionMatrixFile.good() && connectionMatrixFile.is_open()){
		//readline
		getline(connectionMatrixFile,connectionMapFileString);
		if (connectionMapFileString.size()>0)
		{
			std::cout<<"Connection Map output"<<connectionMapFileString<<std::endl;
			//parse into tokens
			strtk::parse(connectionMapFileString,",",connectionMapFileTuple);
			std::cout<<"2222* "<<connectionMapFileTuple.size()<<std::endl;
		}
		else
		{
			//EOF reached; close file
			if(connectionMatrixFile.eof())
			{
				connectionMatrixFile.close();
			}
			//std::cout<<"End of file reached.(One Possibility: Your entries are less than the values found in file.)"<<std::endl;
			//exit (EXIT_SUCCESS);

		}
	}
	else{
		std::cout<<"No such file found"<<std::endl;
		exit (EXIT_FAILURE);
	}

}
//========================================================================================================================
/*
 * This function is used to read experiment file
 */
//========================================================================================================================
void MultipleObjects::FileReading(void){
	std::string token;
	fileReadTuple.clear();

	if(readFile.good() && readFile.is_open()){
		//readline
		getline(readFile,readFileString);
		if (readFileString.size()>0)
		{
			std::cout<<"File output"<<readFileString<<std::endl;
			//parse into tokens
			strtk::parse(readFileString,",",token,fileReadTuple);
		}
		else
		{
			if(readFile.eof())
			{
				std::cout<<"End of file reached.(One Possibility: Your entries of masses are less than the values found in file.)"<<std::endl;
				exit (EXIT_SUCCESS);
			}


		}
	}
	else
	{
		std::cout<<"No such file found"<<std::endl;
		exit (EXIT_FAILURE);
	}

}
//========================================================================================================================
/*
 * This function is used to display and set length of constraints
 */
//========================================================================================================================

void MultipleObjects::ConsoleOutputforLengthSetting(){
	double conX,conY,conZ;

	//std::cin >> conX;
	//std::cin >> conY;
	//std::cin >> conZ;

	//std::cout<<"@@"<<readFileString.substr(2, readFileString.find(",")).c_str()<<std::endl;
	conX = fileReadTuple[0];
	conY = fileReadTuple[1];
	conZ = fileReadTuple[2];

	setLengthofConstraint({conX,conY,conZ});
	//std::cout<<getLengthofConstraint().x_<<","<<getLengthofConstraint().y_<<","<<getLengthofConstraint().z_<<")"<<std::endl;

}
//========================================================================================================================
/*
 * This function is used to do free connection inside mass spring system
 */
//========================================================================================================================
void MultipleObjects::FreeMassSpringConnection(int NumberofMasses){
		for(int i = 0;i<NumberofMasses;i++){
			FileReading();
			ConsoleOutputforLengthSetting();

			std::pair<btRigidBody*,btTransform> Latest = Initialization(getDistributionRange().first,getDistributionRange().second); //min and max of distribution

			myfile<<"Mass Number "<<i<<","<<","<<",";
			//m_dynamicsWorld->addRigidBody(Latest.first);

			if (masses.size()>=1){

				FreeConnection(i,Latest);
			}
			/*if (masses.size()>=1 and i<NumberofMasses-1){
				//Setting constraint length from second go!!!
				std::cout<<"Enter position of mass "<<i+1<< std::endl;
				ConsoleOutputforLengthSetting();
			}
			*/


		}
		myfile<<std::endl;

		//Entering time-steps for Active Spring
		int timestepforActiveSpring;
		std::string quitTimeInput;

		if(mapOfActiveSpring.size()>=1){
						while(true){
						std::cout<<"Enter timeSteps for the activation of Active spring"<<std::endl;
						std::cin>>timestepforActiveSpring;
						listoftimeStepforActiveSpring.push_back(timestepforActiveSpring);

						std::cout<<"Done?"<<std::endl;
						std::cin>> quitTimeInput;

						if(quitTimeInput == "y"){
							std::sort(listoftimeStepforActiveSpring.begin(),listoftimeStepforActiveSpring.end());
							break;
						}
					}
				}

		//Now apply force on masses
		ApplyForce();


}
//========================================================================================================================
/*
 * This is used by FreeMassSpringConnection
 * @param: i = current mass
 */
//========================================================================================================================
void MultipleObjects::FreeConnection(int _i,std::pair<btRigidBody*,btTransform> _Latest)
{
	std::string connection;
	bool connectionConstraint;

	//std::cout<<"Do you want to connect mass "<<_i<<" to any other previous mass(true/false)"<<std::endl;
	//std::cin>>connection;

	//if(connection == "t")
		//connectionConstraint = true;
	//else
		//connectionConstraint = false;
	//if(connectionConstraint){
		//addConnection(_Latest,_i);
	//}

	//check size to determine if there is any connection
	int _size = fileReadTuple.end() - (fileReadTuple.begin() + 3);
	std::cout<<"sizeoftuple"<<_size<<std::endl;

	if(_size >= 0){
		addConnection(_Latest,_i);
	}

}
//========================================================================================================================
/*
 * This is used for nonlinear combination of the first mass spring model
 * It is sigmoidal in nature
 */
//========================================================================================================================
void MultipleObjects::ANN(int NoOfHiddenNeuronsinEachLayer,int NoOfLayers,int NoOfOutputNeurons,int NoOfInputNeurons){
	//BackPropagation* bpg = new BackPropagation(NoOfLayers,NoOfOutputNeurons,NoOfInputNeurons,NoOfHiddenNeuronsinEachLayer);







}
//========================================================================================================================
/*
 * This is used to generate Random float numbers
 * @param : a = min value for selection
 * @param : b = max value for selection
 */
//========================================================================================================================
double RandomFloat(float a, float b) {
    float random = ((float) rand()) / (float) RAND_MAX;
    float diff = b - a;
    float r = random * diff;
    return a + r;
}

//========================================================================================================================
/*
 * This is used to compute Integration of Volterra Series
 *  @param : n = number of intervals
 *  @param : minIntegral = lower limit
 *  @param : maxIntegral = upper limit
 */
//========================================================================================================================
/*void MultipleObjects::SimpsonsRule(double n,double minIntegral,double maxIntegral){
	//TODO
	//Function is left

	double delta_x = (maxIntegral-minIntegral)/n;
	std::vector<double> interval;
	double temp = minIntegral;

	double sum;

	//storing value of the interval
	for(int i=0;i<=n;i++){
		interval.push_back(temp);
		temp += delta_x;
	}
	int t=0; //used for handling index of intervals

	//Calculating values of each function step
	// f(xo),f(x1),f(x2)....

	sum += Function(interval[t]); //for zero interval

	for(int i=1;i<n;i++)
	{
		int flag = 0;


		if(flag ==0){
			sum = sum + 4 * Function(interval[++t]);
			flag=1;
		}
	else{
		sum = sum + 2 * Function(interval[++t]);
		flag = 0;

		}
	}
	 sum = sum + Function(interval[++t]); // for last interval
	 sum = sum * (delta_x/3);
	 printf("\n\n  I = %f  ",sum);



	/*for(int i=0;i<interval.size();i++){
		std::cout<<interval[i]<<std::endl;
	}*/

//}

//TODO equilibrium position of Spring
//I have done it
//_springNeutralLength = get the initial linear position as the form of equilibrium position

//========================================================================================================================
// It is to subtract one vector to another. You can also use
							//operator -=() from btVector3 class
//========================================================================================================================
btVector3 MultipleObjects::SubtractVector(const btVector3& v1, const btVector3& v2)
{
	return btVector3(
			v1.m_floats[0] - v2.m_floats[0],
			v1.m_floats[1] - v2.m_floats[1],
			v1.m_floats[2] - v2.m_floats[2]);
}

//========================================================================================================================
/*
 * Implemented Hooke's Law only for checking purposes
 */
//========================================================================================================================
btVector3 MultipleObjects::LinearForceOfSpring(btVector3 _springCurrentLength,btVector3 _springNeutralLength,btScalar springStiffness){
	//Implementing Hooke's Law
	//I am not sure whether the result comes out to be Vector3 if not use btVector3 instead of btScalar.
	//Use distance between two btVector3 and get the btVector3 as the resultant.
	// btDistance(Vector1,Vector2)
	// See : http://code.google.com/p/bullet/source/browse/trunk/src/LinearMath/btVector3.h?r=2612

	btVector3 delta = SubtractVector(_springCurrentLength,_springNeutralLength);
	btVector3 linearForce = delta.operator *=(-springStiffness);

	//btVector3 linearForce = -1 * springStiffness * delta;
	//btVector3 linearForce = -1 * springStiffness * btScalar(fabs(_springCurrentLength - _springNeutralLength));

	return linearForce;
}
//========================================================================================================================
//========================================================================================================================
void MultipleObjects::NonLinearForceOfSpring(void){


}
//========================================================================================================================
/*
 * This is the function used to compute Volterra series
 */
//TODO
//This volterra function implementation is left
//========================================================================================================================
void MultipleObjects::Function(double index){

}
//========================================================================================================================
/*
 * This is used for computing Volterra series of first mass spring model
 */
//========================================================================================================================
void MultipleObjects::VolterraSeries(void){
	//Memory for Volterra Series
	std::vector<int> Memory;

	//implementing equation 2 of paper 1
	double y;
	double tou1 = RandomFloat(0,0.2),tou2 = RandomFloat(0,0.2);;
	double u1 = 0.1, u2 = 0.1;
	double sigma1 = 0.05 ,sigma2 = 0.05;
	//	Function
	double h2 = exp(pow((tou1 - u1),2)/(2*pow(sigma1,2))) + exp(pow((tou2 - u2),2)/(2*pow(sigma2,2)));


	//std::cout<<h2<<std::endl;



}
//========================================================================================================================
//This function sets the spring stiffness value of connected by the mass that is provided in the argument along with stiffness value //TODO
//========================================================================================================================
void MultipleObjects::setSpringStiffnessConnectedbyMass(int _massNumber,double stiffnessValue){
	springAtEquilibrium[_massNumber].stiffness = stiffnessValue;

}
//========================================================================================================================
//This function gives the spring stiffness value of connected by the mass that is provided in the argument//TODO
//========================================================================================================================
double MultipleObjects::getSpringStiffnessConnectedbyMass(int _massNumber){
	return springAtEquilibrium[_massNumber].stiffness;

}
//========================================================================================================================
//TODO
// For doing collision
// http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=9&t=2961

/*
 * This is used to add fixed object in feedforward connection
 */
//========================================================================================================================
std::pair <btRigidBody*,btTransform> MultipleObjects::addBox(double height){
	btBoxShape* box = new btBoxShape(btVector3(btScalar(0.5),btScalar(0.5),btScalar(0.5)));
	btVector3 inertia(0,0,0);

	btScalar mass(0.);

	//Adding a box on the screen
	m_collisionShapes.push_back(box);

	btTransform t;
	t.setIdentity();
	t.setOrigin(btVector3(10,height,0));
	fixedPoints.push_back(btVector3(10,height,0));

	btDefaultMotionState* myMotionState = new btDefaultMotionState(t);
	btRigidBody::btRigidBodyConstructionInfo info(mass,myMotionState,box,inertia); //motion state would actually be non-null in most real usages
	info.m_restitution = 1.3f;
	info.m_friction = 1.5f;
	btRigidBody* rb = new btRigidBody(info);
	m_dynamicsWorld->addRigidBody(rb);

	return std::make_pair(rb,t);

}
//========================================================================================================================
//========================================================================================================================
Triplet MultipleObjects::LengthofSpringforFeedForward(double x,double y,double z){
	lengthofSpringforfeedforward = {x,y,z};
	return lengthofSpringforfeedforward;
}
//========================================================================================================================
/*
 * This is used for providing connection in FeedForward Fashion
 */
//========================================================================================================================
void MultipleObjects::FeedForwardMassSpringConnection(int NoOfMasses){
	for (int i=0;i<NoOfMasses;i++){

		double height = WindowsDimensions - (5 * i);

		//Length of Spring taken randomly from 1 to 10
		double length = RandomFloat(1.0,10.0);


				/*
				 * For each axis, if
				 *
				 * lower limit = upper limit
				 * The axis is locked
				 *
				 * lower limit < upper limit
				 * The axis is limited between the specified values
				 *
				 * lower limit > upper limit
				 * The axis is free and has no limits
				 */

				/*//////////Adjusting length of spring by the user////////////
				 * If user wants to adjust spring length by his own values then just uncomment this part of the code and comment this* line.
				 *
				 *
				std::string adjustLengthofSpring;
				double springlength_;

				std::cout<<"Do you want to adjust length of spring"<<std::endl;
				std::cin >> adjustLengthofSpring;
				if (adjustLengthofSpring == "t"){
					std::cout<< "Enter length of spring"<<std::endl;
					std::cin >> springlength_;
					Triplet lengthofSpring = LengthofSpring(0.,height,springlength_);
				}
				else{
					Triplet lengthofSpring = LengthofSpring(0.,height,length);
				}
				*
				*////////////////////////////////////////////////////////////





		//This is the line that you have to comment while uncommenting adjusting length of spring by user.
		Triplet lengthofSpring = LengthofSpringforFeedForward(0.,height,length);


		std::pair<btRigidBody*,btTransform> Latest = Initialization(getDistributionRange().first,getDistributionRange().second); //min and max of distribution

		myfile<<"Mass Number "<<i<<","<<","<<",";
		//m_dynamicsWorld->addRigidBody(Latest.first);

		//to make the motion horizontal(by fixing all the axes)
		setLowerAngularLimitofSpring(btVector3(0,0,0));
		setUpperAngularLimitofSpring(btVector3(0,0,0));

		//TODO
		//It is left to implement!! Suggestion: to use btCompoundShape or use Yann's reply
		//http://bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=9&t=6792
		//For now I have fixed it via box as a joint

		std::pair <btRigidBody*,btTransform> joint = addBox(height);
		setUpperLimitofSpring(btVector3(0.,0.5,0.));
		setLowerLimitofSpring(btVector3(0.,-0.5,0.));

		addSpringConstraint(Latest.first,Latest.second,joint.first,joint.second);


	}

	//Apply forces on individual masses
	ApplyForce();

	/*std::string applicationofForce;
	int massNumber;

	std::cout<<"Do you want to apply force on any mass " << std::endl;
	std::cin>>applicationofForce;

	if (applicationofForce == "t"){
	while(true){

			std::cout << "Enter mass number"<<std::endl;
			std::cin >> massNumber;

			ApplyLinearForce(massNumber);

			std::cout << "Do you want to add more forces on other masses?"<<std::endl;
			std::cin >> applicationofForce;

			if(applicationofForce == "f"){
				break;
			}
		}
	}*/
}
//========================================================================================================================
/*
 * This function is general function which is used to apply forces on individual masses
 * It basically collects all the mass indexes onto which force is going to apply at particular timeStep
 */
//========================================================================================================================
void MultipleObjects::ApplyForce(){
	std::string applicationofForce;
	int massNumber;

	std::cout<<"Do you want to apply force on any mass " << std::endl;
	std::cin>>applicationofForce;

	if (applicationofForce == "t"){
		while(true){

				std::cout << "Enter mass number"<<std::endl;
				std::cin >> massNumber;
				if(massNumber < masses.size())
					listofMassesOntoWhichForceIsApplied.push_back(massNumber);
				else
					std::cout<<"This mass is not present, Try again"<<std::endl;

				std::cout << "Do you want to add forces on other masses?"<<std::endl;
				std::cin >> applicationofForce;

				if(applicationofForce == "f"){
					break;
				}
			}
		}

	int timestepforForces;
	std::string quitTimeInput;

	if(listofMassesOntoWhichForceIsApplied.size()>=1){
		while(true){
			std::cout<<"Enter timeSteps for the application of forces on mass(es)"<<std::endl;
			std::cin>>timestepforForces;

			listoftimeStepforApplyingForceonMass.push_back(timestepforForces);

			std::cout<<"Done?"<<std::endl;
			std::cin>> quitTimeInput;

			if(quitTimeInput == "y"){
				std::sort(listoftimeStepforApplyingForceonMass.begin(),listoftimeStepforApplyingForceonMass.end());
				break;
				}
		}
	}



}
//========================================================================================================================
/*
 * This function is used to apply feedforward connection
 */
//========================================================================================================================
void MultipleObjects::FeedForwardConnection(void){

}
//========================================================================================================================
/*
 * This function is used to apply linear force on the mass spring system
 * TODO try this : m_bodies[0]->applyCentralForce(btVector3(0,force,0).rotate(m_bodies[0]->getWorldTransform().getRotation().getAxis(), m_bodies[0]->getWorldTransform().getRotation().getAngle()));
 */
//========================================================================================================================
void MultipleObjects::ApplyLinearForce(int massNumber, int _t){
	//btVector3* Vector3D = new btVector3();

	//std::string applicationofForce;
	//std::cout<<"Do you want to apply force on mass " << massNumber << std::endl;
	//std::cin>>applicationofForce;

	//if (applicationofForce == "t"){
		std::cout << "force applied" <<std::endl;
		if (masses.size() > massNumber){
				//btScalar springNeutralLength = Vector3D->distance(springEquilibriumPointandStiffness[massNumber].first);
				//btScalar springCurrentLength = Vector3D->distance(btVector3(masses[massNumber].second.getOrigin().getX(),masses[massNumber].second.getOrigin().getY(),masses[massNumber].second.getOrigin().getZ()));
				/*
				btVector3 springNeutralLength = massRestingLength[massNumber];
				btVector3 springCurrentLength = btVector3(masses[massNumber].second.getOrigin().getX(),masses[massNumber].second.getOrigin().getY(),masses[massNumber].second.getOrigin().getZ());

				btVector3 ForceApplied = LinearForceOfSpring(springCurrentLength,springNeutralLength,btScalar(springEquilibriumPointandStiffness[massNumber].second));

				//TODO
				//One option is to apply force on different position/index of the spring using applyForce(forceMagnitude,RelativePosition on Spring)
				masses[massNumber].first->applyCentralForce(ForceApplied);//after calculating the magnitude of force from Hooke's law it is applied centrally on the mass number.
				*/

			//////////New Strategy
			double f1 = 2.11, f2 = 3.73, f3 = 4.33;
			double forceX = sin(2.0 * M_PI * f1 * _t) * sin(2. * M_PI * f2 * _t) * sin(2. * M_PI * f3 * _t);

			btVector3 relativeForce = btVector3(forceX,0,0);
			btMatrix3x3& objRotation = masses[massNumber].first->getWorldTransform().getBasis();

			//To avoid extra calculation, comment the following
			//btVector3 correctedForce = (masses[massNumber].second * relativeForce) - masses[massNumber].second.getOrigin();

			btVector3 correctedForce = objRotation * relativeForce;
			masses[massNumber].first->applyCentralForce(correctedForce);
	}


}
//========================================================================================================================
/*
 * This function is used to apply nonlinear force on the mass spring system
 */
//========================================================================================================================
void MultipleObjects::ApplyNonlinearForce(void){

}
//========================================================================================================================
/*
 * This is mainly used for the training purposes
 */
//========================================================================================================================
void MultipleObjects::Training(void){






}
//========================================================================================================================
/*
 * This function is used to connect one mass to another via spring connection
 * @param : massNumber = connection from
 */
//========================================================================================================================
void MultipleObjects::addConnection(std::pair<btRigidBody*,btTransform> MassTransformPair,int massNumber){
	std::string multiple;
	//open mappin information of file

	/*
	 * Free Connection Code
	 */
	btTransform frameInA,frameInB;


	int flag;
	int massNo; //Connection to
	bool connection3 = true;
	int typeofConnection;

	//if(selection){
	for(int i = 3;i<fileReadTuple.size();i++){
		massNo = fileReadTuple[i];
		std::cout<<"=========================Connection from first file"<<massNo<<" and "<<massNumber<<std::endl;
		//while(connection3 == true){

			//std::cout<<"Enter mass number which you wanted to connect with mass "<<massNumber<<std::endl;
			//std::cin>>massNo;
			//std::vector<btRigidBody>::iterator iter;
			//for (iter = masses.begin();iter!= masses.end();iter++){
				//if(iter->getMa)

			if(massNo > (masses.size()-1)){
				std::cout<<"This mass does not exist, Try again"<<std::endl;
				continue;
			}
			//std::cout<<"What type of connection do you want"<<std::endl;
			//std::cout<<"1. Spring Connection \n"
			//		   "2. Point to Point Connection(Ball-Socket) \n"
			//		   "3. Slider Connection \n"
			//		   "4. ConeTwist Connection"<<std::endl;

			//std::cin>>typeofConnection;

			//search for the type of connection
			ConnectionMapFileReading();
			std::cout<<"here1"<<connectionMapFileTuple[0]<<" here2 "<<connectionMapFileTuple[1]<<std::endl;


			if(connectionMapFileTuple[0] == massNumber and connectionMapFileTuple[1] == massNo){ //if there is a connection for the current mass number
				typeofConnection = connectionMapFileTuple[2];
				std::cout<<"ConnectionType"<<typeofConnection<<std::endl;
				flag = 0;




			switch (typeofConnection){
			case 1:
			{
				//std::string checkActiveSpring;
				std::cout<<"Spring"<<std::endl;

				//MassTransformPair.first->

				//btVector3 newCentreWithoutDepth1 = MassTransformPair.first->getCenterOfMassPosition();
				//newCentreWithoutDepth1.setX(fabs(MassTransformPair.first->getCenterOfMassPosition().getX() - masses[massNo].first->getCenterOfMassPosition().getX()));

				//frameInA.setOrigin(newCentreWithoutDepth1);
				frameInA = btTransform::getIdentity();
				frameInA.setOrigin(btVector3(btScalar(10.),btScalar(0.),btScalar(0.)));

				//frameInA.setOrigin(MassTransformPair.second.getOrigin());

				//frameInB.setOrigin(masses[massNo].first->getCenterOfMassPosition());
				frameInB = btTransform::getIdentity();
				frameInB.setOrigin(btVector3(btScalar(0.),btScalar(0.),btScalar(0.)));

				//addSpringConstraint(MassTransformPair.first,MassTransformPair.second,masses[massNo].first,masses[massNo].second);
				addSpringConstraint(MassTransformPair.first,frameInA,masses[massNo].first,frameInB);
				setTypeofconnection(1);

				//std::cout<<"Is it Active Spring(t/f)"<<std::endl;
				//std::cin>>checkActiveSpring;
				//if(checkActiveSpring=="t"){
				if(connectionMapFileTuple[3] == 1){
					std::cout<<"ActiveSpring hai"<<std::endl;
					setActiveSpring({massNumber,massNo,1,0.0}); // Here we are not interested in current length
				}
				break;
			}

			case 2:
			{
				addPointToPointConstraint(MassTransformPair.first,MassTransformPair.second,masses[massNo].first,masses[massNo].second);
				setTypeofconnection(2);
				break;
			}
			case 3:
			{
				addSliderConstraint(MassTransformPair.first,MassTransformPair.second,masses[massNo].first,masses[massNo].second);
				setTypeofconnection(3);
				break;
			}
			case 4:
			{
				addConeTwistConstraint(MassTransformPair.first,MassTransformPair.second,masses[massNo].first,masses[massNo].second);
				setTypeofconnection(4);
				break;
			}
			//TODO : default is ignored here
		}


			//addSpringConstraint(MassTransformPair.first,MassTransformPair.second,masses[massNo].first,masses[massNo].second);

			//Creating a map of connection between different masses
			massMappingForFreeConnection.push_back({massNumber,massNo,getTypeofconnection(),0.0});

			//std::cout<<"Do you want more connections t/f"<<std::endl;
			//std::cin>>multiple;

			//if(multiple == "t")
				//connection3 = true;
			//else
				//connection3 = false;
	}
}
	//TODO complete this case
	/*if(flag!=0 and (3 - fileReadTuple.size())<0)
		std::cout<<"The connection from "<<massNo <<" and "<<massNumber<<" does not exists in ConnectionMap file"<<std::endl;
		*/




	//}
/*	else // for single connection
	{
		addConstraint(MassTransformPair.first,MassTransformPair.second,masses[massNo].first,masses[massNo].second);
	}
*/
}
//========================================================================================================================
/*
 * For reseting the system
 */
//========================================================================================================================
void	MultipleObjects::clientResetScene()
//TODO using keyboard is left
{
	exitPhysics();
	Interface();
}
//========================================================================================================================
/*
 * For setting spring to be active
 * @param : _mapofactiveSpring = the spring constraint that are active springs
 */
//========================================================================================================================
void MultipleObjects::setActiveSpring(ConnectionMap _mapofactiveSpring){
	int featureSelectedForActiveSpring;
	/*
	 * ************************************************************************************
	 * Different codes for features of Active-Spring is determined
	 * 1 = Only Current Length
	 * 2 = Only Stiffness
	 * 3 = Resting Length
	 * 4 = Current Length and Stiffness
	 * 5 = Current Length and Resting Length
	 * 6 = Stiffness and Resting Length
	 * ************************************************************************************
	 */
	//TODO do for all cases
	std::cout<<"What features you want in Active-Spring: \n"
			"Enter the code\n"
			"1 = Only Current Length\n"
			"2 = Only Stiffness\n"
			"3 = Resting Length\n"
			"4 = Current Length and Stiffness \n"
			"5 = Current Length and Resting Length \n"
			"6 = Stiffness and Resting Length"
			<<std::endl;
	std::cin>>featureSelectedForActiveSpring;
	setCodeForFeatureSelectedInActiveSpring(featureSelectedForActiveSpring);

	mapOfActiveSpring.push_back(std::make_pair(getCodeForFeatureSelectedInActiveSpring(),_mapofactiveSpring));

}
//========================================================================================================================
/*
 * This function will update all the features of all active spring
 */
//========================================================================================================================
void MultipleObjects::UpdateFeaturesofAllActiveSpring(void){

	//Search for the index of spring connection in the pool of free-connection map

	int indexofActiveSpring;
	double mass1,mass2;
	btVector3 currentPositionOfMass;


	for(int j=0;j<mapOfActiveSpring.size();j++){
		for(int i=0;i<massMappingForFreeConnection.size();i++){

			//Identify spring
			if(massMappingForFreeConnection[i].connectionFrom == mapOfActiveSpring[j].second.connectionFrom and massMappingForFreeConnection[i].connectionTo == mapOfActiveSpring[j].second.connectionTo){
				indexofActiveSpring = i;
				setCodeForFeatureSelectedInActiveSpring(mapOfActiveSpring[j].first);
				//Identify both masses
				mass1 = massMappingForFreeConnection[i].connectionFrom;
				mass2 = massMappingForFreeConnection[i].connectionTo;

				//Current Position of Spring
				currentPositionOfMass = masses[mass1].second.getOrigin();
			}
		}

		//get the current code
	switch(getCodeForFeatureSelectedInActiveSpring()){
		case 1://Current Length
		{
			btScalar new_length = setCurrentLengthofActiveSpring(currentPositionOfMass,mass1);
			//Update length in massMappingForFreeConnection
			/*massMappingForFreeConnection[indexofActiveSpring] = {massMappingForFreeConnection[indexofActiveSpring].connectionFrom,
															 	 massMappingForFreeConnection[indexofActiveSpring].connectionTo,
															 	 massMappingForFreeConnection[indexofActiveSpring].connectionType,
															 	 new_length};*/
			massMappingForFreeConnection[indexofActiveSpring].currentlength = new_length;
			break;
		}
		case 2://Stiffness
		{
			setSpringStiffnessForActiveSpring(getSpringStiffness(),indexofActiveSpring,mass1,mass2);
			break;
		}
		case 3://Resting Length
		{
			btScalar new_length = setRestingLengthForActiveSpring(indexofActiveSpring,mass1,mass2);
			massMappingForFreeConnection[indexofActiveSpring].currentlength = new_length;
			break;
		}
		case 4://Current length and Stiffness
		{
			//First Stiffness is adjusted then the position is changed
			setSpringStiffnessForActiveSpring(getSpringStiffness(),indexofActiveSpring,mass1,mass2);

			btScalar new_length = setCurrentLengthofActiveSpring(currentPositionOfMass,mass1);
			massMappingForFreeConnection[indexofActiveSpring].currentlength = new_length;

			break;
		}
		case 5:{//TODO Current Length and Resting Length
			break;
		}
		case 6:{//Stiffness and Resting Length
			//First Stiffness is adjusted then the position is changed
			setSpringStiffnessForActiveSpring(getSpringStiffness(),indexofActiveSpring,mass1,mass2);

			btScalar new_length = setRestingLengthForActiveSpring(indexofActiveSpring,mass1,mass2);
			massMappingForFreeConnection[indexofActiveSpring].currentlength = new_length;
			break;
		}

	}
}

}

//========================================================================================================================
/*
 * Function implemented to set the current length of ActiveSpring
 * TODO: in separate classes do take timeStep in account
 * @param currentLengthofActiveSpring = the current length of spring
 * @param  _activeSpringIndex = it is the index of the mass among all the masses that behave as active spring
 * ret = new length of spring
 * This function applies a function on currentLengthofActiveSpring to set a new value for active spring length.
 */
//========================================================================================================================
btScalar MultipleObjects::setCurrentLengthofActiveSpring(const btVector3& _currentLengthofActiveSpring,int _activeSpringIndex){
	btVector3 _newCurrentLengthofActiveSpring = _currentLengthofActiveSpring;

	_newCurrentLengthofActiveSpring = 	btVector3((_newCurrentLengthofActiveSpring.getX()/(timeStep/10.0)),(_newCurrentLengthofActiveSpring.getY()/(timeStep/10.0)),(_newCurrentLengthofActiveSpring.getZ()/(timeStep/10.0)));

	//Calculation of new length
	btScalar new_length = btDistance(_currentLengthofActiveSpring,massRestingLength[_activeSpringIndex]);

	//Update mass new position
	masses[_activeSpringIndex].second.setOrigin(btVector3(_newCurrentLengthofActiveSpring));
	masses[_activeSpringIndex].first->getMotionState()->setWorldTransform(masses[_activeSpringIndex].second);
	masses[_activeSpringIndex].first->setCenterOfMassTransform(masses[_activeSpringIndex].second);
	masses[_activeSpringIndex].first->activate();

	std::cout<<"UPDATE"<<masses[_activeSpringIndex].second.getOrigin().getX()<<" "<<masses[_activeSpringIndex].second.getOrigin().getY()<<"  "<<masses[_activeSpringIndex].second.getOrigin().getZ()<<std::endl;
	return new_length;
}
//========================================================================================================================
/*
 * Function implemented to set the stiffness of ActiveSpring
 * @param : it is the index of the mass among all the masses that behave as active spring
 * TODO: in separate classes do take timeStep in account
 */
//========================================================================================================================
void MultipleObjects::setSpringStiffnessForActiveSpring(double _springStiffnessForActiveSpring,int _activeSpringIndex,double _massValue1,double _massValue2){
	/*
	 * For now the function that I have used is:
	 * F = mg = Kx
	 * x = displacement = (timeStep/10)
	 */
	double mass1 = massValues[_massValue1];
	double mass2 = massValues[_massValue2];

	//std::cout<<"mass of spring1 "<<mass1<<std::endl;
	//std::cout<<"mass of spring2 "<<mass2<<std::endl;

	double differenceOfMasses = fabs(mass1 - mass2);

	int indexofBody1 = massMappingForFreeConnection[_activeSpringIndex].connectionFrom;
	int indexofBody2 = massMappingForFreeConnection[_activeSpringIndex].connectionTo;

	//Formula
	//_springStiffnessForActiveSpring = (masses[indexofBody1].first->getInvMass() * m_dynamicsWorld->getGravity().getY())/(timeStep/10.0);
	_springStiffnessForActiveSpring = (differenceOfMasses * fabs(m_dynamicsWorld->getGravity().getY()))/(timeStep/10.0);

	setSpringStiffness(_springStiffnessForActiveSpring);
	setCalledFromActiveSpring(true);

	//store indexofBody1 and indexofBody2 somewhere and call in addSpringConnection
	setIndexOfActiveSpringForStiffness(std::make_pair(indexofBody1,indexofBody2));

	addSpringConstraint(masses[indexofBody1].first,masses[indexofBody1].second,masses[indexofBody2].first,masses[indexofBody2].second);


	std::cout<<"SPRING = "<<getSpringStiffness()<<std::endl;
}


//========================================================================================================================
/*
 * Function implemented to set the resting length of ActiveSpring
 * @param  _activeSpringIndex = it is the index of the mass among all the masses that behave as active spring
 * ret = new length of spring
 * TODO: in separate classes do take timeStep in account
 */
//========================================================================================================================
btScalar MultipleObjects::setRestingLengthForActiveSpring(int _activeSpringIndex,double _connectionTo,double _connectionFrom){
	bool _isConnectionTo;

	//Calculate new resting length
	btVector3 old_restinglength = massRestingLength[_activeSpringIndex];
	btVector3 new_restinglength = old_restinglength/(timeStep/10.0);

	//Update resting length
	massRestingLength[_activeSpringIndex] = new_restinglength;

	//Update SpringAtEquilibrium Vector
	//Search for the connection index in springAtEquilibrium
	if(springAtEquilibrium.size()>=1){
		for(int search = 0;search<springAtEquilibrium.size();search++){
			if((springAtEquilibrium[search].connectionTo == massRestingLength[_connectionTo] and springAtEquilibrium[search].connectionFrom == massRestingLength[_connectionFrom])
			   || (springAtEquilibrium[search].connectionFrom == massRestingLength[_connectionTo] and springAtEquilibrium[search].connectionTo == massRestingLength[_connectionFrom])){
				//Confirm w.r.t resting length
				std::cout<<"HOGYA"<<std::endl;
				if(old_restinglength == springAtEquilibrium[search].connectionTo)
					_isConnectionTo = true;
				else
					_isConnectionTo = false;
				//Update new resting length in SpringAtEquilibrium
				if(_isConnectionTo)//Update ConnectionTo of SpringAtEquilibrium
					springAtEquilibrium[search].connectionTo = new_restinglength;
				else //Update ConnectionFrom of SpringAtEquilibrium
					springAtEquilibrium[search].connectionFrom = new_restinglength;
			}
		}
	}

	//Update mass new position
	masses[_activeSpringIndex].second.setOrigin(btVector3(masses[_activeSpringIndex].second.getOrigin() - new_restinglength));
	masses[_activeSpringIndex].first->getMotionState()->setWorldTransform(masses[_activeSpringIndex].second);
	masses[_activeSpringIndex].first->setCenterOfMassTransform(masses[_activeSpringIndex].second);
	masses[_activeSpringIndex].first->activate();



	//Change distance of current length and the resting length
	btScalar new_length = btDistance(masses[_activeSpringIndex].second.getOrigin(),massRestingLength[_activeSpringIndex]);
	return new_length;

}

//========================================================================================================================
/*
 * For destroying all the conserved memory locations
 */
//========================================================================================================================
void	MultipleObjects::exitPhysics()
{

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

	myfile.close();
	readFile.close();
	connectionMatrixFile.close();
}
//========================================================================================================================

//TODO
//For Nonlinear Spring use
// http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=9&t=2863

//For reset at the runtime:
//override method for keyboard. Include a number that will again initialize the values for masses.




