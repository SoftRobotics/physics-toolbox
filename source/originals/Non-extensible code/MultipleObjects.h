/*
 * MultipleObjects.h
 *
 *  Created on: Aug 8, 2013
 *      Author: Marium Zeeshan
 */

#ifndef MULTIPLEOBJECTS_H_
#define MULTIPLEOBJECTS_H_

#ifndef MULTIPLEOBJECTS_DEMO_H
#define MULTIPLEOBJECTS_DEMO_H

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

#include <vector>
#include <sstream>
#include <fstream>
#include <tr1/random>
#include <ctime>
#include <cmath>
#include <iomanip>
#include <algorithm>
#include <boost/bind.hpp>

#include "strtk.hpp"

//#include "BackPropagation.h"

#define _USE_MATH_DEFINES


class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;


struct Triplet
{
  double  x_, y_, z_;
};


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
struct ConnectionMap
{
	double connectionFrom;
	double connectionTo;
	int connectionType;
	btScalar currentlength;
};


class MultipleObjects : public PlatformDemoApplication
{

	//keep the collision shapes, for deletion/cleanup

	std::ofstream myfile; //output file of readouts

	std::ifstream readFile; // input file for getting positions of masses

	std::ifstream connectionMatrixFile; //contains type of connections

	btBroadphaseInterface*	m_broadphase;

	btCollisionDispatcher*	m_dispatcher;

	btConstraintSolver*	m_solver;

	btDefaultCollisionConfiguration* m_collisionConfiguration;

	public:

	MultipleObjects()
	{
		_groundDimensions = {200.,20.,80.}; 	//default 200 X 20 X 80 initialization
		_groundOrigin = {0.,-40.,0.};	//default 0 X -50 X 0 initialization
		lengthofConstraint = {0.,0.,0.}; //initialize with origin
		distMin_ = 1.0; //for mass and position
		distMax_ = 5.0; //for mass and position
		upperLimitofSpring = btVector3(5., 0., 0.);
		lowerLimitofSpring = btVector3(-5., 0., 0.);
		lowerAngularLimit = btVector3(0.f, 0.f, -1.5f);
		upperAngularLimit = btVector3(0.f, 0.f, 1.5f);
		springStiffness = 39.478f;
		springDamping = 0.5f;
		calledFromActiveSpring = false;
	}

	virtual ~MultipleObjects()
	{
		exitPhysics();
	}
//X
	void	initPhysics();
//X
	void	exitPhysics();
//X
	void Interface();
//X
	void addSpringConstraint(btRigidBody* body1,btTransform t1,btRigidBody* body2,btTransform t2);
//X
	void addSliderConstraint(btRigidBody* body1,btTransform t1,btRigidBody* body2,btTransform t2);
//X
	void addPointToPointConstraint(btRigidBody* body1,btTransform t1,btRigidBody* body2,btTransform t2); //a.k.a Ball socket
//X
	void addConeTwistConstraint(btRigidBody* body1,btTransform t1,btRigidBody* body2,btTransform t2);

	// XXXX
	void getResults(std::vector<std::pair<btRigidBody*,btTransform> > _massInformation);

	void setActiveSpring(ConnectionMap _mapofactiveSpring);

	double getSpringStiffnessConnectedbyMass(int _massNumber);

	void setSpringStiffnessConnectedbyMass(int _massNumber,double stiffnessValue);

	void setSimulationTime(double _time);

	double getSimulationTime();

	void Function(double index);

	void FileReading(void);

	void FreeMassSpringConnection(int NumberofMasses);

	btVector3 SubtractVector(const btVector3& v1, const btVector3& v2);

	btVector3 LinearForceOfSpring(btVector3 _springCurrentLength,btVector3 _springNeutralLength,btScalar springStiffness);

	Triplet LengthofSpringforFeedForward(double x,double y,double z);

	void NonLinearForceOfSpring(void);

	std::pair<btRigidBody*,btTransform> addMass(double x,double y,double z,double radius,bool fixed,bool activate,double massValue);

	void addConnection(std::pair<btRigidBody*,btTransform> MassTransformPair,int massNumber);

	void addGround();

	void SimpsonsRule(double n,double minIntegral,double maxIntegral);
//X
	std::pair<double,double> getDistributionRange();
//X
	void setDistributionRange(double min_,double max_);

	void UpdateFeaturesofAllActiveSpring(void);
//X
	Triplet getGroundDimensions();
//X
	void setGroundDimensions(double _x,double _y,double _z);
//X
	Triplet getGroundOrigin();
//X
	void setGroundOrigin(double _x,double _y,double _z);
//X TODO return rehgaya hai
	std::pair<btRigidBody*,btTransform> Initialization(double min,double max);
//X
	void setGravity(double y);
//X
	std::pair <btRigidBody*,btTransform> addBox(double height);
//X
	void FeedForwardMassSpringConnection(int NoOfMasses);

	void ConsoleOutputforLengthSetting();

	void FeedForwardConnection(void);

	void VolterraSeries();

	void ANN(int NoOfHiddenNeurons,int NoOfHiddenLayers,int NoOfOutputLayers,int NoOfInputLayers);
//X
	void RNNConnection(int NumberofMasses);

	void FreeConnection(int _i,std::pair<btRigidBody*,btTransform> _Latest);
//X
	void setUpperLimitofSpring(btVector3 _uplimspring);
//X
	void setLowerLimitofSpring(btVector3 _lowlimspring);
//X
	void setLowerAngularLimitofSpring(btVector3 _lowanglim);
//X
	void setUpperAngularLimitofSpring(btVector3 _upanglim);

	void Training();

	void ApplyLinearForce(int massNumber,int _t);
//X
	bool isCalledFromActiveSpring(){
		return calledFromActiveSpring;
	}

	void ApplyNonlinearForce(void);
//X
	virtual void clientMoveAndDisplay();
//X
	virtual void displayCallback();
//X
	virtual void	clientResetScene();

	static DemoApplication* Create()
	{
		MultipleObjects* demo = new MultipleObjects;
		demo->myinit();
		demo->Interface();
		return demo;
	}
//X
	double getSpringStiffness() const
	{
		return springStiffness;
	}
//X
	void setSpringStiffness(double springStiffness)
	{
		this->springStiffness = springStiffness;
	}
//X
	double getSpringDamping() const
	{
		return springDamping;
	}
//X
	void setSpringDamping(double springDamping)
	{
		this->springDamping = springDamping;
	}
//X
	int getMode() const
	{
		return mode;
	}
//X
	void setMode(int mode)
	{
		this->mode = mode;
	}
//X
	int getTypeofconnection() const
	{
		return typeofconnection;
	}
//X
	void setTypeofconnection(int typeofconnection)
	{
		this->typeofconnection = typeofconnection;
	}

	const Triplet& getLengthofConstraint() const
	{
		return lengthofConstraint;
	}

	void setLengthofConstraint(const Triplet& lengthofconstraint)
	{
		this->lengthofConstraint.x_ = lengthofconstraint.x_;
		this->lengthofConstraint.y_ = lengthofconstraint.y_;
		this->lengthofConstraint.z_ = lengthofconstraint.z_;
	}

	const btVector3& getCurrentLengthofActiveSpring() const
	{
		return currentLengthofActiveSpring;
	}

	const btVector3& getRestingLengthForActiveSpring() const
	{
		return restingLengthForActiveSpring;
	}

	double getSpringStiffnessForActiveSpring() const
	{
		return springStiffnessForActiveSpring;
	}

	btScalar setCurrentLengthofActiveSpring(const btVector3& _currentLengthofActiveSpring,int _activeSpringIndex);

	btScalar setRestingLengthForActiveSpring(int _activeSpringIndex,double _connectionTo,double _connectionFrom);

	void ApplyForce();

	void setSpringStiffnessForActiveSpring(double springStiffnessForActiveSpring,int _activeSpringIndex,double _massValue1,double _massValue2);

	int getCodeForFeatureSelectedInActiveSpring() const
	{
		return codeForFeatureSelectedInActiveSpring;
	}

	void setCodeForFeatureSelectedInActiveSpring(int codeForFeatureSelectedInActiveSpring)
	{
		this->codeForFeatureSelectedInActiveSpring = codeForFeatureSelectedInActiveSpring;
	}

	void setCalledFromActiveSpring(bool calledFromActiveSpring)
	{
		this->calledFromActiveSpring = calledFromActiveSpring;
	}

	const std::pair<int,int>& getIndexOfActiveSpringForStiffness() const
	{
		return indexOfActiveSpringForStiffness;
	}

	void setIndexOfActiveSpringForStiffness(const std::pair<int,int>& indexOfActiveSpringForStiffness)
	{
		this->indexOfActiveSpringForStiffness = indexOfActiveSpringForStiffness;
	}
	void ConnectionMapFileReading(void);

private:
	//X
	std::vector<std::pair<btRigidBody*,btTransform> > masses;

	//X
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;
	//X
	std::vector<btVector3> massRestingLength; //Rest points of Mass
	//X
	std::vector<btVector3> fixedPoints; //Position of fixed points in feed-forward network
//X
	std::vector<ConnectionMap> massMappingForFreeConnection;
//X
	std::vector<SpringConnectionAtEquilibrium> springAtEquilibrium; //Used for setting stiffness at the time of active spring
//X
	std::vector<double> massValues;


//X
	double _timeStep;

	//std::vector<Triplet> Co3D;

//X
	Triplet _groundDimensions;
//X
	Triplet _groundOrigin;
//X
	double springStiffness;
//X
	double springDamping;
//X
	btVector3 upperLimitofSpring;
//X
	btVector3 lowerLimitofSpring;
//X
	btVector3 lowerAngularLimit;
//X
	btVector3 upperAngularLimit;

	////////For Reading/////////////
	//For positions
	std::vector<double> fileReadTuple;
	std::string readFileString;
	//For connection map
	std::string connectionMapFileString;
	std::vector<double> connectionMapFileTuple;
	/////////////////////////////////

	Triplet lengthofSpringforfeedforward;
	Triplet lengthofConstraint;
//X
	int mode;
//X
	int typeofconnection;


	//Active Spring
	//X
	double springStiffnessForActiveSpring;
	btVector3 restingLengthForActiveSpring;
	btVector3 currentLengthofActiveSpring;
	int codeForFeatureSelectedInActiveSpring;
	bool calledFromActiveSpring;
	std::pair<int,int> indexOfActiveSpringForStiffness;

	std::vector<std::pair <int,ConnectionMap> > mapOfActiveSpring; //feature selected for the active spring and the connection map between two masses
	std::vector<int> listoftimeStepforActiveSpring; //maintaining list of time-step in which active spring is active
	////X

	//X
	//Apply Force
	std::vector<int> listoftimeStepforApplyingForceonMass; //maintaining list of time-step on which force is applied on masses
	std::vector<int> listofMassesOntoWhichForceIsApplied; //holds list of masses on which force is applied
	////
//X

	//X
	double distMin_,
		   distMax_;

};

#endif //MULTIPLEOBJECTS_DEMO_H



#endif /* MULTIPLEOBJECTS_H_ */
