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

//#include "BackPropagation.h"


#define PlatformDemoApplication GlutDemoApplication


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


class MultipleObjects : public PlatformDemoApplication
{

	//keep the collision shapes, for deletion/cleanup

	std::ofstream myfile;

	btBroadphaseInterface*	m_broadphase;

	btCollisionDispatcher*	m_dispatcher;

	btConstraintSolver*	m_solver;

	btDefaultCollisionConfiguration* m_collisionConfiguration;

	public:

	MultipleObjects()
	{
		_groundDimensions = {200.,20.,80.}; 	//default 200 X 20 X 80 initialization
		_groundOrigin = {0.,-40.,0.};	//default 0 X -50 X 0 initialization
		distMin_ = 1.; //for mass and position
		distMax_ = 5.; //for mass and position
		upperLimitofSpring = btVector3(5., 0., 0.);
		lowerLimitofSpring = btVector3(-5., 0., 0.);
		lowerAngularLimit = btVector3(0.f, 0.f, -1.5f);
		upperAngularLimit = btVector3(0.f, 0.f, 1.5f);
		springStiffness = 39.478f;
		springDamping = 0.5f;
	}

	virtual ~MultipleObjects()
	{
		exitPhysics();
	}

	void	initPhysics();

	void	exitPhysics();

	void Interface();

	void addSpringConstraint(btRigidBody* body1,btTransform t1,btRigidBody* body2,btTransform t2);

	void getResults(std::vector<std::pair<btRigidBody*,btTransform> > _massInformation);

	double getSpringStiffnessConnectedbyMass(int _massNumber);

	void setSpringStiffnessConnectedbyMass(int _massNumber,double stiffnessValue);

	void setSimulationTime(double _time);

	double getSimulationTime();

	void Function(double index);

	void FreeMassSpringConnection(int NumberofMasses);

	btVector3 SubtractVector(const btVector3& v1, const btVector3& v2);

	btVector3 LinearForceOfSpring(btVector3 _springCurrentLength,btVector3 _springNeutralLength,btScalar springStiffness);

	Triplet LengthofSpring(double x,double y,double z);

	void NonLinearForceOfSpring(void);

	std::pair<btRigidBody*,btTransform> addMass(double x,double y,double z,double radius,bool fixed,bool activate);

	void addConnection(std::pair<btRigidBody*,btTransform> MassTransformPair,int massNumber);

	void addGround();

	void SimpsonsRule(double n,double minIntegral,double maxIntegral);

	std::pair<double,double> getDistributionRange();

	void setDistributionRange(double min_,double max_);

	Triplet getGroundDimensions();

	void setGroundDimensions(double _x,double _y,double _z);

	Triplet getGroundOrigin();

	void setGroundOrigin(double _x,double _y,double _z);

	std::pair<btRigidBody*,btTransform> Initialization(double min,double max);

	void setGravity(double y);

	std::pair <btRigidBody*,btTransform> addBox(double height);

	void FeedForwardMassSpringConnection(int NoOfMasses);

	void FeedForwardConnection(void);

	void VolterraSeries();

	void ANN(int NoOfHiddenNeurons,int NoOfHiddenLayers,int NoOfOutputLayers,int NoOfInputLayers);

	void RNNConnection(int NumberofMasses);

	void FreeConnection(int _i,std::pair<btRigidBody*,btTransform> _Latest);

	void setUpperLimitofSpring(btVector3 _uplimspring);

	void setLowerLimitofSpring(btVector3 _lowlimspring);

	void setLowerAngularLimitofSpring(btVector3 _lowanglim);

	void setUpperAngularLimitofSpring(btVector3 _upanglim);

	void Training();

	void ApplyLinearForce(int massNumber);

	void ApplyNonlinearForce(void);

	virtual void clientMoveAndDisplay();

	virtual void displayCallback();

	virtual void	clientResetScene();

	static DemoApplication* Create()
	{
		MultipleObjects* demo = new MultipleObjects;
		demo->myinit();
		demo->initPhysics();
		return demo;
	}

	double getSpringStiffness() const
	{
		return springStiffness;
	}

	void setSpringStiffness(double springStiffness)
	{
		this->springStiffness = springStiffness;
	}

	double getSpringDamping() const
	{
		return springDamping;
	}

	void setSpringDamping(double springDamping)
	{
		this->springDamping = springDamping;
	}

	int getMode() const
	{
		return mode;
	}

	void setMode(int mode)
	{
		this->mode = mode;
	}

private:
	std::vector<std::pair<btRigidBody*,btTransform> > masses;

	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;

	std::vector<std::pair<btVector3,double> >springEquilibriumPointandStiffness; //Rest points and stiffness of spring

	std::vector<btVector3> fixedPoints; //Position of fixed points in feedforward network

	std::vector<std::pair <int,int> > massMappingForFreeConnection;

	double _timeStep;

	//std::vector<Triplet> Co3D;
	Triplet _groundDimensions;
	Triplet _groundOrigin;
	btVector3 upperLimitofSpring;
	btVector3 lowerLimitofSpring;
	btVector3 lowerAngularLimit;
	btVector3 upperAngularLimit;
	Triplet lengthofSpring;
	double springStiffness;
	double springDamping;
	int mode;


	double distMin_,
		   distMax_;

};

#endif //MULTIPLEOBJECTS_DEMO_H



#endif /* MULTIPLEOBJECTS_H_ */
