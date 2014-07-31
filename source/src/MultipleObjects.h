/*
 * Copyright © 2013 Marium Zeeshan at University of Zurich; 2014 René
 * Bernhardsgrütter, Christoph Walter Senn at Zurich University of Applied
 * Sciences; 2014 Helmut Hauser at University of Zurich
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the “Software”), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef MULTIPLEOBJECTS_H_
#define MULTIPLEOBJECTS_H_

#ifndef MULTIPLEOBJECTS_DEMO_H
#define MULTIPLEOBJECTS_DEMO_H

#ifdef _WINDOWS
#include "Win32DemoApplication.h"
#define PlatformDemoApplication Win32DemoApplication
#else
#include "OpenGL/GlutDemoApplication.h"
#define PlatformDemoApplication GlutDemoApplication
#endif

#include "BulletDynamics/Dynamics/btDynamicsWorld.h"
#include "OpenGL/GlutDemoApplication.h"
#include "LinearMath/btAlignedObjectArray.h"
#include <vector>
#include <list>
#include <sstream>
#include <fstream>
#include <tr1/random>
#include <ctime>
#include <cmath>
#include <iomanip>
#include <algorithm>
#include <boost/bind.hpp>
#include "strtk.hpp"
#include "Bullet/BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.h"

#define _USE_MATH_DEFINES

class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

struct Triplet {
    double x_, y_, z_;
};

struct SpringConnectionAtEquilibrium {
    btVector3 connectionFrom;
    btVector3 connectionTo;
    double stiffness;

    btScalar getNeutralLength() {
        btScalar length = btDistance(connectionFrom, connectionTo);
        return length;
    }
};

struct ConnectionMap {
    unsigned int connectionFrom;
    unsigned int connectionTo;
    unsigned int connectionType;
    btScalar currentlength;
};

class MultipleObjects : public PlatformDemoApplication {
    std::ofstream outputFile;
    
    std::ofstream velocitiesFile;
    
    std::ofstream effectorFile;
    
    std::ifstream massesFile;

    std::ifstream connectionMapFile;

    btBroadphaseInterface* m_broadphase;

    btCollisionDispatcher* m_dispatcher;

    btConstraintSolver* m_solver;

    btDefaultCollisionConfiguration* m_collisionConfiguration;

public:

    MultipleObjects() {
        _groundDimensions = {200., 20., 80.}; //default 200 X 20 X 80 initialization
        _groundOrigin = {0., -20.5      , 0.}; //default 0 X -50 X 0 initialization
        lengthofConstraint = {0., 0., 0.}; //initialize with origin
        distMin_ = 1.0; //for mass and position
        distMax_ = 5.0; //for mass and position
        upperLimitOfSpring = btVector3(5., 0., 0.);
        lowerLimitOfSpring = btVector3(-5., 0., 0.);
        lowerAngularLimit = btVector3(0.f, 0.f, -1.5f);
        upperAngularLimit = btVector3(0.f, 0.f, 1.5f);
        springStiffness = 300;//40; // ~10E3 is fine
        springDamping = 1E-4f; // the smaller the value the more the springs are damped
        calledFromActiveSpring = false;
    }

    virtual ~MultipleObjects() {
        exitPhysics();
    }

    void setParameters(std::string massesFilePath, std::string connectionMapFilePath, std::string outputFilePath, std::string inputTrajectoryFilePath, std::string weightsFilePath, bool floor, bool training);

    int readNumberOfMassesFromMassesFile();

    void initPhysics();

    void exitPhysics();

    void Interface();

    void addSpringConstraint(btRigidBody* body1, btTransform t1, btRigidBody* body2, btTransform t2);

    void addSliderConstraint(btRigidBody* body1, btTransform t1, btRigidBody* body2, btTransform t2);

    void addPointToPointConstraint(btRigidBody* body1, btTransform t1, btRigidBody* body2, btTransform t2); //a.k.a Ball socket

    void addRobotBaseConstraint(btRigidBody* body1, btTransform t1, btRigidBody* body2, btTransform t2);

    void addRobotJointConstraint(btRigidBody* body1, btTransform t1, btRigidBody* body2, btTransform t2);

    void addFixedConstraint(btRigidBody* body1, btTransform t1, btRigidBody* body2, btTransform t2);

    void addConeTwistConstraint(btRigidBody* body1, btTransform t1, btRigidBody* body2, btTransform t2);

    void getResults(std::vector<std::pair<btRigidBody*, btTransform> > _massInformation);

    void setActiveSpring(ConnectionMap _mapofactiveSpring);

    double getSpringStiffnessConnectedbyMass(int _massNumber);

    void setSpringStiffnessConnectedbyMass(int _massNumber, double stiffnessValue);

    void setSimulationTime(double _time);

    double getSimulationTime();

    void readNextMassFileTuple(void);

    std::vector<std::vector<double> > readFileToDoubleVector(std::string path);

    void FreeMassSpringConnection(int NumberofMasses);

    btVector3 SubtractVector(const btVector3& v1, const btVector3& v2);

    btVector3 LinearForceOfSpring(btVector3 _springCurrentLength, btVector3 _springNeutralLength, btScalar springStiffness);

    Triplet LengthOfSpringforFeedForward(double x, double y, double z);

    void NonLinearForceOfSpring(void);

    std::pair<btRigidBody*, btTransform> addMass(double x, double y, double z, double radius, char type, bool activate, double massValue);

    void addConnection(std::pair<btRigidBody*, btTransform> MassTransformPair, unsigned int massNumber);

    void addGround();

    void SimpsonsRule(double n, double minIntegral, double maxIntegral);

    std::pair<double, double> getDistributionRange();

    void setDistributionRange(double min_, double max_);

    void UpdateFeaturesOfAllActiveSpring(void);

    Triplet getGroundDimensions();

    void setGroundDimensions(double _x, double _y, double _z);

    Triplet getGroundOrigin();

    void setGroundOrigin(double _x, double _y, double _z);

    std::pair<btRigidBody*, btTransform> Initialization(double min, double max);

    void setGravity(double y);

    std::pair <btRigidBody*, btTransform> addBox(double height);

    void FeedForwardMassSpringConnection(int NoOfMasses);

    void ConsoleOutputforLengthSetting();

    void FeedForwardConnection(void);

    void VolterraSeries();

    void ANN(int NoOfHiddenNeurons, int NoOfHiddenLayers, int NoOfOutputLayers, int NoOfInputLayers);

    void RNNConnection(int NumberofMasses);

    void FreeConnection(int _i, std::pair<btRigidBody*, btTransform> _Latest);

    void setUpperLimitOfSpring(btVector3 _uplimspring);

    void setLowerLimitOfSpring(btVector3 _lowlimspring);

    void setLowerAngularLimitOfSpring(btVector3 _lowanglim);

    void setUpperAngularLimitOfSpring(btVector3 _upanglim);

    void Training();

    void ApplyLinearForce(unsigned int massNumber, int _t);
    
    void openLoop();
    
    void singleStep();
    
    void closedLoop();
    
    void goToStartPosition(double, double);
    
    double getLengthOfSpring(btGeneric6DOFSpringConstraint);

    bool isCalledFromActiveSpring() {
        return calledFromActiveSpring;
    }

    void ApplyNonlinearForce(void);

    bool excludedMass(std::pair<btRigidBody*, btTransform> mass);
    
    virtual void clientMoveAndDisplay();

    virtual void displayCallback();

    virtual void clientResetScene();
    
    static DemoApplication* Create() {
        MultipleObjects* demo = new MultipleObjects;
        demo->myinit();
        demo->Interface();
        return demo;
    }

    double getSpringStiffness() const {
        return springStiffness;
    }

    void setSpringStiffness(double springStiffness) {
        this->springStiffness = springStiffness;
    }

    double getSpringDamping() const {
        return springDamping;
    }

    void setSpringDamping(double springDamping) {
        this->springDamping = springDamping;
    }

    int getMode() const {
        return mode;
    }

    void setMode(int mode) {
        this->mode = mode;
    }

    unsigned int getTypeOfconnection() const {
        return typeofconnection;
    }

    void setTypeOfconnection(int typeofconnection) {
        this->typeofconnection = typeofconnection;
    }

    const Triplet& getLengthOfConstraint() const {
        return lengthofConstraint;
    }

    void setLengthOfConstraint(const Triplet& lengthofconstraint) {
        this->lengthofConstraint.x_ = lengthofconstraint.x_;
        this->lengthofConstraint.y_ = lengthofconstraint.y_;
        this->lengthofConstraint.z_ = lengthofconstraint.z_;
    }

    const btVector3& getCurrentLengthofActiveSpring() const {
        return currentLengthofActiveSpring;
    }

    const btVector3& getRestingLengthForActiveSpring() const {
        return restingLengthForActiveSpring;
    }

    double getSpringStiffnessForActiveSpring() const {
        return springStiffnessForActiveSpring;
    }

    btScalar setCurrentLengthOfActiveSpring(const btVector3& _currentLengthofActiveSpring, int _activeSpringIndex);

    btScalar setRestingLengthForActiveSpring(int _activeSpringIndex, double _connectionTo, double _connectionFrom);

    void ApplyForce();

    void setSpringStiffnessForActiveSpring(double springStiffnessForActiveSpring, int _activeSpringIndex, double _massValue1, double _massValue2);

    int getCodeForFeatureSelectedInActiveSpring() const {
        return codeForFeatureSelectedInActiveSpring;
    }

    void setCodeForFeatureSelectedInActiveSpring(int codeForFeatureSelectedInActiveSpring) {
        this->codeForFeatureSelectedInActiveSpring = codeForFeatureSelectedInActiveSpring;
    }

    void setCalledFromActiveSpring(bool calledFromActiveSpring) {
        this->calledFromActiveSpring = calledFromActiveSpring;
    }

    const std::pair<int, int>& getIndexOfActiveSpringForStiffness() const {
        return indexOfActiveSpringForStiffness;
    }

    void setIndexOfActiveSpringForStiffness(const std::pair<int, int>& indexOfActiveSpringForStiffness) {
        this->indexOfActiveSpringForStiffness = indexOfActiveSpringForStiffness;
    }

    void readNextConnectionMapFileTupel(void);



private:

    std::vector<std::pair<btRigidBody*, btTransform> > masses;
    
    std::vector<std::pair<btRigidBody*, btTransform> > massesToExclude;
    
    std::pair<btRigidBody*, btTransform> endEffector;

    btAlignedObjectArray<btCollisionShape*> m_collisionShapes;

    std::vector<btVector3> massRestingLength; //Rest points of Mass

    std::vector<btVector3> fixedPoints; //Position of fixed points in feed-forward network

    std::vector<ConnectionMap> massMappingForFreeConnection;

    std::vector<SpringConnectionAtEquilibrium> springAtEquilibrium; //Used for setting stiffness at the time of active spring

    std::vector<double> massValues;

    double _timeStep;

    Triplet _groundDimensions;

    Triplet _groundOrigin;

    double springStiffness;

    double springDamping;

    btVector3 upperLimitOfSpring;

    btVector3 lowerLimitOfSpring;

    btVector3 lowerAngularLimit;

    btVector3 upperAngularLimit;

    ////////For Reading/////////////
    //For positions
    std::vector<double> lastReadMassFileTuple;
    std::string lastReadMassFileLine;

    //For connection map
    std::vector<double> lastReadConnectionMapFileTuple;
    std::string lastReadConnectionMapFileLine;
    /////////////////////////////////

    Triplet lengthOfSpringforfeedforward;

    Triplet lengthofConstraint;

    int mode;

    unsigned int typeofconnection;

    //Active Spring
    double springStiffnessForActiveSpring;
    btVector3 restingLengthForActiveSpring;
    btVector3 currentLengthofActiveSpring;
    int codeForFeatureSelectedInActiveSpring;
    bool calledFromActiveSpring;
    std::pair<int, int> indexOfActiveSpringForStiffness;

    std::vector<std::pair <int, ConnectionMap> > mapOfActiveSpring; //feature selected for the active spring and the connection map between two masses
    std::vector<int> listOftimeStepforActiveSpring; //maintaining list of time-step in which active spring is active

    //Apply Force
    std::vector<int> listOfTimeStepforApplyingForceonMass; //maintaining list of time-step on which force is applied on masses
    std::vector<int> listOfMassesOntoWhichForceIsApplied; //holds list of masses on which force is applied

    double distMin_, distMax_;
};

#endif //MULTIPLEOBJECTS_DEMO_H

#endif /* MULTIPLEOBJECTS_H_ */
