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

// scaling of the objects (0.1 = 20 centimeter boxes )
#define SCALING 1.
#define START_POS_X -5
#define START_POS_Y -5
#define START_POS_Z -3

#define ARM_RADIUS 0.05f
#define ARM_LENGTH 5.f
#define MASS_RADIUS 0.15f

#define DIFFERENCE_DIVIDER 5
#define MAX_ANGULAR_VELOCITY 5*SIMD_RADS_PER_DEG

#define WindowsDimensions 30

#define HINGE_TYPE btHingeConstraint
#define CSV_COMMENT_CHARACTER '#'
#define CSV_NEW_LINE_CHARACTER '\n'
#define CSV_VALUE_SEPARATOR " \t,"
#define SOLVER_ITERATIONS 725
#define APPLY_FORCES_ON_MASSES_DEFAULT false
#define MASS_OF_MASSES 0.01
#define MASS_FACTOR_ARM 1000
#define GRAVITY_CONSTANT -0//-9.81
#define FADEOUT 6284
#define FADE_OVER 6284
#define SINGLE_STEP_RESPONSE false


#include "MultipleObjects.h"
#include "OpenGL/GlutStuff.h"
#include "Bullet/btBulletDynamicsCommon.h"
#include <stdio.h>
#include "OpenGL/GLDebugDrawer.h"
#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"
#include <iostream>
#include <list>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <log4cxx/logger.h>
#include <log4cxx/propertyconfigurator.h>

// Initialize logger for this object
log4cxx::LoggerPtr logger(log4cxx::Logger::getLogger("MultipleObjects"));


static GLDebugDrawer gDebugDrawer;
static double timeStep = 0.0;

bool skipMenu = false;
bool training = true;
bool closed = false;
long stepCounter = 0;
std::string massesFilePath;
std::string connectionMapFilePath;
std::string outputFilePath;
std::string velocitiesFilePath = "../Bapp/devArm/circle/velocities.csv";
std::string inputTrajectoryFilePath;
std::string inputWeightsFilePath;
std::string debugFilePath = "../Bapp/devArm/circle/debug.csv";
std::string effectorFilePath = "../Bapp/devArm/circle/effector.csv";
std::ofstream debugFile;
bool gotFloor = false;
bool startPosition = false;
bool writeValues = false;
int numberMasses = 0;
float dt = 1.f / 420.f;

btGeneric6DOFConstraint* robotBase;
std::list<HINGE_TYPE*> robotJoints;
std::vector<std::vector<double> > motorInput;
std::vector<std::vector<double> > weightsInput;
std::vector<std::vector<double> >::iterator weightsInputIterator;
std::vector<std::vector<double> >::iterator motorInputIterator;
std::list<btGeneric6DOFSpringConstraint*> springs;

typedef std::tr1::ranlux64_base_01 Engine;
typedef std::tr1::normal_distribution<double> Distribution;

Engine engine;

void MultipleObjects::setParameters(std::string _massesFilePath, std::string _connectionMapFilePath, std::string _outputFilePath, std::string _inputTrajectory, std::string _inputWeights, bool _floor, bool _training) {
    massesFilePath = _massesFilePath;
    connectionMapFilePath = _connectionMapFilePath;
    outputFilePath = _outputFilePath;
    inputTrajectoryFilePath = _inputTrajectory;
    inputWeightsFilePath = _inputWeights;
    gotFloor = _floor;
    numberMasses = readNumberOfMassesFromMassesFile();

    skipMenu = true;
    training = _training;
}

int MultipleObjects::readNumberOfMassesFromMassesFile() {
    std::vector<std::vector<double> > massLine = readFileToDoubleVector(massesFilePath);
    return massLine.size();
}

void MultipleObjects::clientMoveAndDisplay() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    LOG4CXX_DEBUG(logger, "====================================================");
    LOG4CXX_DEBUG(logger, "INFORMATION OF SIMULATION");
    LOG4CXX_DEBUG(logger, "====================================================");
    LOG4CXX_DEBUG(logger, "Time Step " << ++timeStep);
    LOG4CXX_DEBUG(logger, "Gravity  (" << m_dynamicsWorld->getGravity().getX() << "," << m_dynamicsWorld->getGravity().getY() << "," << m_dynamicsWorld->getGravity().getZ() << ")");


    //For the ACTIVE SPRING
    if (listOftimeStepforActiveSpring.size() >= 1) {

        for (unsigned int i = 0; i < listOftimeStepforActiveSpring.size(); i++) {
            if (timeStep == listOftimeStepforActiveSpring[i]) {
                UpdateFeaturesOfAllActiveSpring();
            }
            if (listOftimeStepforActiveSpring[i] > timeStep) {
                break;
            }
        }
    }

    //For applying FORCES
    if (listOfTimeStepforApplyingForceonMass.size() >= 1) {
        for (unsigned int f = 0; f < listOfTimeStepforApplyingForceonMass.size(); f++) {

            if (timeStep == listOfTimeStepforApplyingForceonMass[f]) {
                //Apply force on the listed masses
                // For now: Linear forces are applied
                for (unsigned int m = 0; m < listOfMassesOntoWhichForceIsApplied.size(); m++) {
                    ApplyLinearForce(listOfMassesOntoWhichForceIsApplied[m], timeStep);

                }

            }
            if (listOfTimeStepforApplyingForceonMass[f] > timeStep) {
                break;
            }
        }
    }

    btTransform trans;
    for (unsigned int i = 0; i < masses.size(); i++) {
        masses[i].first->getMotionState()->getWorldTransform(trans);
        LOG4CXX_DEBUG(logger, "Mass " << i << " Forces " << masses[i].first->getTotalForce().getX() << " " << masses[i].first->getTotalForce().getY() << " " << masses[i].first->getTotalForce().getZ());
        LOG4CXX_DEBUG(logger, "Mass " << i << " Torque " << masses[i].first->getTotalTorque().getX() << " " << masses[i].first->getTotalTorque().getY() << " " << masses[i].first->getTotalTorque().getZ());

        if (masses[i].first->isActive()) {
            LOG4CXX_DEBUG(logger, "Mass " << i);
            LOG4CXX_DEBUG(logger, "("
                    << trans.getOrigin().getX() << ","
                    << trans.getOrigin().getY() << ","
                    << trans.getOrigin().getZ() << ")");

            double lengthX = masses[i].second.getOrigin().getX();
            double lengthY = masses[i].second.getOrigin().getY();
            double lengthZ = masses[i].second.getOrigin().getZ();

            if (getMode() == 1) //FeedForward Network
            {
                btScalar ChangeOfLength = btDistance(btVector3(lengthX, lengthY, lengthZ), massRestingLength[i]);

                LOG4CXX_DEBUG(logger, "Change of Length of Spring of Mass " << i << " = " << ChangeOfLength);

                btScalar LengthOfSpring = btDistance(btVector3(lengthX, lengthY, lengthZ), fixedPoints[i]);
                LOG4CXX_DEBUG(logger, "Current Length of spring of Mass " << i << " = " << LengthOfSpring);
                LOG4CXX_DEBUG(logger, "Angular velocity of mass" << i << " = " << " (" << masses[i].first->getAngularVelocity().getX() << " , " << masses[i].first->getAngularVelocity().getY() << " , " << masses[i].first->getAngularVelocity().getZ() << " ) rad/s");
                LOG4CXX_DEBUG(logger, "Speed of mass " << i << " = " << masses[i].first->getLinearVelocity().length());

            }

            if (getMode() == 2) { // For Recurrent Network
                LOG4CXX_DEBUG(logger, "Angular velocity of mass" << i << " = " << " (" << masses[i].first->getAngularVelocity().getX() << " , " << masses[i].first->getAngularVelocity().getY() << " , " << masses[i].first->getAngularVelocity().getZ() << " ) rad/s");
                for (unsigned int j = 0; j < i; j++) {
                    //Finding the length of spring each of the connection
                    double lengthX_new = masses[j].second.getOrigin().getX();
                    double lengthY_new = masses[j].second.getOrigin().getY();
                    double lengthZ_new = masses[j].second.getOrigin().getZ();

                    // I think change of length can be possible but it wont give any useful info..

                    btScalar LengthOfSpring = btDistance(btVector3(lengthX, lengthY, lengthZ), btVector3(lengthX_new, lengthY_new, lengthZ_new));
                    LOG4CXX_DEBUG(logger, "Current Length of spring between Mass " << j << " and " << i << " = " << LengthOfSpring);

                }
            }

            if (getMode() == 3) { //Connection of any-type (of your choice)
                LOG4CXX_DEBUG(logger, "Angular velocity of mass" << i << " = " << " (" << masses[i].first->getAngularVelocity().getX() << " , " << masses[i].first->getAngularVelocity().getY() << " , " << masses[i].first->getAngularVelocity().getZ() << " ) rad/s");
                if (writeValues && !excludedMass(masses[i])) {
                    velocitiesFile << masses[i].first->getLinearVelocity().getX(); // << masses[i].first->getLinearVelocity().getY() << " , " << masses[i].first->getLinearVelocity().getZ() << " , ";
                    if (i < masses.size() - 1) {
                        velocitiesFile << " , ";
                    }
                }
                if (massMappingForFreeConnection.size() >= 1) {
                    for (unsigned int j = 0; j < massMappingForFreeConnection.size(); j++) {
                        if (i == massMappingForFreeConnection[j].connectionFrom) {
                            //calculate length
                            double lengthX_new = masses[massMappingForFreeConnection[j].connectionTo].second.getOrigin().getX();
                            double lengthY_new = masses[massMappingForFreeConnection[j].connectionTo].second.getOrigin().getY();
                            double lengthZ_new = masses[massMappingForFreeConnection[j].connectionTo].second.getOrigin().getZ();

                            btScalar LengthOfSpring = btDistance(btVector3(lengthX, lengthY, lengthZ), btVector3(lengthX_new, lengthY_new, lengthZ_new));

                            //Update Length of Spring
                            massMappingForFreeConnection[j] = {massMappingForFreeConnection[j].connectionFrom, massMappingForFreeConnection[j].connectionTo, getTypeOfconnection(), LengthOfSpring};

                            LOG4CXX_DEBUG(logger, "Current Length of spring between Mass " << massMappingForFreeConnection[j].connectionFrom << " and " << massMappingForFreeConnection[j].connectionTo << " = " << massMappingForFreeConnection[j].currentlength);
                        }
                    }
                }
            }
        } else if (writeValues) {
            outputFile << "NaN,NaN,NaN,";
        }
    }
    if (writeValues) {
        //outputFile << std::endl;
        velocitiesFile << std::endl;
    }
    {
        static bool once = true;
        if (m_dynamicsWorld->getDebugDrawer() && once) {
            m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawConstraints + btIDebugDraw::DBG_DrawConstraintLimits);
            once = false;
        }
    }

    // Step simulation
    {
        //during idle mode, just run 1 simulation step maximum
        int maxSimSubSteps = m_idle ? 1 : 1;
        if (m_idle)
            dt = 1.0f / 420.f;

        if (startPosition) {
            writeValues = true;
        }

        if (!SINGLE_STEP_RESPONSE) {
            if (!training && closed) {
                LOG4CXX_INFO(logger, "Feedback loop is: closed");
                closedLoop();
            } else {
                LOG4CXX_INFO(logger, "Feedback loop is: open");
                openLoop();
            }
            effectorFile << endEffector.first->getCenterOfMassPosition().getX() << "," << endEffector.first->getCenterOfMassPosition().getY() << "\n";
            effectorFile.flush();
        } else {
            singleStep();
        }

        if (writeValues) {
            std::list<btGeneric6DOFSpringConstraint*>::iterator springIterator;

            for (springIterator = springs.begin(); springIterator != springs.end(); ++springIterator) {
                if (springIterator != springs.begin())
                    outputFile << ",";
                outputFile << getLengthOfSpring((**springIterator));
            }
            outputFile << std::endl;
        }
        
        maxSimSubSteps = 1;
        
        //dt = ((float)getDeltaTimeMicroseconds()) * 0.001f;
        m_dynamicsWorld->stepSimulation(dt, maxSimSubSteps, dt);

        //optional but useful: debug drawing
        //        m_dynamicsWorld->debugDrawWorld();

    }

    renderme();
    glFlush();
    swapBuffers();
}

void MultipleObjects::openLoop() {
    std::list<HINGE_TYPE*>::iterator Iterator;


    if (motorInputIterator == motorInput.end()) {
        motorInputIterator = motorInput.begin();
    }

    std::vector<double> currentInputs = (*motorInputIterator);

    if (0 >= currentInputs.size()) {
        LOG4CXX_ERROR(logger, "Index out of bound");
        exit(EXIT_FAILURE);
    }

    unsigned int i = 0;
    if (!startPosition) {
        //goToStartPosition(6.5397, 0.75247);
        goToStartPosition(6.3523,0.37638);
    } else {
        btVector3* temp = new btVector3();
        robotBase->getAngularLowerLimit(*temp);
        btScalar angle = temp->getZ();

        int axis = 5;
        robotBase->setLimit(axis, angle + currentInputs[i], angle + currentInputs[i]);
        ++i;
        for (Iterator = robotJoints.begin(); Iterator != robotJoints.end(); ++Iterator, ++i) {
            if (i >= currentInputs.size()) {
                LOG4CXX_ERROR(logger, "Index out of bound");
                exit(EXIT_FAILURE);
            }
            angle = (*Iterator)->getLowerLimit();
            (*Iterator)->setLimit(angle + currentInputs[i], angle + currentInputs[i]);
        }
        ++motorInputIterator;

        if (++stepCounter > FADEOUT) {
            closed = true;
        }
    }
}

void MultipleObjects::singleStep() {
    std::list<HINGE_TYPE*>::iterator Iterator;
    
    if (motorInputIterator == motorInput.end()) {
        motorInputIterator = motorInput.begin();
    }

    int axis = 5;
    robotBase->setLimit(axis, SIMD_RADS_PER_DEG, SIMD_RADS_PER_DEG);
    for (Iterator = robotJoints.begin(); Iterator != robotJoints.end(); ++Iterator) {
        (*Iterator)->setLimit(0, 0);
    }
    startPosition = true;
}

bool MultipleObjects::excludedMass(std::pair<btRigidBody*, btTransform> mass) {
    for (unsigned int i = 0; i < massesToExclude.size(); ++i) {
        if (mass == massesToExclude[i])
            return true;
    }

    return false;
}

double MultipleObjects::getLengthOfSpring(btGeneric6DOFSpringConstraint spring) {
    return btVector3(spring.getRigidBodyA().getCenterOfMassPosition() - spring.getRigidBodyB().getCenterOfMassPosition()).length();
}

void MultipleObjects::goToStartPosition(double alpha, double beta) {
    std::list<HINGE_TYPE*>::iterator Iterator;

    // adjust alpha and beta (mod with 2*PI)
    while (alpha > SIMD_2_PI)
        alpha -= SIMD_2_PI;
    while (alpha < 0)
        alpha += SIMD_2_PI;
    while (beta > SIMD_2_PI)
        beta -= SIMD_2_PI;
    while (beta < 0)
        beta += SIMD_2_PI;


    bool baseInPosition = false;
    btVector3* temp = new btVector3();
    robotBase->getAngularLowerLimit(*temp);
    btScalar angle = temp->getZ();
    btScalar difference = alpha - angle;
    robotBase->setLimit(5, angle + difference / DIFFERENCE_DIVIDER, angle + difference / DIFFERENCE_DIVIDER);

    if (difference < 0.05 && difference > -0.05) {
        baseInPosition = true;
    }

    bool jointsInPosition = true;

    for (Iterator = robotJoints.begin(); Iterator != robotJoints.end(); ++Iterator) {
        angle = (*Iterator)->getLowerLimit();
        difference = beta - angle;
        (*Iterator)->setLimit(angle + difference / DIFFERENCE_DIVIDER, angle + difference / DIFFERENCE_DIVIDER);
        if (difference > 0.05 || difference < -0.05) {
            jointsInPosition = false;
        }
    }
    if (baseInPosition && jointsInPosition)
        startPosition = true;
}

void MultipleObjects::closedLoop() {
    static double fadeoverStep = 0;
    weightsInput = readFileToDoubleVector(inputWeightsFilePath);
    std::list<HINGE_TYPE*>::iterator motorIterator;

    weightsInputIterator = weightsInput.begin();
    std::vector<double> currentWeights = (*weightsInputIterator);

    if (0 >= currentWeights.size()) {
        LOG4CXX_ERROR(logger, "Index out of bound");
        exit(EXIT_FAILURE);
    }

    btVector3* temp = new btVector3();
    robotBase->getAngularLowerLimit(*temp);
    btScalar angle = temp->getZ();
    double dAngle = 0;


    std::list<btGeneric6DOFSpringConstraint*>::iterator springIterator;
    unsigned int j = 0;
    for (springIterator = springs.begin(), j = 0; springIterator != springs.end() && j < currentWeights.size(); ++springIterator, ++j) {
        dAngle += getLengthOfSpring((**springIterator)) * currentWeights[j];
    }


    if (motorInputIterator == motorInput.end()) {
        motorInputIterator = motorInput.begin();
    }

    std::vector<double> currentInputs = (*motorInputIterator);

    if (fadeoverStep <= FADE_OVER) {
        dAngle = fadeoverStep / FADE_OVER * dAngle + (1 - fadeoverStep / FADE_OVER) * currentInputs[0];
    }

    robotBase->setLimit(5, dAngle + angle, dAngle + angle);
    debugFile << dAngle << ",";

    ++weightsInputIterator;

    int i = 1;

    for (motorIterator = robotJoints.begin(); motorIterator != robotJoints.end(); ++motorIterator, ++weightsInputIterator, ++i) {
        currentWeights = (*weightsInputIterator);
        angle = (*motorIterator) -> getLowerLimit();
        dAngle = 0;
        for (springIterator = springs.begin(), j = 0; springIterator != springs.end() && j < currentWeights.size(); ++springIterator, ++j) {
            dAngle += getLengthOfSpring((**springIterator)) * currentWeights[j];
        }

        if (fadeoverStep <= FADE_OVER) {
            dAngle = fadeoverStep / FADE_OVER * dAngle + (1 - fadeoverStep / FADE_OVER) * currentInputs[i];
        }

        (*motorIterator)->setLimit(dAngle + angle, dAngle + angle);

        debugFile << dAngle << std::endl;
    }
    debugFile.flush();
    ++motorInputIterator;
    ++weightsInputIterator;
    ++fadeoverStep;
}

void MultipleObjects::setGravity(double y) {
    m_dynamicsWorld->setGravity(btVector3(0, y, 0));
}

void MultipleObjects::getResults(std::vector<std::pair<btRigidBody*, btTransform> > _massInformation) {
}

void MultipleObjects::setSimulationTime(double _time) {
    _timeStep = _time;
    timeStep = _timeStep;
}

double MultipleObjects::getSimulationTime(void) {
    _timeStep = timeStep;
    return _timeStep;
}

void MultipleObjects::displayCallback(void) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    renderme();

    //optional but useful: debug drawing to detect problems
    if (m_dynamicsWorld)
        m_dynamicsWorld->debugDrawWorld();

    glFlush();
    swapBuffers();
}

Triplet MultipleObjects::getGroundDimensions() {
    return _groundDimensions;
}

Triplet MultipleObjects::getGroundOrigin() {
    return _groundOrigin;
}

std::pair<double, double> MultipleObjects::getDistributionRange() {
    return std::make_pair(distMin_, distMax_);
}

void MultipleObjects::setDistributionRange(double min_, double max_) {
    distMin_ = min_;
    distMax_ = max_;
}

double random_normal(Engine &eng, Distribution dist) {
    return dist(eng);
}

std::pair<btRigidBody*, btTransform> MultipleObjects::Initialization(double min, double max) {
    Distribution myDist(min, max);

    //To place them on plane, Taking it half because box is placed on the origin symmetrically
    Distribution CoX(getGroundOrigin().x_ / 2.0, getGroundDimensions().x_ / 2.0);
    Distribution CoY(getGroundOrigin().y_ / 2.0, getGroundDimensions().y_ / 2.0);
    Distribution CoZ(getGroundOrigin().z_ / 2.0, getGroundDimensions().z_ / 2.0);

    double mass, x, y, z, radius;
    std::string type;

    myDist.reset();
    CoX.reset();
    CoY.reset();
    CoZ.reset();

    mass = MASS_OF_MASSES;
    radius = MASS_RADIUS;

    massValues.push_back(mass);

    if (getMode() == 2) {
        x = random_normal(engine, CoX);
        y = random_normal(engine, CoY);
        z = random_normal(engine, CoZ);
        setLengthOfConstraint({x, y, z});
    }

    bool activateMass;
    type = std::string(lastReadMassFileLine, 0, 1);
    activateMass = true;
    std::pair<btRigidBody*, btTransform> newMass;

    if (getMode() == 1) {
        setLengthOfConstraint({lengthOfSpringforfeedforward.x_, lengthOfSpringforfeedforward.y_, lengthOfSpringforfeedforward.z_});
        newMass = addMass(getLengthOfConstraint().x_, getLengthOfConstraint().y_, getLengthOfConstraint().z_, radius, type[0], activateMass, mass);

    } else
        newMass = addMass(getLengthOfConstraint().x_, getLengthOfConstraint().y_, getLengthOfConstraint().z_, radius, type[0], activateMass, mass);

    massRestingLength.push_back(btVector3(getLengthOfConstraint().x_, getLengthOfConstraint().y_, getLengthOfConstraint().z_));

    LOG4CXX_DEBUG(logger, "Mass No = " << masses.size() - 1 << ": \t Mass = " << mass << "\t Position = (" << getLengthOfConstraint().x_ << "," << getLengthOfConstraint().y_ << "," << getLengthOfConstraint().z_ << ")");
    return newMass;
}

void MultipleObjects::setGroundOrigin(double _x, double _y, double _z) {
    _groundOrigin.x_ = _x;
    _groundOrigin.y_ = _y;
    _groundOrigin.z_ = _z;

}

void MultipleObjects::setGroundDimensions(double _x, double _y, double _z) {
    _groundDimensions.x_ = _x;
    _groundDimensions.y_ = _y;
    _groundDimensions.z_ = _z;

}

void MultipleObjects::addGround() {
    btBoxShape* groundShape = new btBoxShape(btVector3(btScalar(_groundDimensions.x_), btScalar(_groundDimensions.y_), btScalar(_groundDimensions.z_)));
    groundShape->initializePolyhedralFeatures();
    m_collisionShapes.push_back(groundShape);

    btTransform groundTransform;
    groundTransform.setIdentity();
    groundTransform.setOrigin(btVector3(btScalar(_groundOrigin.x_), btScalar(_groundOrigin.y_), btScalar(_groundOrigin.z_)));


    {
        btScalar mass(0.);
        bool isDynamic = (mass != 0.f);
        btVector3 localInertia(0, 0, 0);
        if (isDynamic)
            groundShape->calculateLocalInertia(mass, localInertia);

        btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
        btRigidBody* body = new btRigidBody(rbInfo);

        body->setGravity(btVector3(0, GRAVITY_CONSTANT, 0));
        body->setFriction(btScalar(0.7)); // coefficient of friction = 0.7 (Almost equivalent to Aluminium)
        body->setRestitution(btScalar(0.)); //non-elastic, coefficient of restitution = 0
        body->setDamping(btScalar(0.9), btScalar(0.9)); //linear and angular damping

        m_dynamicsWorld->addRigidBody(body);
    }
}

void MultipleObjects::initPhysics() {
    btBoxShape* groundShape = new btBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));
    groundShape->initializePolyhedralFeatures();
    m_collisionShapes.push_back(groundShape);

    btTransform groundTransform;
    groundTransform.setIdentity();
    groundTransform.setOrigin(btVector3(0, -50, 0));

    {
        btScalar mass(0.);
        bool isDynamic = (mass != 0.f);
        btVector3 localInertia(0, 0, 0);

        if (isDynamic)
            groundShape->calculateLocalInertia(mass, localInertia);

        btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
        btRigidBody* body = new btRigidBody(rbInfo);

        m_dynamicsWorld->addRigidBody(body);
    }

    {
        btSphereShape* sphere = new btSphereShape(4);
        m_collisionShapes.push_back(sphere);

        btScalar mass(MASS_OF_MASSES);
        bool isDynamic = (mass != 0.f);

        btVector3 localInertia(10, 10, 0);
        if (isDynamic)
            sphere->calculateLocalInertia(mass, localInertia);

        btTransform t;
        t.setIdentity();
        t.setOrigin(btVector3(0, -3, 0));

        btDefaultMotionState* myMotionState = new btDefaultMotionState(t);
        btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, sphere, localInertia);
        btRigidBody* body = new btRigidBody(rbInfo);

        body->setRestitution(btScalar(1));
        m_dynamicsWorld->addRigidBody(body);

        //Add another Sphere
        btSphereShape* sphere2 = new btSphereShape(1);
        m_collisionShapes.push_back(sphere2);
        btScalar mass2(0.f);
        bool isDynamictwo = (mass2 != 0.f);
        btVector3 localInertia2(0, 0, 0);

        if (isDynamictwo)
            sphere->calculateLocalInertia(mass2, localInertia2);

        btTransform t2;
        t2.setIdentity();
        t2.setOrigin(btVector3(10, 5, 10));

        btDefaultMotionState* myMotionState2 = new btDefaultMotionState(t2);
        btRigidBody::btRigidBodyConstructionInfo rbInfotwo(mass2, myMotionState2, sphere2, localInertia);
        btRigidBody* bodytwo = new btRigidBody(rbInfotwo);
        m_dynamicsWorld->addRigidBody(bodytwo);

        btGeneric6DOFSpringConstraint* pGen6DOFSpring = new btGeneric6DOFSpringConstraint(*bodytwo, *body, t2, t, true);
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
        pGen6DOFSpring->setDamping(5, 0.3f);
        pGen6DOFSpring->setEquilibriumPoint();

        m_dynamicsWorld->addConstraint(pGen6DOFSpring, true);
    }
}

std::pair <btRigidBody*, btTransform> MultipleObjects::addMass(double x, double y, double z, double radius, char type, bool activate, double massValue) {
    //Adding sphere as mass
    btSphereShape* sphere = new btSphereShape(radius);
    btCylinderShape* armShape;
    m_collisionShapes.push_back(sphere);
    btTransform t;
    btRigidBody* body;
    btScalar mass = btScalar(massValue);

    std::pair<btRigidBody*, btTransform> massTransformPair;

    t.setIdentity();


    switch (type) {
        case 'f':
            mass = 0.;
        case 't':
            t.setOrigin(btVector3(btScalar(x), btScalar(y), btScalar(z)));
            t.getBasis().setEulerZYX(0, 0, 0);
            body = localCreateRigidBody(mass, t, sphere);
            massTransformPair = std::make_pair(body, t);
            break;
        case 'i':
            t.setOrigin(btVector3(btScalar(x), btScalar(y), btScalar(z)));
            t.getBasis().setEulerZYX(0, 0, 0);
            body = localCreateRigidBody(mass, t, sphere);
            massTransformPair = std::make_pair(body, t);
            massesToExclude.push_back(massTransformPair);
            break;
        case 'e':
            t.setOrigin(btVector3(btScalar(x), btScalar(y), btScalar(z)));
            t.getBasis().setEulerZYX(0, 0, 0);
            body = localCreateRigidBody(mass, t, sphere);
            massTransformPair = std::make_pair(body, t);
            massesToExclude.push_back(massTransformPair);
            endEffector = massTransformPair;
            break;
        case 'm':
            t.setOrigin(btVector3(btScalar(x), btScalar(y), btScalar(z)));
            t.getBasis().setEulerZYX(0, 0, 0);
            body = localCreateRigidBody(mass, t, sphere);
            massTransformPair = std::make_pair(body, t);
            massesToExclude.push_back(massTransformPair);
            break;
        case 'r':
            t.setOrigin(btVector3(btScalar(x), btScalar(y + ARM_LENGTH), btScalar(z)));
            t.getBasis().setEulerZYX(0, 0, 0);
            armShape = new btCylinderShape(btVector3(ARM_RADIUS, ARM_LENGTH, ARM_RADIUS));
            body = localCreateRigidBody(btScalar(mass * MASS_FACTOR_ARM), t, armShape);
            massTransformPair = std::make_pair(body, t);
            massesToExclude.push_back(massTransformPair);
            break;
        default:
            break;
    }


    masses.push_back(massTransformPair);

    if (activate)
        activate = DISABLE_DEACTIVATION;

    body->setActivationState(activate);
    body->setActivationState(DISABLE_DEACTIVATION);

    return std::make_pair(body, t);
}

void MultipleObjects::addSliderConstraint(btRigidBody* body1, btTransform t1, btRigidBody* body2, btTransform t2) {
    btSliderConstraint* pSlider = new btSliderConstraint(*body1, *body2, t1, t2, false);

    pSlider->setLowerAngLimit(0.0F);
    pSlider->setUpperAngLimit(0.0F);

    pSlider->setDbgDrawSize(btScalar(5.f));
    m_dynamicsWorld->addConstraint(pSlider, true);
}

void MultipleObjects::addConeTwistConstraint(btRigidBody* body1, btTransform t1, btRigidBody* body2, btTransform t2) {
    btConeTwistConstraint* coneTwist = new btConeTwistConstraint(*body1, *body2, t1, t2);
    coneTwist->setLimit(btScalar(SIMD_PI * 0.5 * 0.6f), btScalar(SIMD_PI * 0.5f), btScalar(SIMD_PI) * 0.8f, 1.0f);

    coneTwist->setDbgDrawSize(btScalar(5.f));
    m_dynamicsWorld->addConstraint(coneTwist, true);
}

void MultipleObjects::addPointToPointConstraint(btRigidBody* body1, btTransform t1, btRigidBody* body2, btTransform t2) {
    btVector3 pivotInA = body1->getCenterOfMassTransform().inverse().getOrigin(); //Finding center of body1
    btVector3 pivotInB = body2->getCenterOfMassTransform().inverse().getOrigin(); //Finding center of body2
    btVector3* tmp = new btVector3(pivotInA.x() - pivotInB.x(), pivotInA.y() - pivotInB.y(), pivotInA.z() - pivotInB.z());
    pivotInA = *tmp;
    tmp = new btVector3(0, 0, 0);
    pivotInB = *tmp;

    btPoint2PointConstraint* p2p = new btPoint2PointConstraint(*body1, *body2, pivotInA, pivotInB);

    p2p->setDbgDrawSize(btScalar(5.f));
    m_dynamicsWorld->addConstraint(p2p);

    pivotInA = body2->getCenterOfMassPosition();
    pivotInB = body1->getCenterOfMassPosition();
    p2p = new btPoint2PointConstraint(*body1, *body2, pivotInA, pivotInB);

    p2p->setDbgDrawSize(btScalar(5.f));
    m_dynamicsWorld->addConstraint(p2p);
}

/*
 * This is used to add the shoulder joint.
 * @param : body1 = shoulder mass
 * @param : body2 = upper arm of robot
 * @param : t1 = transform of body1
 * @param : t2 = transform of body2
 */
void MultipleObjects::addRobotBaseConstraint(btRigidBody* body1, btTransform t1, btRigidBody* body2, btTransform t2) {

    btVector3 pivotInA = body1->getCenterOfMassTransform().inverse().getOrigin(); //Finding center of body1
    btVector3 pivotInB = body2->getCenterOfMassTransform().inverse().getOrigin(); //Finding center of body2
    btVector3* tmp = new btVector3(pivotInA.x() - pivotInB.x(), pivotInA.y() - pivotInB.y(), pivotInA.z() - pivotInB.z());
    pivotInA = *tmp;
    tmp = new btVector3(0, 0, 0);
    pivotInB = *tmp;

    btTransform ctWorldTransform;
    ctWorldTransform.setIdentity();
    ctWorldTransform.setOrigin(pivotInB);
    btTransform transformInA = body1->getCenterOfMassTransform().inverse() * ctWorldTransform;
    btTransform transformInB = body2->getCenterOfMassTransform().inverse() * ctWorldTransform;
    tmp = new btVector3(0, 0, 0);
    transformInB.setOrigin(pivotInB);
    transformInA.setOrigin(pivotInA);

    btGeneric6DOFConstraint* hinge = new btGeneric6DOFConstraint(*body1, *body2, transformInA, transformInB, false);
    hinge->setDbgDrawSize(btScalar(5.f));
    hinge->setLimit(1, 0, 0);
    hinge->setLimit(2, 0, 0);
    hinge->setLimit(3, 0, 0);
    hinge->setLimit(4, 0, 0);
    hinge->setLimit(5, 0, 0); //-SIMD_HALF_PI / 2, SIMD_PI + SIMD_HALF_PI / 2);
    hinge->setLimit(6, 0, 0);

    for (int i = 0; i < 6; ++i) {
        hinge->setParam(BT_CONSTRAINT_STOP_CFM, 0, i);
        hinge->setParam(BT_CONSTRAINT_ERP, 0.8, i);
        hinge->getRotationalLimitMotor(i)->m_enableMotor = true;
        hinge->getRotationalLimitMotor(i)->m_targetVelocity = SIMD_INFINITY;
        hinge->getRotationalLimitMotor(i)->m_limitSoftness = btScalar(1.0);
        hinge->getRotationalLimitMotor(i)->m_normalCFM = btScalar(0.0);
        hinge->getRotationalLimitMotor(i)->m_maxMotorForce = SIMD_INFINITY;
    }


    hinge->setOverrideNumSolverIterations(SOLVER_ITERATIONS);

    hinge->enableFeedback(true);

    hinge->setBreakingImpulseThreshold(SIMD_INFINITY);


    robotBase = hinge;

    m_dynamicsWorld->addConstraint(hinge, true);
}

/*
 * This is used to add a robot joint.
 * @param : body1 = first arm segment
 * @param : body2 = second arm segment
 * @param : t1 = transform of body1
 * @param : t2 = transform of body2
 */
void MultipleObjects::addRobotJointConstraint(btRigidBody* body1, btTransform t1, btRigidBody* body2, btTransform t2) {

    btVector3 pivotInA = body1->getCenterOfMassTransform().inverse().getOrigin(); //Finding center of body1
    btVector3 pivotInB = body2->getCenterOfMassTransform().inverse().getOrigin(); //Finding center of body1
    btTransform local1, local2;
    btVector3 hingePos = btVector3(pivotInA.x() - pivotInB.x(), pivotInA.y() - pivotInB.y(), pivotInA.z() - pivotInB.z());
    hingePos /= 2;
    local1.setIdentity();
    local1.setOrigin(hingePos); //rotate point
    local2.setIdentity();
    local2 = body2->getWorldTransform().inverse() * body1->getWorldTransform() * local1;

    btHingeConstraint* hinge = new btHingeConstraint(*body1, *body2, local1, local2, false);
    hinge->setDbgDrawSize(btScalar(5.f));

    for (int i = 0; i < 6; ++i) {
        hinge->setParam(BT_CONSTRAINT_STOP_CFM, 0, i);
        hinge->setParam(BT_CONSTRAINT_ERP, 0.8, i);
    }
    
    
    
    hinge->setOverrideNumSolverIterations(SOLVER_ITERATIONS);

    hinge->setLimit(0, 0);

    hinge->setBreakingImpulseThreshold(SIMD_INFINITY);

    robotJoints.push_back(hinge);

    m_dynamicsWorld->addConstraint(hinge, true);
}

/*
 * This is used to add a constraint with a fixed distance between two bodies.
 * @param : body1 = first body
 * @param : body2 = second body
 * @param : t1 = transform of body1
 * @param : t2 = transform of body2
 */
void MultipleObjects::addFixedConstraint(btRigidBody* body1, btTransform t1, btRigidBody* body2, btTransform t2) {

    btVector3 pivotInA = body1->getCenterOfMassTransform().inverse().getOrigin(); //Finding center of body1
    btVector3 pivotInB = body2->getCenterOfMassTransform().inverse().getOrigin(); //Finding center of body2
    btVector3* tmp = new btVector3(pivotInA.x() - pivotInB.x(), pivotInA.y() - pivotInB.y(), pivotInA.z() - pivotInB.z());
    pivotInA = *tmp;
    tmp = new btVector3(0, 0, 0);
    pivotInB = *tmp;

    btTransform ctWorldTransform;
    ctWorldTransform.setIdentity();
    ctWorldTransform.setOrigin(pivotInB);
    btTransform transformInA = body1->getCenterOfMassTransform().inverse() * ctWorldTransform;
    btTransform transformInB = body2->getCenterOfMassTransform().inverse() * ctWorldTransform;
    tmp = new btVector3(0, 0, 0);
    transformInB.setOrigin(pivotInB);
    transformInA.setOrigin(pivotInA);

    btGeneric6DOFConstraint* fixed = new btGeneric6DOFConstraint(*body1, *body2, transformInA, transformInB, false);

    fixed->setDbgDrawSize(btScalar(5.f));
    fixed->setLimit(1, 0, 0);
    fixed->setLimit(2, 0, 0);
    fixed->setLimit(3, 0, 0);
    fixed->setLimit(4, 0, 0);
    fixed->setLimit(5, 0, 0);
    fixed->setLimit(6, 0, 0);

    fixed->setAngularLowerLimit(btVector3(0, 0, 0));
    fixed->setAngularUpperLimit(btVector3(0, 0, 0));

    for (int i = 1; i <= 6; ++i) {
        fixed->setParam(BT_CONSTRAINT_CFM, 0, i);
        fixed->setParam(BT_CONSTRAINT_ERP, 0.8, i);
    }

    fixed->setOverrideNumSolverIterations(SOLVER_ITERATIONS);
    fixed->setBreakingImpulseThreshold(SIMD_INFINITY);

    m_dynamicsWorld->addConstraint(fixed, true);
}

/*
 * This is used to add Spring constraint
 * @param : body1 = one side of the spring
 * @param : body2 = other side of the spring
 * @param : t1 = transform of body1
 * @param : t2 = transform of body2
 */
void MultipleObjects::addSpringConstraint(btRigidBody* body1, btTransform t1, btRigidBody* body2, btTransform t2) {
    btGeneric6DOFSpringConstraint* pGen6DOFSpring = new btGeneric6DOFSpringConstraint(*body1, *body2, t1, t2, true);

    pGen6DOFSpring->setLinearUpperLimit(upperLimitOfSpring);
    pGen6DOFSpring->setLinearLowerLimit(lowerLimitOfSpring);


    pGen6DOFSpring->setAngularLowerLimit(btVector3(0.f, 0.f, 0.f));
    pGen6DOFSpring->setAngularUpperLimit(btVector3(0.f, 0.f, 0.f));

    m_dynamicsWorld->addConstraint(pGen6DOFSpring, true);

    pGen6DOFSpring->setDbgDrawSize(btScalar(5.f));

    pGen6DOFSpring->setOverrideNumSolverIterations(SOLVER_ITERATIONS);
    pGen6DOFSpring->setBreakingImpulseThreshold(SIMD_INFINITY);

    double erp = dt * getSpringStiffness() / (dt * getSpringStiffness() + 1 / getSpringDamping());
    double cfm = 1 / (dt * getSpringStiffness() + 1 / getSpringDamping()) / dt;

    for (int i = 0; i < 6; ++i) {
        pGen6DOFSpring->enableSpring(i, true);
        pGen6DOFSpring->setStiffness(i, getSpringStiffness());
        pGen6DOFSpring->setDamping(i, getSpringDamping());
    }
    pGen6DOFSpring->setEquilibriumPoint();


    springs.push_back(pGen6DOFSpring);

    if (isCalledFromActiveSpring() && springAtEquilibrium.size() >= 1) {
        //search for the connection as it is called by Active Spring
        //and update stiffness as well as Resting Length.
        for (unsigned int act = 0; act < springAtEquilibrium.size(); act++) {

            if ((massRestingLength[getIndexOfActiveSpringForStiffness().first] == springAtEquilibrium[act].connectionFrom && massRestingLength[getIndexOfActiveSpringForStiffness().second] == springAtEquilibrium[act].connectionTo)
                    || (massRestingLength[getIndexOfActiveSpringForStiffness().second] == springAtEquilibrium[act].connectionFrom && massRestingLength[getIndexOfActiveSpringForStiffness().first] == springAtEquilibrium[act].connectionTo)) {
                //unset the function variable and just update stiffness at this position
                setCalledFromActiveSpring(false);
                springAtEquilibrium[act].stiffness = getSpringStiffness();
            }
        }
    } else {
        // add new instance for new spring
        springAtEquilibrium.push_back({t1.getOrigin(), t2.getOrigin(), getSpringStiffness()});
    }
}

void MultipleObjects::setUpperLimitOfSpring(btVector3 _uplimspring) {
    upperLimitOfSpring = _uplimspring;
}

void MultipleObjects::setLowerAngularLimitOfSpring(btVector3 _lowlimspring) {
    lowerLimitOfSpring = _lowlimspring;
}

void MultipleObjects::setUpperAngularLimitOfSpring(btVector3 _uplimspring) {
    upperAngularLimit = _uplimspring;
}

void MultipleObjects::setLowerLimitOfSpring(btVector3 _lowlimspring) {
    lowerAngularLimit = _lowlimspring;
}

void MultipleObjects::Interface() {
    setTexturing(true);
    setShadows(true);
    setCameraDistance(btScalar(SCALING * 20.));

    ///collision configuration contains default setup for memory, collision setup
    m_collisionConfiguration = new btDefaultCollisionConfiguration();

    ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
    m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
    m_broadphase = new btDbvtBroadphase();

    ///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
    btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
    m_solver = sol;
    m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
    m_dynamicsWorld->setGravity(btVector3(0, GRAVITY_CONSTANT, 0));
    m_dynamicsWorld->setDebugDrawer(&gDebugDrawer);
    btContactSolverInfo &solverInfo = m_dynamicsWorld->getSolverInfo();
    solverInfo.m_numIterations = SOLVER_ITERATIONS;


    if (skipMenu) {
        outputFile.open(outputFilePath);
        massesFile.open(massesFilePath);
        effectorFile.open(effectorFilePath);
        connectionMapFile.open(connectionMapFilePath);
        debugFile.open(debugFilePath);
        velocitiesFile.open(velocitiesFilePath);

        if (gotFloor) {
            addGround();
        }

        setMode(3);
        FreeMassSpringConnection(numberMasses);

        // Setting the flag for String random selection!!
        srand(time(NULL));
        engine.seed((unsigned int) time(NULL)); //initializing generator to January 1, 1970
        motorInput = readFileToDoubleVector(inputTrajectoryFilePath);
        motorInputIterator = motorInput.begin();
    } else {
        outputFile.open(outputFilePath);
        massesFile.open(massesFilePath);
        connectionMapFile.open(connectionMapFilePath);
        numberMasses = readNumberOfMassesFromMassesFile();

        std::string groundAddition;
        std::cout << "Do you want Ground in simulation(t/f)?" << std::endl;
        std::cin >> groundAddition;

        if (groundAddition == "t") {
            addGround();
        }

        // Setting the flag for String random selection!!
        srand(time(NULL));
        engine.seed((unsigned int) time(NULL)); //initializing generator to January 1, 1970

        int connectionType;

        if (numberMasses > 1) {
            while (true) {
                std::cout << "Which type of connection do you want?\n"
                        "Select the number:\n"
                        "=====================================\n"
                        "1.FeedForward Mass Spring Connection \n"
                        "2. Recurrent Mass Spring Connection  \n"
                        "3. Connection of your own \n"
                        << std::endl;

                std::cin >> connectionType;

                if (connectionType == 1) {
                    if (numberMasses > 0) {
                        setMode(1);
                        FeedForwardMassSpringConnection(numberMasses);
                        break;

                    }
                } else if (connectionType == 2) {
                    if (numberMasses > 1) {
                        setMode(2);
                        RNNConnection(numberMasses);
                        break;
                    }
                } else if (connectionType == 3) {
                    if (numberMasses > 1) {
                        setMode(3);
                        FreeMassSpringConnection(numberMasses);
                        break;
                    }
                } else
                    std::cout << "Try Again!!" << std::endl;
            }
        } else {
            //for one mass

            readNextMassFileTuple();
            ConsoleOutputforLengthSetting();
            std::pair<btRigidBody*, btTransform> Latest = Initialization(getDistributionRange().first, getDistributionRange().second); //min and max of distribution
            m_dynamicsWorld->addRigidBody(Latest.first);
        }
    }
}

void MultipleObjects::RNNConnection(int numberOfMasses) {
    for (int i = 0; i < numberOfMasses; i++) {
        std::pair<btRigidBody*, btTransform> NewMass = Initialization(getDistributionRange().first, getDistributionRange().second); //min and max of distribution

        for (int j = 0; j < i; j++) {
            if (masses.size() > 1) {
                //Do connection for all masses that were before

                addSpringConstraint(NewMass.first, NewMass.second, masses[j].first, masses[j].second);
            }
        }
    }

    ApplyForce();
}

void MultipleObjects::readNextConnectionMapFileTupel(void) {
    lastReadConnectionMapFileTuple.clear();

    while (connectionMapFile.good() && connectionMapFile.is_open()) {
        getline(connectionMapFile, lastReadConnectionMapFileLine);
        boost::algorithm::trim(lastReadConnectionMapFileLine);

        if (lastReadConnectionMapFileLine.size() > 0
                && lastReadConnectionMapFileLine[0] != CSV_COMMENT_CHARACTER
                && lastReadConnectionMapFileLine[0] != CSV_NEW_LINE_CHARACTER) {
            LOG4CXX_DEBUG(logger, "Connection Map file output: " << lastReadConnectionMapFileLine);
            strtk::parse(lastReadConnectionMapFileLine,
                    CSV_VALUE_SEPARATOR,
                    lastReadConnectionMapFileTuple);
            return;
        } else if (connectionMapFile.eof()) {
            connectionMapFile.close();

            return;
        }
    }

    std::cout << "[-] Connection Map file not found" << std::endl;
    exit(EXIT_FAILURE);
}

void MultipleObjects::readNextMassFileTuple(void) {
    lastReadMassFileTuple.clear();

    while (massesFile.good() && massesFile.is_open()) {
        getline(massesFile, lastReadMassFileLine);
        boost::algorithm::trim(lastReadMassFileLine);

        if (lastReadMassFileLine.size() > 0
                && lastReadMassFileLine[0] != CSV_COMMENT_CHARACTER
                && lastReadMassFileLine[0] != CSV_NEW_LINE_CHARACTER) {
            LOG4CXX_DEBUG(logger, "Masses file output: " << lastReadMassFileLine);
            strtk::parse(lastReadMassFileLine, CSV_VALUE_SEPARATOR, lastReadMassFileTuple);
            return;
        } else if (massesFile.eof()) {
            LOG4CXX_WARN(logger, "End of file reached. (One Possibility: Your entries of masses are less than the values found in file.)");
            exit(EXIT_SUCCESS);

            return;
        }
    }

    LOG4CXX_WARN(logger, "Masses file not found");
    exit(EXIT_FAILURE);
}

// if there is a compile time error and the vector of vectors ends with >> change it to > >

std::vector<std::vector<double> > MultipleObjects::readFileToDoubleVector(std::string path) {
    std::ifstream file;
    std::string line;
    std::vector<std::vector<double> > tuples;

    file.open(path);

    if (file.good() && file.is_open()) {
        while (!file.eof()) {
            getline(file, line);
            boost::algorithm::trim(line);

            if (line.size() > 0
                    && line[0] != CSV_COMMENT_CHARACTER
                    && line[0] != CSV_NEW_LINE_CHARACTER) {
                LOG4CXX_DEBUG(logger, "Read file line: " << line);
                std::vector<double> tuple;
                strtk::parse(line, CSV_VALUE_SEPARATOR, tuple);
                tuples.push_back(tuple);
            }
        }
    } else {

        LOG4CXX_WARN(logger, "File not found: " << path);
        exit(EXIT_FAILURE);
    }

    return tuples;
}

void MultipleObjects::ConsoleOutputforLengthSetting() {

    double conX, conY, conZ;

    conX = lastReadMassFileTuple[0];
    conY = lastReadMassFileTuple[1];
    conZ = lastReadMassFileTuple[2];

    setLengthOfConstraint({conX, conY, conZ});
}

void MultipleObjects::FreeMassSpringConnection(int numberOfMasses) {
    for (int i = 0; i < numberOfMasses; i++) {
        readNextMassFileTuple();
        ConsoleOutputforLengthSetting();
        std::pair<btRigidBody*, btTransform> Latest = Initialization(getDistributionRange().first, getDistributionRange().second); //min and max of distribution

        if (masses.size() >= 1) {
            FreeConnection(i, Latest);
        }
    }


    //Entering time-steps for Active Spring
    int timestepforActiveSpring;
    std::string quitTimeInput;

    if (mapOfActiveSpring.size() >= 1) {
        while (true) {
            std::cout << "Enter timeSteps for the activation of Active spring" << std::endl;
            std::cin >> timestepforActiveSpring;
            listOftimeStepforActiveSpring.push_back(timestepforActiveSpring);

            std::cout << "Done?" << std::endl;
            std::cin >> quitTimeInput;

            if (quitTimeInput == "y") {
                std::sort(listOftimeStepforActiveSpring.begin(), listOftimeStepforActiveSpring.end());

                break;
            }
        }
    }

    ApplyForce();
}

void MultipleObjects::FreeConnection(int _i, std::pair<btRigidBody*, btTransform> _Latest) {
    //check size to determine if there is any connection
    int _size = lastReadMassFileTuple.end() - (lastReadMassFileTuple.begin() + 3);
    LOG4CXX_DEBUG(logger, "size of tuple = " << _size);

    if (_size >= 0) {

        addConnection(_Latest, _i);
    }
}

void MultipleObjects::ANN(int NoOfHiddenNeuronsinEachLayer, int NoOfLayers, int NoOfOutputNeurons, int NoOfInputNeurons) {
}

double RandomFloat(float a, float b) {
    float random = ((float) rand()) / (float) RAND_MAX;
    float diff = b - a;
    float r = random * diff;

    return a + r;
}

btVector3 MultipleObjects::SubtractVector(const btVector3& v1, const btVector3 & v2) {

    return btVector3(
            v1.m_floats[0] - v2.m_floats[0],
            v1.m_floats[1] - v2.m_floats[1],
            v1.m_floats[2] - v2.m_floats[2]);
}

btVector3 MultipleObjects::LinearForceOfSpring(btVector3 _springCurrentLength, btVector3 _springNeutralLength, btScalar springStiffness) {
    btVector3 delta = SubtractVector(_springCurrentLength, _springNeutralLength);
    btVector3 linearForce = delta.operator *=(-springStiffness);

    return linearForce;
}

void MultipleObjects::NonLinearForceOfSpring(void) {
}

void MultipleObjects::setSpringStiffnessConnectedbyMass(int _massNumber, double stiffnessValue) {
    springAtEquilibrium[_massNumber].stiffness = stiffnessValue;
}

double MultipleObjects::getSpringStiffnessConnectedbyMass(int _massNumber) {
    return springAtEquilibrium[_massNumber].stiffness;
}

std::pair <btRigidBody*, btTransform> MultipleObjects::addBox(double height) {
    btBoxShape* box = new btBoxShape(btVector3(btScalar(0.5), btScalar(0.5), btScalar(0.5)));
    btVector3 inertia(0, 0, 0);

    btScalar mass(0.);

    //Adding a box on the screen
    m_collisionShapes.push_back(box);

    btTransform t;
    t.setIdentity();
    t.setOrigin(btVector3(10, height, 0));
    fixedPoints.push_back(btVector3(10, height, 0));

    btDefaultMotionState* myMotionState = new btDefaultMotionState(t);
    btRigidBody::btRigidBodyConstructionInfo info(mass, myMotionState, box, inertia); //motion state would actually be non-null in most real usages
    info.m_restitution = 1.3f;
    info.m_friction = 1.5f;
    btRigidBody* rb = new btRigidBody(info);
    m_dynamicsWorld->addRigidBody(rb);

    return std::make_pair(rb, t);
}

Triplet MultipleObjects::LengthOfSpringforFeedForward(double x, double y, double z) {

    lengthOfSpringforfeedforward = {x, y, z};
    return lengthOfSpringforfeedforward;
}

void MultipleObjects::FeedForwardMassSpringConnection(int NoOfMasses) {
    for (int i = 0; i < NoOfMasses; i++) {

        double height = WindowsDimensions - (5 * i);
        std::pair<btRigidBody*, btTransform> Latest = Initialization(getDistributionRange().first, getDistributionRange().second); //min and max of distribution


        //to make the motion horizontal(by fixing all the axes)
        setLowerAngularLimitOfSpring(btVector3(0, 0, 0));
        setUpperAngularLimitOfSpring(btVector3(0, 0, 0));

        std::pair <btRigidBody*, btTransform> joint = addBox(height);
        setUpperLimitOfSpring(btVector3(0., 0.5, 0.));
        setLowerLimitOfSpring(btVector3(0., -0.5, 0.));

        addSpringConstraint(Latest.first, Latest.second, joint.first, joint.second);
    }

    ApplyForce();
}

void MultipleObjects::ApplyForce() {
    if (skipMenu && !APPLY_FORCES_ON_MASSES_DEFAULT) {
        return;
    }

    std::string applicationOfForce;
    unsigned int massNumber;

    std::cout << "Do you want to apply force on any mass " << std::endl;
    std::cin >> applicationOfForce;

    if (applicationOfForce == "t") {
        while (true) {
            std::cout << "Enter mass number" << std::endl;
            std::cin >> massNumber;

            if (massNumber < masses.size()) {
                listOfMassesOntoWhichForceIsApplied.push_back(massNumber);
            } else {
                std::cout << "This mass is not present, Try again" << std::endl;
            }

            std::cout << "Do you want to add forces on other masses?" << std::endl;
            std::cin >> applicationOfForce;

            if (applicationOfForce == "f") {
                break;
            }
        }
    }

    int timestepforForces;
    std::string quitTimeInput;

    if (listOfMassesOntoWhichForceIsApplied.size() >= 1) {
        while (true) {
            std::cout << "Enter timeSteps for the application of forces on mass(es)" << std::endl;
            std::cin >> timestepforForces;

            listOfTimeStepforApplyingForceonMass.push_back(timestepforForces);

            std::cout << "Done?" << std::endl;
            std::cin >> quitTimeInput;

            if (quitTimeInput == "y") {
                std::sort(listOfTimeStepforApplyingForceonMass.begin(), listOfTimeStepforApplyingForceonMass.end());

                break;
            }
        }
    }
}

void MultipleObjects::FeedForwardConnection(void) {
}

void MultipleObjects::ApplyLinearForce(unsigned int massNumber, int _t) {
    std::cout << "force applied" << std::endl;

    if (masses.size() > massNumber) {

        double f1 = 2.11, f2 = 3.73, f3 = 4.33;
        double forceX = sin(2.0 * M_PI * f1 * _t) * sin(2. * M_PI * f2 * _t) * sin(2. * M_PI * f3 * _t);

        btVector3 relativeForce = btVector3(forceX, 0, 0);
        btMatrix3x3& objRotation = masses[massNumber].first->getWorldTransform().getBasis();

        btVector3 correctedForce = objRotation * relativeForce;
        masses[massNumber].first->applyCentralForce(correctedForce);
    }
}

void MultipleObjects::ApplyNonlinearForce(void) {
}

void MultipleObjects::Training(void) {
}

void MultipleObjects::addConnection(std::pair<btRigidBody*, btTransform> MassTransformPair, unsigned int massNumber) {
    btTransform frameInA, frameInB;

    unsigned int indexOfMassToConnectTo; //Connection to
    unsigned int connectionType;

    for (unsigned int i = 3; i < lastReadMassFileTuple.size(); i++) {
        indexOfMassToConnectTo = lastReadMassFileTuple[i];
        LOG4CXX_INFO(logger, "=========================");
        if (indexOfMassToConnectTo == 7 && massNumber == 8) {
            ;
        }

        LOG4CXX_INFO(logger, "Connect masses with the indices " << indexOfMassToConnectTo << " and " << massNumber);

        if (indexOfMassToConnectTo > (masses.size() - 1)) {
            LOG4CXX_WARN(logger, "The mass with the index " << indexOfMassToConnectTo << " does not exist in the masses file - please try again");
            continue;
        }

        //search for the type of connection
        readNextConnectionMapFileTupel();

        //check if there is a connection for the current mass number
        if (lastReadConnectionMapFileTuple[0] == massNumber and lastReadConnectionMapFileTuple[1] == indexOfMassToConnectTo) {
            connectionType = lastReadConnectionMapFileTuple[2];
            LOG4CXX_INFO(logger, "Connection type: " << connectionType);

            switch (connectionType) {
                case 1:
                {
                    /* TODO: remove commented code, once simulation is stable */
                    frameInA = btTransform::getIdentity();
                    //frameInA.setOrigin(btVector3(btScalar(10.), btScalar(0.), btScalar(0.)));

                    frameInA.setOrigin(btVector3(btScalar(0.), btScalar(0.), btScalar(0.)));
                    //btVector3 pivotInA = (*(MassTransformPair.first)).getCenterOfMassTransform().getOrigin();
                    //btVector3 pivotInB = (masses[indexOfMassToConnectTo].first)->getCenterOfMassTransform().getOrigin();
                    //frameInA.setOrigin(btVector3((pivotInA.x() - pivotInB.x())/2, (pivotInA.y() - pivotInB.y())/2, (pivotInA.z() - pivotInB.z())/2));
                    //frameInB = btTransform::getIdentity();
                    //frameInB.setOrigin(btVector3(pivotInB.x() - pivotInA.x(), pivotInB.y() - pivotInA.y(), pivotInB.z() - pivotInA.z()));
                    //frameInB.setOrigin(btVector3((pivotInB.x() - pivotInA.x()) / 2, (pivotInB.y() - pivotInA.y()) / 2, (pivotInB.z() - pivotInA.z()) / 2));
                    //frameInB.setOrigin(btVector3(btScalar(0.), btScalar(0.), btScalar(0.)));

                    btTransform inv = masses[indexOfMassToConnectTo].first->getCenterOfMassTransform().inverse();

                    btTransform globalFrameA = MassTransformPair.first->getCenterOfMassTransform() * frameInA;

                    frameInB = inv * globalFrameA;


                    addSpringConstraint(MassTransformPair.first, frameInA, masses[indexOfMassToConnectTo].first, frameInB);
                    setTypeOfconnection(1);

                    if (lastReadConnectionMapFileTuple[3] == 1) {
                        setActiveSpring({massNumber, indexOfMassToConnectTo, 1, 0.0}); // Here we are not interested in current length
                    }

                    break;
                }

                case 2:
                {
                    addPointToPointConstraint(MassTransformPair.first, MassTransformPair.second, masses[indexOfMassToConnectTo].first, masses[indexOfMassToConnectTo].second);
                    setTypeOfconnection(2);
                    break;
                }
                case 3:
                {
                    addSliderConstraint(MassTransformPair.first, MassTransformPair.second, masses[indexOfMassToConnectTo].first, masses[indexOfMassToConnectTo].second);
                    setTypeOfconnection(3);
                    break;
                }
                case 4:
                {
                    addConeTwistConstraint(MassTransformPair.first, MassTransformPair.second, masses[indexOfMassToConnectTo].first, masses[indexOfMassToConnectTo].second);
                    setTypeOfconnection(4);
                    break;
                }
                case 5:
                {
                    addFixedConstraint(MassTransformPair.first, MassTransformPair.second, masses[indexOfMassToConnectTo].first, masses[indexOfMassToConnectTo].second);
                    setTypeOfconnection(5);
                    break;

                }
                case 6:
                {
                    addRobotBaseConstraint(MassTransformPair.first, MassTransformPair.second, masses[indexOfMassToConnectTo].first, masses[indexOfMassToConnectTo].second);
                    setTypeOfconnection(6);
                    break;
                }
                case 7:
                {
                    addRobotJointConstraint(MassTransformPair.first, MassTransformPair.second, masses[indexOfMassToConnectTo].first, masses[indexOfMassToConnectTo].second);
                    setTypeOfconnection(7);
                    break;
                }
                default:
                {

                    break;
                }
            }

            //Creating a map of connection between different masses
            massMappingForFreeConnection.push_back({massNumber, indexOfMassToConnectTo, getTypeOfconnection(), 0.0});
        } else {
            LOG4CXX_WARN(logger, "No connection for given mass indices found");
        }
    }
}

void MultipleObjects::clientResetScene() {

    exitPhysics();
    Interface();
}

void MultipleObjects::setActiveSpring(ConnectionMap _mapOfactiveSpring) {

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

    std::cout << "What features you want in Active-Spring: \n"
            "Enter the code\n"
            "1 = Only Current Length\n"
            "2 = Only Stiffness\n"
            "3 = Resting Length\n"
            "4 = Current Length and Stiffness \n"
            "5 = Current Length and Resting Length \n"
            "6 = Stiffness and Resting Length"
            << std::endl;
    std::cin >> featureSelectedForActiveSpring;
    setCodeForFeatureSelectedInActiveSpring(featureSelectedForActiveSpring);

    mapOfActiveSpring.push_back(std::make_pair(getCodeForFeatureSelectedInActiveSpring(), _mapOfactiveSpring));
}

void MultipleObjects::UpdateFeaturesOfAllActiveSpring(void) {
    //Search for the index of spring connection in the pool of free-connection map
    int indexOfActiveSpring;
    double mass1, mass2;
    btVector3 currentPositionOfMass;

    for (unsigned int j = 0; j < mapOfActiveSpring.size(); j++) {
        for (unsigned int i = 0; i < massMappingForFreeConnection.size(); i++) {

            //Identify spring
            if (massMappingForFreeConnection[i].connectionFrom == mapOfActiveSpring[j].second.connectionFrom and massMappingForFreeConnection[i].connectionTo == mapOfActiveSpring[j].second.connectionTo) {
                indexOfActiveSpring = i;
                setCodeForFeatureSelectedInActiveSpring(mapOfActiveSpring[j].first);

                //Identify both masses
                mass1 = massMappingForFreeConnection[i].connectionFrom;
                mass2 = massMappingForFreeConnection[i].connectionTo;

                //Current Position of Spring
                currentPositionOfMass = masses[mass1].second.getOrigin();
            }
        }

        //get the current code
        switch (getCodeForFeatureSelectedInActiveSpring()) {
            case 1://Current Length
            {
                btScalar new_length = setCurrentLengthOfActiveSpring(currentPositionOfMass, mass1);
                massMappingForFreeConnection[indexOfActiveSpring].currentlength = new_length;
                break;
            }
            case 2://Stiffness
            {
                setSpringStiffnessForActiveSpring(getSpringStiffness(), indexOfActiveSpring, mass1, mass2);
                break;
            }
            case 3://Resting Length
            {
                btScalar new_length = setRestingLengthForActiveSpring(indexOfActiveSpring, mass1, mass2);
                massMappingForFreeConnection[indexOfActiveSpring].currentlength = new_length;
                break;
            }
            case 4://Current length and Stiffness
            {
                //First Stiffness is adjusted then the position is changed
                setSpringStiffnessForActiveSpring(getSpringStiffness(), indexOfActiveSpring, mass1, mass2);

                btScalar new_length = setCurrentLengthOfActiveSpring(currentPositionOfMass, mass1);
                massMappingForFreeConnection[indexOfActiveSpring].currentlength = new_length;

                break;
            }
            case 5:
            {//TODO Current Length and Resting Length
                break;
            }
            case 6:
            {//Stiffness and Resting Length
                //First Stiffness is adjusted then the position is changed
                setSpringStiffnessForActiveSpring(getSpringStiffness(), indexOfActiveSpring, mass1, mass2);

                btScalar new_length = setRestingLengthForActiveSpring(indexOfActiveSpring, mass1, mass2);
                massMappingForFreeConnection[indexOfActiveSpring].currentlength = new_length;

                break;
            }
            default:
                break;
        }
    }

}

btScalar MultipleObjects::setCurrentLengthOfActiveSpring(const btVector3& _currentLengthOfActiveSpring, int _activeSpringIndex) {
    btVector3 _newCurrentLengthOfActiveSpring = _currentLengthOfActiveSpring;

    _newCurrentLengthOfActiveSpring = btVector3((_newCurrentLengthOfActiveSpring.getX() / (timeStep / 10.0)), (_newCurrentLengthOfActiveSpring.getY() / (timeStep / 10.0)), (_newCurrentLengthOfActiveSpring.getZ() / (timeStep / 10.0)));

    //Calculation of new length
    btScalar new_length = btDistance(_currentLengthOfActiveSpring, massRestingLength[_activeSpringIndex]);

    //Update mass new position
    masses[_activeSpringIndex].second.setOrigin(btVector3(_newCurrentLengthOfActiveSpring));
    masses[_activeSpringIndex].first->getMotionState()->setWorldTransform(masses[_activeSpringIndex].second);
    masses[_activeSpringIndex].first->setCenterOfMassTransform(masses[_activeSpringIndex].second);
    masses[_activeSpringIndex].first->activate();

    std::cout << "UPDATE" << masses[_activeSpringIndex].second.getOrigin().getX() << " " << masses[_activeSpringIndex].second.getOrigin().getY() << "  " << masses[_activeSpringIndex].second.getOrigin().getZ() << std::endl;

    return new_length;
}

void MultipleObjects::setSpringStiffnessForActiveSpring(double _springStiffnessForActiveSpring, int _activeSpringIndex, double _massValue1, double _massValue2) {

    /*
     * For now the function that I have used is:
     * F = mg = Kx
     * x = displacement = (timeStep/10)
     */
    double mass1 = massValues[_massValue1];
    double mass2 = massValues[_massValue2];
    double differenceOfMasses = fabs(mass1 - mass2);

    int indexOfBody1 = massMappingForFreeConnection[_activeSpringIndex].connectionFrom;
    int indexOfBody2 = massMappingForFreeConnection[_activeSpringIndex].connectionTo;

    //Formula
    _springStiffnessForActiveSpring = (differenceOfMasses * fabs(m_dynamicsWorld->getGravity().getY())) / (timeStep / 10.0);

    setSpringStiffness(_springStiffnessForActiveSpring);
    setCalledFromActiveSpring(true);

    //store indexofBody1 and indexofBody2 somewhere and call in addSpringConnection
    setIndexOfActiveSpringForStiffness(std::make_pair(indexOfBody1, indexOfBody2));
    addSpringConstraint(masses[indexOfBody1].first, masses[indexOfBody1].second, masses[indexOfBody2].first, masses[indexOfBody2].second);

    std::cout << "SPRING = " << getSpringStiffness() << std::endl;
}

btScalar MultipleObjects::setRestingLengthForActiveSpring(int _activeSpringIndex, double _connectionTo, double _connectionFrom) {
    bool _isConnectionTo;

    //Calculate new resting length
    btVector3 old_restinglength = massRestingLength[_activeSpringIndex];
    btVector3 new_restinglength = old_restinglength / (timeStep / 10.0);

    //Update resting length
    massRestingLength[_activeSpringIndex] = new_restinglength;

    //Update SpringAtEquilibrium Vector
    //Search for the connection index in springAtEquilibrium
    if (springAtEquilibrium.size() >= 1) {
        for (unsigned int search = 0; search < springAtEquilibrium.size(); search++) {
            if ((springAtEquilibrium[search].connectionTo == massRestingLength[_connectionTo] and springAtEquilibrium[search].connectionFrom == massRestingLength[_connectionFrom])
                    || (springAtEquilibrium[search].connectionFrom == massRestingLength[_connectionTo] and springAtEquilibrium[search].connectionTo == massRestingLength[_connectionFrom])) {
                //Confirm w.r.t resting length
                std::cout << "HOGYA" << std::endl;
                if (old_restinglength == springAtEquilibrium[search].connectionTo)
                    _isConnectionTo = true;
                else
                    _isConnectionTo = false;
                //Update new resting length in SpringAtEquilibrium
                if (_isConnectionTo)//Update ConnectionTo of SpringAtEquilibrium
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
    btScalar new_length = btDistance(masses[_activeSpringIndex].second.getOrigin(), massRestingLength[_activeSpringIndex]);

    return new_length;
}

void MultipleObjects::exitPhysics() {
    //cleanup in the reverse order of creation/initialization
    for (int i = m_dynamicsWorld->getNumConstraints() - 1; i >= 0; i--) {
        btTypedConstraint* constraint = m_dynamicsWorld->getConstraint(i);
        m_dynamicsWorld->removeConstraint(constraint);
        delete constraint;
    }

    //remove the rigidbodies from the dynamics world and delete them
    int i;
    for (i = m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--) {
        btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
        btRigidBody* body = btRigidBody::upcast(obj);
        if (body && body->getMotionState()) {
            delete body->getMotionState();
        }
        m_dynamicsWorld->removeCollisionObject(obj);
        delete obj;
    }

    //delete collision shapes
    for (int j = 0; j < m_collisionShapes.size(); j++) {
        btCollisionShape* shape = m_collisionShapes[j];
        delete shape;
    }

    m_collisionShapes.clear();

    delete m_dynamicsWorld;
    delete m_solver;
    delete m_broadphase;
    delete m_dispatcher;
    delete m_collisionConfiguration;

    outputFile.close();
    massesFile.close();
    connectionMapFile.close();
}
