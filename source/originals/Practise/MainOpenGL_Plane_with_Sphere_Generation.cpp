#include <iostream>
#include <SDL.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <vector>
#include "btBulletDynamicsCommon.h"
#include <stdlib.h>
#include <stdio.h>
#include <tchar.h>
#include <windows.h>
#include <windowsx.h>

GLUquadricObj* quad;
//just for now!!! make it globally just for the sake of simplicity
btDynamicsWorld* world;
btDispatcher* dispatcher;
btCollisionConfiguration* collisionconfig;
btBroadphaseInterface* broadphase;
btConstraintSolver* solver;
std::vector<btRigidBody*> bodies;


btRigidBody* addSphereShape(float rad,float x,float y,float z,float mass){
	//add static objects 
	//Body : sphere
	btTransform t;
	t.setIdentity(); //no rotation at all
	t.setOrigin(btVector3(x,y,z));
	btSphereShape* sphere  = new btSphereShape(rad);//set the plane at x-z position
	btVector3 inertia(0,0,0);
	if(mass!= 0.0) //dynamic
		sphere->calculateLocalInertia(mass,inertia);

	btMotionState* motion = new btDefaultMotionState(t);
	btRigidBody::btRigidBodyConstructionInfo info(mass,motion,sphere,inertia);
	btRigidBody* body = new btRigidBody(info);
	world->addRigidBody(body);
	bodies.push_back(body);
	return body;

}


void init(float angle)
{
	collisionconfig = new btDefaultCollisionConfiguration();
	dispatcher = new btCollisionDispatcher(collisionconfig);
	broadphase = new btDbvtBroadphase();
	solver = new btSequentialImpulseConstraintSolver();
	world = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionconfig);
	world->setGravity(btVector3(0,-10,0));//gravity in the negative direction
	

	//Add plane
	btTransform t;
	t.setIdentity(); //no rotation at all
	t.setOrigin(btVector3(0,0,0));
	btStaticPlaneShape* plane  = new btStaticPlaneShape(btVector3(0,1,0),0);//set the plane at x-z position, 0 = original position
	btMotionState* motion = new btDefaultMotionState(t);
	btRigidBody::btRigidBodyConstructionInfo info(0.0,motion,plane);
	btRigidBody* body = new btRigidBody(info);
	world->addRigidBody(body);
	bodies.push_back(body);
	
	addSphereShape(1.0,0.0,20,0,1.0); //1Kg of mass (dynamic body)

	glClearColor(0,0,0,1);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(angle,640.0/480,1,1000);
	glMatrixMode(GL_MODELVIEW);
	quad  = gluNewQuadric();
	glEnable(GL_DEPTH_TEST);
}

	
void renderSphere(btRigidBody* sphere)
{
	if(sphere->getCollisionShape()->getShapeType()!= SPHERE_SHAPE_PROXYTYPE)	//sphere should have to be sphere_shape_proxytype
		return;

	glColor3f(1,0,0);
	float r = ((btSphereShape*) sphere->getCollisionShape())->getRadius();
	btTransform t;
	sphere->getMotionState()->getWorldTransform(t); //position of the current sphere
	float mat[16];//4X4
	t.getOpenGLMatrix(mat);
	glPushMatrix();
	glMultMatrixf(mat); //translation, rotation to the current sphere
	gluSphere(quad,r,20,20); //rendering sphere ; 20 and 20 is resolution
	glPopMatrix();

}

void renderPlane(btRigidBody* plane)
{
	if(plane->getCollisionShape()->getShapeType()!= STATIC_PLANE_PROXYTYPE)	//plane should have to be static
		return;

	glColor3f(0.8,0.8,0.8); //light grey
	
	btTransform t;
	plane->getMotionState()->getWorldTransform(t); //position of the current plane
	float mat[16];//4X4 matrix
	t.getOpenGLMatrix(mat);
	glPushMatrix();
	glMultMatrixf(mat); //translation, rotation to the current plane
	glBegin(GL_QUADS);
		glVertex3f(-1000,0,1000);
		glVertex3f(-1000,0,-1000);//left closer vertex
		glVertex3f(1000,0,-1000);
		glVertex3f(1000,0,1000);
	glEnd();
	glPopMatrix();

}

void display()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	
	//go thorugh all the objects in the scene
	for(int i=0;i<bodies.size();i++){
		if(bodies[i]->getCollisionShape()->getShapeType() == STATIC_PLANE_PROXYTYPE)
			renderPlane(bodies[i]);
		else if(bodies[i]->getCollisionShape()->getShapeType() == SPHERE_SHAPE_PROXYTYPE)
			renderSphere(bodies[i]);

	}

	
	
}

int _tmain() //used this because it can trasnlate to either main() or wmain()
{
	SDL_Init(SDL_INIT_EVERYTHING);
	SDL_SetVideoMode(640,480,32,SDL_OPENGL);
	Uint32 start;
	SDL_Event event;
	bool running = true;
	float angle = 50;
	init(angle);
	while(running)
	{
		start = SDL_GetTicks();
		while(SDL_PollEvent(&event))
		{
			switch(event.type)
			{
			case SDL_QUIT:
				running = false;
				break;
			case SDL_KEYDOWN:
				switch(event.key.keysym.sym)
				{
				case SDLK_ESCAPE:
					running = false;
					break;
				case SDLK_y:
					break;
				case SDLK_SPACE:
					break; 
				}
				break;
			case SDL_KEYUP:
				switch(event.key.keysym.sym)
				{
				}
				break;
			case SDL_MOUSEBUTTONDOWN:
				break;
			}
		}
		world->stepSimulation(1.0/60.0);
		display();
		SDL_GL_SwapBuffers();
		if(1000.0/60 >SDL_GetTicks()-start)
			SDL_Delay(1000.0/60 - (SDL_GetTicks() - start));
	}

	//render all the bodies
	for(int i=0;i<bodies.size();i++){
		world->removeCollisionObject(bodies[i]);
		btMotionState* motionState = bodies[i]->getMotionState();
		btCollisionShape* shape = bodies[i]->getCollisionShape();//due their dynamical allocation
		delete bodies[i];
		delete shape;
		delete motionState;
	}

	delete dispatcher;
	delete collisionconfig;
	delete solver;
	delete world;
	delete broadphase;
SDL_Quit();
gluDeleteQuadric(quad);

return 0;
}

