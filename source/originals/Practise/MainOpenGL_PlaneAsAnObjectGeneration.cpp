#include <iostream>
#include <SDL.h>
#include <GL/gl.h>
#include <GL/glu.h>
//#include "camera.h"
//#include "functions.h"
#include <vector>
//#include "collision.h"
//#include "collisionplane.h"
#include "btBulletDynamicsCommon.h"


//camera cam;
GLUquadricObj* quad;

//just for now!!! make it globally just for the sake of simplicity
btDynamicsWorld* world;
btDispatcher* dispatcher;
btCollisionConfiguration* collisionconfig;
btBroadphaseInterface* broadphase;
btConstraintSolver* solver;
std::vector<btRigidBody*> bodies;


void init(float angle)
{
	collisionconfig = new btDefaultCollisionConfiguration();
	dispatcher = new btCollisionDispatcher(collisionconfig);
	broadphase = new btDbvtBroadphase();
	solver = new btSequentialImpulseConstraintSolver();
	world = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionconfig);
	world->setGravity(btVector3(0,-10,0));//gravity in the negative direction

	//add static objects
	//Body : ground
	btTransform t;
	t.setIdentity(); //no rotation at all
	t.setOrigin(btVector3(0,0,0));
	btStaticPlaneShape* plane  = new btStaticPlaneShape(btVector3(0,1,0),0);//set the plane at x-z position
	btMotionState* motion = new btDefaultMotionState(t);
	btRigidBody::btRigidBodyConstructionInfo info(0.0,motion,plane);
	btRigidBody* body = new btRigidBody(info);
	world->addRigidBody(body);
	bodies.push_back(body);





	glClearColor(0,0,0,1);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(angle,640.0/480,1,1000);
	glMatrixMode(GL_MODELVIEW);
	quad  = gluNewQuadric();
	glEnable(GL_DEPTH_TEST);
}

void display()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	//gluSphere(quad,1.0,30,30);
	glColor3f(1,0,0);
	gluCylinder(quad,1.0,1.0,5.0,30,1);
}

int main()
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

	delete dispatcher;
	delete collisionconfig;
	delete solver;
	delete world;
	delete broadphase;
SDL_Quit();
gluDeleteQuadric(quad);
}

