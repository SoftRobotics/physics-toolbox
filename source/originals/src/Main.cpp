/*
 * Main.cpp
 *
 *  Created on: Oct 26, 2013
 *      Author: Marium Zeeshan
 */

#include "Interface.h"

#include "GlutStuff.h"
#include "btBulletDynamicsCommon.h"
#include "LinearMath/btHashMap.h"



int main(int argc,char** argv)
{
	//TODO Logging

	Interface interface;
	//ccdDemo.initPhysics();
	interface.initPhysics();
	//ccdDemo.setGravity(-0.0);
	interface.setDebugMode(btIDebugDraw::DBG_DrawConstraints+btIDebugDraw::DBG_DrawConstraintLimits);


#ifdef CHECK_MEMORY_LEAKS
	interface.exitPhysics();
#else
	return glutmain(argc, argv,2000,1060,"Mass Spring System",&interface);
#endif
	return 0;
}





