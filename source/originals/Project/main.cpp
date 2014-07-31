/*
 * main.cpp
 *
 *  Created on: Aug 5, 2013
 *      Author: marium
 */
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2007 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

//#include "Basic.h"
#include "MultipleObjects.h"
#include "GlutStuff.h"
#include "btBulletDynamicsCommon.h"
#include "LinearMath/btHashMap.h"



int main(int argc,char** argv)
{

	//BasicDemo ccdDemo;
	MultipleObjects ccdDemo;
	//ccdDemo.initPhysics();
	ccdDemo.Interface();
	//ccdDemo.RNNConnection(4);
	//ccdDemo.SimpsonsRule(6,0,0.2);
	//ccdDemo.Training();

	//ccdDemo.Initialization(0,1);


#ifdef CHECK_MEMORY_LEAKS
	ccdDemo.exitPhysics();
#else
	return glutmain(argc, argv,1024,600,"Mass Spring System",&ccdDemo);
#endif

	//default glut doesn't return from mainloop
	return 0;
}





