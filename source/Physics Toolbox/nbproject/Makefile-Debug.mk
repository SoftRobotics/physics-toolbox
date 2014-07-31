#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Environment
MKDIR=mkdir
CP=cp
GREP=grep
NM=nm
CCADMIN=CCadmin
RANLIB=ranlib
CC=gcc
CCC=g++
CXX=g++
FC=gfortran
AS=as

# Macros
CND_PLATFORM=GNU-Linux-x86
CND_DLIB_EXT=so
CND_CONF=Debug
CND_DISTDIR=dist
CND_BUILDDIR=build

# Include project Makefile
include Makefile

# Object Directory
OBJECTDIR=${CND_BUILDDIR}/${CND_CONF}/${CND_PLATFORM}

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/_ext/1506833081/btAxisSweep3.o \
	${OBJECTDIR}/_ext/1506833081/btBroadphaseProxy.o \
	${OBJECTDIR}/_ext/1506833081/btCollisionAlgorithm.o \
	${OBJECTDIR}/_ext/1506833081/btDbvt.o \
	${OBJECTDIR}/_ext/1506833081/btDbvtBroadphase.o \
	${OBJECTDIR}/_ext/1506833081/btDispatcher.o \
	${OBJECTDIR}/_ext/1506833081/btMultiSapBroadphase.o \
	${OBJECTDIR}/_ext/1506833081/btOverlappingPairCache.o \
	${OBJECTDIR}/_ext/1506833081/btQuantizedBvh.o \
	${OBJECTDIR}/_ext/1506833081/btSimpleBroadphase.o \
	${OBJECTDIR}/_ext/1230660710/SphereTriangleDetector.o \
	${OBJECTDIR}/_ext/1230660710/btActivatingCollisionAlgorithm.o \
	${OBJECTDIR}/_ext/1230660710/btBox2dBox2dCollisionAlgorithm.o \
	${OBJECTDIR}/_ext/1230660710/btBoxBoxCollisionAlgorithm.o \
	${OBJECTDIR}/_ext/1230660710/btBoxBoxDetector.o \
	${OBJECTDIR}/_ext/1230660710/btCollisionDispatcher.o \
	${OBJECTDIR}/_ext/1230660710/btCollisionObject.o \
	${OBJECTDIR}/_ext/1230660710/btCollisionWorld.o \
	${OBJECTDIR}/_ext/1230660710/btCompoundCollisionAlgorithm.o \
	${OBJECTDIR}/_ext/1230660710/btCompoundCompoundCollisionAlgorithm.o \
	${OBJECTDIR}/_ext/1230660710/btConvex2dConvex2dAlgorithm.o \
	${OBJECTDIR}/_ext/1230660710/btConvexConcaveCollisionAlgorithm.o \
	${OBJECTDIR}/_ext/1230660710/btConvexConvexAlgorithm.o \
	${OBJECTDIR}/_ext/1230660710/btConvexPlaneCollisionAlgorithm.o \
	${OBJECTDIR}/_ext/1230660710/btDefaultCollisionConfiguration.o \
	${OBJECTDIR}/_ext/1230660710/btEmptyCollisionAlgorithm.o \
	${OBJECTDIR}/_ext/1230660710/btGhostObject.o \
	${OBJECTDIR}/_ext/1230660710/btHashedSimplePairCache.o \
	${OBJECTDIR}/_ext/1230660710/btInternalEdgeUtility.o \
	${OBJECTDIR}/_ext/1230660710/btManifoldResult.o \
	${OBJECTDIR}/_ext/1230660710/btSimulationIslandManager.o \
	${OBJECTDIR}/_ext/1230660710/btSphereBoxCollisionAlgorithm.o \
	${OBJECTDIR}/_ext/1230660710/btSphereSphereCollisionAlgorithm.o \
	${OBJECTDIR}/_ext/1230660710/btSphereTriangleCollisionAlgorithm.o \
	${OBJECTDIR}/_ext/1230660710/btUnionFind.o \
	${OBJECTDIR}/_ext/1463567378/btBox2dShape.o \
	${OBJECTDIR}/_ext/1463567378/btBoxShape.o \
	${OBJECTDIR}/_ext/1463567378/btBvhTriangleMeshShape.o \
	${OBJECTDIR}/_ext/1463567378/btCapsuleShape.o \
	${OBJECTDIR}/_ext/1463567378/btCollisionShape.o \
	${OBJECTDIR}/_ext/1463567378/btCompoundShape.o \
	${OBJECTDIR}/_ext/1463567378/btConcaveShape.o \
	${OBJECTDIR}/_ext/1463567378/btConeShape.o \
	${OBJECTDIR}/_ext/1463567378/btConvex2dShape.o \
	${OBJECTDIR}/_ext/1463567378/btConvexHullShape.o \
	${OBJECTDIR}/_ext/1463567378/btConvexInternalShape.o \
	${OBJECTDIR}/_ext/1463567378/btConvexPointCloudShape.o \
	${OBJECTDIR}/_ext/1463567378/btConvexPolyhedron.o \
	${OBJECTDIR}/_ext/1463567378/btConvexShape.o \
	${OBJECTDIR}/_ext/1463567378/btConvexTriangleMeshShape.o \
	${OBJECTDIR}/_ext/1463567378/btCylinderShape.o \
	${OBJECTDIR}/_ext/1463567378/btEmptyShape.o \
	${OBJECTDIR}/_ext/1463567378/btHeightfieldTerrainShape.o \
	${OBJECTDIR}/_ext/1463567378/btMinkowskiSumShape.o \
	${OBJECTDIR}/_ext/1463567378/btMultiSphereShape.o \
	${OBJECTDIR}/_ext/1463567378/btMultimaterialTriangleMeshShape.o \
	${OBJECTDIR}/_ext/1463567378/btOptimizedBvh.o \
	${OBJECTDIR}/_ext/1463567378/btPolyhedralConvexShape.o \
	${OBJECTDIR}/_ext/1463567378/btScaledBvhTriangleMeshShape.o \
	${OBJECTDIR}/_ext/1463567378/btShapeHull.o \
	${OBJECTDIR}/_ext/1463567378/btSphereShape.o \
	${OBJECTDIR}/_ext/1463567378/btStaticPlaneShape.o \
	${OBJECTDIR}/_ext/1463567378/btStridingMeshInterface.o \
	${OBJECTDIR}/_ext/1463567378/btTetrahedronShape.o \
	${OBJECTDIR}/_ext/1463567378/btTriangleBuffer.o \
	${OBJECTDIR}/_ext/1463567378/btTriangleCallback.o \
	${OBJECTDIR}/_ext/1463567378/btTriangleIndexVertexArray.o \
	${OBJECTDIR}/_ext/1463567378/btTriangleIndexVertexMaterialArray.o \
	${OBJECTDIR}/_ext/1463567378/btTriangleMesh.o \
	${OBJECTDIR}/_ext/1463567378/btTriangleMeshShape.o \
	${OBJECTDIR}/_ext/1463567378/btUniformScalingShape.o \
	${OBJECTDIR}/_ext/1919507387/btContactProcessing.o \
	${OBJECTDIR}/_ext/1919507387/btGImpactBvh.o \
	${OBJECTDIR}/_ext/1919507387/btGImpactCollisionAlgorithm.o \
	${OBJECTDIR}/_ext/1919507387/btGImpactQuantizedBvh.o \
	${OBJECTDIR}/_ext/1919507387/btGImpactShape.o \
	${OBJECTDIR}/_ext/1919507387/btGenericPoolAllocator.o \
	${OBJECTDIR}/_ext/1919507387/btTriangleShapeEx.o \
	${OBJECTDIR}/_ext/1919507387/gim_box_set.o \
	${OBJECTDIR}/_ext/1919507387/gim_contact.o \
	${OBJECTDIR}/_ext/1919507387/gim_memory.o \
	${OBJECTDIR}/_ext/1919507387/gim_tri_collision.o \
	${OBJECTDIR}/_ext/1196390564/btContinuousConvexCollision.o \
	${OBJECTDIR}/_ext/1196390564/btConvexCast.o \
	${OBJECTDIR}/_ext/1196390564/btGjkConvexCast.o \
	${OBJECTDIR}/_ext/1196390564/btGjkEpa2.o \
	${OBJECTDIR}/_ext/1196390564/btGjkEpaPenetrationDepthSolver.o \
	${OBJECTDIR}/_ext/1196390564/btGjkPairDetector.o \
	${OBJECTDIR}/_ext/1196390564/btMinkowskiPenetrationDepthSolver.o \
	${OBJECTDIR}/_ext/1196390564/btPersistentManifold.o \
	${OBJECTDIR}/_ext/1196390564/btPolyhedralContactClipping.o \
	${OBJECTDIR}/_ext/1196390564/btRaycastCallback.o \
	${OBJECTDIR}/_ext/1196390564/btSubSimplexConvexCast.o \
	${OBJECTDIR}/_ext/1196390564/btVoronoiSimplexSolver.o \
	${OBJECTDIR}/_ext/1145006819/btKinematicCharacterController.o \
	${OBJECTDIR}/_ext/472263850/btConeTwistConstraint.o \
	${OBJECTDIR}/_ext/472263850/btContactConstraint.o \
	${OBJECTDIR}/_ext/472263850/btFixedConstraint.o \
	${OBJECTDIR}/_ext/472263850/btGearConstraint.o \
	${OBJECTDIR}/_ext/472263850/btGeneric6DofConstraint.o \
	${OBJECTDIR}/_ext/472263850/btGeneric6DofSpringConstraint.o \
	${OBJECTDIR}/_ext/472263850/btHinge2Constraint.o \
	${OBJECTDIR}/_ext/472263850/btHingeConstraint.o \
	${OBJECTDIR}/_ext/472263850/btPoint2PointConstraint.o \
	${OBJECTDIR}/_ext/472263850/btSequentialImpulseConstraintSolver.o \
	${OBJECTDIR}/_ext/472263850/btSliderConstraint.o \
	${OBJECTDIR}/_ext/472263850/btSolve2LinearConstraint.o \
	${OBJECTDIR}/_ext/472263850/btTypedConstraint.o \
	${OBJECTDIR}/_ext/472263850/btUniversalConstraint.o \
	${OBJECTDIR}/_ext/1202644710/Bullet-C-API.o \
	${OBJECTDIR}/_ext/1202644710/btDiscreteDynamicsWorld.o \
	${OBJECTDIR}/_ext/1202644710/btRigidBody.o \
	${OBJECTDIR}/_ext/1202644710/btSimpleDynamicsWorld.o \
	${OBJECTDIR}/_ext/818275416/btMultiBody.o \
	${OBJECTDIR}/_ext/818275416/btMultiBodyConstraint.o \
	${OBJECTDIR}/_ext/818275416/btMultiBodyConstraintSolver.o \
	${OBJECTDIR}/_ext/818275416/btMultiBodyDynamicsWorld.o \
	${OBJECTDIR}/_ext/818275416/btMultiBodyJointLimitConstraint.o \
	${OBJECTDIR}/_ext/818275416/btMultiBodyJointMotor.o \
	${OBJECTDIR}/_ext/818275416/btMultiBodyPoint2Point.o \
	${OBJECTDIR}/_ext/1317251118/btDantzigLCP.o \
	${OBJECTDIR}/_ext/1317251118/btMLCPSolver.o \
	${OBJECTDIR}/_ext/1780737382/btRaycastVehicle.o \
	${OBJECTDIR}/_ext/1780737382/btWheelInfo.o \
	${OBJECTDIR}/_ext/1815990081/btDefaultSoftBodySolver.o \
	${OBJECTDIR}/_ext/1815990081/btSoftBody.o \
	${OBJECTDIR}/_ext/1815990081/btSoftBodyConcaveCollisionAlgorithm.o \
	${OBJECTDIR}/_ext/1815990081/btSoftBodyHelpers.o \
	${OBJECTDIR}/_ext/1815990081/btSoftBodyRigidBodyCollisionConfiguration.o \
	${OBJECTDIR}/_ext/1815990081/btSoftRigidCollisionAlgorithm.o \
	${OBJECTDIR}/_ext/1815990081/btSoftRigidDynamicsWorld.o \
	${OBJECTDIR}/_ext/1815990081/btSoftSoftCollisionAlgorithm.o \
	${OBJECTDIR}/_ext/1130245248/btAlignedAllocator.o \
	${OBJECTDIR}/_ext/1130245248/btConvexHull.o \
	${OBJECTDIR}/_ext/1130245248/btConvexHullComputer.o \
	${OBJECTDIR}/_ext/1130245248/btGeometryUtil.o \
	${OBJECTDIR}/_ext/1130245248/btPolarDecomposition.o \
	${OBJECTDIR}/_ext/1130245248/btQuickprof.o \
	${OBJECTDIR}/_ext/1130245248/btSerializer.o \
	${OBJECTDIR}/_ext/1130245248/btVector3.o \
	${OBJECTDIR}/_ext/1360937237/MultipleObjects.o \
	${OBJECTDIR}/_ext/965943543/DemoApplication.o \
	${OBJECTDIR}/_ext/965943543/GLDebugDrawer.o \
	${OBJECTDIR}/_ext/965943543/GLDebugFont.o \
	${OBJECTDIR}/_ext/965943543/GL_DialogDynamicsWorld.o \
	${OBJECTDIR}/_ext/965943543/GL_DialogWindow.o \
	${OBJECTDIR}/_ext/965943543/GL_ShapeDrawer.o \
	${OBJECTDIR}/_ext/965943543/GL_Simplex1to4.o \
	${OBJECTDIR}/_ext/965943543/GlutDemoApplication.o \
	${OBJECTDIR}/_ext/965943543/GlutStuff.o \
	${OBJECTDIR}/_ext/965943543/RenderTexture.o \
	${OBJECTDIR}/_ext/965943543/Win32AppMain.o \
	${OBJECTDIR}/_ext/965943543/Win32DemoApplication.o \
	${OBJECTDIR}/_ext/965943543/stb_image.o \
	${OBJECTDIR}/_ext/1360937237/main.o


# C Compiler Flags
CFLAGS=

# CC Compiler Flags
CCFLAGS=
CXXFLAGS=

# Fortran Compiler Flags
FFLAGS=

# Assembler Flags
ASFLAGS=

# Link Libraries and Options
LDLIBSOPTIONS=-lGL -lGLU -lglut `pkg-config --libs liblog4cxx`  

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS}
	"${MAKE}"  -f nbproject/Makefile-${CND_CONF}.mk ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/physics_toolbox

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/physics_toolbox: ${OBJECTFILES}
	${MKDIR} -p ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}
	${LINK.cc} -o ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/physics_toolbox ${OBJECTFILES} ${LDLIBSOPTIONS}

${OBJECTDIR}/_ext/1506833081/btAxisSweep3.o: ../src/Bullet/BulletCollision/BroadphaseCollision/btAxisSweep3.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1506833081
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1506833081/btAxisSweep3.o ../src/Bullet/BulletCollision/BroadphaseCollision/btAxisSweep3.cpp

${OBJECTDIR}/_ext/1506833081/btBroadphaseProxy.o: ../src/Bullet/BulletCollision/BroadphaseCollision/btBroadphaseProxy.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1506833081
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1506833081/btBroadphaseProxy.o ../src/Bullet/BulletCollision/BroadphaseCollision/btBroadphaseProxy.cpp

${OBJECTDIR}/_ext/1506833081/btCollisionAlgorithm.o: ../src/Bullet/BulletCollision/BroadphaseCollision/btCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1506833081
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1506833081/btCollisionAlgorithm.o ../src/Bullet/BulletCollision/BroadphaseCollision/btCollisionAlgorithm.cpp

${OBJECTDIR}/_ext/1506833081/btDbvt.o: ../src/Bullet/BulletCollision/BroadphaseCollision/btDbvt.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1506833081
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1506833081/btDbvt.o ../src/Bullet/BulletCollision/BroadphaseCollision/btDbvt.cpp

${OBJECTDIR}/_ext/1506833081/btDbvtBroadphase.o: ../src/Bullet/BulletCollision/BroadphaseCollision/btDbvtBroadphase.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1506833081
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1506833081/btDbvtBroadphase.o ../src/Bullet/BulletCollision/BroadphaseCollision/btDbvtBroadphase.cpp

${OBJECTDIR}/_ext/1506833081/btDispatcher.o: ../src/Bullet/BulletCollision/BroadphaseCollision/btDispatcher.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1506833081
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1506833081/btDispatcher.o ../src/Bullet/BulletCollision/BroadphaseCollision/btDispatcher.cpp

${OBJECTDIR}/_ext/1506833081/btMultiSapBroadphase.o: ../src/Bullet/BulletCollision/BroadphaseCollision/btMultiSapBroadphase.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1506833081
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1506833081/btMultiSapBroadphase.o ../src/Bullet/BulletCollision/BroadphaseCollision/btMultiSapBroadphase.cpp

${OBJECTDIR}/_ext/1506833081/btOverlappingPairCache.o: ../src/Bullet/BulletCollision/BroadphaseCollision/btOverlappingPairCache.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1506833081
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1506833081/btOverlappingPairCache.o ../src/Bullet/BulletCollision/BroadphaseCollision/btOverlappingPairCache.cpp

${OBJECTDIR}/_ext/1506833081/btQuantizedBvh.o: ../src/Bullet/BulletCollision/BroadphaseCollision/btQuantizedBvh.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1506833081
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1506833081/btQuantizedBvh.o ../src/Bullet/BulletCollision/BroadphaseCollision/btQuantizedBvh.cpp

${OBJECTDIR}/_ext/1506833081/btSimpleBroadphase.o: ../src/Bullet/BulletCollision/BroadphaseCollision/btSimpleBroadphase.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1506833081
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1506833081/btSimpleBroadphase.o ../src/Bullet/BulletCollision/BroadphaseCollision/btSimpleBroadphase.cpp

${OBJECTDIR}/_ext/1230660710/SphereTriangleDetector.o: ../src/Bullet/BulletCollision/CollisionDispatch/SphereTriangleDetector.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1230660710
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1230660710/SphereTriangleDetector.o ../src/Bullet/BulletCollision/CollisionDispatch/SphereTriangleDetector.cpp

${OBJECTDIR}/_ext/1230660710/btActivatingCollisionAlgorithm.o: ../src/Bullet/BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1230660710
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1230660710/btActivatingCollisionAlgorithm.o ../src/Bullet/BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.cpp

${OBJECTDIR}/_ext/1230660710/btBox2dBox2dCollisionAlgorithm.o: ../src/Bullet/BulletCollision/CollisionDispatch/btBox2dBox2dCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1230660710
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1230660710/btBox2dBox2dCollisionAlgorithm.o ../src/Bullet/BulletCollision/CollisionDispatch/btBox2dBox2dCollisionAlgorithm.cpp

${OBJECTDIR}/_ext/1230660710/btBoxBoxCollisionAlgorithm.o: ../src/Bullet/BulletCollision/CollisionDispatch/btBoxBoxCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1230660710
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1230660710/btBoxBoxCollisionAlgorithm.o ../src/Bullet/BulletCollision/CollisionDispatch/btBoxBoxCollisionAlgorithm.cpp

${OBJECTDIR}/_ext/1230660710/btBoxBoxDetector.o: ../src/Bullet/BulletCollision/CollisionDispatch/btBoxBoxDetector.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1230660710
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1230660710/btBoxBoxDetector.o ../src/Bullet/BulletCollision/CollisionDispatch/btBoxBoxDetector.cpp

${OBJECTDIR}/_ext/1230660710/btCollisionDispatcher.o: ../src/Bullet/BulletCollision/CollisionDispatch/btCollisionDispatcher.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1230660710
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1230660710/btCollisionDispatcher.o ../src/Bullet/BulletCollision/CollisionDispatch/btCollisionDispatcher.cpp

${OBJECTDIR}/_ext/1230660710/btCollisionObject.o: ../src/Bullet/BulletCollision/CollisionDispatch/btCollisionObject.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1230660710
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1230660710/btCollisionObject.o ../src/Bullet/BulletCollision/CollisionDispatch/btCollisionObject.cpp

${OBJECTDIR}/_ext/1230660710/btCollisionWorld.o: ../src/Bullet/BulletCollision/CollisionDispatch/btCollisionWorld.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1230660710
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1230660710/btCollisionWorld.o ../src/Bullet/BulletCollision/CollisionDispatch/btCollisionWorld.cpp

${OBJECTDIR}/_ext/1230660710/btCompoundCollisionAlgorithm.o: ../src/Bullet/BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1230660710
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1230660710/btCompoundCollisionAlgorithm.o ../src/Bullet/BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.cpp

${OBJECTDIR}/_ext/1230660710/btCompoundCompoundCollisionAlgorithm.o: ../src/Bullet/BulletCollision/CollisionDispatch/btCompoundCompoundCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1230660710
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1230660710/btCompoundCompoundCollisionAlgorithm.o ../src/Bullet/BulletCollision/CollisionDispatch/btCompoundCompoundCollisionAlgorithm.cpp

${OBJECTDIR}/_ext/1230660710/btConvex2dConvex2dAlgorithm.o: ../src/Bullet/BulletCollision/CollisionDispatch/btConvex2dConvex2dAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1230660710
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1230660710/btConvex2dConvex2dAlgorithm.o ../src/Bullet/BulletCollision/CollisionDispatch/btConvex2dConvex2dAlgorithm.cpp

${OBJECTDIR}/_ext/1230660710/btConvexConcaveCollisionAlgorithm.o: ../src/Bullet/BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1230660710
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1230660710/btConvexConcaveCollisionAlgorithm.o ../src/Bullet/BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.cpp

${OBJECTDIR}/_ext/1230660710/btConvexConvexAlgorithm.o: ../src/Bullet/BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1230660710
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1230660710/btConvexConvexAlgorithm.o ../src/Bullet/BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.cpp

${OBJECTDIR}/_ext/1230660710/btConvexPlaneCollisionAlgorithm.o: ../src/Bullet/BulletCollision/CollisionDispatch/btConvexPlaneCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1230660710
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1230660710/btConvexPlaneCollisionAlgorithm.o ../src/Bullet/BulletCollision/CollisionDispatch/btConvexPlaneCollisionAlgorithm.cpp

${OBJECTDIR}/_ext/1230660710/btDefaultCollisionConfiguration.o: ../src/Bullet/BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1230660710
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1230660710/btDefaultCollisionConfiguration.o ../src/Bullet/BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.cpp

${OBJECTDIR}/_ext/1230660710/btEmptyCollisionAlgorithm.o: ../src/Bullet/BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1230660710
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1230660710/btEmptyCollisionAlgorithm.o ../src/Bullet/BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.cpp

${OBJECTDIR}/_ext/1230660710/btGhostObject.o: ../src/Bullet/BulletCollision/CollisionDispatch/btGhostObject.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1230660710
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1230660710/btGhostObject.o ../src/Bullet/BulletCollision/CollisionDispatch/btGhostObject.cpp

${OBJECTDIR}/_ext/1230660710/btHashedSimplePairCache.o: ../src/Bullet/BulletCollision/CollisionDispatch/btHashedSimplePairCache.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1230660710
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1230660710/btHashedSimplePairCache.o ../src/Bullet/BulletCollision/CollisionDispatch/btHashedSimplePairCache.cpp

${OBJECTDIR}/_ext/1230660710/btInternalEdgeUtility.o: ../src/Bullet/BulletCollision/CollisionDispatch/btInternalEdgeUtility.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1230660710
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1230660710/btInternalEdgeUtility.o ../src/Bullet/BulletCollision/CollisionDispatch/btInternalEdgeUtility.cpp

${OBJECTDIR}/_ext/1230660710/btManifoldResult.o: ../src/Bullet/BulletCollision/CollisionDispatch/btManifoldResult.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1230660710
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1230660710/btManifoldResult.o ../src/Bullet/BulletCollision/CollisionDispatch/btManifoldResult.cpp

${OBJECTDIR}/_ext/1230660710/btSimulationIslandManager.o: ../src/Bullet/BulletCollision/CollisionDispatch/btSimulationIslandManager.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1230660710
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1230660710/btSimulationIslandManager.o ../src/Bullet/BulletCollision/CollisionDispatch/btSimulationIslandManager.cpp

${OBJECTDIR}/_ext/1230660710/btSphereBoxCollisionAlgorithm.o: ../src/Bullet/BulletCollision/CollisionDispatch/btSphereBoxCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1230660710
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1230660710/btSphereBoxCollisionAlgorithm.o ../src/Bullet/BulletCollision/CollisionDispatch/btSphereBoxCollisionAlgorithm.cpp

${OBJECTDIR}/_ext/1230660710/btSphereSphereCollisionAlgorithm.o: ../src/Bullet/BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1230660710
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1230660710/btSphereSphereCollisionAlgorithm.o ../src/Bullet/BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.cpp

${OBJECTDIR}/_ext/1230660710/btSphereTriangleCollisionAlgorithm.o: ../src/Bullet/BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1230660710
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1230660710/btSphereTriangleCollisionAlgorithm.o ../src/Bullet/BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.cpp

${OBJECTDIR}/_ext/1230660710/btUnionFind.o: ../src/Bullet/BulletCollision/CollisionDispatch/btUnionFind.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1230660710
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1230660710/btUnionFind.o ../src/Bullet/BulletCollision/CollisionDispatch/btUnionFind.cpp

${OBJECTDIR}/_ext/1463567378/btBox2dShape.o: ../src/Bullet/BulletCollision/CollisionShapes/btBox2dShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1463567378
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1463567378/btBox2dShape.o ../src/Bullet/BulletCollision/CollisionShapes/btBox2dShape.cpp

${OBJECTDIR}/_ext/1463567378/btBoxShape.o: ../src/Bullet/BulletCollision/CollisionShapes/btBoxShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1463567378
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1463567378/btBoxShape.o ../src/Bullet/BulletCollision/CollisionShapes/btBoxShape.cpp

${OBJECTDIR}/_ext/1463567378/btBvhTriangleMeshShape.o: ../src/Bullet/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1463567378
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1463567378/btBvhTriangleMeshShape.o ../src/Bullet/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.cpp

${OBJECTDIR}/_ext/1463567378/btCapsuleShape.o: ../src/Bullet/BulletCollision/CollisionShapes/btCapsuleShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1463567378
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1463567378/btCapsuleShape.o ../src/Bullet/BulletCollision/CollisionShapes/btCapsuleShape.cpp

${OBJECTDIR}/_ext/1463567378/btCollisionShape.o: ../src/Bullet/BulletCollision/CollisionShapes/btCollisionShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1463567378
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1463567378/btCollisionShape.o ../src/Bullet/BulletCollision/CollisionShapes/btCollisionShape.cpp

${OBJECTDIR}/_ext/1463567378/btCompoundShape.o: ../src/Bullet/BulletCollision/CollisionShapes/btCompoundShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1463567378
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1463567378/btCompoundShape.o ../src/Bullet/BulletCollision/CollisionShapes/btCompoundShape.cpp

${OBJECTDIR}/_ext/1463567378/btConcaveShape.o: ../src/Bullet/BulletCollision/CollisionShapes/btConcaveShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1463567378
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1463567378/btConcaveShape.o ../src/Bullet/BulletCollision/CollisionShapes/btConcaveShape.cpp

${OBJECTDIR}/_ext/1463567378/btConeShape.o: ../src/Bullet/BulletCollision/CollisionShapes/btConeShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1463567378
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1463567378/btConeShape.o ../src/Bullet/BulletCollision/CollisionShapes/btConeShape.cpp

${OBJECTDIR}/_ext/1463567378/btConvex2dShape.o: ../src/Bullet/BulletCollision/CollisionShapes/btConvex2dShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1463567378
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1463567378/btConvex2dShape.o ../src/Bullet/BulletCollision/CollisionShapes/btConvex2dShape.cpp

${OBJECTDIR}/_ext/1463567378/btConvexHullShape.o: ../src/Bullet/BulletCollision/CollisionShapes/btConvexHullShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1463567378
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1463567378/btConvexHullShape.o ../src/Bullet/BulletCollision/CollisionShapes/btConvexHullShape.cpp

${OBJECTDIR}/_ext/1463567378/btConvexInternalShape.o: ../src/Bullet/BulletCollision/CollisionShapes/btConvexInternalShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1463567378
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1463567378/btConvexInternalShape.o ../src/Bullet/BulletCollision/CollisionShapes/btConvexInternalShape.cpp

${OBJECTDIR}/_ext/1463567378/btConvexPointCloudShape.o: ../src/Bullet/BulletCollision/CollisionShapes/btConvexPointCloudShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1463567378
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1463567378/btConvexPointCloudShape.o ../src/Bullet/BulletCollision/CollisionShapes/btConvexPointCloudShape.cpp

${OBJECTDIR}/_ext/1463567378/btConvexPolyhedron.o: ../src/Bullet/BulletCollision/CollisionShapes/btConvexPolyhedron.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1463567378
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1463567378/btConvexPolyhedron.o ../src/Bullet/BulletCollision/CollisionShapes/btConvexPolyhedron.cpp

${OBJECTDIR}/_ext/1463567378/btConvexShape.o: ../src/Bullet/BulletCollision/CollisionShapes/btConvexShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1463567378
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1463567378/btConvexShape.o ../src/Bullet/BulletCollision/CollisionShapes/btConvexShape.cpp

${OBJECTDIR}/_ext/1463567378/btConvexTriangleMeshShape.o: ../src/Bullet/BulletCollision/CollisionShapes/btConvexTriangleMeshShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1463567378
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1463567378/btConvexTriangleMeshShape.o ../src/Bullet/BulletCollision/CollisionShapes/btConvexTriangleMeshShape.cpp

${OBJECTDIR}/_ext/1463567378/btCylinderShape.o: ../src/Bullet/BulletCollision/CollisionShapes/btCylinderShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1463567378
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1463567378/btCylinderShape.o ../src/Bullet/BulletCollision/CollisionShapes/btCylinderShape.cpp

${OBJECTDIR}/_ext/1463567378/btEmptyShape.o: ../src/Bullet/BulletCollision/CollisionShapes/btEmptyShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1463567378
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1463567378/btEmptyShape.o ../src/Bullet/BulletCollision/CollisionShapes/btEmptyShape.cpp

${OBJECTDIR}/_ext/1463567378/btHeightfieldTerrainShape.o: ../src/Bullet/BulletCollision/CollisionShapes/btHeightfieldTerrainShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1463567378
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1463567378/btHeightfieldTerrainShape.o ../src/Bullet/BulletCollision/CollisionShapes/btHeightfieldTerrainShape.cpp

${OBJECTDIR}/_ext/1463567378/btMinkowskiSumShape.o: ../src/Bullet/BulletCollision/CollisionShapes/btMinkowskiSumShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1463567378
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1463567378/btMinkowskiSumShape.o ../src/Bullet/BulletCollision/CollisionShapes/btMinkowskiSumShape.cpp

${OBJECTDIR}/_ext/1463567378/btMultiSphereShape.o: ../src/Bullet/BulletCollision/CollisionShapes/btMultiSphereShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1463567378
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1463567378/btMultiSphereShape.o ../src/Bullet/BulletCollision/CollisionShapes/btMultiSphereShape.cpp

${OBJECTDIR}/_ext/1463567378/btMultimaterialTriangleMeshShape.o: ../src/Bullet/BulletCollision/CollisionShapes/btMultimaterialTriangleMeshShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1463567378
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1463567378/btMultimaterialTriangleMeshShape.o ../src/Bullet/BulletCollision/CollisionShapes/btMultimaterialTriangleMeshShape.cpp

${OBJECTDIR}/_ext/1463567378/btOptimizedBvh.o: ../src/Bullet/BulletCollision/CollisionShapes/btOptimizedBvh.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1463567378
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1463567378/btOptimizedBvh.o ../src/Bullet/BulletCollision/CollisionShapes/btOptimizedBvh.cpp

${OBJECTDIR}/_ext/1463567378/btPolyhedralConvexShape.o: ../src/Bullet/BulletCollision/CollisionShapes/btPolyhedralConvexShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1463567378
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1463567378/btPolyhedralConvexShape.o ../src/Bullet/BulletCollision/CollisionShapes/btPolyhedralConvexShape.cpp

${OBJECTDIR}/_ext/1463567378/btScaledBvhTriangleMeshShape.o: ../src/Bullet/BulletCollision/CollisionShapes/btScaledBvhTriangleMeshShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1463567378
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1463567378/btScaledBvhTriangleMeshShape.o ../src/Bullet/BulletCollision/CollisionShapes/btScaledBvhTriangleMeshShape.cpp

${OBJECTDIR}/_ext/1463567378/btShapeHull.o: ../src/Bullet/BulletCollision/CollisionShapes/btShapeHull.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1463567378
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1463567378/btShapeHull.o ../src/Bullet/BulletCollision/CollisionShapes/btShapeHull.cpp

${OBJECTDIR}/_ext/1463567378/btSphereShape.o: ../src/Bullet/BulletCollision/CollisionShapes/btSphereShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1463567378
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1463567378/btSphereShape.o ../src/Bullet/BulletCollision/CollisionShapes/btSphereShape.cpp

${OBJECTDIR}/_ext/1463567378/btStaticPlaneShape.o: ../src/Bullet/BulletCollision/CollisionShapes/btStaticPlaneShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1463567378
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1463567378/btStaticPlaneShape.o ../src/Bullet/BulletCollision/CollisionShapes/btStaticPlaneShape.cpp

${OBJECTDIR}/_ext/1463567378/btStridingMeshInterface.o: ../src/Bullet/BulletCollision/CollisionShapes/btStridingMeshInterface.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1463567378
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1463567378/btStridingMeshInterface.o ../src/Bullet/BulletCollision/CollisionShapes/btStridingMeshInterface.cpp

${OBJECTDIR}/_ext/1463567378/btTetrahedronShape.o: ../src/Bullet/BulletCollision/CollisionShapes/btTetrahedronShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1463567378
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1463567378/btTetrahedronShape.o ../src/Bullet/BulletCollision/CollisionShapes/btTetrahedronShape.cpp

${OBJECTDIR}/_ext/1463567378/btTriangleBuffer.o: ../src/Bullet/BulletCollision/CollisionShapes/btTriangleBuffer.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1463567378
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1463567378/btTriangleBuffer.o ../src/Bullet/BulletCollision/CollisionShapes/btTriangleBuffer.cpp

${OBJECTDIR}/_ext/1463567378/btTriangleCallback.o: ../src/Bullet/BulletCollision/CollisionShapes/btTriangleCallback.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1463567378
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1463567378/btTriangleCallback.o ../src/Bullet/BulletCollision/CollisionShapes/btTriangleCallback.cpp

${OBJECTDIR}/_ext/1463567378/btTriangleIndexVertexArray.o: ../src/Bullet/BulletCollision/CollisionShapes/btTriangleIndexVertexArray.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1463567378
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1463567378/btTriangleIndexVertexArray.o ../src/Bullet/BulletCollision/CollisionShapes/btTriangleIndexVertexArray.cpp

${OBJECTDIR}/_ext/1463567378/btTriangleIndexVertexMaterialArray.o: ../src/Bullet/BulletCollision/CollisionShapes/btTriangleIndexVertexMaterialArray.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1463567378
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1463567378/btTriangleIndexVertexMaterialArray.o ../src/Bullet/BulletCollision/CollisionShapes/btTriangleIndexVertexMaterialArray.cpp

${OBJECTDIR}/_ext/1463567378/btTriangleMesh.o: ../src/Bullet/BulletCollision/CollisionShapes/btTriangleMesh.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1463567378
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1463567378/btTriangleMesh.o ../src/Bullet/BulletCollision/CollisionShapes/btTriangleMesh.cpp

${OBJECTDIR}/_ext/1463567378/btTriangleMeshShape.o: ../src/Bullet/BulletCollision/CollisionShapes/btTriangleMeshShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1463567378
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1463567378/btTriangleMeshShape.o ../src/Bullet/BulletCollision/CollisionShapes/btTriangleMeshShape.cpp

${OBJECTDIR}/_ext/1463567378/btUniformScalingShape.o: ../src/Bullet/BulletCollision/CollisionShapes/btUniformScalingShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1463567378
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1463567378/btUniformScalingShape.o ../src/Bullet/BulletCollision/CollisionShapes/btUniformScalingShape.cpp

${OBJECTDIR}/_ext/1919507387/btContactProcessing.o: ../src/Bullet/BulletCollision/Gimpact/btContactProcessing.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1919507387
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1919507387/btContactProcessing.o ../src/Bullet/BulletCollision/Gimpact/btContactProcessing.cpp

${OBJECTDIR}/_ext/1919507387/btGImpactBvh.o: ../src/Bullet/BulletCollision/Gimpact/btGImpactBvh.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1919507387
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1919507387/btGImpactBvh.o ../src/Bullet/BulletCollision/Gimpact/btGImpactBvh.cpp

${OBJECTDIR}/_ext/1919507387/btGImpactCollisionAlgorithm.o: ../src/Bullet/BulletCollision/Gimpact/btGImpactCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1919507387
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1919507387/btGImpactCollisionAlgorithm.o ../src/Bullet/BulletCollision/Gimpact/btGImpactCollisionAlgorithm.cpp

${OBJECTDIR}/_ext/1919507387/btGImpactQuantizedBvh.o: ../src/Bullet/BulletCollision/Gimpact/btGImpactQuantizedBvh.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1919507387
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1919507387/btGImpactQuantizedBvh.o ../src/Bullet/BulletCollision/Gimpact/btGImpactQuantizedBvh.cpp

${OBJECTDIR}/_ext/1919507387/btGImpactShape.o: ../src/Bullet/BulletCollision/Gimpact/btGImpactShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1919507387
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1919507387/btGImpactShape.o ../src/Bullet/BulletCollision/Gimpact/btGImpactShape.cpp

${OBJECTDIR}/_ext/1919507387/btGenericPoolAllocator.o: ../src/Bullet/BulletCollision/Gimpact/btGenericPoolAllocator.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1919507387
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1919507387/btGenericPoolAllocator.o ../src/Bullet/BulletCollision/Gimpact/btGenericPoolAllocator.cpp

${OBJECTDIR}/_ext/1919507387/btTriangleShapeEx.o: ../src/Bullet/BulletCollision/Gimpact/btTriangleShapeEx.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1919507387
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1919507387/btTriangleShapeEx.o ../src/Bullet/BulletCollision/Gimpact/btTriangleShapeEx.cpp

${OBJECTDIR}/_ext/1919507387/gim_box_set.o: ../src/Bullet/BulletCollision/Gimpact/gim_box_set.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1919507387
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1919507387/gim_box_set.o ../src/Bullet/BulletCollision/Gimpact/gim_box_set.cpp

${OBJECTDIR}/_ext/1919507387/gim_contact.o: ../src/Bullet/BulletCollision/Gimpact/gim_contact.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1919507387
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1919507387/gim_contact.o ../src/Bullet/BulletCollision/Gimpact/gim_contact.cpp

${OBJECTDIR}/_ext/1919507387/gim_memory.o: ../src/Bullet/BulletCollision/Gimpact/gim_memory.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1919507387
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1919507387/gim_memory.o ../src/Bullet/BulletCollision/Gimpact/gim_memory.cpp

${OBJECTDIR}/_ext/1919507387/gim_tri_collision.o: ../src/Bullet/BulletCollision/Gimpact/gim_tri_collision.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1919507387
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1919507387/gim_tri_collision.o ../src/Bullet/BulletCollision/Gimpact/gim_tri_collision.cpp

${OBJECTDIR}/_ext/1196390564/btContinuousConvexCollision.o: ../src/Bullet/BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1196390564
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1196390564/btContinuousConvexCollision.o ../src/Bullet/BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.cpp

${OBJECTDIR}/_ext/1196390564/btConvexCast.o: ../src/Bullet/BulletCollision/NarrowPhaseCollision/btConvexCast.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1196390564
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1196390564/btConvexCast.o ../src/Bullet/BulletCollision/NarrowPhaseCollision/btConvexCast.cpp

${OBJECTDIR}/_ext/1196390564/btGjkConvexCast.o: ../src/Bullet/BulletCollision/NarrowPhaseCollision/btGjkConvexCast.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1196390564
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1196390564/btGjkConvexCast.o ../src/Bullet/BulletCollision/NarrowPhaseCollision/btGjkConvexCast.cpp

${OBJECTDIR}/_ext/1196390564/btGjkEpa2.o: ../src/Bullet/BulletCollision/NarrowPhaseCollision/btGjkEpa2.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1196390564
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1196390564/btGjkEpa2.o ../src/Bullet/BulletCollision/NarrowPhaseCollision/btGjkEpa2.cpp

${OBJECTDIR}/_ext/1196390564/btGjkEpaPenetrationDepthSolver.o: ../src/Bullet/BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1196390564
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1196390564/btGjkEpaPenetrationDepthSolver.o ../src/Bullet/BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.cpp

${OBJECTDIR}/_ext/1196390564/btGjkPairDetector.o: ../src/Bullet/BulletCollision/NarrowPhaseCollision/btGjkPairDetector.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1196390564
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1196390564/btGjkPairDetector.o ../src/Bullet/BulletCollision/NarrowPhaseCollision/btGjkPairDetector.cpp

${OBJECTDIR}/_ext/1196390564/btMinkowskiPenetrationDepthSolver.o: ../src/Bullet/BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1196390564
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1196390564/btMinkowskiPenetrationDepthSolver.o ../src/Bullet/BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.cpp

${OBJECTDIR}/_ext/1196390564/btPersistentManifold.o: ../src/Bullet/BulletCollision/NarrowPhaseCollision/btPersistentManifold.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1196390564
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1196390564/btPersistentManifold.o ../src/Bullet/BulletCollision/NarrowPhaseCollision/btPersistentManifold.cpp

${OBJECTDIR}/_ext/1196390564/btPolyhedralContactClipping.o: ../src/Bullet/BulletCollision/NarrowPhaseCollision/btPolyhedralContactClipping.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1196390564
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1196390564/btPolyhedralContactClipping.o ../src/Bullet/BulletCollision/NarrowPhaseCollision/btPolyhedralContactClipping.cpp

${OBJECTDIR}/_ext/1196390564/btRaycastCallback.o: ../src/Bullet/BulletCollision/NarrowPhaseCollision/btRaycastCallback.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1196390564
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1196390564/btRaycastCallback.o ../src/Bullet/BulletCollision/NarrowPhaseCollision/btRaycastCallback.cpp

${OBJECTDIR}/_ext/1196390564/btSubSimplexConvexCast.o: ../src/Bullet/BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1196390564
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1196390564/btSubSimplexConvexCast.o ../src/Bullet/BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.cpp

${OBJECTDIR}/_ext/1196390564/btVoronoiSimplexSolver.o: ../src/Bullet/BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1196390564
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1196390564/btVoronoiSimplexSolver.o ../src/Bullet/BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.cpp

${OBJECTDIR}/_ext/1145006819/btKinematicCharacterController.o: ../src/Bullet/BulletDynamics/Character/btKinematicCharacterController.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1145006819
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1145006819/btKinematicCharacterController.o ../src/Bullet/BulletDynamics/Character/btKinematicCharacterController.cpp

${OBJECTDIR}/_ext/472263850/btConeTwistConstraint.o: ../src/Bullet/BulletDynamics/ConstraintSolver/btConeTwistConstraint.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/472263850
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/472263850/btConeTwistConstraint.o ../src/Bullet/BulletDynamics/ConstraintSolver/btConeTwistConstraint.cpp

${OBJECTDIR}/_ext/472263850/btContactConstraint.o: ../src/Bullet/BulletDynamics/ConstraintSolver/btContactConstraint.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/472263850
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/472263850/btContactConstraint.o ../src/Bullet/BulletDynamics/ConstraintSolver/btContactConstraint.cpp

${OBJECTDIR}/_ext/472263850/btFixedConstraint.o: ../src/Bullet/BulletDynamics/ConstraintSolver/btFixedConstraint.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/472263850
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/472263850/btFixedConstraint.o ../src/Bullet/BulletDynamics/ConstraintSolver/btFixedConstraint.cpp

${OBJECTDIR}/_ext/472263850/btGearConstraint.o: ../src/Bullet/BulletDynamics/ConstraintSolver/btGearConstraint.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/472263850
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/472263850/btGearConstraint.o ../src/Bullet/BulletDynamics/ConstraintSolver/btGearConstraint.cpp

${OBJECTDIR}/_ext/472263850/btGeneric6DofConstraint.o: ../src/Bullet/BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/472263850
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/472263850/btGeneric6DofConstraint.o ../src/Bullet/BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.cpp

${OBJECTDIR}/_ext/472263850/btGeneric6DofSpringConstraint.o: ../src/Bullet/BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/472263850
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/472263850/btGeneric6DofSpringConstraint.o ../src/Bullet/BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.cpp

${OBJECTDIR}/_ext/472263850/btHinge2Constraint.o: ../src/Bullet/BulletDynamics/ConstraintSolver/btHinge2Constraint.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/472263850
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/472263850/btHinge2Constraint.o ../src/Bullet/BulletDynamics/ConstraintSolver/btHinge2Constraint.cpp

${OBJECTDIR}/_ext/472263850/btHingeConstraint.o: ../src/Bullet/BulletDynamics/ConstraintSolver/btHingeConstraint.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/472263850
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/472263850/btHingeConstraint.o ../src/Bullet/BulletDynamics/ConstraintSolver/btHingeConstraint.cpp

${OBJECTDIR}/_ext/472263850/btPoint2PointConstraint.o: ../src/Bullet/BulletDynamics/ConstraintSolver/btPoint2PointConstraint.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/472263850
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/472263850/btPoint2PointConstraint.o ../src/Bullet/BulletDynamics/ConstraintSolver/btPoint2PointConstraint.cpp

${OBJECTDIR}/_ext/472263850/btSequentialImpulseConstraintSolver.o: ../src/Bullet/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/472263850
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/472263850/btSequentialImpulseConstraintSolver.o ../src/Bullet/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.cpp

${OBJECTDIR}/_ext/472263850/btSliderConstraint.o: ../src/Bullet/BulletDynamics/ConstraintSolver/btSliderConstraint.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/472263850
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/472263850/btSliderConstraint.o ../src/Bullet/BulletDynamics/ConstraintSolver/btSliderConstraint.cpp

${OBJECTDIR}/_ext/472263850/btSolve2LinearConstraint.o: ../src/Bullet/BulletDynamics/ConstraintSolver/btSolve2LinearConstraint.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/472263850
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/472263850/btSolve2LinearConstraint.o ../src/Bullet/BulletDynamics/ConstraintSolver/btSolve2LinearConstraint.cpp

${OBJECTDIR}/_ext/472263850/btTypedConstraint.o: ../src/Bullet/BulletDynamics/ConstraintSolver/btTypedConstraint.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/472263850
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/472263850/btTypedConstraint.o ../src/Bullet/BulletDynamics/ConstraintSolver/btTypedConstraint.cpp

${OBJECTDIR}/_ext/472263850/btUniversalConstraint.o: ../src/Bullet/BulletDynamics/ConstraintSolver/btUniversalConstraint.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/472263850
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/472263850/btUniversalConstraint.o ../src/Bullet/BulletDynamics/ConstraintSolver/btUniversalConstraint.cpp

${OBJECTDIR}/_ext/1202644710/Bullet-C-API.o: ../src/Bullet/BulletDynamics/Dynamics/Bullet-C-API.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1202644710
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1202644710/Bullet-C-API.o ../src/Bullet/BulletDynamics/Dynamics/Bullet-C-API.cpp

${OBJECTDIR}/_ext/1202644710/btDiscreteDynamicsWorld.o: ../src/Bullet/BulletDynamics/Dynamics/btDiscreteDynamicsWorld.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1202644710
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1202644710/btDiscreteDynamicsWorld.o ../src/Bullet/BulletDynamics/Dynamics/btDiscreteDynamicsWorld.cpp

${OBJECTDIR}/_ext/1202644710/btRigidBody.o: ../src/Bullet/BulletDynamics/Dynamics/btRigidBody.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1202644710
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1202644710/btRigidBody.o ../src/Bullet/BulletDynamics/Dynamics/btRigidBody.cpp

${OBJECTDIR}/_ext/1202644710/btSimpleDynamicsWorld.o: ../src/Bullet/BulletDynamics/Dynamics/btSimpleDynamicsWorld.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1202644710
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1202644710/btSimpleDynamicsWorld.o ../src/Bullet/BulletDynamics/Dynamics/btSimpleDynamicsWorld.cpp

${OBJECTDIR}/_ext/818275416/btMultiBody.o: ../src/Bullet/BulletDynamics/Featherstone/btMultiBody.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/818275416
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/818275416/btMultiBody.o ../src/Bullet/BulletDynamics/Featherstone/btMultiBody.cpp

${OBJECTDIR}/_ext/818275416/btMultiBodyConstraint.o: ../src/Bullet/BulletDynamics/Featherstone/btMultiBodyConstraint.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/818275416
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/818275416/btMultiBodyConstraint.o ../src/Bullet/BulletDynamics/Featherstone/btMultiBodyConstraint.cpp

${OBJECTDIR}/_ext/818275416/btMultiBodyConstraintSolver.o: ../src/Bullet/BulletDynamics/Featherstone/btMultiBodyConstraintSolver.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/818275416
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/818275416/btMultiBodyConstraintSolver.o ../src/Bullet/BulletDynamics/Featherstone/btMultiBodyConstraintSolver.cpp

${OBJECTDIR}/_ext/818275416/btMultiBodyDynamicsWorld.o: ../src/Bullet/BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/818275416
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/818275416/btMultiBodyDynamicsWorld.o ../src/Bullet/BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.cpp

${OBJECTDIR}/_ext/818275416/btMultiBodyJointLimitConstraint.o: ../src/Bullet/BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/818275416
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/818275416/btMultiBodyJointLimitConstraint.o ../src/Bullet/BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.cpp

${OBJECTDIR}/_ext/818275416/btMultiBodyJointMotor.o: ../src/Bullet/BulletDynamics/Featherstone/btMultiBodyJointMotor.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/818275416
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/818275416/btMultiBodyJointMotor.o ../src/Bullet/BulletDynamics/Featherstone/btMultiBodyJointMotor.cpp

${OBJECTDIR}/_ext/818275416/btMultiBodyPoint2Point.o: ../src/Bullet/BulletDynamics/Featherstone/btMultiBodyPoint2Point.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/818275416
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/818275416/btMultiBodyPoint2Point.o ../src/Bullet/BulletDynamics/Featherstone/btMultiBodyPoint2Point.cpp

${OBJECTDIR}/_ext/1317251118/btDantzigLCP.o: ../src/Bullet/BulletDynamics/MLCPSolvers/btDantzigLCP.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1317251118
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1317251118/btDantzigLCP.o ../src/Bullet/BulletDynamics/MLCPSolvers/btDantzigLCP.cpp

${OBJECTDIR}/_ext/1317251118/btMLCPSolver.o: ../src/Bullet/BulletDynamics/MLCPSolvers/btMLCPSolver.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1317251118
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1317251118/btMLCPSolver.o ../src/Bullet/BulletDynamics/MLCPSolvers/btMLCPSolver.cpp

${OBJECTDIR}/_ext/1780737382/btRaycastVehicle.o: ../src/Bullet/BulletDynamics/Vehicle/btRaycastVehicle.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1780737382
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1780737382/btRaycastVehicle.o ../src/Bullet/BulletDynamics/Vehicle/btRaycastVehicle.cpp

${OBJECTDIR}/_ext/1780737382/btWheelInfo.o: ../src/Bullet/BulletDynamics/Vehicle/btWheelInfo.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1780737382
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1780737382/btWheelInfo.o ../src/Bullet/BulletDynamics/Vehicle/btWheelInfo.cpp

${OBJECTDIR}/_ext/1815990081/btDefaultSoftBodySolver.o: ../src/Bullet/BulletSoftBody/btDefaultSoftBodySolver.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1815990081
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1815990081/btDefaultSoftBodySolver.o ../src/Bullet/BulletSoftBody/btDefaultSoftBodySolver.cpp

${OBJECTDIR}/_ext/1815990081/btSoftBody.o: ../src/Bullet/BulletSoftBody/btSoftBody.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1815990081
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1815990081/btSoftBody.o ../src/Bullet/BulletSoftBody/btSoftBody.cpp

${OBJECTDIR}/_ext/1815990081/btSoftBodyConcaveCollisionAlgorithm.o: ../src/Bullet/BulletSoftBody/btSoftBodyConcaveCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1815990081
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1815990081/btSoftBodyConcaveCollisionAlgorithm.o ../src/Bullet/BulletSoftBody/btSoftBodyConcaveCollisionAlgorithm.cpp

${OBJECTDIR}/_ext/1815990081/btSoftBodyHelpers.o: ../src/Bullet/BulletSoftBody/btSoftBodyHelpers.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1815990081
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1815990081/btSoftBodyHelpers.o ../src/Bullet/BulletSoftBody/btSoftBodyHelpers.cpp

${OBJECTDIR}/_ext/1815990081/btSoftBodyRigidBodyCollisionConfiguration.o: ../src/Bullet/BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1815990081
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1815990081/btSoftBodyRigidBodyCollisionConfiguration.o ../src/Bullet/BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.cpp

${OBJECTDIR}/_ext/1815990081/btSoftRigidCollisionAlgorithm.o: ../src/Bullet/BulletSoftBody/btSoftRigidCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1815990081
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1815990081/btSoftRigidCollisionAlgorithm.o ../src/Bullet/BulletSoftBody/btSoftRigidCollisionAlgorithm.cpp

${OBJECTDIR}/_ext/1815990081/btSoftRigidDynamicsWorld.o: ../src/Bullet/BulletSoftBody/btSoftRigidDynamicsWorld.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1815990081
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1815990081/btSoftRigidDynamicsWorld.o ../src/Bullet/BulletSoftBody/btSoftRigidDynamicsWorld.cpp

${OBJECTDIR}/_ext/1815990081/btSoftSoftCollisionAlgorithm.o: ../src/Bullet/BulletSoftBody/btSoftSoftCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1815990081
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1815990081/btSoftSoftCollisionAlgorithm.o ../src/Bullet/BulletSoftBody/btSoftSoftCollisionAlgorithm.cpp

${OBJECTDIR}/_ext/1130245248/btAlignedAllocator.o: ../src/Bullet/LinearMath/btAlignedAllocator.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1130245248
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1130245248/btAlignedAllocator.o ../src/Bullet/LinearMath/btAlignedAllocator.cpp

${OBJECTDIR}/_ext/1130245248/btConvexHull.o: ../src/Bullet/LinearMath/btConvexHull.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1130245248
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1130245248/btConvexHull.o ../src/Bullet/LinearMath/btConvexHull.cpp

${OBJECTDIR}/_ext/1130245248/btConvexHullComputer.o: ../src/Bullet/LinearMath/btConvexHullComputer.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1130245248
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1130245248/btConvexHullComputer.o ../src/Bullet/LinearMath/btConvexHullComputer.cpp

${OBJECTDIR}/_ext/1130245248/btGeometryUtil.o: ../src/Bullet/LinearMath/btGeometryUtil.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1130245248
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1130245248/btGeometryUtil.o ../src/Bullet/LinearMath/btGeometryUtil.cpp

${OBJECTDIR}/_ext/1130245248/btPolarDecomposition.o: ../src/Bullet/LinearMath/btPolarDecomposition.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1130245248
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1130245248/btPolarDecomposition.o ../src/Bullet/LinearMath/btPolarDecomposition.cpp

${OBJECTDIR}/_ext/1130245248/btQuickprof.o: ../src/Bullet/LinearMath/btQuickprof.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1130245248
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1130245248/btQuickprof.o ../src/Bullet/LinearMath/btQuickprof.cpp

${OBJECTDIR}/_ext/1130245248/btSerializer.o: ../src/Bullet/LinearMath/btSerializer.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1130245248
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1130245248/btSerializer.o ../src/Bullet/LinearMath/btSerializer.cpp

${OBJECTDIR}/_ext/1130245248/btVector3.o: ../src/Bullet/LinearMath/btVector3.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1130245248
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1130245248/btVector3.o ../src/Bullet/LinearMath/btVector3.cpp

${OBJECTDIR}/_ext/1360937237/MultipleObjects.o: ../src/MultipleObjects.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1360937237
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1360937237/MultipleObjects.o ../src/MultipleObjects.cpp

${OBJECTDIR}/_ext/965943543/DemoApplication.o: ../src/OpenGL/DemoApplication.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/965943543
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/965943543/DemoApplication.o ../src/OpenGL/DemoApplication.cpp

${OBJECTDIR}/_ext/965943543/GLDebugDrawer.o: ../src/OpenGL/GLDebugDrawer.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/965943543
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/965943543/GLDebugDrawer.o ../src/OpenGL/GLDebugDrawer.cpp

${OBJECTDIR}/_ext/965943543/GLDebugFont.o: ../src/OpenGL/GLDebugFont.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/965943543
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/965943543/GLDebugFont.o ../src/OpenGL/GLDebugFont.cpp

${OBJECTDIR}/_ext/965943543/GL_DialogDynamicsWorld.o: ../src/OpenGL/GL_DialogDynamicsWorld.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/965943543
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/965943543/GL_DialogDynamicsWorld.o ../src/OpenGL/GL_DialogDynamicsWorld.cpp

${OBJECTDIR}/_ext/965943543/GL_DialogWindow.o: ../src/OpenGL/GL_DialogWindow.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/965943543
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/965943543/GL_DialogWindow.o ../src/OpenGL/GL_DialogWindow.cpp

${OBJECTDIR}/_ext/965943543/GL_ShapeDrawer.o: ../src/OpenGL/GL_ShapeDrawer.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/965943543
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/965943543/GL_ShapeDrawer.o ../src/OpenGL/GL_ShapeDrawer.cpp

${OBJECTDIR}/_ext/965943543/GL_Simplex1to4.o: ../src/OpenGL/GL_Simplex1to4.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/965943543
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/965943543/GL_Simplex1to4.o ../src/OpenGL/GL_Simplex1to4.cpp

${OBJECTDIR}/_ext/965943543/GlutDemoApplication.o: ../src/OpenGL/GlutDemoApplication.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/965943543
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/965943543/GlutDemoApplication.o ../src/OpenGL/GlutDemoApplication.cpp

${OBJECTDIR}/_ext/965943543/GlutStuff.o: ../src/OpenGL/GlutStuff.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/965943543
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/965943543/GlutStuff.o ../src/OpenGL/GlutStuff.cpp

${OBJECTDIR}/_ext/965943543/RenderTexture.o: ../src/OpenGL/RenderTexture.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/965943543
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/965943543/RenderTexture.o ../src/OpenGL/RenderTexture.cpp

${OBJECTDIR}/_ext/965943543/Win32AppMain.o: ../src/OpenGL/Win32AppMain.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/965943543
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/965943543/Win32AppMain.o ../src/OpenGL/Win32AppMain.cpp

${OBJECTDIR}/_ext/965943543/Win32DemoApplication.o: ../src/OpenGL/Win32DemoApplication.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/965943543
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/965943543/Win32DemoApplication.o ../src/OpenGL/Win32DemoApplication.cpp

${OBJECTDIR}/_ext/965943543/stb_image.o: ../src/OpenGL/stb_image.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/965943543
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/965943543/stb_image.o ../src/OpenGL/stb_image.cpp

${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1360937237
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../src/OpenGL -I../src/Bullet `pkg-config --cflags liblog4cxx` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1360937237/main.o ../src/main.cpp

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${CND_BUILDDIR}/${CND_CONF}
	${RM} ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/physics_toolbox

# Subprojects
.clean-subprojects:

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc
