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

#include <stdio.h>
#include <iostream>
#include <string>
#include <log4cxx/logger.h>
#include <log4cxx/propertyconfigurator.h>
#include "MultipleObjects.h"
#include "OpenGL/GlutStuff.h"
#include "btBulletDynamicsCommon.h"
#include "LinearMath/btHashMap.h"
#include "main.h"

#define WINDOW_TITLE "Physics Toolbox"
#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 600
#define FLOOR_BOOL false
#define TRAINING_BOOL true
std::string LOG_CONFIGURATION = "../src/log4cxx.properties";
std::string MASSES_FILE = "../Bapp/devArm/circle/masses.csv";
std::string CONNECTION_MAP_FILE = "../Bapp/devArm/circle/connectionMap.csv";
std::string TORQUES_FILE = "../Bapp/devArm/circle/angles.csv";
std::string OUTPUT_FILE = "../Bapp/devArm/circle/output.csv";
std::string WEIGHTS_FILE = "../Bapp/devArm/circle/weights.csv";
log4cxx::LoggerPtr mainLogger(log4cxx::Logger::getLogger("Main"));

int start(std::string massesFilePath, std::string connectionMapFilePath, std::string outputFilePath, std::string motorTorquesFilePath, std::string weightsFilePath, bool floor, bool training) {
    MultipleObjects multipleObjects;
    multipleObjects.setParameters(massesFilePath, connectionMapFilePath, outputFilePath, motorTorquesFilePath, weightsFilePath, floor, training);
    LOG4CXX_INFO(mainLogger, "Set all parameters, show interface now.");
    multipleObjects.Interface();
    multipleObjects.setDebugMode(btIDebugDraw::DBG_DrawConstraints + btIDebugDraw::DBG_DrawConstraintLimits);
#ifdef CHECK_MEMORY_LEAKS
    multipleObjects.exitPhysics();
#else
    char *argv [1];
    int argc = 1;
    argv [0] = strdup("robotArm");
    return glutmain(argc, argv, WINDOW_WIDTH, WINDOW_HEIGHT, WINDOW_TITLE, &multipleObjects);
#endif
    return 0;
}

int main(int argc, char** argv) {
    log4cxx::PropertyConfigurator::configure(LOG_CONFIGURATION);
    LOG4CXX_INFO(mainLogger, "C++ Robot Arm Simulator");

    for (int i = 0; i < argc; i++) {
        LOG4CXX_DEBUG(mainLogger, "Argument " << i << ": " << argv[i]);
    }

    if (argc == 8) {
        char* massesFilePath = argv[1];
        char* connectionMapFilePath = argv[2];
        char* outputFilePath = argv[3];
        char* torquesFilePath = argv[4];
        char* weightsFilePath = argv[5];
        bool showFloor = std::string(argv[6]).compare("true") == 0;
        bool isTraining = std::string(argv[7]).compare("true") == 0;

        LOG4CXX_INFO(mainLogger, "Run with the provided arguments and skip menu:");
        LOG4CXX_INFO(mainLogger, "  masses file:         " + std::string(massesFilePath));
        LOG4CXX_INFO(mainLogger, "  connection map file: " + std::string(connectionMapFilePath));
        LOG4CXX_INFO(mainLogger, "  output file:         " + std::string(outputFilePath));
        LOG4CXX_INFO(mainLogger, "  trajectory file:     " + std::string(torquesFilePath));
        LOG4CXX_INFO(mainLogger, "  weights file:        " + std::string(weightsFilePath));
        LOG4CXX_INFO(mainLogger, "  show floor:          " + (showFloor ? std::string("true") : std::string("false")));
        LOG4CXX_INFO(mainLogger, "  is training:         " + (isTraining ? std::string("true") : std::string("false")));

        return start(massesFilePath, connectionMapFilePath, outputFilePath, torquesFilePath, weightsFilePath, showFloor, isTraining);
    }

    if (argc > 2) {
        showExpectedArguments();
        return 1;
    }

    LOG4CXX_INFO(mainLogger, "Run with the defaults and show menu.");
    return start(MASSES_FILE, CONNECTION_MAP_FILE, OUTPUT_FILE, TORQUES_FILE, WEIGHTS_FILE, FLOOR_BOOL, true);
//    return start(MASSES_FILE, CONNECTION_MAP_FILE, OUTPUT_FILE, TORQUES_FILE, WEIGHTS_FILE, FLOOR_BOOL, false);
}
