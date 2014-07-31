/*
 * FileHandling.cpp
 *
 *  Created on: Oct 29, 2013
 *      Author: marium
 */

#include "FileHandling.h"

FileHandling::FileHandling() {
	// TODO Auto-generated constructor stub

}

FileHandling::~FileHandling() {
}

void FileHandling::writeInFile(std::string writeLine){
	myfile<<writeLine;

}

void FileHandling::openFiles(){
	myfile.open("/home/marium/Desktop/Experiments/Experiment 4/MassPositions_Experiment3_WithoutGravity.csv");

	//Open file to read
	readFile.open("/home/marium/Desktop/Experiments/Experiment 4/Experiment_4.csv");
	connectionMatrixFile.open("/home/marium/Desktop/Experiments/Experiment 4/Experiment4_ConnectionMapping.csv");
}

void FileHandling::closeFiles(){
	myfile.close();
	readFile.close();
	connectionMatrixFile.close();
}
