/*
 * FileHandling.h
 *
 *  Created on: Oct 29, 2013
 *      Author: marium
 */

#ifndef FILEHANDLING_H_
#define FILEHANDLING_H_

#include <sstream>
#include <fstream>
#include <vector>

class FileHandling {
	// File Handling IO
	std::ofstream myfile; //output file of readouts
	std::ifstream readFile; // input file for getting positions of masses
	std::ifstream connectionMatrixFile; //contains type of connections

public:
	FileHandling();
	virtual ~FileHandling();

	void writeInFile(std::string writeLetter);
	void openFiles();
	void closeFiles();

private:
	//For positions
	std::vector<double> fileReadTuple;
	std::string readFileString;

	//For connection map
	std::string connectionMapFileString;
	std::vector<double> connectionMapFileTuple;
};

#endif /* FILEHANDLING_H_ */
