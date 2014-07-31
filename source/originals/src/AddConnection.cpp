/*
 * AddConnection.cpp
 *
 *  Created on: Oct 26, 2013
 *      Author: marium
 */

#include "AddConnection.h"

AddConnection::AddConnection() {
this->_connectionType = -1;

}

AddConnection::~AddConnection() {
}

int AddConnection::getConnectionType() const {
	return _connectionType;
}

void AddConnection::setConnectionType(int connectionType) {
	_connectionType = connectionType;
}
