/*
 * AddConnection.h
 *
 *  Created on: Oct 26, 2013
 *      Author: marium
 */

#ifndef ADDCONNECTION_H_
#define ADDCONNECTION_H_

class AddConnection {
public:
	AddConnection();
	virtual ~AddConnection();

	int getConnectionType() const;
	void setConnectionType(int connectionType);

private:
	int _connectionType;

};

#endif /* ADDCONNECTION_H_ */
