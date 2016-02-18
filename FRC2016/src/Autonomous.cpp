#include "Autonomous.h"

#include <stdio.h>
#include <map>

#include "WPILib.h"

RobotDrive *Autonomous::drive = NULL;
Gyro *Autonomous::gyro = NULL;
Encoder *Autonomous::leftEnc = NULL;
Encoder *Autonomous::rightEnc = NULL;

const std::map<std::string, bool (*)()> Autonomous::crossFunctions =
{
	{"Low Bar", &Autonomous::lowBar},
	{"Portcullis", &Autonomous::portcullis},
	{"Cheval De Frise", &Autonomous::chevalDeFrise},
	{"Moat", &Autonomous::moat},
	{"Ramparts", &Autonomous::ramparts},
	{"Drawbridge", &Autonomous::drawBridge},
	{"Sally Port", &Autonomous::sallyPort},
	{"Rock Wall", &Autonomous::rockWall},
	{"Rough Terrain", &Autonomous::roughTerrain}
};

void Autonomous::init(RobotDrive *drive, Gyro *gyro, Encoder *leftEnc, Encoder *rightEnc) {
	Autonomous::drive = drive;
	Autonomous::gyro = gyro;
	Autonomous::leftEnc = leftEnc;
	Autonomous::rightEnc = rightEnc;
}

bool Autonomous::approachOnly() {
	float angle = gyro->GetAngle();
	drive->Drive(-0.5, -angle * .03);
	if (leftEnc->GetDistance() > 78.11 || rightEnc->GetDistance() > 78.11) {
		return true;
	}
	return false;
}

//cross functions
bool Autonomous::lowBar() {
	float angle = gyro->GetAngle();
	drive->Drive(-0.5, -angle * .03);
	if (leftEnc->GetDistance() > 121.61 || rightEnc->GetDistance() > 121.61) {
		return true;
	}
	return false;
}

bool Autonomous::portcullis() {
	//Nope.
	return false;
}
bool Autonomous::chevalDeFrise() {
	//Maybe.
	return false;
}
bool Autonomous::moat() {
	float angle = gyro->GetAngle();
	drive->Drive(-0.5, -angle * .03);
	if (leftEnc->GetDistance() > 121.61 || rightEnc->GetDistance() > 121.61) {
		return true;
	}
	return false;
}
bool Autonomous::ramparts() {
	float angle = gyro->GetAngle();
	drive->Drive(-0.5, -angle * .03);
	if (leftEnc->GetDistance() > 121.61 || rightEnc->GetDistance() > 121.61) {
		return true;
	}
	return false;
}
bool Autonomous::drawBridge() {
	//No
	return false;
}
bool Autonomous::sallyPort() {
	//No
	return false;
}
bool Autonomous::rockWall() {
	float angle = gyro->GetAngle();
	drive->Drive(-0.5, -angle * .03);
	if (leftEnc->GetDistance() > 121.61 || rightEnc->GetDistance() > 121.61) {
		return true;
	}
	return false;
}
bool Autonomous::roughTerrain() {
	float angle = gyro->GetAngle();
	drive->Drive(-0.5, -angle * .03);
	if (leftEnc->GetDistance() > 121.61 || rightEnc->GetDistance() > 121.61) {
		return true;
	}
	return false;
}
