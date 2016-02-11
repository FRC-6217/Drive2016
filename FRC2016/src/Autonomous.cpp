#include "Autonomous.h"

#include <stdio.h>
#include <map>

#include "WPILib.h"

RobotDrive *Autonomous::drive = NULL;

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

void Autonomous::init(RobotDrive *drive, Gyro *gyro, Encoder *enc) {
	Autonomous::drive = drive;
	Autonomous::gyro = gyro;
	Autonomous::enc = enc;
}

bool Autonomous::doNothing() {
	Wait(0.1);
	printf("Doing Nothing.\n");
	return true;
}

//cross functions
bool Autonomous::lowBar() {
	return false;
}

bool Autonomous::portcullis() {
	return false;
}
bool Autonomous::chevalDeFrise() {
	return false;
}
bool Autonomous::moat() {
	return false;
}
bool Autonomous::ramparts() {
	return false;
}
bool Autonomous::drawBridge() {
	return false;
}
bool Autonomous::sallyPort() {
	return false;
}
bool Autonomous::rockWall() {
	return false;
}
bool Autonomous::roughTerrain() {
	return false;
}
