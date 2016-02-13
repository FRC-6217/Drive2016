#ifndef AUTONOMOUS_H_
#define AUTONOMOUS_H_

#include <map>
#include "WPILib.h"

class Autonomous {
	static RobotDrive *drive;
	static Gyro *gyro;
	static Encoder *leftEnc;
	static Encoder *rightEnc;
	//other sensors here

public:
	const static std::map<std::string, bool (*)()> crossFunctions;

	static void init(RobotDrive *drive, Gyro *gyro, Encoder *leftEnc, Encoder *rightEnc);

	static bool approachOnly();
	//cross functions
	static bool lowBar();
	static bool portcullis();
	static bool chevalDeFrise();
	static bool moat();
	static bool ramparts();
	static bool drawBridge();
	static bool sallyPort();
	static bool rockWall();
	static bool roughTerrain();

};

#endif
