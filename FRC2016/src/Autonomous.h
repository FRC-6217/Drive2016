#ifndef AUTONOMOUS_H_
#define AUTONOMOUS_H_

#include <map>
#include "WPILib.h"

//Vision constants
//TODO: configure constants
const double TARGET_WIDTH = 1.33;
const double VIEW_WIDTH = 9.0;
const double VIEW_ANGLE = 49;

const double CAMERA_POS = 5.0; //TODO: set this to the right value
const double CAMERA_TOLERANCE = 1.0; //... and this

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

	//other
	static void alignWithGoal(RobotDrive *drive, VictorSP *launch1, VictorSP *launch2, VictorSP *winch, std::shared_ptr<NetworkTable> table);

};

#endif
