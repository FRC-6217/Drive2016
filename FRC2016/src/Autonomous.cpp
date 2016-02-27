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
	{"Rough Terrain", &Autonomous::roughTerrain},
	{"Spy Bot", &Autonomous::spyBot}
};

//maps minimum distances (ft) to shooter power.
//TODO: fill table with real values
std::map<double, double> powerLookup = {
		{2.0, 0.1},
};

void Autonomous::init(RobotDrive *drive, Gyro *gyro, Encoder *leftEnc, Encoder *rightEnc) {
	Autonomous::drive = drive;
	Autonomous::gyro = gyro;
	Autonomous::leftEnc = leftEnc;
	Autonomous::rightEnc = rightEnc;
}

bool Autonomous::spyBot() {
	return true;
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

bool Autonomous::alignWithGoal(RobotDrive *drive, VictorSP *launch1, VictorSP *launch2, VictorSP *winch, Victor *otherWinch, std::shared_ptr<NetworkTable> table, Timer *timer) {
	std::vector<double> areaArray = table->GetNumberArray("area", llvm::ArrayRef<double>());
	std::vector<double> centerArray = table->GetNumberArray("centerX", llvm::ArrayRef<double>());
	std::vector<double> widthArray = table->GetNumberArray("width", llvm::ArrayRef<double>());

	int largest = -1;
	double size = 0;
	for (unsigned int i = 0; i < areaArray.size(); i++) {
		if (areaArray.at(i) > size) {
			largest = i;
			size = areaArray.at(i);
		}
	}
	if (largest > -1) {
		double width = widthArray.at(largest);
		printf("area:%f, centerX:%f, width:%f\n", areaArray.at(largest), centerArray.at(largest), widthArray.at(largest));

		if (centerArray.at(largest) - CAMERA_POS > CAMERA_TOLERANCE) {
			drive->ArcadeDrive(0.0,0.5);
		} else if (centerArray.at(largest) - CAMERA_POS < CAMERA_TOLERANCE) {
			drive->ArcadeDrive(0.0,-0.5);
		} else {
			double distance = TARGET_WIDTH * VIEW_WIDTH / (2 * width * tan(VIEW_ANGLE));

			double time = 0.0;
			for (std::map<double,double>::iterator it = powerLookup.begin(); it != powerLookup.end(); it++) {
				if (it->first < distance) {
					time = it->second;
				}
			}

			if (timer->Get() < time) {
				winch->Set(-0.5);
				otherWinch->Set(0.5);
			} else {
				winch->Set(0.0);
				otherWinch->Set(0.0);
				return true;
			}

			//launch1->Set(powerLookup[distance]);
			//launch2->Set(powerLookup[distance]);
			//winch->Set(0.0);
		}
	}
	return false;
}
