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
	if (leftEnc->GetDistance() > 48.11 || rightEnc->GetDistance() > 48.11) {
		return true;
	}
	return false;
}

//cross functions
bool Autonomous::lowBar() {
	float angle = gyro->GetAngle();
	drive->Drive(0.5, -angle * .03);
	if (leftEnc->GetDistance() < -115 || rightEnc->GetDistance() < -115) {
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
	drive->Drive(-1.0, -angle * .03);
	if (leftEnc->GetDistance() > 115 || rightEnc->GetDistance() > 115) {
		return true;
	}
	return false;
}
bool Autonomous::ramparts() {
	float angle = gyro->GetAngle();
	drive->Drive(-0.8, -angle * .03);
	if (leftEnc->GetDistance() > 115 || rightEnc->GetDistance() > 115) {
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
	drive->Drive(-1.0, -angle * .03);
	if (leftEnc->GetDistance() > 115 || rightEnc->GetDistance() > 115) {
		return true;
	}
	return false;
}
bool Autonomous::roughTerrain() {
	float angle = gyro->GetAngle();
	drive->Drive(-1.0, -angle * .03);
	if (leftEnc->GetDistance() > 115 || rightEnc->GetDistance() > 115) {
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
			launch1->Set(0.0);
			launch2->Set(0.0);
			winch->Set(0.0);
			otherWinch->Set(0.0);

			drive->ArcadeDrive(0.0,0.5);
			timer->Reset();
		} else if (centerArray.at(largest) - CAMERA_POS < CAMERA_TOLERANCE) {
			launch1->Set(0.0);
			launch2->Set(0.0);
			winch->Set(0.0);
			otherWinch->Set(0.0);

			drive->ArcadeDrive(0.0,-0.5);
			timer->Reset();
		} else {
			double distance = TARGET_WIDTH * VIEW_WIDTH / (2 * width * tan(VIEW_ANGLE));

			double time = 0.0;
			for (std::map<double,double>::iterator it = powerLookup.begin(); it != powerLookup.end(); it++) {
				if (it->first < distance) {
					time = it->second;
				}
			}

			if (timer->Get() < 1.0) {
				otherWinch->Set(-0.5);
			} else {
				otherWinch->Set(0.0);
			}

			if (timer->Get() < time) {
				winch->Set(-0.5);
			} else {
				winch->Set(0.0);
				return true;
			}
			
			launch1->Set(1.0);
			launch2->Set(1.0);
			//launch1->Set(powerLookup[distance]);
			//launch2->Set(powerLookup[distance]);
			//winch->Set(0.0);

			drive->ArcadeDrive(0.0,0.0);
		}
	} else {
		launch1->Set(0.0);
		launch2->Set(0.0);
		winch->Set(0.0);
		otherWinch->Set(0.0);
		drive->ArcadeDrive(0.0,0.0);
	}
	return false;
}
