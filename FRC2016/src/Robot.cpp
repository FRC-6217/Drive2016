#include <vector>

#include "WPILib.h"
#include "Autonomous.h"

class Robot: public IterativeRobot
{
private:
	LiveWindow *lw = LiveWindow::GetInstance();
	SendableChooser *chooser;
	const std::string autoNameDefault = "Approch Only";
	const std::vector<std::string> autoNames = {
			"Low Bar",
			"High"
	};

	std::string autoSelected;

	VictorSP *leftMotor, *rightMotor;
	RobotDrive *drive;

	Joystick *stick;

	DoubleSolenoid *test1;
	Solenoid *test2;

	bool defenseCrossed;
	bool done;

	void RobotInit()
	{
		chooser = new SendableChooser();
		chooser->AddDefault(autoNameDefault, (void*)&autoNameDefault);
		for (std::map<std::string, bool (*)()>::const_iterator it = Autonomous::crossFunctions.begin(); it!= Autonomous::crossFunctions.end(); it++) {
			chooser->AddObject(it->first, (void*)&(it->first));
		}
		SmartDashboard::PutData("Auto Modes", chooser);

		leftMotor = new VictorSP(1);
		rightMotor = new VictorSP(0);

		rightMotor->SetInverted(true);
		leftMotor->SetInverted(true);
		drive = new RobotDrive(leftMotor, rightMotor);

		drive->SetMaxOutput(0.5);

		stick = new Joystick(0);

		//test1 = new DoubleSolenoid(0,1);
		//test2 = new Solenoid(0);
	}


	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the GetString line to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the if-else structure below with additional strings.
	 * If using the SendableChooser make sure to add them to the chooser code above as well.
	 */
	void AutonomousInit()
	{
		autoSelected = *((std::string*)chooser->GetSelected());
		//std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;

		defenseCrossed = false;
		done = false;
	}

	void AutonomousPeriodic()
	{
		if(done) {
			//doNothing(); //Wait? Why do I have a function for this?
		} else {
		if (autoSelected == "Approach Only") {
			//only touch defense, doesn't matter what
		} else if (!defenseCrossed) {
				if(Autonomous::crossFunctions.find(autoSelected) != Autonomous::crossFunctions.end()) {
					bool (*crossFunction)() = Autonomous::crossFunctions.at(autoSelected);
					defenseCrossed = crossFunction();
				} else {
					//doNothing();
				}
			} else {
				//after we cross...
			}
		}
	}

	void TeleopInit()
	{
		test1->Set(DoubleSolenoid::Value::kOff);
	}

	void TeleopPeriodic()
	{
		drive->ArcadeDrive(stick);
		drive->SetMaxOutput((1-stick->GetThrottle())/2);
		printf("%f\n", (1-stick->GetThrottle())/2);
		//leftMotor->Set(0.1);
		//rightMotor->Set(0.1);
	}

	void TestPeriodic()
	{
		lw->Run();
		/*printf("Forward\n");
		test1->Set(DoubleSolenoid::Value::kForward);
		//test2->Set(true);
		Wait(4.0);
		printf("Reverse\n");
		test1->Set(DoubleSolenoid::Value::kReverse);
		//test2->Set(false);
		Wait(5.0);
		*/


	}
};

START_ROBOT_CLASS(Robot)
