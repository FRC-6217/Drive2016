#include <vector>

#include "WPILib.h"
#include "Autonomous.h"

class Robot: public IterativeRobot
{
private:
	LiveWindow *lw = LiveWindow::GetInstance();
	SendableChooser *chooser;
	const std::string autoNameDefault = "Approach Only";

	std::string autoSelected;

	//VictorSP *leftMotor, *rightMotor;
	RobotDrive *drive;

	Joystick *stick;

	DoubleSolenoid *test1;
	Solenoid *test2;

	VictorSP *launch1, *launch2;

	Gyro *gyro;
	Encoder *leftEnc;
	Encoder *rightEnc;

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

		//leftMotor = new VictorSP(1);
		//rightMotor = new VictorSP(0);

		//rightMotor->SetInverted(true);
		//leftMotor->SetInverted(true);

		//Front Left, Back Left, Front Right, Back Right
		drive = new RobotDrive(2, 3, 0, 1);
		drive->SetInvertedMotor(RobotDrive::MotorType::kFrontLeftMotor, true);
		drive->SetInvertedMotor(RobotDrive::MotorType::kRearLeftMotor, true);
		drive->SetInvertedMotor(RobotDrive::MotorType::kFrontRightMotor, true);
		drive->SetInvertedMotor(RobotDrive::MotorType::kRearRightMotor, true);

		drive->SetMaxOutput(0.5);

		stick = new Joystick(0);

		test1 = new DoubleSolenoid(4,5);
		//test2 = new Solenoid(0);

		launch1 = new VictorSP(4);
		launch2 = new VictorSP(5);
		launch1->SetInverted(true);

		gyro = new AnalogGyro(1);
		leftEnc = new Encoder(0, 1, false, Encoder::EncodingType::k1X);
		rightEnc = new Encoder(2,3, false, Encoder::EncodingType::k1X);
		//Configure for inches.
		leftEnc->SetDistancePerPulse(10);
		rightEnc->SetDistancePerPulse(10);
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

		Autonomous::init(drive, gyro, leftEnc, rightEnc);

		//Make sure to reset the encoder!
		leftEnc->Reset();
		rightEnc->Reset();
	}

	void AutonomousPeriodic()
	{
		if(done) {
			//doNothing(); //Wait? Why do I have a function for this?
		} else {
		if (autoSelected == "Approach Only") {
			Autonomous::approachOnly();
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
		leftEnc->Reset();
		rightEnc->Reset();
	}

	void TeleopPeriodic()
	{
		//printf("Left Encoder: %i, Right Encoder: %i, Gyro: %f\n", leftEnc->Get(), rightEnc->Get(), gyro->GetAngle());

		drive->ArcadeDrive(stick);
		drive->SetMaxOutput((1-stick->GetThrottle())/2);
		printf("%f\n", (1-stick->GetThrottle())/2);
		//leftMotor->Set(0.1);
		//rightMotor->Set(0.1);

		if (stick->GetTrigger()) {
			launch1->Set(1.0);
			launch2->Set(1.0);
		} else if (stick->GetRawButton(2)) {
			launch1->Set(-1.0);
			launch2->Set(-1.0);
		} else {
			launch1->Set(0.0);
			launch2->Set(0.0);
		}
		/*//TEST CODE
		if (stick->GetRawButton(4)) {
			test1->Set(DoubleSolenoid::Value::kForward);
		} else if (stick->GetRawButton(6)) {
			test1->Set(DoubleSolenoid::Value::kReverse);
		} else {
			test1->Set(DoubleSolenoid::Value::kOff);
		}*/

	}

	void TestPeriodic()
	{
		lw->Run();
		printf("Left Encoder: %i, Right Encoder: %i, Gyro: %f\n", leftEnc->Get(), rightEnc->Get(), gyro->GetAngle());

	}
};

START_ROBOT_CLASS(Robot)
