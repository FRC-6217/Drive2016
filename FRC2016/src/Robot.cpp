#include <vector>
#include <string>
#include <map>
#include <cmath>
#include <unistd.h>

#include "WPILib.h"
#include "Autonomous.h"

//TODO: get GRIP running on this, and test with NetworkTables
//Possibly done?

class Robot: public IterativeRobot
{
private:
	LiveWindow *lw = LiveWindow::GetInstance();
	SendableChooser *chooser;
	const std::string autoNameDefault = "Approach Only";

	SendableChooser *posChooser;
	const std::string posDefault = "0";

	SendableChooser *goalChooser;
	const std::string goalDefault = "High";

	std::string autoSelected;
	double rotation;
	std::string goal;

	double posToDegrees[5] = {-34.83, -22.66, -7.92, 7.92, 22.66};

	//VictorSP *leftMotor, *rightMotor;
	RobotDrive *drive;

	Joystick *driveStick;
	Joystick *shootStick;


	DoubleSolenoid *launchPiston;
	DoubleSolenoid *tiltPiston;

	VictorSP *frontLeft, *backLeft, *frontRight, *backRight;
	VictorSP *launch1, *launch2;
	VictorSP *winch;

	Gyro *gyro;
	Encoder *leftEnc;
	Encoder *rightEnc;

	bool defenseCrossed;
	bool done;

	int autoCounter;

	IMAQdxSession session;
	Image *frame;
	IMAQdxError imaqError;
	std::unique_ptr<AxisCamera> camera;

	std::shared_ptr<NetworkTable> table;

	int powerCounter = 0;
	const double POWER_MAX = 10;

	void RobotInit()
	{
		frame = imaqCreateImage(IMAQ_IMAGE_RGB, 0);
		camera.reset(new AxisCamera("axis-camera.local"));

		table = NetworkTable::GetTable("GRIP/myContoursReport");

		chooser = new SendableChooser();
		chooser->AddDefault(autoNameDefault, (void*)&autoNameDefault);
		for (std::map<std::string, bool (*)()>::const_iterator it = Autonomous::crossFunctions.begin(); it!= Autonomous::crossFunctions.end(); it++) {
			chooser->AddObject(it->first, (void*)&(it->first));
		}
		SmartDashboard::PutData("Auto Modes", chooser);

		posChooser = new SendableChooser();
		posChooser->AddDefault(posDefault, (void*)&posToDegrees[stoi(posDefault)]);
		for (int i = 1; i < 6; i++) {
			posChooser->AddObject(std::to_string(i), (void*)&posToDegrees[i]);
		}
		SmartDashboard::PutData("Position", posChooser);

//		goalChooser = new SendableChooser();
//		goalChooser->AddDefault(goalDefault, (void*)&goalDefault);
//		goalChooser->AddObject("Low", (void*)"Low");

		drive = new RobotDrive(2,3,0,1);
		//drive = new RobotDrive(frontLeft, backLeft, frontRight, backRight);

		drive->SetInvertedMotor(RobotDrive::MotorType::kFrontLeftMotor, true);
		drive->SetInvertedMotor(RobotDrive::MotorType::kRearLeftMotor, true);
		drive->SetInvertedMotor(RobotDrive::MotorType::kFrontRightMotor, true);
		drive->SetInvertedMotor(RobotDrive::MotorType::kRearRightMotor, true);

		drive->SetMaxOutput(0.5);

		driveStick = new Joystick(0);
		shootStick = new Joystick(1);

		//test1 = new DoubleSolenoid(4,5);
		//test2 = new Solenoid(0);

		launch1 = new VictorSP(4);
		launch2 = new VictorSP(5);
		launch1->SetInverted(true);

		winch = new VictorSP(6);

		gyro = new AnalogGyro(1);
		leftEnc = new Encoder(2, 3, false, Encoder::EncodingType::k1X);
		rightEnc = new Encoder(0,1, false, Encoder::EncodingType::k1X);
		//Configure for inches.t551
		leftEnc->SetDistancePerPulse(-0.06);
		rightEnc->SetDistancePerPulse(0.06);


		Autonomous::init(drive, gyro, leftEnc, rightEnc);

		if (fork() == 0) {
		            system("/home/lvuser/grip &");
		}
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

		rotation = *((double*)posChooser->GetSelected());

		//goal = *((std::string*)goalChooser->GetSelected());
		goal = "High";

		defenseCrossed = false;
		done = false;

		//Make sure to reset the encoder!
		leftEnc->Reset();
		rightEnc->Reset();
		gyro->Reset();
		autoCounter = 0;
	}


	void AutonomousPeriodic()
	{
		printf("Distance: %f\n", rightEnc->GetDistance());
		if(done) {
			autoCounter++;
			if (autoCounter > 10) {
				launchPiston->Set(DoubleSolenoid::Value::kReverse);
			}
		} else {
		if (autoSelected == "Approach Only") {
			done = Autonomous::approachOnly();
		} else if (!defenseCrossed) {
				if(Autonomous::crossFunctions.find(autoSelected) != Autonomous::crossFunctions.end()) {
					bool (*crossFunction)() = Autonomous::crossFunctions.at(autoSelected);
					defenseCrossed = crossFunction();
				} else {
					//doNothing();
				}
			} else {
				//after we cross...
				float difference = gyro->GetAngle() - rotation;

				if (difference > 10) {
					drive->ArcadeDrive(0.0,difference * 0.3);
				} else {
					if (goal == "High") {
							Autonomous::alignWithGoal(drive, launch1, launch2, winch, table);
							launchPiston->Set(DoubleSolenoid::Value::kForward);
					} else {

					}
					done = true;
				}
			}
		}
	}

	void TeleopInit()
	{
		leftEnc->Reset();
		rightEnc->Reset();
		gyro->Reset();

		powerCounter = 0;
	}

	void TeleopPeriodic()
	{
		camera->GetImage(frame);
		imaqDrawShapeOnImage(frame, frame, { 10, 10, 100, 100 }, DrawMode::IMAQ_DRAW_VALUE, ShapeMode::IMAQ_SHAPE_OVAL, 0.0f);
		CameraServer::GetInstance()->SetImage(frame);


		printf("Left Encoder: %i, Right Encoder: %i, Gyro: %f\n", leftEnc->Get(), rightEnc->Get(), gyro->GetAngle());

		drive->ArcadeDrive(driveStick);
		drive->SetMaxOutput((1-driveStick->GetThrottle())/2);
		//printf("%f\n", (1-stick->GetThrottle())/2);
		//leftMotor->Set(0.1);
		//rightMotor->Set(0.1);

		if (shootStick->GetRawAxis(3) > 0.5) {
			launch1->Set(1.0);
			launch2->Set(1.0);
		} else if (shootStick->GetRawAxis(2) > 0.5) {
			printf("Power Counter: %i\n", powerCounter);
			if (powerCounter < POWER_MAX) {
				powerCounter++;
				launch1->Set(-0.7);
				launch2->Set(-0.7);
			} else {
				launch1->Set(-0.3);
				launch2->Set(-0.3);
			}
			Wait(0.1);
		} else {
			launch1->Set(0.0);
			launch2->Set(0.0);
			powerCounter = 0.0;
		}

		if (shootStick->GetRawButton(5)) {
			winch->Set(-0.5);
		} else if (shootStick->GetRawButton(6)) {
			winch->Set(0.5);
		} else {
			winch->Set(0.0);
		}

		if (shootStick->GetRawButton(1)) {
			launchPiston->Set(DoubleSolenoid::Value::kForward);
		} else {
			launchPiston->Set(DoubleSolenoid::Value::kReverse);
		}

		if (shootStick->GetRawButton(2)) {
			tiltPiston->Set(DoubleSolenoid::Value::kForward);
		} else {
			tiltPiston->Set(DoubleSolenoid::Value::kReverse);
		}
	}

	void TestPeriodic()
	{
		lw->Run();
		printf("Left Encoder: %i, Right Encoder: %i, Gyro: %f\n", leftEnc->Get(), rightEnc->Get(), gyro->GetAngle());

	}
};

START_ROBOT_CLASS(Robot)
