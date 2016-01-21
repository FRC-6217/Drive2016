#include "WPILib.h"

class Robot: public IterativeRobot
{
private:
	LiveWindow *lw = LiveWindow::GetInstance();
	SendableChooser *chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";
	std::string autoSelected;

	VictorSP *leftMotor, *rightMotor;
	RobotDrive *drive;
	Joystick *stick;

	DoubleSolenoid *test1;

	void RobotInit()
	{
		chooser = new SendableChooser();
		chooser->AddDefault(autoNameDefault, (void*)&autoNameDefault);
		chooser->AddObject(autoNameCustom, (void*)&autoNameCustom);
		SmartDashboard::PutData("Auto Modes", chooser);

		leftMotor = new VictorSP(1);
		rightMotor = new VictorSP(0);

		rightMotor->SetInverted(true);
		leftMotor->SetInverted(true);
		drive = new RobotDrive(leftMotor, rightMotor);

		drive->SetMaxOutput(0.5);

		stick = new Joystick(0);

		test1 = new DoubleSolenoid(0,1);
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

		if(autoSelected == autoNameCustom){
			//Custom Auto goes here
		} else {
			//Default Auto goes here
		}
	}

	void AutonomousPeriodic()
	{
		if(autoSelected == autoNameCustom){
			test1->Set(DoubleSolenoid::Value::kForward);
			Wait(1.0);
			test1->Set(DoubleSolenoid::Value::kReverse);
			Wait(1.0);
		} else {
			//Default Auto goes here
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
	}
};

START_ROBOT_CLASS(Robot)
