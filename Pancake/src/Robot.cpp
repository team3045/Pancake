
#include "WPILib.h"
#include "Commands/Command.h"
#include "Commands/ExampleCommand.h"
#include "CommandBase.h"

class Robot: public IterativeRobot  {
	RobotDrive rdF;
	RobotDrive rdR;
	Joystick jsE;
	Joystick jsL;
	Joystick jsR;
	Victor vctR;
	Victor vctL;
	//Victor vctE;
	Compressor cps;
	DoubleSolenoid ds1;
	DoubleSolenoid ds2;
	Encoder enc;
	//DigitalInput diMinSwitch;


public:
	Robot() :
		rdF(0, 3), rdR(1, 4), jsE(2), jsL(1), jsR(0), vctR(5), vctL(2), cps(), ds1(0, 1), ds2(2, 3), enc(0, 1, false, Encoder::EncodingType::k4X){
	}

private:
	void RobotInit() {
		SmartDashboard::PutNumber("Encoder", -1);
	}

	void AutonomousInit() {
	}

	void AutonomousPeriodic() {
	}

	void TeleopInit()
	{
		this->cps.Start();


	}

	void Drive() {
		float leftInput = 1 * this->jsL.GetY();
		float rightInput = 1 * this->jsR.GetY();
		float RightMidWheelInput = 1 * this->jsR.GetY();
		float LeftMidWheelInput = 1 * this->jsL.GetY();
		this->rdF.SetLeftRightMotorOutputs(leftInput, rightInput);
		this->rdR.SetLeftRightMotorOutputs(leftInput, rightInput);
		this->vctR.Set(-1 * RightMidWheelInput);
		this->vctL.Set(LeftMidWheelInput);
		//this->rdF.SetLeftRightMotorOutputs(1, 1);
		//this->rdR.SetLeftRightMotorOutputs(-1, -1);
		//this->vctR.Set(-1);
		//this->vctL.Set(1);


	}

	void TeleopPeriodic() {
		int gearshift = 0;
		int count = enc->Get();
		this->Drive();
		//this->cps.Start();


		/*if (this->jsL.GetRawButton(1)) {
			this->cps.Start();
		}*/

		 if (this->jsL.GetRawButton(2)) { //Close
			this->ds1.Set(DoubleSolenoid::kForward);
			gearshift = 1;
			printf( "{%d},\n", gearshift);
		} else if (this->jsL.GetRawButton(3)) //Open
				{
			this->ds1.Set(DoubleSolenoid::kReverse);
			gearshift = -1;
			printf( "{%d},\n", gearshift);
		} else {
			this->ds1.Set(DoubleSolenoid::kOff);

			//printf( "{%d},\n", gearshift);
		}

		if (this->jsL.GetRawButton(2)) {
			this->ds2.Set(DoubleSolenoid::kForward);
		} else if (this->jsL.GetRawButton(3)) {
			this->ds2.Set(DoubleSolenoid::kReverse);
		} else {
			this->ds2.Set(DoubleSolenoid::kOff);
		}

		printf("%d,\n", count);
	}

	void TestPeriodic()
	{

	}
};

START_ROBOT_CLASS(Robot)

