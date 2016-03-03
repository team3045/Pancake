
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
	CANTalon talS1;
	//CANTalon talS2;
	CANTalon talI1;
	CANTalon talI2;
	Compressor cps;
	DoubleSolenoid ds1;
	DoubleSolenoid ds2;
	Solenoid s1;
	Solenoid s2;
	Encoder lenc;
	Encoder renc;
	//DigitalInput diMinSwitch;
	//PIDController spdcontrol;


public:
	Robot() :
		rdF(0, 3), rdR(1, 4), jsE(2), jsL(1), jsR(0), vctR(5), vctL(2), talS1(1), talI1(0), talI2(2), cps(), ds1(0), ds2(1), s1(2), s2(3), lenc(0, 1, false, Encoder::EncodingType::k2X),
		renc(2, 3, false, Encoder::EncodingType::k2X){
	}

private:
	void RobotInit() {
		//SmartDashboard::PutNumber("Encoder", -1);
		lenc.Reset();
		lenc.SetMaxPeriod(.1);
		lenc.SetMinRate(10);
		lenc.SetDistancePerPulse(2.8125);
		lenc.SetReverseDirection(false);
		lenc.SetSamplesToAverage(7);
		renc.Reset();
		renc.SetMaxPeriod(.1);
		renc.SetMinRate(10);
		renc.SetDistancePerPulse(2.8125);
		renc.SetReverseDirection(false);
		renc.SetSamplesToAverage(7);

		SmartDashboard::PutBoolean("Autonomous 0", true);
		SmartDashboard::PutBoolean("Autonomous 1", true);
		SmartDashboard::PutBoolean("Autonomous 2", true);
		SmartDashboard::PutBoolean("Autonomous 3", true);
		SmartDashboard::PutBoolean("Autonomous 4", true);
	}


	int pointer = 0;
	int index = 0;

	void AutonomousInit() {
		cps.Start();
		if (SmartDashboard::GetBoolean("Autonomous 0", true)) {
			index = 0;
		} else if (SmartDashboard::GetBoolean("Autonomous 1", true)) {
			index = 1;
		} else if (SmartDashboard::GetBoolean("Autonomous 2", true)) {
			index = 2;
		} else if (SmartDashboard::GetBoolean("Autonomous 3", true)) {
			index = 3;
		} else if (SmartDashboard::GetBoolean("Autonomous 4", true)) {
			index = 4;
		}
	}

	void AutonomousPeriodic() {
		float leftInput;
		float rightInput;
		double lcount = this->lenc.GetDistance();
		double rcount = this->renc.GetDistance();

		if (index == 0) {
			if (rcount <= 4000 && lcount <= 4000) {
				rdF.SetLeftRightMotorOutputs(1, -1);
				rdR.SetLeftRightMotorOutputs(1, -1);
				vctR.Set(-1);
				vctL.Set(1);
			}
		} else if (index == 1) {

		} else if (index == 2) {
		} else if (index == 3) {
		} else if (index == 4) {
		}
	}

	void TeleopInit()
	{
		this->cps.Start();
		printf( "HelloWorld!,\n");



	}

	void Drive() {
		float leftInput = 1 * jsL.GetY();
		float rightInput = 1 * jsR.GetY();
		float RightMidWheelInput = 1 * jsR.GetY();
		float LeftMidWheelInput = 1 * jsL.GetY();
		rdF.SetLeftRightMotorOutputs(leftInput, rightInput);
		rdR.SetLeftRightMotorOutputs(leftInput, rightInput);
		vctR.Set(-1 * RightMidWheelInput);
		vctL.Set(LeftMidWheelInput);
	}

	void Shooter() {
		float shooterInput = 1 * jsE.GetY();
		talI1.Set(shooterInput);
		talI2.Set(shooterInput);
		 /*
		 *if (jsE.GetRawButton(3)) {
		 *	s1.Set(true);
		 *	s2.Set(true);
		 *} else if (jsE.GetRawButton(4)) {
		 *	s1.Set(false);
		 *	s2.Set(false);
		 *}
		 */
	}

	void PID() {
		talS1.SetControlMode(CANSpeedController::kSpeed);
		talS1.SetFeedbackDevice(CANTalon::QuadEncoder);

		if (jsE.GetRawButton(1)) {
			talS1.SetPID(1.0, 0.0, 0.0, 0.0);
			//talS2.Set(talS1.Get());
		}
	}

	void Gearshift() {
		int gearshift = -3;

		if (this->jsL.GetRawButton(2)) { //Close
			this->ds1.Set(DoubleSolenoid::kForward);
			gearshift = 1;
			printf( "{%d},\n", gearshift);
		} else if (this->jsL.GetRawButton(3)) { //Open
			this->ds1.Set(DoubleSolenoid::kReverse);
			gearshift = -1;
			printf( "{%d},\n", gearshift);
		} else {
			this->ds1.Set(DoubleSolenoid::kOff);
			printf( "{%d},\n", gearshift);
		}

		if (this->jsL.GetRawButton(2)) {
			this->ds2.Set(DoubleSolenoid::kForward);
		} else if (this->jsL.GetRawButton(3)) {
			this->ds2.Set(DoubleSolenoid::kReverse);
		} else {
			this->ds2.Set(DoubleSolenoid::kOff);
		}
	}

	double lastCountl = -1;
	double lastCountr = -1;

	void TeleopPeriodic() {
		//printf("Hello!,\n");
		double lcount = this->lenc.GetDistance();
		if (lcount != lastCountl) {
			printf("{%.2f},\n", lcount);
		}
		lastCountl = lcount;

		double rcount = renc.GetDistance();
		if (rcount != lastCountr) {
			printf("{%.2f},\n", rcount);
		}
		lastCountr = rcount;

		Drive();
		Gearshift();
		//PID();
		Shooter();

		//gearshift code

	}

	void TestPeriodic()
	{

	}
};

START_ROBOT_CLASS(Robot)

