
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
	//CANTalon talS1;
	//CANTalon talS2;
	CANTalon talI1;
	//CANTalon talI2;
	Compressor cps;
	DoubleSolenoid ds1;
	DoubleSolenoid ds2;
	//DoubleSolenoid ds3;
	//DoubleSolenoid ds4;
	//DoubleSolenoid ds5;
	Encoder enc;
	//DigitalInput diMinSwitch;


public:
	Robot() :
		rdF(0, 3), rdR(1, 4), jsE(2), jsL(1), jsR(0), vctR(5), vctL(2), talI1(0), cps(), ds1(0, 1), ds2(2, 3), enc(0, 1, false, Encoder::EncodingType::k2X){
	}

private:
	void RobotInit() {
		//SmartDashboard::PutNumber("Encoder", -1);
		this->enc.Reset();
		this->enc.SetMaxPeriod(.1);
		this->enc.SetMinRate(10);
		this->enc.SetDistancePerPulse(2.8125);
		this->enc.SetReverseDirection(false);
		this->enc.SetSamplesToAverage(7);
	}

	void AutonomousInit() {
		this->cps.Start();
	}

	void AutonomousPeriodic() {
		/*
		 *
		 */
	}

	void TeleopInit()
	{
		this->cps.Start();
		printf( "HelloWorld!,\n");


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
	}

	void Shooter() {
		float shooterInput = 1 * this->jsE.GetY();
		 this->talI1.Set(shooterInput);
		 //this->talI2.Set(shooterInput);

		 //PID  system enter here or shooter wheels
		 /*if (this->jsE.GetRawButton(1)) {
		 	this->talS1.SetPID(0.3,0,0,0.025);
		 	this->talS1.SetIzone(0.0);
		 }*/
		 /*} else if (this->jsE.GetRawButton(2)) {
		 *	this->talS1.Set(-1);
		 *	this->talS2.Set(1);
		 *} else {
		 *	this->vctS1.Set(0);
		 *	this->vctS2.Set(0);
		 *}
		 *
		 *if (this->jsE.GetRawButton(5)) {
		 *	this->ds3.Set(DoubleSolenoid::kForward);
		 *} else if (this->jsE.GetRawButton(6)) {
		 *	this->ds3.Set(DoubleSolenoid::kReverse);
		 *} else {
		 *	this->ds3.Set(DoubleSolenoid::kOff);
		 *}
		 *
		 *if (this->jsE.GetRawButton(3)) {
		 *	this->ds4.Set(DoubleSolenoid::kForward);
		 *} else if (this->jsE.GetRawButton(4)) {
		 *	this->ds4.Set(DoubleSolenoid::kReverse);
		 *} else {
		 *	this->ds4.Set(DoubleSolenoid::kOff);
		 *}
		 *
		 *if (this->jsE.GetRawButton(7)) {
		 *	this->ds5.Set(DoubleSolenoid::kForward);
		 *} else if (this->jsE.GetRawButton(8)) {
		 *	this->ds5.Set(DoubleSolenoid::kReverse);
		 *} else {
		 *	this->ds5.Set(DoubleSolenoid::kOff);
		 *}
		 */
	}

	double lastCount = -1;

	void TeleopPeriodic() {
		int gearshift = -3;
		//printf("Hello!,\n");
		double count = this->enc.GetDistance();
		if (count != lastCount) {
			printf("{%.2f},\n", count);
		}
		lastCount = count;

		this->Drive();
			this->Shooter();

		//gearshift code
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


	}

	void TestPeriodic()
	{

	}
};

START_ROBOT_CLASS(Robot)

