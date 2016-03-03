
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
	CANTalon talS2;
	CANTalon talI1;
	Compressor cps;
	DoubleSolenoid ds1;
	DoubleSolenoid ds2;
	Solenoid s1;
	Solenoid s2;
	Solenoid SolenoidIntake;
	Encoder lenc;
	Encoder renc;
	Encoder shooterenc;
	Timer timer;
	//DigitalInput diMinSwitch;
	//PIDController spdcontrol;


public:
	Robot() :
		rdF(0, 3), rdR(1, 4), jsE(2), jsL(1), jsR(0), vctR(5), vctL(2), talS1(1), talS2(2), talI1(0), cps(), ds1(0), ds2(1), s1(2), s2(3), SolenoidIntake(4), lenc(0, 1, false, Encoder::EncodingType::k2X),
		renc(2, 3, false, Encoder::EncodingType::k2X), shooterenc(4, 5, false, Encoder::EncodingType::k2X){
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
		double shooterwheels = this->shooterenc.GetDistance();

		if (index == 0) {
			if (rcount <= 4000 && lcount <= 4000) {
				rdF.SetLeftRightMotorOutputs(1, -1);
				rdR.SetLeftRightMotorOutputs(1, -1);
				vctR.Set(-1);
				vctL.Set(1);
			}
		} else if (index == 1) {
			if () {  //use vision code to define if statement
				s1.Set(true);
				while (shooterwheels < 1200) {
					talS1.Set(-1);
					talS2.Set(1);
					if (shooterwheels > 200) {
						s2.Set(true);
					}
				}
			}
		} else if (index == 2) {
			s1.Set(true);
			while (shooterwheels < 1200) {
				talS1.Set(-1);
				/talS2.Set(1);
								if (shooterwheels > 200) {
									s2.Set(true);
								}
							}
		} else if (index == 3) {
		} else if (index == 4) {
			if (rcount <= 4000 && lcount <= 4000) {
				rdF.SetLeftRightMotorOutputs(1, -1);
				rdR.SetLeftRightMotorOutputs(1, -1);
				vctR.Set(-1);
				vctL.Set(1);
			}
			if () {  //use vision code to define if statement
				s1.Set(true);
				while (shooterwheels < 1200) {
					talS1.Set(-1);
					talS2.Set(1);
					if (shooterwheels > 200) {
						s2.Set(true);
					}
				}
			}
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
		float time = timer.Get();
		float shooterInput = 1 * jsE.GetY();
		talI1.Set(shooterInput);

		 if (jsE.GetRawButton(3)) {
			timer.Start();
			timer.Reset();
		 	s1.Set(true);
		 	if (time >= 0) {
		 		s2.Set(true);
		 		timer.Stop();
		 	}
		 } else if (jsE.GetRawButton(4)) {
			 timer.Reset();
			 timer.Start();
			 s1.Set (false);
			 if (time >= 0) {
			 	s2.Set (false);
			 	timer.Stop();
			 }
		 } else if (jsE.GetRawButton(5)) {
			 timer.Reset();
			 timer.Start();
			 s1.Set (false);
			 if (time >= 0) {
			 	s2.Set(true);
			  	timer.Stop();
			  }
		 } else if (jsE.GetRawButton(6)) {
			 timer.Reset();
			 timer.Start();
			 s1.Set (true);
			 if (time >= 0) {
			 	s2.Set(false);
			  	timer.Stop();
			  }
		 }
	}

	void Random() {
		if (jsE.GetRawButton(1)) {
			SolenoidIntake.Set(true);
		} else if (jsE.GetRawButton(2)) {
			SolenoidIntake.Set(false);
		}
	}

	void PID() {
		float shootSpeed = talS1.Get();
		talS1.SetControlMode(CANSpeedController::kSpeed);
		talS1.SetFeedbackDevice(CANTalon::QuadEncoder);

		if (jsE.GetRawButton(7)) {
			talS1.SetPID(1.0, 0.0, 0.0, 0.0);
			talS2.Set(shootSpeed);
		}
	}

	void Gearshift() {
		int gearshift = -3;

		if (jsL.GetRawButton(2)) { //Close
			ds1.Set(DoubleSolenoid::kForward);
			gearshift = 1;
			printf( "{%d},\n", gearshift);
		} else if (jsL.GetRawButton(3)) { //Open
			ds1.Set(DoubleSolenoid::kReverse);
			gearshift = -1;
			printf( "{%d},\n", gearshift);
		} else {
			ds1.Set(DoubleSolenoid::kOff);
			printf( "{%d},\n", gearshift);
		}

		if (jsL.GetRawButton(2)) {
			ds2.Set(DoubleSolenoid::kForward);
		} else if (jsL.GetRawButton(3)) {
			ds2.Set(DoubleSolenoid::kReverse);
		} else {
			ds2.Set(DoubleSolenoid::kOff);
		}
	}

	double lastCountl = -1;
	double lastCountr = -1;

	void TeleopPeriodic() {
		//printf("Hello!,\n");
		talS1.SetFeedbackDevice(CANTalon::QuadEncoder);
		double lcount = talS1.GetEncPosition();
		//double lcount = this->lenc.GetDistance();
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
		Random();
		PID();
		Shooter();

	}

	void TestPeriodic()
	{

	}
};

START_ROBOT_CLASS(Robot)

