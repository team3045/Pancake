#include "WPILib.h"
#include "Commands/Command.h"
#include "Commands/ExampleCommand.h"
#include "CommandBase.h"
#include "AHRS.h"

	// I/O Map
	// Talons:
	//    shooterControl CANTalon 1 // shooter CIM motor (with quadrature encoder)
	//	  shooterFollow CANTalon 2 // shooter CIM motor (follower, without encoder)
	//	  feeder CANTalon 0 // intake motor
	//	  TBD CANTalon xxx // turret
	// Joysticks:
	//	jsE
	//	jsL
	//	jsR
	// Victors:
	//	robotDriveFront Victor PWM 0
	//	robotDriveFront Victor PWM 3
	//	robotDriveRear Victor PWM 1
	//	robotDriveRear Victor PWM 4
	//	midWheelRight Victor PWM 5
	//  midWheelLeft Victor PWM 2
	// Encoders:
	//   leftdriveenc 0 1 // left hand of drive chassis
	//   rightdriveenc 2 3 // right hand of drive chassis
	//   shooterenc 4 5 // ? shooter encoder
	// PCMs:
	//	 0 : 12 volt
	//	 1 : 24 volt
	// Solenoids
	//	bottomRamp Solenoid PCM 1 0
	//	topRamp Solenoid PCM 1 1
	//  SolenoidIntake Solenoid PCM 1 2
	// DoubleSolenoids
	//	leftShift DoubleSolenoid PCM 0 0 1
	//	rightShift DoubleSolenoid PCM 0 2 3
	// DIO
	//	limitSwitchA DigitalInput 8
	//	limitSwitchB DigitalInput 9

class Robot: public IterativeRobot  {
	Victor v0;
	Victor v1;
	Victor v3;
	Victor v4;
	RobotDrive robotDriveFront;
	RobotDrive robotDriveRear;
	Joystick jsE;
	Joystick jsL;
	Joystick jsR;
	Victor midWheelRight;
	Victor midWheelLeft;
	CANTalon shooterControl;
	CANTalon shooterFollow;
	CANTalon feeder;
	CANTalon turret;
	CANTalon feeder2;
	Compressor cps;
	DoubleSolenoid leftShift;
	DoubleSolenoid rightShift;
	DoubleSolenoid SolenoidIntake;
	Solenoid bottomRamp;
	Solenoid topRamp;
	Encoder leftdriveenc;
	Encoder rightdriveenc;
	Encoder shooterenc;
	Timer timer;
	std::shared_ptr<NetworkTable> table;
	AHRS *ahrs;
	LiveWindow *lw;
	DigitalInput limitSwitchA;
	DigitalInput limitSwitchB;
	int autoLoopCounter;


public:
	Robot() :
		v0(0), v1(1), v3(3), v4(4), robotDriveFront(v0, v3), robotDriveRear(v1, v4), jsE(2), jsL(1), jsR(0),
		midWheelRight(5), midWheelLeft(2), shooterControl(1), shooterFollow(3), feeder(0), turret(2), feeder2(5),
		cps(), leftShift(0, 0, 1), rightShift(0, 2, 3), SolenoidIntake (0, 4, 5), bottomRamp(1, 1), topRamp(1, 0),
		leftdriveenc(0, 1, false, Encoder::EncodingType::k2X),
		rightdriveenc(2, 3, false, Encoder::EncodingType::k2X),
		shooterenc(4, 5, false, Encoder::EncodingType::k2X),
		table(NULL),
		ahrs(NULL),
		lw(NULL),
		limitSwitchA(8), limitSwitchB(9), autoLoopCounter(0){
	}

private:
	void RobotInit() {
		//Initializes control motor of shooter to voltage mode
		shooterControl.SetControlMode(CANSpeedController::kPercentVbus);
		//shooterControl.SetFeedbackDevice(CANTalon::QuadEncoder);
		//shooterControl.ConfigEncoderCodesPerRev(20);
		shooterControl.ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Brake);
		shooterControl.Set(0);

		//Initializes follow motor of shooter to follow mode
		shooterFollow.SetControlMode(CANSpeedController::kFollower);
		//shooterFollow.SetClosedLoopOutputDirection(true);
		shooterFollow.ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Brake);
		shooterFollow.Set(1);

		//Initializes feeder motors to voltagge mode
		feeder.SetControlMode(CANSpeedController::kPercentVbus);
		feeder2.SetControlMode(CANSpeedController::kPercentVbus);

		turret.SetControlMode(CANSpeedController::kPercentVbus);

		//Initializes gearbox encoder on left side of drive train
		leftdriveenc.Reset();
		leftdriveenc.SetMaxPeriod(.1);
		leftdriveenc.SetMinRate(10);
		leftdriveenc.SetDistancePerPulse(2.8125);
		leftdriveenc.SetReverseDirection(false);
		leftdriveenc.SetSamplesToAverage(7);

		//Initializes gearbox encoder on right side of drive train
		rightdriveenc.Reset();
		rightdriveenc.SetMaxPeriod(.1);
		rightdriveenc.SetMinRate(10);
		rightdriveenc.SetDistancePerPulse(2.8125);
		rightdriveenc.SetReverseDirection(false);
		rightdriveenc.SetSamplesToAverage(7);

		//Initializes encoder on control motor of shooter
		shooterenc.Reset();
		shooterenc.SetMaxPeriod(.1);
		shooterenc.SetMinRate(10);
		shooterenc.SetDistancePerPulse(2.8125);
		shooterenc.SetReverseDirection(false);
		shooterenc.SetSamplesToAverage(7);

		table = NetworkTable::GetTable("datatable");
		lw = LiveWindow::GetInstance();

		system("/home/lvuser/grip &");
		/*try {
			ahrs = new AHRS(I2C::Port::kOnboard);
		} catch (std::exception ex ) {
			std::string err_string = "Error instantiating gyro: ";
			err_string += ex.what();
			DriverStation::ReportError(err_string.c_str());
		}
		if ( ahrs ) {
			LiveWindow::GetInstance()->AddSensor("IMU", "Gyro", ahrs);
		}*/
		//sets up Autonomous Control
		SmartDashboard::PutBoolean("Autonomous 0", true);
		SmartDashboard::PutBoolean("Autonomous 1", false);
		SmartDashboard::PutBoolean("Autonomous 2", false);
		SmartDashboard::PutBoolean("Autonomous 3", false);
		SmartDashboard::PutBoolean("Autonomous 4", false);
	}


	int pointer = 0;
	int index = 0;

	void AutonomousInit() {
		cps.Start();

		timer.Reset();
		timer.Start();

		SolenoidIntake.Set(DoubleSolenoid::kForward);

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

	//	front L 0
	//	front R 3
	//	rear L 1
	//	rear R 4
	//	mid L 2
	//	mid R 5 (why -1 * ?)
	void AutonomousPeriodic() {

		double lcount = this->leftdriveenc.GetDistance();
		double rcount = this->rightdriveenc.GetDistance();

		auto grip = NetworkTable::GetTable("grip");

		auto Xcenter = grip->GetNumberArray("ContoursReport/centerX", 1);
		auto Ycenter = grip->GetNumberArray("ContoursReport/centerY", 1);

		float time = timer.Get();

		if (index == 0) {
			// MPH - is the gearshift default correct for autonomous?
			float leftInput = -0.37; //EGH - sets left and right voltage input of drive train
			float rightInput = -0.37;
			leftdriveenc.Reset();
			rightdriveenc.Reset();
			if (time < 7.0) {
				robotDriveFront.SetLeftRightMotorOutputs(leftInput, rightInput); // MPH - maybe 0.25, 0.25 ? why is Right negative?  it's positive on TeleOp?
				robotDriveRear.SetLeftRightMotorOutputs(leftInput, rightInput); // maybe 0.25, 0.25 ? why is Right negative?  it's positive on TeleOp?
				midWheelLeft.Set(leftInput);
				midWheelRight.Set(-1 * rightInput); // EGH - midWheelRight victor is inverted - sets value inverted
			} else {
				robotDriveFront.SetLeftRightMotorOutputs(0, 0); //stops robot
				robotDriveRear.SetLeftRightMotorOutputs(0, 0);
				midWheelLeft.Set(0);
				midWheelRight.Set(0);
			}
			//printf("{%.2f},\n", rcount);
			printf("{%.2f},\n", time);
		} /*else if (index == 1) {
			double TurnRatio;
			TurnRatio = ((Xcenter[1])/(240));

				turret.Set((TurnRatio - 1));

			//if () {  //use vision code to define if statement
				shooterControl.SetControlMode(CANSpeedController::kPercentVbus);
				float time = timer.Get();
				bottomRamp.Set(false);
				topRamp.Set(true);

				timer.Reset();
				timer.Start();
				if (time < 2.0) {
					shooterControl.Set(1);
					if (0 < time < 2.0) {
						bottomRamp.Set(true);
						topRamp.Set(false);
					}
				}
		} else if (index == 2) {
			shooterControl.SetControlMode(CANSpeedController::kPercentVbus);
			float time = timer.Get();
			bottomRamp.Set(false);
			topRamp.Set(true);

			timer.Reset();
			timer.Start();
			if (time < 2.0) {
				shooterControl.Set(1);
				if (0 < time < 2.0) {
					bottomRamp.Set(true);
					topRamp.Set(false);
				}
			}
		} else if (index == 3) {
			shooterControl.SetControlMode(CANSpeedController::kPercentVbus);
			float time = timer.Get();
			bottomRamp.Set(false);
			topRamp.Set(true);

			timer.Reset();
			timer.Start();
			if (time < 2.0) {
				shooterControl.Set(1);
				if (0 < time < 2.0) {
					bottomRamp.Set(true);
					topRamp.Set(false);
				}
			}
		} else if (index == 4) {
			if (rcount <= 4000 && lcount <= 4000) {
				robotDriveFront.SetLeftRightMotorOutputs(1, -1);
				robotDriveRear.SetLeftRightMotorOutputs(1, -1);
				midWheelRight.Set(-1);
				midWheelLeft.Set(1);
			}
			shooterControl.SetControlMode(CANSpeedController::kPercentVbus);
			float time = timer.Get();
			bottomRamp.Set(false);
			topRamp.Set(true);

			timer.Reset();
			timer.Start();
			if (time < 2.0) {
				shooterControl.Set(1);
				if (0 < time < 2.0) {
					bottomRamp.Set(true);
					topRamp.Set(false);
				}
			}
		}*/
	}

	void TeleopInit()
	{
		this->cps.Start(); //starts compressor for rest of teleop - IMPORTANT
	}

	//	front L 0
	//	front R 3
	//	rear L 1
	//	rear R 4
	//	mid L 2
	//	mid R 5 (why -1 * ?) // MPH?
	void Drive() {
		//printf("Drive\n");
		float leftInput = 1 * jsL.GetY(); //Gets joystick input for tank drive
		float rightInput = 1 * jsR.GetY();
		float LeftMidWheelInput = 1 * jsL.GetY();
		float RightMidWheelInput = 1 * jsR.GetY();
		robotDriveFront.SetLeftRightMotorOutputs(leftInput, rightInput); //sets right and left front of drive train to joystick input value
		robotDriveRear.SetLeftRightMotorOutputs(leftInput, rightInput);
		midWheelLeft.Set(LeftMidWheelInput);
		midWheelRight.Set(-1 * RightMidWheelInput); // RightMidWheelInput victor inverted - (-1) sets the output value equivalent to rightInput
	}

	double lastCounta = 0;

	void Turret() {
		//if (jsE.GetX() > 0) {
			turret.SetControlMode(CANSpeedController::kPercentVbus);
			float turretInput = (0.10) * jsE.GetX();
			turret.Set(turretInput);
		//} else if (jsE.GetX() == 0) {
			if (jsE.GetRawButton(4)) {
				turret.SetControlMode(CANSpeedController::kPosition);
				turret.SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
				turret.SetClosedLoopOutputDirection(true);
				turret.SetPID(0.30, 0.01, 0.001);
				turret.SetEncPosition(0);
				turret.SetPosition(0);
			}
		//}
	}


	int lastswitch = 0;
	int sw = 0;

	void Shooter() { //
		float time = timer.Get();
		sw = limitSwitchA.Get();

		if (sw == 1 && lastswitch == 0) {
			bottomRamp.Set (true);
		}
		//runs feeder wheels
		if (jsR.GetRawButton(2)) { //in
			feeder.Set(1);
			feeder2.Set(-1);
		} else if (jsR.GetRawButton(3)){  //out
			feeder.Set(-1);
			feeder2.Set(1);
		} else { //stop
			feeder.Set(0);
			feeder2.Set(0);
		}


		 if (jsE.GetRawButton(2)) { //bottom piston down
				bottomRamp.Set(false);
			//}
		 } else {
			 bottomRamp.Set (true); //bottom piston up
		 }
		 if (jsE.GetRawButton(3)) { //top piston up
			 topRamp.Set(false);
		 } else {
		 	topRamp.Set(true); //top piston down
		 }

		 if (jsE.GetRawButton(1)) { //triggers shooting - bottom ramp up and top ramp up
			 timer.Reset();
			 timer.Start();
			 bottomRamp.Set(true);
			 //if (time >= 0.05) {
				 topRamp.Set(false);
			 //}
		 }

		 lastswitch = sw;
	}

	int loopCount = 0;

	bool fOn = true;
	bool fOff = true;

	void PID() {
		if (jsE.GetRawButton(6)) { //runs shooter wheels at 5000 rpm using PID loop
			shooterControl.SetControlMode(CANSpeedController::kSpeed);
			shooterControl.SetFeedbackDevice(CANTalon::QuadEncoder);
			shooterControl.SetPID(0.25, 0.01, 0.001, 1.0);
			shooterControl.Set(6000.0);
			feeder.Set(-1);
			//shooterFollow.SetControlMode(CANSpeedController::kPercentVbus);
			//shooterFollow.Set(1);
			//shooterControl.EnableControl();

			if (fOn) {
				printf("motoron \n");
				fOn = false;
				fOff = true;
			}
		} else if (jsE.GetRawButton(7)) { //stops shooter wheels - CAUTION: Voltage mode necessary to stop
			printf ("motoroff \n");
			shooterControl.SetControlMode(CANSpeedController::kPercentVbus);
			shooterControl.Set(0);
			feeder.Set(0);
			//shooterFollow.SetControlMode(CANSpeedController::kPercentVbus);
			//shooterFollow.Set(0);
			//shooterControl.EnableControl();

			if (fOff) {
				printf("motoron \n");
				fOn = true;
				fOff = false;
			}
		}

		int encPosition = shooterControl.GetEncPosition();
		float getX = shooterControl.Get(); // get speed, for example

		int brakeEnabled = shooterControl.GetBrakeEnableDuringNeutral(); // 0 = disabled; nz = brake en
		float outVolt = shooterControl.GetOutputVoltage(); //
		int quadEncoderVelocity = shooterControl.GetEncVel();

		SmartDashboard::PutNumber(  "Shooter Volts",              outVolt);
		SmartDashboard::PutNumber(  "Shooter Speed",              getX);


		if ((loopCount % 100) == 0) {
			printf("loopCount %d ;  pos: %d ; getX: %.2f ; brakeEnabled: %d ; quadEncoderVelocity: %d ; outVolt: %.2f \n",
					loopCount, encPosition, getX, brakeEnabled, quadEncoderVelocity, outVolt);
		}
		loopCount = loopCount + 1;
	}

	void Random () { //drops feeder wheels
		if (jsE.GetRawButton(8)) {
			SolenoidIntake.Set(DoubleSolenoid::kForward);
		} else if (jsE.GetRawButton(9)) {
			SolenoidIntake.Set(DoubleSolenoid::kReverse);
		} else {
			SolenoidIntake.Set(DoubleSolenoid::kOff);
		}
	}
	void Gearshift() { //drive train gearshift (high to low gear)
		if (jsL.GetRawButton(2)) { //Close
			leftShift.Set(DoubleSolenoid::kForward);
			rightShift.Set(DoubleSolenoid::kForward);
		} else if (jsL.GetRawButton(3)) { //Open
			leftShift.Set(DoubleSolenoid::kReverse);
			rightShift.Set(DoubleSolenoid::kReverse);
		} else {
			leftShift.Set(DoubleSolenoid::kOff);
			rightShift.Set(DoubleSolenoid::kOff);
		}
	}

	double lastCountl = -1;
	double lastCountr = -1;

	void TeleopPeriodic() {
		double lcount = leftdriveenc.GetDistance();
		if (lcount != lastCountl) {
			printf("{%.2f},\n", lcount);
		}
		lastCountl = lcount;

		double rcount = rightdriveenc.GetDistance();
		if (rcount != lastCountr) {
			printf("{%.2f},\n", rcount);
		}
		lastCountr = rcount;

		this->Drive(); //starts drivetrain
		this->Gearshift(); //starts gear shift
		//Accelerometer();
		this->Turret(); //starts turret rotation
		this->PID(); //starts shooter wheels
		this->Random(); //starts feeder pneumatics
		this->Shooter(); //starts feeder wheels and ramps

	}

	void TestPeriodic()
	{

	}
};

START_ROBOT_CLASS(Robot)

