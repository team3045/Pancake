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
	Compressor cps;
	DoubleSolenoid leftShift;
	DoubleSolenoid rightShift;
	Solenoid bottomRamp;
	Solenoid topRamp;
	DoubleSolenoid SolenoidIntake;
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
		robotDriveFront(0, 3), robotDriveRear(1, 4), jsE(2), jsL(1), jsR(0),
		midWheelRight(5), midWheelLeft(2), shooterControl(1), shooterFollow(3), feeder(0), turret(2),
		cps(), leftShift(0, 0, 1), rightShift(0, 2, 3), bottomRamp(1, 0), topRamp(1, 1), SolenoidIntake(0, 4, 5),
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
		shooterControl.SetControlMode(CANSpeedController::kPercentVbus);
		//shooterControl.SetFeedbackDevice(CANTalon::QuadEncoder);
		//shooterControl.ConfigEncoderCodesPerRev(20);
		shooterControl.ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Brake);
		shooterControl.Set(0);

		shooterFollow.SetControlMode(CANSpeedController::kFollower);
		//shooterFollow.SetClosedLoopOutputDirection(true);
		shooterFollow.ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Brake);
		shooterFollow.Set(1);

		feeder.SetControlMode(CANSpeedController::kPercentVbus);

		turret.SetControlMode(CANSpeedController::kPercentVbus);

		leftdriveenc.Reset();
		leftdriveenc.SetMaxPeriod(.1);
		leftdriveenc.SetMinRate(10);
		leftdriveenc.SetDistancePerPulse(2.8125);
		leftdriveenc.SetReverseDirection(false);
		leftdriveenc.SetSamplesToAverage(7);

		rightdriveenc.Reset();
		rightdriveenc.SetMaxPeriod(.1);
		rightdriveenc.SetMinRate(10);
		rightdriveenc.SetDistancePerPulse(2.8125);
		rightdriveenc.SetReverseDirection(false);
		rightdriveenc.SetSamplesToAverage(7);

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
		double lcount = this->leftdriveenc.GetDistance();
		double rcount = this->rightdriveenc.GetDistance();
		double shooterwheels = this->shooterenc.GetDistance();

		auto grip = NetworkTable::GetTable("grip");

		auto Xcenter = grip->GetNumberArray("ContoursReport/centerX", 1);
		auto Ycenter = grip->GetNumberArray("ContoursReport/centerY", 1);


		if (index == 0) {
			if (rcount <= 4000 && lcount <= 4000) {
				robotDriveFront.SetLeftRightMotorOutputs(1, -1);
				robotDriveRear.SetLeftRightMotorOutputs(1, -1);
				midWheelRight.Set(-1);
				midWheelLeft.Set(1);
			}
		} else if (index == 1) {
			int TurnRatio;
			TurnRatio = ((Xcenter[1])/(240));

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
		}
	}

	void TeleopInit()
	{
		this->cps.Start();
	}

	void Drive() {
		//printf("Drive\n");
		float leftInput = 1 * jsL.GetY();
		float rightInput = 1 * jsR.GetY();
		float RightMidWheelInput = 1 * jsR.GetY();
		float LeftMidWheelInput = 1 * jsL.GetY();
		robotDriveFront.SetLeftRightMotorOutputs(leftInput, rightInput);
		robotDriveRear.SetLeftRightMotorOutputs(leftInput, rightInput);
		midWheelRight.Set(-1 * RightMidWheelInput);
		midWheelLeft.Set(LeftMidWheelInput);
	}

	double lastCounta = 0;

	void Accelerometer() {
		if ( !ahrs ) return;

		bool reset_yaw_button_pressed = DriverStation::GetInstance().GetStickButton(0,1);
		if ( reset_yaw_button_pressed ) {
		   ahrs->ZeroYaw();
		}

		SmartDashboard::PutBoolean( "IMU_Connected",        ahrs->IsConnected());

		SmartDashboard::PutNumber(  "IMU_Yaw",              ahrs->GetYaw());

		SmartDashboard::PutNumber(  "Displacement_X",       ahrs->GetDisplacementX() );
		SmartDashboard::PutNumber(  "Displacement_Y",       ahrs->GetDisplacementY() );

		/*
		SmartDashboard::PutNumber( "auto_LoopCounter1",     0);
		SmartDashboard::PutBoolean( "IMU_Connected",        ahrs->IsConnected());
		SmartDashboard::PutNumber(  "IMU_Yaw",              ahrs->GetYaw());
		SmartDashboard::PutNumber(  "IMU_Pitch",            ahrs->GetPitch());
		SmartDashboard::PutNumber(  "IMU_Roll",             ahrs->GetRoll());
		SmartDashboard::PutNumber(  "IMU_CompassHeading",   ahrs->GetCompassHeading());
		SmartDashboard::PutNumber(  "IMU_Update_Count",     ahrs->GetUpdateCount());
		SmartDashboard::PutNumber(  "IMU_Byte_Count",       ahrs->GetByteCount());

		SmartDashboard::PutNumber(  "IMU_TotalYaw",         ahrs->GetAngle());
		SmartDashboard::PutNumber(  "IMU_YawRateDPS",       ahrs->GetRate());

		SmartDashboard::PutNumber(  "IMU_Accel_X",          ahrs->GetWorldLinearAccelX());
		SmartDashboard::PutNumber(  "IMU_Accel_Y",          ahrs->GetWorldLinearAccelY());
		SmartDashboard::PutBoolean( "IMU_IsMoving",         ahrs->IsMoving());
		SmartDashboard::PutNumber(  "IMU_Temp_C",           ahrs->GetTempC());
		SmartDashboard::PutBoolean( "IMU_IsCalibrating",    ahrs->IsCalibrating());

		SmartDashboard::PutNumber(  "Velocity_X",           ahrs->GetVelocityX() );
		SmartDashboard::PutNumber(  "Velocity_Y",           ahrs->GetVelocityY() );
		SmartDashboard::PutNumber(  "Displacement_X",       ahrs->GetDisplacementX() );
		SmartDashboard::PutNumber(  "Displacement_Y",       ahrs->GetDisplacementY() );

		SmartDashboard::PutNumber(  "RawGyro_X",            ahrs->GetRawGyroX());
		SmartDashboard::PutNumber(  "RawGyro_Y",            ahrs->GetRawGyroY());
		SmartDashboard::PutNumber(  "RawGyro_Z",            ahrs->GetRawGyroZ());
		SmartDashboard::PutNumber(  "RawAccel_X",           ahrs->GetRawAccelX());
		SmartDashboard::PutNumber(  "RawAccel_Y",           ahrs->GetRawAccelY());
		SmartDashboard::PutNumber(  "RawAccel_Z",           ahrs->GetRawAccelZ());
		SmartDashboard::PutNumber(  "RawMag_X",             ahrs->GetRawMagX());
		SmartDashboard::PutNumber(  "RawMag_Y",             ahrs->GetRawMagY());
		SmartDashboard::PutNumber(  "RawMag_Z",             ahrs->GetRawMagZ());
		SmartDashboard::PutNumber(  "IMU_Temp_C",           ahrs->GetTempC());
		*/

		//AHRS::BoardYawAxis yaw_axis = ahrs->GetBoardYawAxis();
		/*
		SmartDashboard::PutString(  "YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
		SmartDashboard::PutNumber(  "YawAxis",              yaw_axis.board_axis );
		*/

		/*double accel = ahrs->GetRawAccelX();
		if (accel != lastCounta) {
			printf("{%.2f},\n", accel);
		}
		lastCounta = accel;

		if (leftShift.Get() == DoubleSolenoid::kReverse) { //Close
			if (abs(accel - lastCounta) < 0.005) {
				leftShift.Set(DoubleSolenoid::kForward);
			}

		}
		double shift = leftShift.Get();
		printf("shift: %.2f \n", shift);*/
	}

	void Turret() {
		//if (jsE.GetX() > 0) {
			turret.SetControlMode(CANSpeedController::kPercentVbus);
			float turretInput = (0.10) * jsE.GetY();
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

	void Shooter() {
		float time = timer.Get();
		sw = limitSwitchA.Get();

		if (sw == 1 && lastswitch == 0) {
			bottomRamp.Set (true);
		}

		if (jsR.GetRawButton(3)) {
			feeder.Set(1);
		} else if (jsR.GetRawButton(2)){
			feeder.Set(-1);
		} else {
			feeder.Set(0);
		}


		 if (jsE.GetRawButton(2)) {
				bottomRamp.Set(false);
			//}
		 } else {
			 bottomRamp.Set (true);
		 }
		 if (jsE.GetRawButton(3)) {
			 topRamp.Set(false);
		 } else {
		 	topRamp.Set(true);
		 }

		 if (jsE.GetRawButton(1)) {
			 timer.Reset();
			 timer.Start();
			 bottomRamp.Set(true);
			 //if (time >= 0.05) {
				 topRamp.Set(false);
			 //}
		 }

		 lastswitch = sw;
	}

	void Random() {
		if (jsE.GetRawButton(8)) {
			SolenoidIntake.Set(DoubleSolenoid::kForward);
		} else if (jsE.GetRawButton(9)) {
			SolenoidIntake.Set(DoubleSolenoid::kReverse);
		} else {
			SolenoidIntake.Set(DoubleSolenoid::kOff);
		}
	}

	int loopCount = 0;

	bool fOn = true;
	bool fOff = true;

	void PID() {
		if (jsE.GetRawButton(6)) {
			shooterControl.SetControlMode(CANSpeedController::kSpeed);
			shooterControl.SetFeedbackDevice(CANTalon::QuadEncoder);
			shooterControl.SetPID(0.25, 0.01, 0.001, 1.0);
			shooterControl.Set(5000.0);
			feeder.Set(-1);
			//shooterFollow.SetControlMode(CANSpeedController::kPercentVbus);
			//shooterFollow.Set(1);
			//shooterControl.EnableControl();

			if (fOn) {
				printf("motoron \n");
				fOn = false;
				fOff = true;
			}
		} else if (jsE.GetRawButton(7)) {
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

	void Gearshift() {
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
		//double scount = shooterControl.GetEncPosition();
		if (lcount != lastCountl) {
			printf("{%.2f},\n", lcount);
		}
		lastCountl = lcount;

		double rcount = rightdriveenc.GetDistance();
		if (rcount != lastCountr) {
			printf("{%.2f},\n", rcount);
		}
		lastCountr = rcount;

		this->Drive();
		this->Gearshift();
		//Accelerometer();
		this->Turret();
		this->Random();
		this->PID();
		this->Shooter();

	}

	void TestPeriodic()
	{

	}
};

START_ROBOT_CLASS(Robot)

