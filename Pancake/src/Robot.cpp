
#include "WPILib.h"
#include "Commands/Command.h"
#include "Commands/ExampleCommand.h"
#include "CommandBase.h"
#include "AHRS.h"

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
	std::shared_ptr<NetworkTable> table;
	AHRS *ahrs;
	LiveWindow *lw;
	DigitalInput limitSwitchA;
	DigitalInput limitSwitchB;
	int autoLoopCounter;


public:
	Robot() :
		rdF(0, 3), rdR(1, 4), jsE(2), jsL(1), jsR(0),
		vctR(5), vctL(2), talS1(1), talS2(2), talI1(0),
		cps(), ds1(0, 0, 1), ds2(0, 2, 3), s1(1, 0), s2(1, 1), SolenoidIntake(1, 2),
		lenc(0, 1, false, Encoder::EncodingType::k2X),
		renc(2, 3, false, Encoder::EncodingType::k2X),
		shooterenc(4, 5, false, Encoder::EncodingType::k2X),
		table(NULL),
		ahrs(NULL),
		lw(NULL),
		limitSwitchA(1), limitSwitchB(2), autoLoopCounter(0){
	}

private:
	void RobotInit() {
		talS1.SetControlMode(CANSpeedController::kPercentVbus);
		//talS1.SetFeedbackDevice(CANTalon::QuadEncoder);
		//talS1.ConfigEncoderCodesPerRev(20);
		talS1.ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Brake);
		talS1.Set(0);

		talS2.SetControlMode(CANSpeedController::kFollower);
		talS2.SetClosedLoopOutputDirection(true);
		talS2.ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Brake);
		talS2.Set(1);


		talI1.SetControlMode(CANSpeedController::kPercentVbus);
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

		shooterenc.Reset();
		shooterenc.SetMaxPeriod(.1);
		shooterenc.SetMinRate(10);
		shooterenc.SetDistancePerPulse(2.8125);
		shooterenc.SetReverseDirection(false);
		shooterenc.SetSamplesToAverage(7);

		table = NetworkTable::GetTable("datatable");
		lw = LiveWindow::GetInstance();

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
			/*if () {  //use vision code to define if statement
				s1.Set(true);
				while (shooterwheels < 1200) {
					talS1.Set(-1);
					talS2.Set(1);
					if (shooterwheels > 200) {
						s2.Set(true);
					}
				}
			}*/
		} else if (index == 2) {
			s1.Set(true);
			while (shooterwheels < 1200) {
				talS1.Set(-1);
				talS2.Set(1);
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
			/*if () {  //use vision code to define if statement
				s1.Set(true);
				while (shooterwheels < 1200) {
					talS1.Set(-1);
					talS2.Set(1);
					if (shooterwheels > 200) {
						s2.Set(true);
					}
				}
			}*/
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
		rdF.SetLeftRightMotorOutputs(leftInput, rightInput);
		rdR.SetLeftRightMotorOutputs(leftInput, rightInput);
		vctR.Set(-1 * RightMidWheelInput);
		vctL.Set(LeftMidWheelInput);
	}

	double lastCounta = 0;

	void Accelerometer() {
		if ( !ahrs ) return;

		bool reset_yaw_button_pressed = DriverStation::GetInstance().GetStickButton(0,1);
		if ( reset_yaw_button_pressed ) {
		   ahrs->ZeroYaw();
		}
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

		if (ds1.Get() == DoubleSolenoid::kReverse) { //Close
			if (abs(accel - lastCounta) < 0.005) {
				ds1.Set(DoubleSolenoid::kForward);
			}

		}
		double shift = ds1.Get();
		printf("shift: %.2f \n", shift);*/
	}

	int lastswitch = 0;
	int sw = 0;

	void Shooter() {
		float time = timer.Get();
		sw = limitSwitchA.Get();

		if (sw == 1 && lastswitch == 0) {
			s1.Set (true);
		}

		if (jsR.GetRawButton(3)) {
			talI1.Set(1);
		} else if (jsR.GetRawButton(2)){
			talI1.Set(-1);
		} else {
			talI1.Set(0);
		}


		 if (jsE.GetRawButton(2)) {
				s1.Set(false);
			//}
		 } else {
			 s1.Set (true);
		 }
		 if (jsE.GetRawButton(3)) {
			 s2.Set(false);
		 } else {
		 	s2.Set(true);
		 }

		 if (jsE.GetRawButton(1)) {
			 timer.Reset();
			 timer.Start();
			 s1.Set(true);
			 //if (time >= 0.05) {
				 s2.Set(false);
			 //}
		 }

		 lastswitch = sw;
	}

	void Random() {
		if (jsR.GetRawButton(3)) {
			SolenoidIntake.Set(true);
		} else if (jsR.GetRawButton(2)) {
			SolenoidIntake.Set(false);
		}
	}

	int loopCount = 0;

	bool fOn = true;
	bool fOff = true;

	void PID() {
		if (jsE.GetRawButton(6)) {
			talS1.SetControlMode(CANSpeedController::kSpeed);
			talS1.SetFeedbackDevice(CANTalon::QuadEncoder);
			talS1.SetPID(0.25, 0.01, 0.001, 1.0);
			talS1.Set(5000.0);
			//talS1.EnableControl();

			if (fOn) {
				printf("motoron \n");
				fOn = false;
				fOff = true;
			}
		} else if (jsE.GetRawButton(7)) {
			printf ("motoroff \n");
			talS1.SetControlMode(CANSpeedController::kPercentVbus);
			talS1.Set(0);
			//talS1.EnableControl();

			if (fOff) {
				printf("motoron \n");
				fOn = true;
				fOff = false;
			}
		}

		int encPosition = talS1.GetEncPosition();
		float getX = talS1.Get(); // get speed, for example
		int brakeEnabled = talS1.GetBrakeEnableDuringNeutral(); // 0 = disabled; nz = brake en
		float outVolt = talS1.GetOutputVoltage(); //
		int quadEncoderVelocity = talS1.GetEncVel();

		if ((loopCount % 100) == 0) {
			printf("loopCount %d ;  pos: %d ; getX: %.2f ; brakeEnabled: %d ; quadEncoderVelocity: %d ; outVolt: %.2f \n",
					loopCount, encPosition, getX, brakeEnabled, quadEncoderVelocity, outVolt);
		}
		loopCount = loopCount + 1;
	}

	void Gearshift() {
		//int gearshift = -3;

		if (jsL.GetRawButton(2)) { //Close
			ds1.Set(DoubleSolenoid::kForward);
			gearshift = 1;
			//printf( "{%d},\n", gearshift);
		} else if (jsL.GetRawButton(3)) { //Open
			ds1.Set(DoubleSolenoid::kReverse);
			gearshift = -1;
			//printf( "{%d},\n", gearshift);
		} else {
			ds1.Set(DoubleSolenoid::kOff);
			//printf( "{%d},\n", gearshift);
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
		double lcount = lenc.GetDistance();
		//double scount = talS1.GetEncPosition();
		if (lcount != lastCountl) {
			printf("{%.2f},\n", lcount);
		}
		lastCountl = lcount;

		double rcount = renc.GetDistance();
		if (rcount != lastCountr) {
			printf("{%.2f},\n", rcount);
		}
		lastCountr = rcount;

		this->Drive();
		this->Gearshift();
		//Accelerometer();
		//this->Random();
		this->PID();
		this->Shooter();

	}

	void TestPeriodic()
	{

	}
};

START_ROBOT_CLASS(Robot)

