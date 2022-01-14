package org.usfirst.frc.team1806.robot.subsystems;

//import org.omg.CORBA.PRIVATE_MEMBER;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.Kinematics;
import org.usfirst.frc.team1806.robot.RobotMap;
import org.usfirst.frc.team1806.robot.RobotState;
import org.usfirst.frc.team1806.robot.Vision.VisionServer;
import org.usfirst.frc.team1806.robot.loop.Loop;
import org.usfirst.frc.team1806.robot.loop.Looper;
import org.usfirst.frc.team1806.robot.path.Path;
import org.usfirst.frc.team1806.robot.path.PathFollower;
import org.usfirst.frc.team1806.robot.util.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;

/**
 * The DrivetrainSubsystem deals with all of the drivetrain
 * code used on the robot.
 */
public class DriveTrainSubsystem implements Subsystem {

	boolean debug = true;

	VisionServer mVisionServer;
	Target mostRecentTarget;
	double mostRecentTargetTimestamp;

	public enum DriveStates {
		DRIVING, // Ya old normal dirivng
		CREEP, // Creep for percise movement
		VISION, // Vision tracking!
		TURN_TO_HEADING, // Turn using PID
		DRIVE_TO_POSITION, // Drive to Position using SRX PID
		PATH_FOLLOWING,
		VELOCITY_SETPOINT,
		WIGGLE,
		DRIVE_TO_STALL,
		PARKING_BRAKE,
		NOTHING // Used on init
	}

	private static DriveTrainSubsystem mDriveTrainSubsystem = new DriveTrainSubsystem(); //Only ever 1 instance of drivetrain.
	private static final int kLowGearPositionControlSlot = 0;
	private static final int kHighGearVelocityControlSlot = 1;

	public static DriveTrainSubsystem getInstance() {
		return mDriveTrainSubsystem;
	}

	private static double inchesPerSecondToRPM(double inches_per_second) {
		return (inches_per_second / Constants.kDriveInchesPerRevolution) * 60;
	}


	private static double inchesPerSecondToRpm(double inches_per_second) {
		return inchesToRotations(inches_per_second) * 60;
	}

	private static double inchesToRPM(double inches) {
		return inches / Constants.kDriveInchesPerRevolution;
	}

	private static double inchesToRotations(double inches) {
		return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
	}

	/**
	 * Check if the current drive state is using position control.
	 */
	protected static boolean usesTalonPositionControl(DriveStates state) {
		if (state == DriveStates.DRIVE_TO_POSITION ||
				state == DriveStates.TURN_TO_HEADING) {
			return true;
		}
		return false;
	}

	/**
	 * Check if the proposed DriveStates needs VelocityControl
	 * @param state the DriveStates to determine if velocity control is needed
	 * @return true or false depending on if the VelocityControl is needed or not
	 */
	protected static boolean usesTalonVelocityControl(DriveStates state) {
		if (state == DriveStates.VELOCITY_SETPOINT || state == DriveStates.PATH_FOLLOWING || state == DriveStates.VISION) {
			return true;
		}
		return false;
	}

	//Initialize all of the drive motors
	private CANSparkMax masterLeft, masterRight, leftA, rightA, leftB, rightB;
	private DoubleSolenoid shifter;
	private NavX navx;
	private PathFollower mPathFollower;
	private Rotation2d mTargetHeading = new Rotation2d();
	private boolean mIsOnTarget = false;

	private double currentTimeStamp;
	private double lastTimeStamp;
	private double leftEncoderDistance, rightEncoderDistance;
	private double leftVelocity, rightVelocity;

	public DriveStates getmDriveStates() {
		return mDriveStates;
	}

	// State Control
	private DriveStates mDriveStates;
	private RobotState mRobotState = RobotState.getInstance();
	private Path mCurrentPath = null;
	private boolean mIsHighGear = false;
	public static boolean isWantedLowPID = false;
	private boolean mIsBrakeMode = false;

	private Loop mLoop = new Loop() {
		/** Different states that constantly need to be ran
		 * 	Runs different functions based on the current state of the DriveTrain
		 * @param timestamp current robot runtime in seconds
		 */
		@Override
		public synchronized void onLoop(double timestamp) {
			lastTimeStamp = currentTimeStamp;
			currentTimeStamp = timestamp;
			leftEncoderDistance = masterLeft.getEncoder().getPosition();
			rightEncoderDistance = masterRight.getEncoder().getPosition();
			leftVelocity = masterLeft.getEncoder().getVelocity();
			rightVelocity = masterRight.getEncoder().getVelocity();
			synchronized (DriveTrainSubsystem.this) {
				switch (mDriveStates) {
					case CREEP:
						return;
					case DRIVE_TO_POSITION:
						return;
					case DRIVING:
						return;
					case NOTHING:
						return;
					case PATH_FOLLOWING:
						if (mPathFollower != null) {
							updatePathFollower(timestamp);
						}
						return;
					case TURN_TO_HEADING:
						updateTurnToHeading(timestamp);
						return;
					case VELOCITY_SETPOINT:
						return;
					case VISION:
						updateVision();
						return;
					case WIGGLE:
						return;

					case DRIVE_TO_STALL:
						leftDrive(Constants.kStallPower);
						rightDrive(Constants.kStallPower);
						return;
					case PARKING_BRAKE:
						parkingBrakeHandler();
						return;
					default:
						return;

				}
			}

		}

		@Override
		public synchronized void onStart(double timestamp) {
			synchronized (DriveTrainSubsystem.this) {
				setOpenLoop(DriveSignal.NEUTRAL);
				setNeutralMode(false);
				navx.reset();
				setMaxDrivePower(1);
			}
		}

		@Override
		public synchronized void onStop(double timestamp) {
			setOpenLoop(DriveSignal.NEUTRAL);

		}
	};

	/** instantiating the motors
	 *  sets currentLimit
	 */
	public DriveTrainSubsystem() {
		//init the all of the motor controllers
		masterLeft = new CANSparkMax(RobotMap.masterLeft, CANSparkMaxLowLevel.MotorType.kBrushless);
		masterRight = new CANSparkMax(RobotMap.masterRight, CANSparkMaxLowLevel.MotorType.kBrushless);

		leftA = new CANSparkMax(RobotMap.leftA, CANSparkMaxLowLevel.MotorType.kBrushless);
		rightA = new CANSparkMax(RobotMap.rightA, CANSparkMaxLowLevel.MotorType.kBrushless);

		leftB = new CANSparkMax(RobotMap.leftB, CANSparkMaxLowLevel.MotorType.kBrushless);
		rightB = new CANSparkMax(RobotMap.rightB, CANSparkMaxLowLevel.MotorType.kBrushless);

		//Follow for right side
        rightA.follow(masterRight);
        rightB.follow(masterRight);

		// Follow for left side
        leftA.follow(masterLeft);
        leftB.follow(masterLeft);

        masterLeft.setSmartCurrentLimit(85);
        leftA.setSmartCurrentLimit(85);
        leftB.setSmartCurrentLimit(85);

        masterRight.setSmartCurrentLimit(85);
        rightA.setSmartCurrentLimit(85);
        rightB.setSmartCurrentLimit(85);


		//Set Encoders for each side of the talon
        //TODO: configure after REV updates their software. https://www.chiefdelphi.com/t/connecting-external-encoders-to-spark-max/345039
		/*masterLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		masterRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		masterRight.setSensorPhase(true);
		masterLeft.setSensorPhase(true);
		*/
		masterLeft.setInverted(true);
		leftA.setInverted(true);
		leftB.setInverted(true);

//		//Invert the right side
		masterRight.setInverted(false);
		rightA.setInverted(false);
		rightB.setInverted(false);

		leftEncoderDistance = 0;
		rightEncoderDistance = 0;
		leftVelocity = 0;
		rightVelocity = 0;

		mVisionServer = VisionServer.getInstance();
		mostRecentTarget = null;
		mostRecentTargetTimestamp = 0;

		// init solenoids
		shifter = new DoubleSolenoid(RobotMap.module2Number, RobotMap.shiftLow, RobotMap.shiftHigh);
//		init navx
		navx = new NavX(SPI.Port.kMXP);

		reloadGains();
		mDriveStates = DriveStates.NOTHING;
		setMaxDrivePower(1);
	}

	private synchronized void configureTalonsForPositionControl() {
		if (usesTalonPositionControl(mDriveStates)) {
			setMaxDrivePower(1);
			// We entered a position control state.
			System.out.println("Configuring position control");
			masterLeft.setIAccum(0);
			masterRight.setIAccum(0);
			setBrakeMode();
		} else {
			System.out.println("Oh no! DIdn't set Position control");
		}
	}

	private synchronized void configureTalonsForSpeedControl() {
		if (!usesTalonVelocityControl(mDriveStates)) {
			// We entered a velocity control state.
			setMaxDrivePower(1);
			System.out.println("Configuring speed control");
			setBrakeMode();
			updateVelocitySetpoint(0,0);
		}
	}

	public synchronized void forceDoneWithPath() {
		if (mDriveStates == DriveStates.PATH_FOLLOWING && mPathFollower != null) {
			mPathFollower.forceFinish();
		}
		else {
			System.out.println("Robot is not in path following mode");
		}

		mDriveStates = DriveStates.DRIVING;
		setOpenLoop(new DriveSignal(0, 0 , true));
		mPathFollower = null;
	}

	public synchronized Rotation2d getGyroYaw() {
		return navx.getYaw();
		//return new Rotation2d();
	}

	private double zeroRoll = 0;
	public synchronized double getGyroRoll() {
		return navx.getRoll() - zeroRoll;
	}
	public void zeroGyroRoll() {
		zeroRoll = navx.getRoll();
	}

	public double getLeftDistanceInches() {
		return leftEncoderDistance * Constants.kDriveInchesPerRevolution;
	}

	public double getLeftVelocityInchesPerSec() {
		return leftVelocity * Constants.kDriveInchesPerRevolution / 60;
	}

	public double getRightDistanceInches() {
		return rightEncoderDistance * Constants.kDriveInchesPerRevolution;
	}

	public double getRightVelocityInchesPerSec() {
		return rightVelocity * Constants.kDriveInchesPerRevolution / 60;
	}

	public boolean isCreeping() {
		return mDriveStates == DriveStates.CREEP;
	}

	public synchronized boolean isDoneWithPath() {
		if (mDriveStates == DriveStates.PATH_FOLLOWING && mPathFollower != null) {
			boolean isFinished = mPathFollower.isFinished();
			if(isFinished){
				mPathFollower = null;
			}
			return isFinished;
		} else {
			System.out.println("Robot is not in path following mode");
			mPathFollower = null;
			return true;
		}
	}

	public synchronized boolean isDoneWithTurn() {
		if (mDriveStates == DriveStates.TURN_TO_HEADING) {
			return mIsOnTarget;
		} else {
			System.out.println("Robot is not in turn to heading mode");
			return false;
		}
	}

	public boolean isHighGear() {
		return mIsHighGear;
	}

	public boolean isPositionControl(DriveStates state) {
		if (state == DriveStates.DRIVE_TO_POSITION ||
				state == DriveStates.TURN_TO_HEADING) {
			return true;
		} else {
			return false;
		}
	}

	public void leftDrive(double output) {
	    masterLeft.getPIDController().setReference(output, ControlType.kDutyCycle);
	}

	@Override
	public void outputToSmartDashboard() {
		if(debug) {
			SmartDashboard.putNumber(Constants.kDriveTrainKey + "position left (in)", getLeftDistanceInches());
			SmartDashboard.putNumber(Constants.kDriveTrainKey + "position right (in)", getRightDistanceInches());
			SmartDashboard.putNumber(Constants.kDriveTrainKey + "velocity left (in/sec)", getLeftVelocityInchesPerSec());
			SmartDashboard.putNumber(Constants.kDriveTrainKey + "velocity right (in/sec)", getRightVelocityInchesPerSec());
			SmartDashboard.putNumber(Constants.kDriveTrainKey + "encoder count left", leftEncoderDistance);
			SmartDashboard.putNumber(Constants.kDriveTrainKey + "encoder count right", rightEncoderDistance);

			SmartDashboard.putString(Constants.kDriveTrainKey + "drive state", returnDriveState());
			SmartDashboard.putNumber(Constants.kDriveTrainKey + "navX yaw", getGyroYaw().getDegrees());
		   SmartDashboard.putBoolean(Constants.kDriveTrainKey + "is high gear?", isHighGear());

			SmartDashboard.putNumber(Constants.kDriveTrainKey + "temp masterLeft", masterLeft.getMotorTemperature());
			SmartDashboard.putNumber(Constants.kDriveTrainKey + "temp leftA", leftA.getMotorTemperature());
			SmartDashboard.putNumber(Constants.kDriveTrainKey + "temp leftB", leftB.getMotorTemperature());
			SmartDashboard.putNumber(Constants.kDriveTrainKey + "temp masterRight", masterRight.getMotorTemperature());
			SmartDashboard.putNumber(Constants.kDriveTrainKey + "temp rightA", rightA.getMotorTemperature());
			SmartDashboard.putNumber(Constants.kDriveTrainKey + "temp rightB", rightB.getMotorTemperature());

			SmartDashboard.putNumber(Constants.kDriveTrainKey + "amps masterLeft", masterLeft.getOutputCurrent());
			SmartDashboard.putNumber(Constants.kDriveTrainKey + "amps leftA", leftA.getOutputCurrent());
			SmartDashboard.putNumber(Constants.kDriveTrainKey + "amps leftB", leftB.getOutputCurrent());
			SmartDashboard.putNumber(Constants.kDriveTrainKey + "amps masterRight", masterRight.getOutputCurrent());
			SmartDashboard.putNumber(Constants.kDriveTrainKey + "amps rightA", rightA.getOutputCurrent());
			SmartDashboard.putNumber(Constants.kDriveTrainKey + "amps rightB", rightB.getOutputCurrent());

		}
	}


	@Override
	public synchronized void registerEnabledLoops(Looper enabledLooper) {
		enabledLooper.register(mLoop);

	}

	@Override
	public void setDebug(boolean _debug) {
		debug = _debug;
	}

	public synchronized void reloadGains() {
		reloadLowGearPositionGains();
		reloadHighGearVelocityGains();
	}



	/** sets a pid on a motor controller position (high gear high speed)
	 *
	 * @param motorController to set the pid values on
	 */
	public synchronized void reloadHighGearPositionGainsForController(CANSparkMax motorController) {
	    motorController.getPIDController().setP(Constants.kDriveHighGearVelocityKp, kHighGearVelocityControlSlot);
        motorController.getPIDController().setI(Constants.kDriveHighGearVelocityKi, kHighGearVelocityControlSlot);
        motorController.getPIDController().setD(Constants.kDriveHighGearVelocityKd, kHighGearVelocityControlSlot);
        motorController.getPIDController().setFF(Constants.kDriveHighGearVelocityKf, kHighGearVelocityControlSlot);
        motorController.getPIDController().setIZone(Constants.kDriveHighGearVelocityIZone, kHighGearVelocityControlSlot);

        /*TODO: Do we need this?
		motorController.configClosedloopRamp(Constants.kDriveHighGearVelocityRampRate, Constants.kDriveTrainPIDSetTimeout);
         */
	}

	/** sets a pid on a motor controller position (high gear low speed)
	 *
	 * @param motorController to set the pid values on
	 */
	public synchronized void reloadHighGearPositionGainsForControllerLowPID(CANSparkMax motorController) {
        motorController.getPIDController().setP(Constants.kDriveHighGearVelocityLowKp, kHighGearVelocityControlSlot);
        motorController.getPIDController().setI(Constants.kDriveHighGearVelocityLowKi, kHighGearVelocityControlSlot);
        motorController.getPIDController().setD(Constants.kDriveHighGearVelocityLowKd, kHighGearVelocityControlSlot);
        motorController.getPIDController().setFF(Constants.kDriveHighGearVelocityLowKf, kHighGearVelocityControlSlot);
        motorController.getPIDController().setIZone(Constants.kDriveHighGearVelocityLowIZone, kHighGearVelocityControlSlot);

                /*TODO: Do we need this?
		motorController.configClosedloopRamp(Constants.kDriveHighGearVelocityLowRampRate, Constants.kDriveTrainPIDSetTimeout);
         */
	}

	/** reloads the velocity pid based on whether or not the current wanted pid is high speed or low speed
	 *
	 */
	public synchronized void reloadHighGearVelocityGains() {
		if (isWantedLowPID) {
			System.out.println("low PID");
			reloadHighGearPositionGainsForControllerLowPID(masterLeft);
			reloadHighGearPositionGainsForControllerLowPID(masterRight);
		} else {
			System.out.println("high PID");
			reloadHighGearPositionGainsForController(masterLeft);
			reloadHighGearPositionGainsForController(masterRight);
		}
	}

	/** Resets masterLeft and materRight low gear position gains
	 *
	 */
	public synchronized void reloadLowGearPositionGains() {
		reloadLowGearPositionGainsForController(masterLeft);
		reloadLowGearPositionGainsForController(masterRight);
	}
	/** sets a pid on a motor controller position (low gear)
	 *
	 * @param motorController to set the pid values on
	 */
	public synchronized void reloadLowGearPositionGainsForController(CANSparkMax motorController) {
        motorController.getPIDController().setP(Constants.kDriveLowGearPositionKp,kLowGearPositionControlSlot);
        motorController.getPIDController().setI(Constants.kDriveLowGearPositionKi,kLowGearPositionControlSlot);
        motorController.getPIDController().setD(Constants.kDriveLowGearPositionKd,kLowGearPositionControlSlot);
        motorController.getPIDController().setFF(Constants.kDriveLowGearPositionKf,kLowGearPositionControlSlot);
        motorController.getPIDController().setIZone(Constants.kDriveLowGearPositionIZone,kLowGearPositionControlSlot);
        motorController.getPIDController().setSmartMotionMaxVelocity(Constants.kDriveLowGearMaxVelocity,kLowGearPositionControlSlot);
        motorController.getPIDController().setSmartMotionMaxAccel(Constants.kDriveLowGearMaxAccel, Constants.kLiftPositionControlPIDSlot);
        /*TODO:DO we need this?
		motorController.configClosedloopRamp(Constants.kDriveLowGearPositionRampRate, Constants.kDriveTrainPIDSetTimeout);
		*/

	}

	public synchronized void resetNavx() {
		navx.reset();
	}

	public synchronized void resetYaw() {
		navx.zeroYaw();
	}

	public String returnDriveState() {
		return mDriveStates.toString();
	}

	/**
	 *
	 * Drives only the right side at a percent
	 *
	 * @param output Wanted percent
	 */
	public void rightDrive(double output) {
	    masterRight.getPIDController().setReference(output, ControlType.kDutyCycle);
	}

	/**
	 * Sets the talons for brake mode
	 */
	public synchronized void setBrakeMode() {
		//set for auto
        masterLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
		leftA.setIdleMode(CANSparkMax.IdleMode.kBrake);
		leftB.setIdleMode(CANSparkMax.IdleMode.kBrake);

		masterRight.setIdleMode(CANSparkMax.IdleMode.kBrake);
		rightA.setIdleMode(CANSparkMax.IdleMode.kBrake);
		rightB.setIdleMode(CANSparkMax.IdleMode.kBrake);
	}

	/**
	 * Sets the talons up for coast mode
	 */
	public synchronized void setCoastMode() {
		// set for driving
		masterLeft.setIdleMode(CANSparkMax.IdleMode.kCoast);
		leftA.setIdleMode(CANSparkMax.IdleMode.kCoast);
		leftB.setIdleMode(CANSparkMax.IdleMode.kCoast);

		masterRight.setIdleMode(CANSparkMax.IdleMode.kCoast);
		rightA.setIdleMode(CANSparkMax.IdleMode.kCoast);
		rightB.setIdleMode(CANSparkMax.IdleMode.kCoast);

	}

	/**
	 * Used in OI to set the robot up for creep mode
	 *
	 * @param signal Our left and right drivetrain speed
	 */
	public synchronized void setCreepMode(DriveSignal signal) {
		if (mDriveStates != DriveStates.CREEP) {
			mDriveStates = DriveStates.CREEP;
			System.out.println("CREEP");
		}
		masterLeft.getPIDController().setReference(signal.getLeft() / 2, ControlType.kDutyCycle);
		masterRight.getPIDController().setReference(signal.getRight() / 2, ControlType.kDutyCycle);
	}

	//////

	/**
	 * Used to change the robot heading when needed
	 *
	 * @param angle Wanted angle
	 */
	public synchronized void setGyroAngle(Rotation2d angle) {
		navx.reset();
		navx.setAngleAdjustment(angle);
	}

	/**
	 * Used to set highgear
	 * @param wantsHighGear
	 * it's a boolean saying if you want it or not
	 */
	public synchronized void setHighGear(boolean wantsHighGear) {
        if (wantsHighGear != mIsHighGear) {
            mIsHighGear = wantsHighGear;
            shifter.set(wantsHighGear ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
        }
    }

	/**
	 * Sets the neutral mode for the drive train.
	 *
	 * @param brake if 1, the drive train will go into brake mode, 0 will put it into coast mode
	 */
	public synchronized void setNeutralMode(boolean brake) {
		mIsBrakeMode = brake;
		CANSparkMax.IdleMode currentMode = brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast;
		masterRight.setIdleMode(currentMode);
		rightA.setIdleMode(currentMode);
		masterLeft.setIdleMode(currentMode);
		leftA.setIdleMode(currentMode);
	}

	/**
	 * Used to control robot in OI
	 *
	 * @param signal Signal is our left drivetrain and right drivetrian power
	 */
	public synchronized void setOpenLoop(DriveSignal signal) {
		if (mDriveStates != DriveStates.DRIVING) {
			mDriveStates = DriveStates.DRIVING;
			setNeutralMode(false);
		}
        masterLeft.getPIDController().setReference(signal.getLeft(), ControlType.kDutyCycle);
		masterRight.getPIDController().setReference(signal.getRight(), ControlType.kDutyCycle);
	}

	/**
	 * Used to update velocity setpoint and set state
	 *
	 * @param left_inches_per_sec  Left inches per second
	 * @param right_inches_per_sec Right inches per second
	 */
	public synchronized void setVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
		configureTalonsForSpeedControl();
		mDriveStates = DriveStates.VELOCITY_SETPOINT;
		updateVelocitySetpoint(left_inches_per_sec, right_inches_per_sec);
	}

	/**
	 * Used to setup our pure pursuit controller for auto
	 *
	 * @param path     This is the wanted path that we want to drive on
	 * @param reversed If the robot is reversed or not
	 */
	public synchronized void setWantDrivePath(Path path, boolean reversed) {
		reloadHighGearVelocityGains();
		if (mCurrentPath != path || mDriveStates != DriveStates.PATH_FOLLOWING) {
			System.out.println("Setting Path_Following");
			configureTalonsForSpeedControl();
			RobotState.getInstance().resetDistanceDriven();
			mPathFollower = new PathFollower(path, reversed,
					new PathFollower.Parameters(
							new Lookahead(Constants.kMinLookAhead, Constants.kMaxLookAhead,
									Constants.kMinLookAheadSpeed, Constants.kMaxLookAheadSpeed),
							Constants.kInertiaSteeringGain, Constants.kPathFollowingProfileKp,
							Constants.kPathFollowingProfileKi, Constants.kPathFollowingProfileKv,
							Constants.kPathFollowingProfileKffv, Constants.kPathFollowingProfileKffa,
							Constants.kPathFollowingMaxVel, Constants.kPathFollowingMaxAccel,
							Constants.kPathFollowingGoalPosTolerance, Constants.kPathFollowingGoalVelTolerance,
							Constants.kPathStopSteeringDistance));
			mDriveStates = DriveStates.PATH_FOLLOWING;
			mCurrentPath = path;
		} else {
			System.out.println("setting velocity to 0");
			setVelocitySetpoint(0, 0);
		}
	}

	/**
	 * Configures the drivebase to turn to a desired heading
	 */
	public synchronized void setWantTurnToHeading(Rotation2d heading) {
		if (mDriveStates != DriveStates.TURN_TO_HEADING) {
			mDriveStates = DriveStates.TURN_TO_HEADING;
			configureTalonsForPositionControl();
			setMaxDrivePower(Constants.kDriveTurnMaxPower);
			updatePositionSetpoint(getLeftDistanceInches(), getRightDistanceInches());
		}
		if (Math.abs(heading.inverse().rotateBy(mTargetHeading).getDegrees()) > 1E-3) {
			mTargetHeading = heading;
			mIsOnTarget = false;
		}
	}

	boolean wasWigglin = false;
	boolean startingWiggle = false;
	boolean finishingWiggle = false;
	double wiggleAdjust = 0;
	double kWiggle = 6*3.1415;
	double kWiggleAmplitude = .15;
	Timer wiggleTime = new Timer();

	public synchronized void wiggleHandler(boolean wiggleReq) {
		startingWiggle = wiggleReq && !wasWigglin;
		finishingWiggle = !wiggleReq && wasWigglin;


		if(startingWiggle) {
			wiggleTime.reset();
			wiggleTime.start();
			mDriveStates = DriveStates.WIGGLE;
		}
		else if(finishingWiggle) {
			wiggleTime.stop();
			wiggleTime.reset();
			mDriveStates = DriveStates.DRIVING;

			leftDrive(0);
			rightDrive(0);
		}

		if(wiggleReq) {
			wiggleAdjust = Math.sin(wiggleTime.get() * kWiggle) * .2;

			leftDrive(.25 - wiggleAdjust);
			rightDrive(.25 + wiggleAdjust);
		}


		wasWigglin = wiggleReq;
	}


	boolean wasPushing = false;
	boolean startingPush = false;
	boolean finishingPush = false;
	boolean isTimedOut = false;
	double pushTimeStamp = 0;

	public synchronized boolean driveToStall(boolean pushReq, boolean stopReq) {
		startingPush = pushReq && !wasPushing;


		if(startingPush) {
			System.out.println("starting push");
			mDriveStates = DriveStates.DRIVE_TO_STALL;
			leftDrive(Constants.kStallPower);
			rightDrive(Constants.kStallPower);
			pushTimeStamp = currentTimeStamp;
		}

		isTimedOut = (currentTimeStamp - pushTimeStamp > Constants.kStallTimeout);
		finishingPush = (leftVelocity < Constants.kStallSpeed && rightVelocity < Constants.kStallSpeed && currentTimeStamp - pushTimeStamp > Constants.kStallWaitPeriod) || isTimedOut || stopReq;
		wasPushing = pushReq;
		if(leftVelocity < Constants.kStallSpeed && mDriveStates == DriveStates.DRIVE_TO_STALL) {
			System.out.println("time to stall " + (currentTimeStamp - pushTimeStamp));
		}

		if(finishingPush && mDriveStates == DriveStates.DRIVE_TO_STALL) {
			System.out.println("finishing push");
			System.out.println("speed low? " + (leftVelocity < Constants.kStallSpeed ));
			System.out.println("wait period? " + (currentTimeStamp - pushTimeStamp > Constants.kStallWaitPeriod));
			System.out.println("is timed out? " + isTimedOut);
			mDriveStates = DriveStates.DRIVING;
			pushTimeStamp = 0;
			leftDrive(0);
			rightDrive(0);
			return true;
		}
		return false;

	}

	private boolean parkingBrakeIsStopped = false;
	private double parkingBrakePositionLeft = 0.0;
	private double parkingBrakePositionRight = 0.0;

	private synchronized boolean parkingBrakeHandler() {
		if(!parkingBrakeIsStopped) {
			masterLeft.getPIDController().setReference(0, ControlType.kVelocity);
			masterRight.getPIDController().setReference(0, ControlType.kVelocity);
			if(masterRight.getEncoder().getVelocity() < 100 && masterLeft.getEncoder().getVelocity() < 100)  {
				parkingBrakeIsStopped = true;
				parkingBrakePositionLeft = masterLeft.getEncoder().getPosition();
				parkingBrakePositionRight = masterRight.getEncoder().getPosition();
				reloadParkingBrakeGains(masterLeft);
				reloadParkingBrakeGains(masterRight);
			}
		}
		else {
			masterLeft.getPIDController().setReference(parkingBrakePositionLeft, ControlType.kPosition);
			masterRight.getPIDController().setReference(parkingBrakePositionRight, ControlType.kPosition);
		}

		return false;
	}

	private void reloadParkingBrakeGains(CANSparkMax motorController) { //TODO create seperate PID slot if needed
		motorController.getPIDController().setP(Constants.kDriveLowGearPositionKp,kLowGearPositionControlSlot);
		motorController.getPIDController().setI(Constants.kDriveLowGearPositionKi,kLowGearPositionControlSlot);
		motorController.getPIDController().setD(Constants.kDriveLowGearPositionKd,kLowGearPositionControlSlot);
		motorController.getPIDController().setFF(Constants.kDriveLowGearPositionKf,kLowGearPositionControlSlot);
		motorController.getPIDController().setIZone(Constants.kDriveLowGearPositionIZone,kLowGearPositionControlSlot);
	}

	public void startParkingBrake() {
		setBrakeMode();
		mDriveStates = DriveStates.PARKING_BRAKE;
		parkingBrakeIsStopped = false;
	}
	public void stopParkingBrake() {
		mDriveStates = DriveStates.DRIVING;
		parkingBrakeIsStopped = false;
		setCoastMode();
	}




	@Override
	public synchronized void stop() {
		stopDrive();

	}

	/**
	 * Stops the drivetrain completely
	 */
	public synchronized void stopDrive() {
		if (mDriveStates != DriveStates.DRIVING) {
			mDriveStates = DriveStates.DRIVING;
		}
		masterLeft.getPIDController().setReference(0, ControlType.kDutyCycle);
		masterRight.getPIDController().setReference(0, ControlType.kDutyCycle);
	}


	/**
	 * Called periodically when the robot is in path following mode. Updates the path follower with the robots latest
	 * pose, distance driven, and velocity, the updates the wheel velocity setpoints.
	 */
	private synchronized void updatePathFollower(double timestamp) {
		RigidTransform2d robot_pose = mRobotState.getLatestFieldToVehicle().getValue();
		Twist2d command = mPathFollower.update(timestamp, robot_pose,
				RobotState.getInstance().getDistanceDriven(), RobotState.getInstance().getPredictedVelocity().dx);
		if (!mPathFollower.isFinished()) {
			Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);
			updateVelocitySetpoint(setpoint.left, setpoint.right);
			SmartDashboard.putNumber("A Left Side Setpoint: ", setpoint.left);
			SmartDashboard.putNumber("A Right Side Setpoint: ", setpoint.right);
		} else {
			setOpenLoop(new DriveSignal(0, 0, false));
			//updateVelocitySetpoint(0, 0);
		}
	}

	/**
	 * Updates the talons to what position it will go to
	 *
	 * @param left_position_inches  Inches wanted
	 * @param right_position_inches Inches wanted
	 */
	private synchronized void updatePositionSetpoint(double left_position_inches, double right_position_inches) {
		if (usesTalonPositionControl(mDriveStates)) {
		    masterLeft.getPIDController().setReference(inchesToRPM(left_position_inches), ControlType.kPosition, kLowGearPositionControlSlot);
			masterRight.getPIDController().setReference(inchesToRPM(right_position_inches), ControlType.kPosition, kLowGearPositionControlSlot);
		} else {
			System.out.println("Hit a bad position control state");
            masterLeft.getPIDController().setReference(0, ControlType.kDutyCycle);
            masterRight.getPIDController().setReference(0, ControlType.kDutyCycle);
		}
	}

	/**
	 * Updates motor speed when turning to an angle
	 *
	 * @param timestamp Current timestamp, don't worry chris it's not used
	 */
	private synchronized void updateTurnToHeading(double timestamp) {
		final Rotation2d field_to_robot = mRobotState.getLatestFieldToVehicle().getValue().getRotation();
		// Figure out the rotation necessary to turn to face the goal.
		final Rotation2d robot_to_target = field_to_robot.inverse().rotateBy(mTargetHeading);

		// Check if we are on target
		final double kGoalPosTolerance = 3; // degrees TODO:CONSTANTS
		final double kGoalVelTolerance = 25.0; // inches per second
		if (Math.abs(robot_to_target.getDegrees()) < kGoalPosTolerance
				&& Math.abs(getLeftVelocityInchesPerSec()) < kGoalVelTolerance
				&& Math.abs(getRightVelocityInchesPerSec()) < kGoalVelTolerance) {
			// Use the current setpoint and base lock.
			mIsOnTarget = true;
			updatePositionSetpoint(getLeftDistanceInches(), getRightDistanceInches());
			return;
		}

		Kinematics.DriveVelocity wheel_delta = Kinematics
				.inverseKinematics(new Twist2d(0, 0, robot_to_target.getRadians()));
		updatePositionSetpoint(wheel_delta.left + getLeftDistanceInches(),
				wheel_delta.right + getRightDistanceInches());

		SmartDashboard.putNumber("Wanted Right Turn To Heading: ", wheel_delta.right + getRightDistanceInches());
		SmartDashboard.putNumber("Wanted Left Turn to Heading", wheel_delta.left + getLeftDistanceInches());
		SmartDashboard.putNumber("Current Right Turn To Heading", getRightDistanceInches());
		SmartDashboard.putNumber("Current Left Turn To Heading", getLeftDistanceInches());
	}

	/**
	 * Update velocity setpoint is used to send over our desired velocity from pure pursuit control
	 *
	 * @param left_inches_per_sec  Left side inches per second
	 * @param right_inches_per_sec right side inches per second
	 */
	private synchronized void updateVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
		if (usesTalonVelocityControl(mDriveStates)) {
			final double max_desired = Math.max(Math.abs(left_inches_per_sec), Math.abs(right_inches_per_sec));
			final double scale = max_desired > Constants.kDriveHighGearMaxSetpoint
					? Constants.kDriveHighGearMaxSetpoint / max_desired : 1.0;
			masterLeft.getPIDController().setReference(inchesPerSecondToRPM(left_inches_per_sec * scale), ControlType.kVelocity, kHighGearVelocityControlSlot);
			masterRight.getPIDController().setReference(inchesPerSecondToRPM(right_inches_per_sec * scale), ControlType.kVelocity, kHighGearVelocityControlSlot);
            
           /* System.out.println("Left Side Velocity : "+ left_inches_per_sec+ "  " + 
            		"Right Side Veloctiy: "+ right_inches_per_sec);*/


			SmartDashboard.putNumber("A Left Side Velocity", getLeftVelocityInchesPerSec());
			SmartDashboard.putNumber("A Right Side Velocity", getRightVelocityInchesPerSec());
		} else {
			System.out.println("Hit a bad velocity control state");
            masterLeft.getPIDController().setReference(0, ControlType.kDutyCycle);
            masterRight.getPIDController().setReference(0, ControlType.kDutyCycle);
		}
	}

	/**
	 * This method zeros the encoders of both sides of the drivetrain
	 */
	@Override
	public synchronized void zeroSensors() {
//		System.out.println("Zeroing drivetrain sensors...");
        masterLeft.getEncoder().setPosition(0);
        masterRight.getEncoder().setPosition(0);
        leftVelocity = 0;
        rightVelocity = 0;
        leftEncoderDistance =0;
        rightEncoderDistance = 0;
		navx.zeroYaw();
//   	 System.out.println("Drivetrain sensors zeroed!");
	}

	public double getLeftVoltage() {
		return masterLeft.getAppliedOutput();
	}
	public double getRightVoltage() {
		return masterRight.getAppliedOutput();
	}


	@Override
	public void writeToLog() {
		// TODO Auto-generated method stub

	}

	/**
	 * sets up a parking brake for climbing
	 * <p>
	 * TODO: Do it chris
	 */
	public void setParkingBrakeMode() {
		setBrakeMode();
	}

	/**
	 * sets the max power that the drivetrain can go
	 *
	 * @param power
	 */
	public void setMaxDrivePower(double power) {
	    masterLeft.getPIDController().setOutputRange(-power, power);
	    masterRight.getPIDController().setOutputRange(-power, power);

	}

	public void setWantVisionTracking(boolean wantVision){
		if(wantVision && mDriveStates != DriveStates.VISION){
			isWantedLowPID = true;
			mDriveStates = DriveStates.VISION;
			configureTalonsForSpeedControl();
			mostRecentTarget = null;
			mostRecentTargetTimestamp = 0;
		}
		else if(!wantVision && mDriveStates == DriveStates.VISION){
			mDriveStates = DriveStates.DRIVING;
			setCoastMode();
		}

	}

	public void updateVision(){
		double VISION_BASE_SPEED = 40;
		double PROPORTIONAL_GAIN_FOR_VISION = .075;
		Target wantedTarget = TargetHelper.getClosestTargetToRobot();
		double targetTimestamp = mVisionServer.getTargetsTimestamp();
		RigidTransform2d robotPose = RobotState.getInstance().getFieldToVehicle(targetTimestamp - Constants.kVisionExpectedCameraLag);
		if(wantedTarget != null){
			mostRecentTarget = wantedTarget;
			mostRecentTargetTimestamp = targetTimestamp;
		}
		if(mostRecentTarget != null){
			RigidTransform2d correctingRobotPose =RobotState.getInstance().getFieldToVehicle(mostRecentTargetTimestamp - Constants.kVisionExpectedCameraLag);
			double robotToTarget =mostRecentTarget.getRobotToTarget() + correctingRobotPose.getRotation().getDegrees();
			double visionCorrection = (robotToTarget * PROPORTIONAL_GAIN_FOR_VISION);

			updateVelocitySetpoint(VISION_BASE_SPEED - visionCorrection, VISION_BASE_SPEED + visionCorrection);
		}
		else{
			updateVelocitySetpoint(0, 0);
		}


	}

	public float getWorldLinearAccelX() {
		return navx.getWorldLinearAccelX();
	}

	public float getWorldLinearAccelY() {
		return navx.getWorldLinearAccelY();
	}

	public float getWorldLinearAccelZ() {
		return navx.getWorldLinearAccelZ();
	}


	public void goToHatchMode(){
		//nothing to do here
	}

	public void goToCargoMode(){
		//nothing to do here
	}

	public void retractAll() {
		//nothing to do here
	}
}


