package org.usfirst.frc.team1806.robot.subsystems;

import java.util.function.DoubleSupplier;

//import org.omg.CORBA.PRIVATE_MEMBER;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.Kinematics;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotMap;
import org.usfirst.frc.team1806.robot.RobotState;
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

	private static DriveTrainSubsystem mDriveTrainSubsystem = new DriveTrainSubsystem(); // Only ever 1 instance of
																							// drivetrain.

	public static DriveTrainSubsystem getInstance() {
		return mDriveTrainSubsystem;
	}

	private static double inchesToCounts(double inches) {
		return inches / Constants.kDriveInchesPerCount;
	}

	/**
	 * Check if the current drive state is using position control.
	 */
	protected static boolean usesPositionPID(DriveStates state) {
		if (state == DriveStates.DRIVE_TO_POSITION ||
				state == DriveStates.TURN_TO_HEADING) {
			return true;
		}
		return false;
	}

	/**
	 * Check if the proposed DriveStates needs VelocityControl
	 * 
	 * @param state the DriveStates to determine if velocity control is needed
	 * @return true or false depending on if the VelocityControl is needed or not
	 */
	protected static boolean usesVelocityPID(DriveStates state) {
		if (state == DriveStates.VELOCITY_SETPOINT || state == DriveStates.PATH_FOLLOWING
				|| state == DriveStates.VISION) {
			return true;
		}
		return false;
	}

	// Initialize all of the drive motors
	private CANSparkMax leaderLeft, leaderRight, leftA, rightA, leftB, rightB;
	private DoubleSolenoid shifter;
	private NavX navx;
	private PathFollower mPathFollower;
	private Rotation2d mTargetHeading = new Rotation2d();
	private boolean mIsOnTarget = false;

	private double currentTimeStamp;
	private double lastTimeStamp;
	private double leftEncoderDistance, rightEncoderDistance;
	private double leftVelocity, rightVelocity;
	private Encoder leftEncoder, rightEncoder;
	private double visionThrottle;
	private double lowGearPositionMaxPower = 1;
	private PIDController leftHighGearVelocityPID, rightHighGearVelocityPID, leftLoweGearPositionPID, rightLowGearPositionPID;
	private SimpleMotorFeedforward leftHighGearVelocityFeedForward, rightHighGearVelocityFeedForward;
	private CheesyDriveHelper visionCheesyDriveHelper = new CheesyDriveHelper(0.0, 0.2);

	public DriveStates getmDriveStates() {
		return mDriveStates;
	}

	// State Control
	private DriveStates mDriveStates;
	private RobotState mRobotState = RobotState.getInstance();
	private VisionSubsystem mVisionSubsystem = VisionSubsystem.getInstance();
	private Path mCurrentPath = null;
	private boolean mIsHighGear = false;
	public static boolean isWantedLowPID = false;
	private Loop mLoop = new Loop() {
		/**
		 * Different states that constantly need to be ran
		 * Runs different functions based on the current state of the DriveTrain
		 * 
		 * @param timestamp current robot runtime in seconds
		 */
		@Override
		public synchronized void onLoop(double timestamp) {
			lastTimeStamp = currentTimeStamp;
			currentTimeStamp = timestamp;
			leftEncoderDistance = leftEncoder.getDistance();
			rightEncoderDistance =  rightEncoder.getDistance();
			leftVelocity = leftEncoder.getRate();
			rightVelocity = rightEncoder.getRate();
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
				setLowGearPositionControlMaxDrivePower(12);
			}
		}

		@Override
		public synchronized void onStop(double timestamp) {
			setOpenLoop(DriveSignal.NEUTRAL);

		}
	};

	/**
	 * instantiating the motors
	 * sets currentLimit
	 */
	public DriveTrainSubsystem() {
		visionThrottle = 0.0;
		//init encoders
		leftEncoder = new Encoder(Constants.kDIODriveLeftEncoderA, Constants.kDIODriveLeftEncoderB);
		rightEncoder = new Encoder(Constants.kDIODriveRightEncoderA, Constants.kDIODriveRightEncoderB);

		// init the all of the motor controllers
		leaderLeft = new CANSparkMax(RobotMap.leftLeader, CANSparkMaxLowLevel.MotorType.kBrushless);
		leaderRight = new CANSparkMax(RobotMap.rightLeader, CANSparkMaxLowLevel.MotorType.kBrushless);

		leftA = new CANSparkMax(RobotMap.leftFollowerA, CANSparkMaxLowLevel.MotorType.kBrushless);
		rightA = new CANSparkMax(RobotMap.rightFollowerA, CANSparkMaxLowLevel.MotorType.kBrushless);

		leftB = new CANSparkMax(RobotMap.leftFollowerB, CANSparkMaxLowLevel.MotorType.kBrushless);
		rightB = new CANSparkMax(RobotMap.rightFollowerB, CANSparkMaxLowLevel.MotorType.kBrushless);

		// Follow for right side
		rightA.follow(leaderRight);
		rightB.follow(leaderRight);

		// Follow for left side
		leftA.follow(leaderLeft);
		leftB.follow(leaderLeft);

		leaderLeft.setSmartCurrentLimit(85);
		leftA.setSmartCurrentLimit(85);
		leftB.setSmartCurrentLimit(85);

		leaderRight.setSmartCurrentLimit(85);
		rightA.setSmartCurrentLimit(85);
		rightB.setSmartCurrentLimit(85);

		leaderLeft.setInverted(true);
		leftA.setInverted(true);
		leftB.setInverted(true);

		// //Invert the right side
		leaderRight.setInverted(false);
		rightA.setInverted(false);
		rightB.setInverted(false);

		leftEncoderDistance = 0;
		rightEncoderDistance = 0;
		leftVelocity = 0;
		rightVelocity = 0;


		mostRecentTargetTimestamp = 0;

		// init solenoids
		shifter = new DoubleSolenoid( PneumaticsModuleType.CTREPCM, RobotMap.shiftLow,
				RobotMap.shiftHigh);
		// init navx
		navx = new NavX(SPI.Port.kMXP);

		reloadGains();
		mDriveStates = DriveStates.DRIVING;
		setLowGearPositionControlMaxDrivePower(12);
		leftEncoder.setReverseDirection(true);
	}

	private synchronized void configureForPositionControl() {
		if (usesPositionPID(mDriveStates)) {
			setLowGearPositionControlMaxDrivePower(12);
			// We entered a position control state.
			System.out.println("Configuring position control");
			// used to reset Iaccumulator
			setBrakeMode();
		} else {
			System.out.println("Oh no! DIdn't set Position control");
		}
	}

	private synchronized void configureForVelocityControl() {
		if (!usesVelocityPID(mDriveStates)) {
			// We entered a velocity control state.
			setLowGearPositionControlMaxDrivePower(12);
			System.out.println("Configuring speed control");
			setBrakeMode();
			updateVelocitySetpoint(0, 0);
		}
	}

	public synchronized void forceDoneWithPath() {
		if (mDriveStates == DriveStates.PATH_FOLLOWING && mPathFollower != null) {
			mPathFollower.forceFinish();
		} else {
			System.out.println("Robot is not in path following mode");
		}

		mDriveStates = DriveStates.DRIVING;
		setOpenLoop(new DriveSignal(0, 0, true));
		mPathFollower = null;
	}

	public synchronized Rotation2d getGyroYaw() {
		return navx.getYaw();
		// return new Rotation2d();
	}

	private double zeroRoll = 0;

	public synchronized double getGyroRoll() {
		return navx.getRoll() - zeroRoll;
	}

	public void zeroGyroRoll() {
		zeroRoll = navx.getRoll();
	}

	public double getLeftDistanceInches() {
		return leftEncoderDistance * Constants.kDriveInchesPerCount;
	}

	public double getLeftVelocityInchesPerSec() {
		return leftVelocity * Constants.kDriveInchesPerCount / 60;
	}

	public double getRightDistanceInches() {
		return rightEncoderDistance * Constants.kDriveInchesPerCount;
	}

	public double getRightVelocityInchesPerSec() {
		return rightVelocity * Constants.kDriveInchesPerCount / 60;
	}

	public boolean isCreeping() {
		return mDriveStates == DriveStates.CREEP;
	}

	public synchronized boolean isDoneWithPath() {
		if (mDriveStates == DriveStates.PATH_FOLLOWING && mPathFollower != null) {
			boolean isFinished = mPathFollower.isFinished();
			if (isFinished) {
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
		leaderLeft.setVoltage(output);
	}

	@Override
	public void outputToSmartDashboard() {
		if (debug) {
			SmartDashboard.putNumber(Constants.kDriveTrainKey + "position left (in)", getLeftDistanceInches());
			SmartDashboard.putNumber(Constants.kDriveTrainKey + "position right (in)", getRightDistanceInches());
			SmartDashboard.putNumber(Constants.kDriveTrainKey + "velocity left (in/sec)",
					getLeftVelocityInchesPerSec());
			SmartDashboard.putNumber(Constants.kDriveTrainKey + "velocity right (in/sec)",
					getRightVelocityInchesPerSec());
			SmartDashboard.putNumber(Constants.kDriveTrainKey + "encoder count left", leftEncoderDistance);
			SmartDashboard.putNumber(Constants.kDriveTrainKey + "encoder count right", rightEncoderDistance);

			SmartDashboard.putString(Constants.kDriveTrainKey + "drive state", returnDriveState());
			SmartDashboard.putNumber(Constants.kDriveTrainKey + "navX yaw", getGyroYaw().getDegrees());
			SmartDashboard.putBoolean(Constants.kDriveTrainKey + "is high gear?", isHighGear());

			SmartDashboard.putNumber(Constants.kDriveTrainKey + "temp leaderLeft", leaderLeft.getMotorTemperature());
			SmartDashboard.putNumber(Constants.kDriveTrainKey + "temp leftA", leftA.getMotorTemperature());
			SmartDashboard.putNumber(Constants.kDriveTrainKey + "temp leftB", leftB.getMotorTemperature());
			SmartDashboard.putNumber(Constants.kDriveTrainKey + "temp leaderRight", leaderRight.getMotorTemperature());
			SmartDashboard.putNumber(Constants.kDriveTrainKey + "temp rightA", rightA.getMotorTemperature());
			SmartDashboard.putNumber(Constants.kDriveTrainKey + "temp rightB", rightB.getMotorTemperature());

			SmartDashboard.putNumber(Constants.kDriveTrainKey + "amps leaderLeft", leaderLeft.getOutputCurrent());
			SmartDashboard.putNumber(Constants.kDriveTrainKey + "amps leftA", leftA.getOutputCurrent());
			SmartDashboard.putNumber(Constants.kDriveTrainKey + "amps leftB", leftB.getOutputCurrent());
			SmartDashboard.putNumber(Constants.kDriveTrainKey + "amps leaderRight", leaderRight.getOutputCurrent());
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

	/**
	 * sets a pid on a motor controller position (high gear high speed)
	 *
	 * @param motorController to set the pid values on
	 */
	public synchronized void reloadHighGearPositionGainsForController(CANSparkMax motorController) {
		leftHighGearVelocityPID = new PIDController(Constants.kDriveHighGearVelocityKp, Constants.kDriveHighGearVelocityKi, Constants.kDriveHighGearVelocityKd);
		leftHighGearVelocityFeedForward = new SimpleMotorFeedforward(Constants.kDriveHighGearVelocityKs, Constants.kDriveHighGearVelcoityKv, Constants.kDriveHighGearVelocityKa);
		rightHighGearVelocityPID = new PIDController(Constants.kDriveHighGearVelocityKp, Constants.kDriveHighGearVelocityKi, Constants.kDriveHighGearVelocityKd);
		rightHighGearVelocityFeedForward = new SimpleMotorFeedforward(Constants.kDriveHighGearVelocityKs, Constants.kDriveHighGearVelcoityKv, Constants.kDriveHighGearVelocityKa);
	}

	/**
	 * sets a pid on a motor controller position (high gear low speed)
	 *
	 * @param motorController to set the pid values on
	 */
	public synchronized void reloadHighGearPositionGainsForControllerLowPID(CANSparkMax motorController) {
		leftHighGearVelocityPID = new PIDController(Constants.kDriveHighGearVelocityLowKp, Constants.kDriveHighGearVelocityLowKi, Constants.kDriveHighGearVelocityLowKd);
		leftHighGearVelocityFeedForward = new SimpleMotorFeedforward(Constants.kDriveHighGearVelocityLowKs, Constants.kDriveHighGearVelcoityLowKv, Constants.kDriveHighGearVelocityLowKa);
		rightHighGearVelocityPID = new PIDController(Constants.kDriveHighGearVelocityLowKp, Constants.kDriveHighGearVelocityLowKi, Constants.kDriveHighGearVelocityLowKd);
		rightHighGearVelocityFeedForward = new SimpleMotorFeedforward(Constants.kDriveHighGearVelocityLowKs, Constants.kDriveHighGearVelcoityLowKv, Constants.kDriveHighGearVelocityLowKa);
	}

	/**
	 * reloads the velocity pid based on whether or not the current wanted pid is
	 * high speed or low speed
	 *
	 */
	public synchronized void reloadHighGearVelocityGains() {
		if (isWantedLowPID) {
			System.out.println("low PID");
			reloadHighGearPositionGainsForControllerLowPID(leaderLeft);
			reloadHighGearPositionGainsForControllerLowPID(leaderRight);
		} else {
			System.out.println("high PID");
			reloadHighGearPositionGainsForController(leaderLeft);
			reloadHighGearPositionGainsForController(leaderRight);
		}
	}

	/**
	 * Resets leaderLeft and materRight low gear position gains
	 *
	 */
	public synchronized void reloadLowGearPositionGains() {
		reloadLowGearPositionGainsForController(leaderLeft);
		reloadLowGearPositionGainsForController(leaderRight);
	}

	/**
	 * sets a pid on a motor controller position (low gear)
	 *
	 * @param motorController to set the pid values on
	 */
	public synchronized void reloadLowGearPositionGainsForController(CANSparkMax motorController) {
		leftLoweGearPositionPID = new PIDController(Constants.kDriveLowGearPositionKp, Constants.kDriveLowGearPositionKi, Constants.kDriveLowGearPositionKd);
		rightLowGearPositionPID = new PIDController(Constants.kDriveLowGearPositionKp, Constants.kDriveLowGearPositionKi, Constants.kDriveLowGearPositionKd);
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
		leaderRight.setVoltage(output);
	}

	/**
	 * Sets the sparkMaxes for brake mode
	 */
	public synchronized void setBrakeMode() {
		// set for auto
		leaderLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
		leftA.setIdleMode(CANSparkMax.IdleMode.kBrake);
		leftB.setIdleMode(CANSparkMax.IdleMode.kBrake);

		leaderRight.setIdleMode(CANSparkMax.IdleMode.kBrake);
		rightA.setIdleMode(CANSparkMax.IdleMode.kBrake);
		rightB.setIdleMode(CANSparkMax.IdleMode.kBrake);
	}

	/**
	 * Sets the sparkMaxes up for coast mode
	 */
	public synchronized void setCoastMode() {
		// set for driving
		leaderLeft.setIdleMode(CANSparkMax.IdleMode.kCoast);
		leftA.setIdleMode(CANSparkMax.IdleMode.kCoast);
		leftB.setIdleMode(CANSparkMax.IdleMode.kCoast);

		leaderRight.setIdleMode(CANSparkMax.IdleMode.kCoast);
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
		leaderLeft.setVoltage(signal.getLeft() / 2);
		leaderRight.setVoltage(signal.getRight() / 2);
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
	 * 
	 * @param wantsHighGear
	 *                      it's a boolean saying if you want it or not
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
	 * @param brake if 1, the drive train will go into brake mode, 0 will put it
	 *              into coast mode
	 */
	public synchronized void setNeutralMode(boolean brake) {
		CANSparkMax.IdleMode currentMode = brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast;
		leaderRight.setIdleMode(currentMode);
		rightA.setIdleMode(currentMode);
		leaderLeft.setIdleMode(currentMode);
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
		leaderLeft.setVoltage(signal.getLeft() * 12.0);
		leaderRight.setVoltage(signal.getRight() * 12.0);
	}

	/**
	 * Used to update velocity setpoint and set state
	 *
	 * @param left_inches_per_sec  Left inches per second
	 * @param right_inches_per_sec Right inches per second
	 */
	public synchronized void setVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
		configureForVelocityControl();
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
			configureForVelocityControl();
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
			configureForPositionControl();
			setLowGearPositionControlMaxDrivePower(Constants.kDriveTurnMaxPower);
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
	double kWiggle = 6 * 3.1415;
	double kWiggleAmplitude = .15;
	Timer wiggleTime = new Timer();

	public synchronized void wiggleHandler(boolean wiggleReq) {
		startingWiggle = wiggleReq && !wasWigglin;
		finishingWiggle = !wiggleReq && wasWigglin;

		if (startingWiggle) {
			wiggleTime.reset();
			wiggleTime.start();
			mDriveStates = DriveStates.WIGGLE;
		} else if (finishingWiggle) {
			wiggleTime.stop();
			wiggleTime.reset();
			mDriveStates = DriveStates.DRIVING;

			leftDrive(0);
			rightDrive(0);
		}

		if (wiggleReq) {
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

		if (startingPush) {
			System.out.println("starting push");
			mDriveStates = DriveStates.DRIVE_TO_STALL;
			leftDrive(Constants.kStallPower);
			rightDrive(Constants.kStallPower);
			pushTimeStamp = currentTimeStamp;
		}

		isTimedOut = (currentTimeStamp - pushTimeStamp > Constants.kStallTimeout);
		finishingPush = (leftVelocity < Constants.kStallSpeed && rightVelocity < Constants.kStallSpeed
				&& currentTimeStamp - pushTimeStamp > Constants.kStallWaitPeriod) || isTimedOut || stopReq;
		wasPushing = pushReq;
		if (leftVelocity < Constants.kStallSpeed && mDriveStates == DriveStates.DRIVE_TO_STALL) {
			System.out.println("time to stall " + (currentTimeStamp - pushTimeStamp));
		}

		if (finishingPush && mDriveStates == DriveStates.DRIVE_TO_STALL) {
			System.out.println("finishing push");
			System.out.println("speed low? " + (leftVelocity < Constants.kStallSpeed));
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
		if (!parkingBrakeIsStopped) {
			setVelocitySetpoint(0, 0);
			if ( leftEncoder.getRate() < inchesToCounts(4) && rightEncoder.getRate()< inchesToCounts(4)) {
				parkingBrakeIsStopped = true;
				parkingBrakePositionLeft = leftEncoder.getDistance();
				parkingBrakePositionRight = rightEncoder.getDistance();
				reloadLowGearPositionGains();
			}
		} else {
			leaderLeft.setVoltage(leftLoweGearPositionPID.calculate(leftEncoder.getDistance(), parkingBrakePositionLeft));
			leaderRight.setVoltage(rightLowGearPositionPID.calculate(rightEncoder.getDistance(), parkingBrakePositionRight));
		}

		return false;
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
		leaderLeft.setVoltage(0);
		leaderRight.setVoltage(0);
	}

	/**
	 * Called periodically when the robot is in path following mode. Updates the
	 * path follower with the robots latest
	 * pose, distance driven, and velocity, the updates the wheel velocity
	 * setpoints.
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
			// updateVelocitySetpoint(0, 0);
		}
	}

	/**
	 * Updates the sparkMaxes to what position it will go to
	 *
	 * @param left_position_inches  Inches wanted
	 * @param right_position_inches Inches wanted
	 */
	private synchronized void updatePositionSetpoint(double left_position_inches, double right_position_inches) {
		if (usesPositionPID(mDriveStates)) {
			leaderLeft.setVoltage(leftLoweGearPositionPID.calculate(leftEncoder.getDistance(), inchesToCounts(left_position_inches)));
			leaderRight.setVoltage(rightLowGearPositionPID.calculate(rightEncoder.getDistance(), inchesToCounts(right_position_inches)));
		} else {
			System.out.println("Hit a bad position control state");
			leaderLeft.setVoltage(0);
			leaderRight.setVoltage(0);
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
	 * Update velocity setpoint is used to send over our desired velocity from pure
	 * pursuit control
	 *
	 * @param left_inches_per_sec  Left side inches per second
	 * @param right_inches_per_sec right side inches per second
	 */
	private synchronized void updateVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
		if (usesVelocityPID(mDriveStates)) {
			final double max_desired = Math.max(Math.abs(left_inches_per_sec), Math.abs(right_inches_per_sec));
			final double scale = max_desired > Constants.kDriveHighGearMaxSetpoint
					? Constants.kDriveHighGearMaxSetpoint / max_desired
					: 1.0;
			leaderLeft.setVoltage(leftHighGearVelocityFeedForward.calculate(inchesToCounts(left_inches_per_sec * scale)) + leftHighGearVelocityPID.calculate(leftEncoder.getDistance(), inchesToCounts(left_inches_per_sec * scale)));
			leaderRight.setVoltage(rightHighGearVelocityFeedForward.calculate(inchesToCounts(right_inches_per_sec * scale)) + rightHighGearVelocityPID.calculate(rightEncoder.getDistance(), inchesToCounts(right_inches_per_sec * scale)));

			SmartDashboard.putNumber("A Left Side Velocity", getLeftVelocityInchesPerSec());
			SmartDashboard.putNumber("A Right Side Velocity", getRightVelocityInchesPerSec());
		} else {
			System.out.println("Hit a bad velocity control state");
			leaderLeft.setVoltage(0);
			leaderRight.setVoltage(0);
		}
	}

	/**
	 * This method zeros the encoders of both sides of the drivetrain
	 */
	@Override
	public synchronized void zeroSensors() {
		// System.out.println("Zeroing drivetrain sensors...");
		leftEncoder.reset();
		rightEncoder.reset();
		leftVelocity = 0;
		rightVelocity = 0;
		leftEncoderDistance = 0;
		rightEncoderDistance = 0;
		navx.zeroYaw();
		// System.out.println("Drivetrain sensors zeroed!");
	}

	public double getLeftVoltage() {
		return leaderLeft.getAppliedOutput();
	}

	public double getRightVoltage() {
		return leaderRight.getAppliedOutput();
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
	public void setLowGearPositionControlMaxDrivePower(double power) {
		lowGearPositionMaxPower = power;
	}

	public void setWantVisionTracking(double throttle) {
		if (mDriveStates != DriveStates.VISION) {
			mDriveStates = DriveStates.VISION;
			mostRecentTargetTimestamp = 0;
			setBrakeMode();
		}
	}

	public void updateVision() {
		DriveSignal signal = visionCheesyDriveHelper.cheesyDrive(visionThrottle, mVisionSubsystem.getAngleOffsetToTarget() * Constants.visionLineUpProportional, true, false);

		leaderLeft.setVoltage(signal.getLeft() * 12.0);
		leaderRight.setVoltage(signal.getRight() * 12.0);
		
	
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

	public void retractAll() {
		// nothing to do here
	}






	public void setupDriverTab(){


		Robot.getMainDriverTab().addNumber("Left Drive Power", new DoubleSupplier() {

			@Override
			public double getAsDouble() {
				return leaderLeft.getAppliedOutput();
			}
			
		}).withWidget(BuiltInWidgets.kNumberBar).withPosition(1,1); //add .withProperties if neccesary


		Robot.getMainDriverTab().addNumber("Right Drive Power", new DoubleSupplier() {

			@Override
			public double getAsDouble() {
				return leaderLeft.getAppliedOutput();
			}
			
		}).withWidget(BuiltInWidgets.kNumberBar).withPosition(1,2); //add .withProperties if neccesary


	}
}
