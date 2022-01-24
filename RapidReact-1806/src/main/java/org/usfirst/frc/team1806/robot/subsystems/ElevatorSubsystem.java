package org.usfirst.frc.team1806.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotMap;
import org.usfirst.frc.team1806.robot.loop.Loop;
import org.usfirst.frc.team1806.robot.loop.Looper;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;


/**
 * Whew, this subsystem is for our liftactions, we will later implement this
 * into a superstructure
 * so we can interact with our intake for doing things like shooting!
 */
public class ElevatorSubsystem implements Subsystem {
	boolean debug = false;

	public enum ElevatorStates {
		POSITION_CONTROL,
		RESET_TO_BOTTOM,
		RESET_TO_TOP,
		HOLD_POSITION,
		MANUAL_CONTROL,
		IDLE
	}

	private CANSparkMax elevatorLead, elevatorFollow; // gotta have the power
	public DigitalInput bottomLimit, topLimit;

	private double inchesPerCount;
	private double lastTimeStamp;
	private double elevatorWantedPosition;

	private boolean isBrakeMode = false;
	private boolean mIsOnTarget = false;

	private ElevatorStates mElevatorStates;

	private static ElevatorSubsystem mElevatorSubsystem = new ElevatorSubsystem(); // only ever 1 lift

	public ElevatorSubsystem() {
		inchesPerCount = Constants.kElevatorInchesPerCountDefault;
		elevatorLead = new CANSparkMax(RobotMap.elevatorLead, CANSparkMaxLowLevel.MotorType.kBrushless);
		elevatorFollow = new CANSparkMax(RobotMap.elevatorFollow, CANSparkMaxLowLevel.MotorType.kBrushless);
		elevatorLead.setInverted(true);
		elevatorFollow.follow(elevatorLead, true);
		/*
		 * liftLead.setSmartCurrentLimit(130, 80);
		 * liftFollow.setSmartCurrentLimit(130, 80);
		 */
		bottomLimit = new DigitalInput(RobotMap.elevatorBottomLimit);
		topLimit = new DigitalInput(RobotMap.elevatorHighLimit);
		mElevatorStates = ElevatorStates.IDLE;
		elevatorWantedPosition = 0;
		reloadGains();

		elevatorLead.getEncoder().setPositionConversionFactor(48);

		lastTimeStamp = 0;

	}

	@Override
	public void outputToSmartDashboard() {
		if (debug) {
			SmartDashboard.putString(Constants.kLiftKey + "State: ", returnLiftStates().toString());
			SmartDashboard.putNumber(Constants.kLiftKey + "Position (in)", getHeightInInches());
			SmartDashboard.putNumber(Constants.kLiftKey + "Encoder Position", elevatorLead.getEncoder().getPosition());
			SmartDashboard.putNumber(Constants.kLiftKey + "Velocity", elevatorLead.getEncoder().getVelocity());
			SmartDashboard.putNumber(Constants.kLiftKey + "Leader Power Sending", elevatorLead.getAppliedOutput());
			SmartDashboard.putNumber(Constants.kLiftKey + "Follow Power Sending", elevatorFollow.getAppliedOutput());
			SmartDashboard.putBoolean(Constants.kLiftKey + "Bottom limit triggered", areWeAtBottomLimit());
			SmartDashboard.putNumber(Constants.kLiftKey + "Wanted Height", elevatorWantedPosition);
			SmartDashboard.putNumber(Constants.kLiftKey + "Lead Motor Temp", elevatorLead.getMotorTemperature());
			SmartDashboard.putNumber(Constants.kLiftKey + "Follow Motor Temp", elevatorFollow.getMotorTemperature());
			SmartDashboard.putBoolean(Constants.kLiftKey + "is at position?", isAtPosition());
		}

	}

	@Override
	public void stop() {
		mElevatorStates = ElevatorStates.IDLE;
		setLiftIdle();
	}

	@Override
	public synchronized void zeroSensors() {
		elevatorLead.getEncoder().setPosition(0);
	}

	public synchronized void zeroSensorsAtTop() {
		elevatorLead.getEncoder().setPosition(0);
		mElevatorStates = ElevatorStates.POSITION_CONTROL;
	}

	public synchronized void zeroSensorsAtBottom() {
		elevatorLead.getEncoder().setPosition(0);
		mElevatorStates = ElevatorStates.POSITION_CONTROL;
	}

	@Override
	public void registerEnabledLoops(Looper enabledLooper) {
		enabledLooper.register(new Loop() {
			/**
			 * Set lift to idle.
			 * 
			 * @param timestamp current robot runtime in seconds.
			 */
			@Override
			public void onStop(double timestamp) {
				mElevatorStates = ElevatorStates.IDLE;
			}

			/**
			 *
			 * @param timestamp current robot runtime in seconds
			 */
			@Override
			public void onStart(double timestamp) {
				if (Robot.needToPositionControlInTele) {
					setLiftHoldPosition();
				} else {
					mElevatorStates = ElevatorStates.IDLE;
				}
			}

			/**
			 *
			 * @param timestamp current robot runtime in seconds
			 */
			@Override
			public void onLoop(double timestamp) {

				// not moving and not manual
				if (isAtPosition() && mElevatorStates != ElevatorStates.MANUAL_CONTROL) {
					// not moving, at bottom
					if (mElevatorStates == ElevatorStates.RESET_TO_BOTTOM
							|| areWeAtBottomLimit()) {
						mElevatorStates = ElevatorStates.IDLE;
					}
					// not moving, not at bottom
					else {
						mElevatorStates = ElevatorStates.HOLD_POSITION;
						holdPosition();

					}
				}
				elevatorStateLoop();

				lastTimeStamp = timestamp;
			}

			private void elevatorStateLoop() {
				switch (mElevatorStates) {
					case POSITION_CONTROL:
						return;
					case RESET_TO_BOTTOM:
						mIsOnTarget = false;
						return;
					case RESET_TO_TOP:
						mIsOnTarget = false;
						return;
					case HOLD_POSITION:
						holdPosition();
						return;
					case MANUAL_CONTROL:
						return;
					case IDLE:
						elevatorLead.getPIDController().setReference(0, CANSparkMax.ControlType.kDutyCycle);
						return;
					default:
						return;

				}
			}
		});
	}

	@Override
	public void setDebug(boolean _debug) {
		debug = _debug;
	}

	@Override
	public void writeToLog() {

	}

	public static ElevatorSubsystem getInstance() {
		return mElevatorSubsystem;
	}

	public synchronized void goToSetpoint(double setpoint) {
		mElevatorStates = ElevatorStates.POSITION_CONTROL;
		elevatorWantedPosition = setpoint;
		setBrakeMode();
		elevatorLead.getPIDController().setReference(elevatorWantedPosition, CANSparkMax.ControlType.kPosition);
		// System.out.println(mLiftWantedPosition + " " + isReadyForSetpoint());
	}

	public synchronized void goToSetpointInches(double setpointInInches)
	{
		mElevatorStates = ElevatorStates.POSITION_CONTROL;
		elevatorWantedPosition = setpointInInches / inchesPerCount;
	}

	public synchronized void zeroOnBottom() {
		// TODO Auto-generated method stub
		if (mElevatorStates != ElevatorStates.RESET_TO_BOTTOM) {
			mElevatorStates = ElevatorStates.RESET_TO_BOTTOM;
		}
	}

	public synchronized void goToTop() {
		// TODO Auto-generated method stub

	}

	public double getHeightInInches() {
		return getHeightInCounts() / inchesPerCount;
	}

	public double getHeightInCounts() {
		return elevatorLead.getEncoder().getPosition();
	}

	public boolean isOnTarget() {
		return mIsOnTarget;
	}

	public void setBrakeMode() {

		elevatorLead.setIdleMode(CANSparkMax.IdleMode.kBrake);
		elevatorFollow.setIdleMode(CANSparkMax.IdleMode.kBrake);
		isBrakeMode = true;

	}

	public void setCoastMode() {
		elevatorLead.setIdleMode(CANSparkMax.IdleMode.kCoast);
		elevatorFollow.setIdleMode(CANSparkMax.IdleMode.kCoast);
		isBrakeMode = false;
	}

	public boolean isInBrakeMode() {
		return isBrakeMode;
	}

	public void reloadGains() {
		/*
		 * liftLead.setParameter(CANSparkMaxLowLevel.ConfigParameter.kP_0,
		 * Constants.kLiftPositionkP);
		 * liftLead.setParameter(CANSparkMaxLowLevel.ConfigParameter.kI_0,
		 * Constants.kLiftPositionkI);
		 * liftLead.setParameter(CANSparkMaxLowLevel.ConfigParameter.kD_0,
		 * Constants.kLiftPositionkD);
		 * liftLead.setParameter(CANSparkMaxLowLevel.ConfigParameter.kF_0,
		 * Constants.kLiftPositionkF);
		 * liftLead.setParameter(CANSparkMaxLowLevel.ConfigParameter.kIZone_0,
		 * Constants.kLiftPositionIZone);
		 * liftLead.setParameter(CANSparkMaxLowLevel.ConfigParameter.kRampRate,
		 * Constants.kLiftPositionRampRate);
		 */
		elevatorLead.getPIDController().setP(Constants.kElevatorPositionkP);
		elevatorLead.getPIDController().setI(Constants.kElevatorPositionkI);
		elevatorLead.getPIDController().setD(Constants.kElevatorPositionkD);
		elevatorLead.getPIDController().setFF(Constants.kElevatorPositionkF);
		elevatorLead.getPIDController().setIZone(Constants.kElevatorPositionIZone);

	}

	public synchronized void resetToBottom() {
		if (!areWeAtBottomLimit() || Math.abs(elevatorLead.getEncoder().getPosition()) < Constants.kBottomLimitTolerance) {
			mElevatorStates = ElevatorStates.RESET_TO_BOTTOM;
			goToSetpoint(0);
		}
	}

	public synchronized void resetToTop() {
		if (!topLimit.get()) {
			if (mElevatorStates != ElevatorStates.RESET_TO_TOP) {
				mElevatorStates = ElevatorStates.RESET_TO_TOP;
				goToSetpoint(Constants.kLiftTopLimitSwitchPosition);
			}
			elevatorLead.getPIDController().setReference(Constants.elevatorResetSpeed, CANSparkMax.ControlType.kDutyCycle);
		} else {
			zeroSensorsAtTop();
		}
	}

	/**
	 * @return
	 *         Returns whether or not the liftactions is ready to be held at
	 *         position for a cube to be deposited
	 */
	public synchronized boolean isAtPosition() {
		if (mElevatorStates == ElevatorStates.IDLE) {
			return false;
		}
		return Math
				.abs(elevatorWantedPosition - elevatorLead.getEncoder().getPosition()) < Constants.kElevatorPositionTolerance
				&&
				Math.abs(elevatorLead.getEncoder().getVelocity()) < Constants.kElevatorVelocityTolerance;
	}

	/**
	 *
	 * @return
	 *         returns current state of cube
	 */
	public synchronized ElevatorStates returnLiftStates() {
		return mElevatorStates;
	}

	/**
	 * Used to stop the manipulator from running. mostly ran on stop or when first
	 * setting the liftactions up
	 */
	public synchronized void setLiftIdle() {
		mElevatorStates = ElevatorStates.IDLE;
		elevatorLead.getPIDController().setReference(0, CANSparkMax.ControlType.kDutyCycle); // DutyCycle just means voltage on scale of
																				// -1 to 1
	}

	public synchronized void setLiftHoldPosition() {
		mElevatorStates = ElevatorStates.POSITION_CONTROL;
		goToSetpoint(getHeightInCounts());
	}

	/**
	 * Sets up the robot to accept position setpoints
	 */
	public synchronized void updatePositionControl() {
		mElevatorStates = ElevatorStates.POSITION_CONTROL;
		setBrakeMode();
	}

	public synchronized void manualMode(double power) {
		mElevatorStates = ElevatorStates.MANUAL_CONTROL;
		elevatorLead.getPIDController().setReference(power, CANSparkMax.ControlType.kDutyCycle);
		if (Math.abs(power) < .2) {
			goToSetpoint(elevatorLead.getEncoder().getPosition());
			mElevatorStates = ElevatorStates.HOLD_POSITION;
		}
	}

	/**
	 * Used to hold the cube when it is ready to be spat out
	 */
	public synchronized void holdPosition() {
		elevatorLead.getPIDController().setReference(Constants.kElevatorHoldPercentOutput +
				(elevatorWantedPosition - elevatorLead.getEncoder().getPosition()) * Constants.kElevatorHoldkPGain,
				CANSparkMax.ControlType.kDutyCycle);
		if (Math.abs(getHeightInCounts() - elevatorWantedPosition) > Constants.kElevatorPositionTolerance) {
			mElevatorStates = ElevatorStates.POSITION_CONTROL;
			goToSetpoint(elevatorWantedPosition);
		}
	}

	/**
	 * @return
	 *         are we at the bottom limit??
	 */
	public boolean areWeAtBottomLimit() {
		return !bottomLimit.get();
	}

	public double returnLiftHeight() {
		return elevatorLead.getEncoder().getPosition();
	}

	public synchronized boolean bumpHeightUp() {
		if (mElevatorStates == ElevatorStates.POSITION_CONTROL || mElevatorStates == ElevatorStates.HOLD_POSITION) {
			goToSetpoint(elevatorWantedPosition + Constants.kBumpEncoderPosition);
			return true;
		}
		return false;
	}

	public synchronized boolean bumpHeightDown() {
		if (mElevatorStates == ElevatorStates.POSITION_CONTROL || mElevatorStates == ElevatorStates.HOLD_POSITION) {
			goToSetpoint(elevatorWantedPosition - Constants.kBumpEncoderPosition);
			return true;
		}
		return false;
	}

	public void retractAll() {
		goToSetpoint(Constants.kMaxElevatorHeightToNeedToExtendIntake + Constants.kSafeElevatorHeightOffsetToNotHitIntake);
	}
}
