package org.usfirst.frc.team1806.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotMap;
import org.usfirst.frc.team1806.robot.loop.Loop;
import org.usfirst.frc.team1806.robot.loop.Looper;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;


/**
 * Whew, this subsystem is for our liftactions, we will later implement this
 * into a superstructure
 * so we can interact with our intake for doing things like shooting!
 */
public class ElevatorSubsystem implements Subsystem {
	boolean debug = false;

	public enum ElevatorStates {
		POSITION_CONTROL,
		HOLD_POSITION,
		MANUAL_CONTROL,
		IDLE
	}

	private CANSparkMax elevatorLead;

	private double inchesPerCount;
	private double lastTimeStamp;
	private double elevatorWantedPosition;
	private Double heightLeniency = 0.25;

	private AnalogInput mStringPotentiometer;
	private PIDController mPidController;

	private boolean isBrakeMode = false;
	private boolean mIsOnTarget = false;

	private ElevatorStates mElevatorStates;

	private static ElevatorSubsystem mElevatorSubsystem = new ElevatorSubsystem(); // only ever 1 lift

	public ElevatorSubsystem() {
		inchesPerCount = Constants.kElevatorInchesPerCountDefault;
		elevatorLead = new CANSparkMax(RobotMap.elevatorLeader, CANSparkMaxLowLevel.MotorType.kBrushless);
		elevatorLead.setInverted(true);
		mElevatorStates = ElevatorStates.IDLE;
		elevatorWantedPosition = 0;
		reloadGains();

		elevatorLead.getEncoder().setPositionConversionFactor(48);

		lastTimeStamp = 0;
		mStringPotentiometer = new AnalogInput(0);
	}

	@Override
	public void outputToSmartDashboard() {
		if (debug) {
			SmartDashboard.putString(Constants.kLiftKey + "State: ", returnLiftStates().toString());
			SmartDashboard.putNumber(Constants.kLiftKey + "Position (in)", getHeightInInches());
			SmartDashboard.putNumber(Constants.kLiftKey + "Encoder Position", elevatorLead.getEncoder().getPosition());
			SmartDashboard.putNumber(Constants.kLiftKey + "Velocity", elevatorLead.getEncoder().getVelocity());
			SmartDashboard.putNumber(Constants.kLiftKey + "Leader Power Sending", elevatorLead.getAppliedOutput());
			SmartDashboard.putNumber(Constants.kLiftKey + "Wanted Height", elevatorWantedPosition);
			SmartDashboard.putNumber(Constants.kLiftKey + "Lead Motor Temp", elevatorLead.getMotorTemperature());
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

						mElevatorStates = ElevatorStates.HOLD_POSITION;
						holdPosition();
				}
				elevatorStateLoop();

				lastTimeStamp = timestamp;
			}

			private void elevatorStateLoop() {
				switch (mElevatorStates) {
					case POSITION_CONTROL:
						elevatorLead.setVoltage(mPidController.calculate(getHeightInInches(), elevatorWantedPosition));
						return;
					case HOLD_POSITION:
						holdPosition();
						return;
					case MANUAL_CONTROL:
						return;
					case IDLE:
						elevatorLead.setVoltage(0);
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

	public synchronized void goToSetpointInches(double setpointInInches)
	{
		mElevatorStates = ElevatorStates.POSITION_CONTROL;
		elevatorWantedPosition = setpointInInches;
	}

	public synchronized void goToTop() {
		// TODO Auto-generated method stub

	}

	public double getHeightInInches() {
		return getHeightInCounts() / inchesPerCount;
	}

	public double getHeightInCounts() {
		return mStringPotentiometer.getAverageVoltage();
	}

	public boolean isOnTarget() {
		return mIsOnTarget;
	}

	public void setBrakeMode() {

		elevatorLead.setIdleMode(CANSparkMax.IdleMode.kBrake);
		isBrakeMode = true;

	}

	public void setCoastMode() {
		elevatorLead.setIdleMode(CANSparkMax.IdleMode.kCoast);
		isBrakeMode = false;
	}

	public boolean isInBrakeMode() {
		return isBrakeMode;
	}

	public void reloadGains() {
		mPidController = new PIDController(Constants.kElevatorPositionkP, Constants.kElevatorPositionkI, Constants.kElevatorPositionkD);

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
		goToSetpointInches(getHeightInCounts());
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
			goToSetpointInches(elevatorLead.getEncoder().getPosition());
			mElevatorStates = ElevatorStates.HOLD_POSITION;
		}
	}

	/**
	 * Used to hold the cube when it is ready to be spat out
	 */
	public synchronized void holdPosition() {
		elevatorLead.getPIDController().setReference(Constants.kElevatorHoldPercentOutput +
				(elevatorWantedPosition - getHeightInCounts()) * Constants.kElevatorHoldkPGain,
				CANSparkMax.ControlType.kDutyCycle);
		if (Math.abs(getHeightInCounts() - elevatorWantedPosition) > Constants.kElevatorPositionTolerance) {
			mElevatorStates = ElevatorStates.POSITION_CONTROL;
			goToSetpointInches(elevatorWantedPosition);
		}
	}

	public synchronized boolean bumpHeightUp() {
		if (mElevatorStates == ElevatorStates.POSITION_CONTROL || mElevatorStates == ElevatorStates.HOLD_POSITION) {
			goToSetpointInches(elevatorWantedPosition + Constants.kBumpEncoderPosition);
			return true;
		}
		return false;
	}

	public synchronized boolean bumpHeightDown() {
		if (mElevatorStates == ElevatorStates.POSITION_CONTROL || mElevatorStates == ElevatorStates.HOLD_POSITION) {
			goToSetpointInches(elevatorWantedPosition - Constants.kBumpEncoderPosition);
			return true;
		}
		return false;
	}

	public void retractAll() {
		goToSetpointInches(Constants.kMaxElevatorHeightToNeedToExtendIntake + Constants.kSafeElevatorHeightOffsetToNotHitIntake);
	}

	public Boolean isLiftInRange(){
		if (getHeightInInches() >= 0.5){
			return false;
		}
		return true;
	}

	public Boolean heightToCheck(Double height){
		if (height >= getHeightInInches() + heightLeniency && height >= getHeightInInches() - heightLeniency) return false;
		return true;
	}
}
