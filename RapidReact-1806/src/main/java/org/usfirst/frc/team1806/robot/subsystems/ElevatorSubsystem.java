package org.usfirst.frc.team1806.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotMap;
import org.usfirst.frc.team1806.robot.loop.Loop;
import org.usfirst.frc.team1806.robot.loop.Looper;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;


/**
 * Whew, this subsystem is for our liftactions, we will later implement this
 * into a superstructure
 * so we can interact with our intake for doing things like shooting!
 */
public class ElevatorSubsystem implements Subsystem {
	boolean debug = true;

	public enum ElevatorStates {
		POSITION_CONTROL,
		HOLD_POSITION,
		MANUAL_CONTROL,
		IDLE
	}

	private CANSparkMax elevatorLead;
	private double elevatorWantedPosition;
	private Double heightLeniency = 0.25;

	private AnalogInput mStringPotentiometer;
	private PIDController mPidController;

	private boolean isBrakeMode = false;
	private boolean mIsOnTarget = false;

	private ElevatorStates mElevatorStates;

	private static ElevatorSubsystem mElevatorSubsystem = new ElevatorSubsystem(); // only ever 1 lift

	public ElevatorSubsystem() {
		elevatorLead = new CANSparkMax(RobotMap.elevatorLeader, CANSparkMaxLowLevel.MotorType.kBrushless);
		elevatorLead.setSmartCurrentLimit(65);
		elevatorLead.setInverted(true);
		mElevatorStates = ElevatorStates.IDLE;
		elevatorWantedPosition = Constants.kLiftBottomPivotHeight;
		reloadGains();
		elevatorLead.setSecondaryCurrentLimit(200.0);
		mStringPotentiometer = new AnalogInput(0);
		setBrakeMode();
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
			SmartDashboard.putNumber(Constants.kLiftKey + "Voltage", mStringPotentiometer.getVoltage());
		}

	}

	@Override
	public void stop() {
		mElevatorStates = ElevatorStates.IDLE;
		setLiftIdle();
	}

	@Override
	public synchronized void zeroSensors() {
	}

	public synchronized void zeroSensorsAtTop() {

	}

	public synchronized void zeroSensorsAtBottom() {
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
			}

			private void elevatorStateLoop() {
				
				switch (mElevatorStates) {
					case POSITION_CONTROL:
						double reverseClamp = getHeightInInches() < Constants.kLiftSlowDownHeight? 0.05:0.2;
						double output = MathUtil.clamp(mPidController.calculate(getHeightInInches(), elevatorWantedPosition) +Constants.kElevatorHoldPercentOutput, -reverseClamp, 1.0);
						SmartDashboard.putNumber("Elevator PID Output", output);
						elevatorLead.set(output);
						if(isAtPosition()){
							mElevatorStates = ElevatorStates.HOLD_POSITION;
						}
						return;
					case HOLD_POSITION:
						holdPosition();
						return;
					case MANUAL_CONTROL:
						return;
					case IDLE:
						elevatorLead.set(0);
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
		double setpoint = setpointInInches < Constants.kLiftBottomPivotHeight? Constants.kLiftBottomPivotHeight: setpointInInches;
		if(Math.abs(elevatorWantedPosition - setpoint) > 0.0001 || mElevatorStates != ElevatorStates.POSITION_CONTROL)
		{
			mPidController.reset();
		}

		elevatorWantedPosition = setpoint;
		if(!isAtPosition()){
		mElevatorStates = ElevatorStates.POSITION_CONTROL;
		}
	}

	public synchronized void goToTop() {
		// TODO Auto-generated method stub

	}

	public double getHeightInInches() {
		return -getExtensionLength(mStringPotentiometer.getVoltage()) + 48.0;
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
		mPidController = new PIDController(Constants.kElevatorPositionkP, Constants.kElevatorPositionkI, Constants.kElevatorPositionkD, Constants.kLooperDt);

	}

	/**
	 * @return
	 *         Returns whether or not the liftactions is ready to be held at
	 *         position for a cube to be deposited
	 */
	public synchronized boolean isAtPosition() {
		
		return isAtArbitraryPosition(elevatorWantedPosition);
	}

	public synchronized boolean isAtArbitraryPosition(double height){
		if (mElevatorStates == ElevatorStates.IDLE) {
			return false;
		}
		if(height <= Constants.kLiftBottomPivotHeight){
			return  true;
		}
		return Math
				.abs(height - getHeightInInches()) < Constants.kElevatorPositionTolerance;
	}

	public synchronized boolean isAbovePosition(double height){
		return getHeightInInches() > height;
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
		elevatorLead.setVoltage(0); // DutyCycle just means voltage on scale of
																				// -1 to 1
	}

	public synchronized void setLiftHoldPosition() {
		mElevatorStates = ElevatorStates.POSITION_CONTROL;
		goToSetpointInches(getHeightInInches());
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
		if(elevatorWantedPosition <= Constants.kLiftBottomPivotHeight){
			elevatorLead.set(0);
		}
		else{
			elevatorLead.set(Constants.kElevatorHoldPercentOutput +
			(elevatorWantedPosition - getHeightInInches()) * Constants.kElevatorHoldkPGain);
		}
		
		if (!isAtPosition()) {
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

	@Override
	public void setupDriverTab() {
		// TODO Auto-generated method stub

	}

	private Double getExtensionLength(Double X){
		return -6.606 * X + 34.224;
	}

}
