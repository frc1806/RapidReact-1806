package org.usfirst.frc.team1806.robot.subsystems;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.FeatureFlags;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotMap;
import org.usfirst.frc.team1806.robot.loop.Loop;
import org.usfirst.frc.team1806.robot.loop.Looper;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANEncoder.*;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;

import javax.naming.ldap.Control;

/**
 * Whew, this subsystem is for our liftactions, we will later implement this into a superstructure
 * so we can interact with our intake for doing things like shooting!
 */
public class LiftSubsystem  implements Subsystem {
	boolean debug = false;


	public enum LiftStates {
		POSITION_CONTROL,
		RESET_TO_BOTTOM,
		RESET_TO_TOP,
		HOLD_POSITION,
		MANUAL_CONTROL,
		IDLE
	}
	public enum LiftPosition {
		BOTTOM_LIMIT(0),
        TOP_LIMIT(13000),
		TELEOP_HOLD(600),
		SHIP_CARGO(7400),
		CARGO_FEEDER(8250),
		ROCKET_CARGO_LOW(4522),
		ROCKET_CARGO_MID(9000),
		ROCKET_CARGO_HIGH(13100),
		ROCKET_HATCH_MID(5600),
		ROCKET_HATCH_HIGH(10000),
		TEMP_HOLD_POS(0);

		double height;
		LiftPosition(double liftHeight){
			height = liftHeight;
		}

		double getHeight(){
			return height;
		}

		LiftPosition setHeight(double liftHeight) {
			height = liftHeight;
			return this;
		}
	}

	private CANSparkMax liftLead, liftFollow; //gotta have the power
	public DigitalInput bottomLimit, topLimit;

	private CargoIntakeSubsystem mCargoIntakeSubsystem;
	private double intakePneumaticWait ;
	private double lastTimeStamp;
	private boolean needsIntakeOut;
	private boolean wasIntakeOut;

	private boolean isBrakeMode = false;
	private boolean mIsOnTarget = false;

	private LiftStates mLiftStates;
	private LiftPosition mLiftPosition;

	private DoubleSolenoid liftLeaner;

	private static LiftSubsystem mLiftSubsystem = new LiftSubsystem(); //only ever 1 lift

	public LiftSubsystem() {
		liftLead = new CANSparkMax(RobotMap.liftLead, CANSparkMaxLowLevel.MotorType.kBrushless);
		liftFollow = new CANSparkMax(RobotMap.liftFollow, CANSparkMaxLowLevel.MotorType.kBrushless);
		liftLead.setInverted(true);
		liftFollow.follow(liftLead, true);
		/*liftLead.setSmartCurrentLimit(130, 80);
		liftFollow.setSmartCurrentLimit(130, 80);*/
		bottomLimit = new DigitalInput(RobotMap.liftBottomLimit);
		topLimit = new DigitalInput(RobotMap.liftHighLimit);
		mLiftStates = LiftStates.IDLE;
		mLiftPosition = LiftPosition.BOTTOM_LIMIT;
 		reloadGains();

 		liftLead.getEncoder().setPositionConversionFactor(48);

 		intakePneumaticWait = 0;
 		mCargoIntakeSubsystem = CargoIntakeSubsystem.getInstance();
 		lastTimeStamp = 0;
 		needsIntakeOut = false;
 		wasIntakeOut = false;

 		if(FeatureFlags.FF_LIFT_TILT){
 		    //liftLeaner = new DoubleSolenoid(RobotMap.liftStandUp, RobotMap.liftLeanBack);
        }
	}


	@Override
	public void outputToSmartDashboard() {
		if(debug) {
			SmartDashboard.putString(Constants.kLiftKey + "State: ", returnLiftStates().toString());
			SmartDashboard.putString(Constants.kLiftKey + "Position", returnLiftPosition().toString());
			SmartDashboard.putNumber(Constants.kLiftKey + "Encoder Position", liftLead.getEncoder().getPosition());
			SmartDashboard.putNumber(Constants.kLiftKey + "Velocity", liftLead.getEncoder().getVelocity());
			SmartDashboard.putNumber(Constants.kLiftKey + "Leader Power Sending", liftLead.getAppliedOutput());
			SmartDashboard.putNumber(Constants.kLiftKey + "Follow Power Sending", liftFollow.getAppliedOutput());
			SmartDashboard.putBoolean(Constants.kLiftKey + "Bottom limit triggered", areWeAtBottomLimit());
			SmartDashboard.putNumber(Constants.kLiftKey + "Wanted Height", mLiftPosition.getHeight());
			SmartDashboard.putNumber(Constants.kLiftKey + "Lead Motor Temp", liftLead.getMotorTemperature());
			SmartDashboard.putNumber(Constants.kLiftKey + "Follow Motor Temp", liftFollow.getMotorTemperature());
			SmartDashboard.putBoolean(Constants.kLiftKey + "is at position?", isAtPosition());
		}

        }

	@Override
	public void stop() {
        mLiftStates = LiftStates.IDLE;
        setLiftIdle();
	}

	@Override
	public synchronized void zeroSensors() {
		liftLead.getEncoder().setPosition(0);
	}


    public synchronized void zeroSensorsAtTop(){
        liftLead.getEncoder().setPosition(0);
        mLiftStates = LiftStates.POSITION_CONTROL;
    }
    public synchronized void zeroSensorsAtBottom(){
		liftLead.getEncoder().setPosition(0);
        mLiftStates = LiftStates.POSITION_CONTROL;
    }

	@Override
	public void registerEnabledLoops(Looper enabledLooper) {
        enabledLooper.register(new Loop() {
			/**
			 * Set lift to idle.
			 * @param timestamp current robot runtime in seconds.
			 */
			@Override
            public void onStop(double timestamp) {
                mLiftStates = LiftStates.IDLE;
            }

			/**
			 *
			 * @param timestamp current robot runtime in seconds
			 */
            @Override
            public void onStart(double timestamp) {
				if(Robot.needToPositionControlInTele){
					setLiftHoldPosition();
				} else {
					mLiftStates = LiftStates.IDLE;
				}
            }

			/**
			 *
			 * @param timestamp current robot runtime in seconds
			 */
			@Override
            public void onLoop(double timestamp) {

            		//not moving and not manual
					if(isAtPosition() && mLiftStates != LiftStates.MANUAL_CONTROL){
						//not moving, at bottom
						if(mLiftStates == LiftStates.RESET_TO_BOTTOM || mLiftPosition == LiftPosition.BOTTOM_LIMIT
								|| areWeAtBottomLimit()
								){
							mLiftStates = LiftStates.IDLE;
						}
						//not moving, not at bottom
						else{
							mLiftStates = LiftStates.HOLD_POSITION;
							holdPosition();

						}
					}
					liftStateLoop();

					lastTimeStamp = timestamp;
				}



			private void liftStateLoop(){
				switch(mLiftStates) {
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
						liftLead.getPIDController().setReference(0, ControlType.kDutyCycle);
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

	public static LiftSubsystem getInstance() {
		return mLiftSubsystem;
	}

	public synchronized void goToSetpoint(LiftPosition setpoint) {
		mLiftStates = LiftStates.POSITION_CONTROL;
		mLiftPosition = setpoint;
		setBrakeMode();
		/*if(currentLiftCommandNeedsIntakeExtension(setpoint)){
			needsIntakeOut = true;
			wasIntakeOut = mCargoIntakeSubsystem.isOuterIntakeExtended();
			mCargoIntakeSubsystem.extendOuterIntake();
			intakePneumaticWait =0; //Constants.kLiftWaitForExtendIntake;
		}
		else {*/
		liftLead.getPIDController().setReference(mLiftPosition.getHeight(), ControlType.kPosition);
		//}
		//System.out.println(mLiftWantedPosition + "  " + isReadyForSetpoint());
	}
	public synchronized void goToSetpoint(double setpoint) {
		mLiftStates = LiftStates.POSITION_CONTROL;
		mLiftPosition.TEMP_HOLD_POS.setHeight(setpoint);
		mLiftPosition = LiftPosition.TEMP_HOLD_POS;
		setBrakeMode();
		liftLead.getPIDController().setReference(mLiftPosition.getHeight(), ControlType.kPosition);
		//System.out.println(mLiftWantedPosition + "  " + isReadyForSetpoint());
	}

	public synchronized void zeroOnBottom() {
		// TODO Auto-generated method stub
		if(mLiftStates != LiftStates.RESET_TO_BOTTOM) {
			mLiftStates = LiftStates.RESET_TO_BOTTOM;
		}
	}

	public synchronized void goToTop() {
		// TODO Auto-generated method stub

	}

	public double getHeightInInches() {
		return getHeightInCounts() / Constants.kCountsPerInch;
	}

	public double getHeightInCounts() {
		return liftLead.getEncoder().getPosition();
	}

	public boolean isOnTarget() {
		return mIsOnTarget;
	}
	public void setBrakeMode(){

		liftLead.setIdleMode(CANSparkMax.IdleMode.kBrake);
		liftFollow.setIdleMode(CANSparkMax.IdleMode.kBrake);
		isBrakeMode = true;

	}
	public void setCoastMode() {
		liftLead.setIdleMode(CANSparkMax.IdleMode.kCoast);
		liftFollow.setIdleMode(CANSparkMax.IdleMode.kCoast);
		isBrakeMode = false;
	}
	public boolean isInBrakeMode() {
		return isBrakeMode;
	}
	public void reloadGains() {
		/*
		liftLead.setParameter(CANSparkMaxLowLevel.ConfigParameter.kP_0, Constants.kLiftPositionkP);
		liftLead.setParameter(CANSparkMaxLowLevel.ConfigParameter.kI_0, Constants.kLiftPositionkI);
		liftLead.setParameter(CANSparkMaxLowLevel.ConfigParameter.kD_0, Constants.kLiftPositionkD);
		liftLead.setParameter(CANSparkMaxLowLevel.ConfigParameter.kF_0, Constants.kLiftPositionkF);
		liftLead.setParameter(CANSparkMaxLowLevel.ConfigParameter.kIZone_0, Constants.kLiftPositionIZone);
		liftLead.setParameter(CANSparkMaxLowLevel.ConfigParameter.kRampRate, Constants.kLiftPositionRampRate);
		*/
		liftLead.getPIDController().setP(Constants.kLiftPositionkP);
		liftLead.getPIDController().setI(Constants.kLiftPositionkI);
		liftLead.getPIDController().setD(Constants.kLiftPositionkD);
		liftLead.getPIDController().setFF(Constants.kLiftPositionkF);
		liftLead.getPIDController().setIZone(Constants.kLiftPositionIZone);

	}

	public synchronized void resetToBottom() {
		if(!areWeAtBottomLimit() || Math.abs(liftLead.getEncoder().getPosition()) < Constants.kBottomLimitTolerance) {
		        mLiftStates = LiftStates.RESET_TO_BOTTOM;
		        mLiftPosition = LiftPosition.BOTTOM_LIMIT;
				goToSetpoint(LiftPosition.BOTTOM_LIMIT);
		}
	}

	public synchronized void resetToTop() {
        if(!topLimit.get()) {
            if(mLiftStates != LiftStates.RESET_TO_TOP){
                mLiftStates = LiftStates.RESET_TO_TOP;
				mLiftPosition = LiftPosition.TOP_LIMIT;
			}
            liftLead.getPIDController().setReference(Constants.liftSpeed, ControlType.kDutyCycle);
        } else {
            zeroSensorsAtTop();
        }
	}

	/**
	 * @return
	 * Returns whether or not the liftactions is ready to be held at position for a cube to be deposited
	 */
	public synchronized boolean isAtPosition() {
		if(mLiftStates == LiftStates.IDLE){
			return false;
		}
		return Math.abs(mLiftPosition.getHeight() - liftLead.getEncoder().getPosition()) < Constants.kLiftPositionTolerance &&
				Math.abs(liftLead.getEncoder().getVelocity()) < Constants.kLiftVelocityTolerance;
	}
	/**
	 *
	 * @return
	 * returns current state of cube
	 */
	public synchronized LiftStates returnLiftStates() {
		return mLiftStates;
	}

    /**
     * @return
     * returns the position of where the liftactions is eg: moving, scale
     */
	public synchronized LiftPosition returnLiftPosition() {
		return mLiftPosition;
	}

    /**
     * Used to stop the manipulator from running. mostly ran on stop or when first setting the liftactions up
     */
	public synchronized void setLiftIdle(){
	    mLiftStates = LiftStates.IDLE;
        liftLead.getPIDController().setReference(0, ControlType.kDutyCycle); //DutyCycle just means voltage on scale of -1 to 1
    }
	public synchronized  void setLiftHoldPosition(){
		mLiftStates = LiftStates.POSITION_CONTROL;
		goToSetpoint(mLiftPosition.TEMP_HOLD_POS.getHeight());
	}
    /**
     * Sets up the robot to accept position setpoints
     */
    public synchronized void updatePositionControl(){
		mLiftStates = LiftStates.POSITION_CONTROL;
        setBrakeMode();
    }


	public synchronized void goToTeleOpHold(){
    	mLiftPosition = LiftPosition.TELEOP_HOLD;
    	mLiftStates = LiftStates.POSITION_CONTROL;
    	if(isReadyForSetpoint()){
    		goToSetpoint(Constants.kTeleOpHoldHeight);
		}
	}
	private synchronized boolean isReadyForSetpoint(){
    	return (isCurrentModesReady());
	}
	private synchronized boolean isCurrentModesReady(){
    	return mLiftStates == LiftStates.POSITION_CONTROL ||
				mLiftStates == LiftStates.IDLE ||
				mLiftStates == LiftStates.HOLD_POSITION ||
				mLiftStates == LiftStates.RESET_TO_BOTTOM;
	}

	public synchronized void manualMode(double power){
    	mLiftStates = LiftStates.MANUAL_CONTROL;
		liftLead.getPIDController().setReference(power, ControlType.kDutyCycle);
		if(Math.abs(power)< .2) {
			goToSetpoint(liftLead.getEncoder().getPosition());
			mLiftStates = LiftStates.HOLD_POSITION;
		}
	}

	/**
	 * Used to hold the cube when it is ready to be spat out
	 */
	public synchronized void holdPosition(){
		liftLead.getPIDController().setReference(Constants.kLiftHoldPercentOutput +
				( mLiftPosition.getHeight() - liftLead.getEncoder().getPosition()) * Constants.kLiftHoldkPGain, ControlType.kDutyCycle);
    	if(Math.abs(getHeightInCounts() - mLiftPosition.getHeight()) > Constants.kLiftPositionTolerance){
    		mLiftStates = LiftStates.POSITION_CONTROL;
    		goToSetpoint(mLiftPosition);
		}
	}

	/**
	 * @return
	 * are we at the bottom limit??
	 */
	public boolean areWeAtBottomLimit(){
		return !bottomLimit.get();
	}
	public double returnLiftHeight(){
		return liftLead.getEncoder().getPosition();
	}
	public synchronized boolean bumpHeightUp(){
		if(mLiftStates == LiftStates.POSITION_CONTROL || mLiftStates == LiftStates.HOLD_POSITION) {
			mLiftPosition = LiftPosition.TEMP_HOLD_POS.setHeight(mLiftPosition.getHeight() + Constants.kBumpEncoderPosition);
			goToSetpoint(mLiftPosition);
			return true;
		}
		return false;
	}
	public synchronized boolean bumpHeightDown(){
		if(mLiftStates == LiftStates.POSITION_CONTROL || mLiftStates == LiftStates.HOLD_POSITION ){
			mLiftPosition = LiftPosition.TEMP_HOLD_POS.setHeight(mLiftPosition.getHeight() - Constants.kBumpEncoderPosition);
			goToSetpoint(mLiftPosition);
			return true;
		}
		return false;
	}

	public synchronized boolean isNeedingIntakeOut(){
		return needsIntakeOut;
	}

	public void standLiftUp(){
	    if(FeatureFlags.FF_LIFT_TILT){
	        liftLeaner.set(DoubleSolenoid.Value.kForward);
        }
    }

    public void leanLiftBack(){
	    if(FeatureFlags.FF_LIFT_TILT){
	        liftLeaner.set(DoubleSolenoid.Value.kReverse);
        }
    }



	public void goToHatchMode(){
		//cargo intake won't retract if the lift needs it out, then the lift's loop will retract it anyway
	}

	public void goToCargoMode(){
		//oddly nothing to do here
	}

	public void retractAll() {
				LiftPosition.TEMP_HOLD_POS.setHeight(Constants.kMaxLiftHeightToNeedToExtendIntake + Constants.kSafeLiftHeightOffsetToNotHitIntake);
				goToSetpoint(LiftPosition.TELEOP_HOLD);
	}
}
