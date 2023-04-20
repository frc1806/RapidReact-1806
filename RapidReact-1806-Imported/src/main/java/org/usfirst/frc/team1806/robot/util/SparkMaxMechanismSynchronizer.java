package org.usfirst.frc.team1806.robot.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

public class SparkMaxMechanismSynchronizer {
    private CANSparkMax left;
    private CANSparkMax right;

    private double wantedPosition;
    private double wantedSpeed;
    private double synchronizerProportional;
    private double throttleLetOffDistance;
    private double velocityTolerance;
    private double positionTolerance;
    private SynchronizerState mSynchronizerState;

    public enum SynchronizerState {
        IDLE,
        MOVING,
        COMPLETE,
        MANUAL
    }

    public SparkMaxMechanismSynchronizer(CANSparkMax left, CANSparkMax right) {
        this.left = left;
        this.right = right;
        velocityTolerance = 1;
        positionTolerance = 1;
        throttleLetOffDistance = 1;
        synchronizerProportional= 0;
        mSynchronizerState = SynchronizerState.IDLE;
    }

    public double getEncoderPosLeft() {
        return left.getEncoder().getPosition();
    }

    public double getEncoderPosRight() {
        return right.getEncoder().getPosition();
    }

    public double getEncoderVelocityLeft() {
        return left.getEncoder().getVelocity();

    }

    public double getEncoderVelocityRight() {
        return right.getEncoder().getVelocity();
    }

    public void reloadLeftMotorGains(double p, double i, double d, double f, double iZone) {
        left.getPIDController().setP(p);
        left.getPIDController().setI(i);
        left.getPIDController().setD(d);
        left.getPIDController().setFF(f);
        left.getPIDController().setIZone(iZone);

    }

    public void reloadRightMotorGains(double p, double i, double d, double f, double iZone) {
        right.getPIDController().setP(p);
        right.getPIDController().setI(i);
        right.getPIDController().setD(d);
        right.getPIDController().setFF(f);
        right.getPIDController().setIZone(iZone);
    }

    public void reloadBothMotorGains(double p, double i, double d, double f, double iZone) {
        reloadLeftMotorGains(p, i, d, f, iZone);
        reloadRightMotorGains(p, i, d, f, iZone);
    }

    public void configureSynchronizerParameters(double p, double throttleLetOffDistance, double velocityTolerance, double positionTolerance){
        synchronizerProportional = p;
        this.throttleLetOffDistance = throttleLetOffDistance;
        this.velocityTolerance = velocityTolerance;
        this.positionTolerance = positionTolerance;
    }

    public void resetEncoders(){
        left.getEncoder().setPosition(0.0);
        right.getEncoder().setPosition(0.0);
    }

    public void update(){
        switch (mSynchronizerState){
            case IDLE:
                stop();
                break;
            case MOVING:
                updateMovingSetpoints();
                if(isFinished()){
                    mSynchronizerState = SynchronizerState.COMPLETE;
                }
                break;
            case COMPLETE:
                stop();
                break;
            case MANUAL:
                break;
            default:
                stop();
                break;
        }
    }

    public void setWantedMovement(double position, double speed){
        wantedPosition = position;
        wantedSpeed = Math.abs(speed);
        mSynchronizerState = SynchronizerState.MOVING;
    }
    public void stop(){
        left.getPIDController().setReference(0.0, ControlType.kDutyCycle);
        right.getPIDController().setReference(0.0, ControlType.kDutyCycle);
    }

    public void stopSynchonizer() {
        mSynchronizerState = SynchronizerState.IDLE;
        stop();
    }

    public void updateMovingSetpoints(){
        double leftEncoder = getEncoderPosLeft();
        double rightEncoder = getEncoderPosRight();
        boolean leftMovementPositive = leftEncoder - wantedPosition < 0;
        boolean rightMovementPositive = rightEncoder - wantedPosition < 0;
        double leftTargetSpeed = (rightEncoder - leftEncoder)* (leftMovementPositive?synchronizerProportional:-synchronizerProportional) + Math.min(wantedSpeed, (wantedSpeed*Math.abs(wantedPosition-leftEncoder)/throttleLetOffDistance));
        double rightTargetSpeed = (leftEncoder - rightEncoder)* (rightMovementPositive?synchronizerProportional: -synchronizerProportional) + Math.min(wantedSpeed, (wantedSpeed*Math.abs(wantedPosition-rightEncoder)/throttleLetOffDistance));
        left.getPIDController().setReference(leftMovementPositive?leftTargetSpeed:-leftTargetSpeed,ControlType.kVelocity);
        right.getPIDController().setReference(rightMovementPositive?rightTargetSpeed:-rightTargetSpeed,ControlType.kVelocity);
    }

    private boolean isFinished(){
        return Math.abs(left.getEncoder().getVelocity()) < velocityTolerance && Math.abs(left.getEncoder().getPosition() - wantedPosition) < positionTolerance
                && Math.abs(right.getEncoder().getVelocity()) < velocityTolerance && Math.abs(right.getEncoder().getPosition() - wantedPosition) < positionTolerance;
    }

    /*
        Force the synchronizer to not update the motors. Consumers will need to write values to the motors for manual mode as this class wants nothing to do with manual control.
     */
    public void forceToManual(){
        mSynchronizerState = SynchronizerState.MANUAL;
    }

    public SynchronizerState getState(){
        return mSynchronizerState;
    }
}

