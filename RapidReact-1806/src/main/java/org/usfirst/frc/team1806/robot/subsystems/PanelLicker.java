package org.usfirst.frc.team1806.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.RobotMap;
import org.usfirst.frc.team1806.robot.loop.Loop;
import org.usfirst.frc.team1806.robot.loop.Looper;

public class PanelLicker implements Subsystem{

    private static boolean DEBUG = false;

    private static PanelLicker mPanelLicker = null; //new PanelLicker();
    private LickPosition mLickPosition;
    private LickState mLickState;
    public TalonSRX mLickMotor; //Don't lick 775 pros, they don't taste good.


    public static PanelLicker getInstance(){
       return mPanelLicker;
    }

    /**
     * The loop for the Panel Licker.
     */
    private Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            //Do nothing
        }

        /**
         * Does various things based on the current {@link LickState}.
         * @param timestamp current robot runtime in seconds
         */
        @Override
        public void onLoop(double timestamp) {
            switch(mLickState){
                case IDLE:
                    default:
                        mLickMotor.set(ControlMode.PercentOutput, 0);
                        break;
                case MOVING:
                    if(isTongueInPosition()){
                        mLickState = LickState.AT_POSITION;
                    }
                    break;
                case AT_POSITION:
                    mLickMotor.set(ControlMode.PercentOutput, 0);
                    break;
                case MANUAL:
                    break;

            }
        }

        @Override
        public void onStop(double timestamp) {

        }
    };

    @Override
    public void writeToLog() {

    }

    @Override
    public void outputToSmartDashboard() {
        if(DEBUG){
            SmartDashboard.putNumber("Lick Position", mLickMotor.getSelectedSensorPosition());
        }
    }

    @Override
    public void stop() {
        mLickMotor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void zeroSensors() {
        mLickMotor.setSelectedSensorPosition(0);
    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
        enabledLooper.register(mLoop);
    }

    @Override
    public void setDebug(boolean _debug) {

    }

    @Override
    public void goToHatchMode() {
        lickToPosition(LickPosition.IN);
    }

    @Override
    public void goToCargoMode() {
        lickToPosition(LickPosition.REJECT_PICKUP);
    }

    @Override
    public void retractAll() {
        lickToPosition(LickPosition.IN);
    }

    /**
     * Positions that the Panel Licker can be in
     */
    public enum LickPosition {
        IN(0),
        REJECT_PICKUP(5),
        HOLD_BALL(20),
        TRANSFER_TO_SQUID(2048);


        private int mPosition;

        /**
         *
         * @param position encoder counts for the position.
         */
        LickPosition(int position){
            mPosition = position;
        }

        /**
         *
         * @return an int representing the encoder counts for the given position.
         */
        public int getmPosition() {
            return mPosition;
        }
    }

    public enum LickState{
        IDLE,
        MOVING,
        AT_POSITION,
        MANUAL
    }

    /**
     * Initialize the panel licker.
     */
    private PanelLicker(){
        mLickMotor = new TalonSRX(RobotMap.tongueMotor);
        mLickPosition = LickPosition.IN;
        mLickState = LickState.IDLE;
        reloadGains();
    }

    /**
     * reloads the PID gains for the panel licker.
     */
    public void reloadGains(){
        mLickMotor.config_kP(0, Constants.kLickKp);
        mLickMotor.config_kI(0, Constants.kLickKi);
        mLickMotor.config_kD(0, Constants.kLickKd);
        mLickMotor.config_kF(0, Constants.kLickKf);
        mLickMotor.config_IntegralZone(0, Constants.kLickKiZone);
    }

    /**
     * puts the tongue to a {@link LickPosition}
     */
    public void lickToPosition(LickPosition lickPosition){
        mLickPosition = lickPosition;
        mLickState = LickState.MOVING;
        mLickMotor.set(ControlMode.Position, lickPosition.getmPosition());
    }

    /**
     *
     * @return return true if tongue is in tolerance of the set position, false if not.
     */
    private boolean isTongueInPosition(){
        int lickPos = mLickMotor.getSelectedSensorPosition();
        return lickPos > mLickPosition.getmPosition() - Constants.kLickPositionTolerance && lickPos < mLickPosition.getmPosition() + Constants.kLickPositionTolerance;
    }


    /**
     *
     * @param power The power to move the tongue with. If the absolute value is less than .2 and the lick was being manually controlled, this will set the {@link PanelLicker} to an idle state.
     *              A power of less than .2 will not interfere with exiting control modes.
     */
    public void lickManually(double power){
        if(Math.abs(power) > .2){
            mLickState = LickState.MANUAL;
            mLickMotor.set(ControlMode.PercentOutput, power);
        }
        else{
            if(mLickState == LickState.MANUAL){
                mLickState = LickState.IDLE;
            }
        }
    }

    /**
     * Sets whether or not to debug the panel licker.
     * @param DEBUG whether or not to debug the panel licker. If true, will start outputting to smart dashboard.
     */

    public static void setDEBUG(boolean DEBUG) {
        PanelLicker.DEBUG = DEBUG;
    }

    /**
     *
     * @return whether or not this subsystem is outputting to smart dashboard
     */
    public static boolean isDEBUG() {
        return DEBUG;
    }

    /**
     *
     * @return the current set {@link LickPosition}
     */
    public LickPosition getmLickPosition() {
        return mLickPosition;
    }

    /**
     *
     * @return the current set {@link LickState}
     */
    public LickState getmLickState() {
        return mLickState;
    }
}
