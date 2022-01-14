package org.usfirst.frc.team1806.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SensorUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.RobotMap;
import org.usfirst.frc.team1806.robot.loop.Looper;

public class SquidSubsystem implements Subsystem {

    boolean debug = false;
    private DoubleSolenoid mSquidSolenoid;
    private DoubleSolenoid mSquidExtender;
    private DigitalInput mHatchDetector;

    private static SquidSubsystem mSquidSubsystem = new SquidSubsystem();
    public static SquidSubsystem getInstance(){
        return mSquidSubsystem;
    }

    private SquidSubsystem(){

        mSquidSolenoid = new DoubleSolenoid(RobotMap.squidOpenPort, RobotMap.squidClosePort);
        mSquidExtender = new DoubleSolenoid(RobotMap.squidExtendForward, RobotMap.squidExtendBackward);
        mHatchDetector = new DigitalInput(RobotMap.hatchDetector);

    }

    public void writeToLog(){

    }

    public void outputToSmartDashboard(){
    if(debug) {
        SmartDashboard.putString(Constants.kSquidKey + "State", mSquidSolenoid.get().name());
        SmartDashboard.putString(Constants.kSquidKey + "Extend" , mSquidExtender.get().name());
    }

    }

    public void stop(){

    }

    public void zeroSensors(){

    }

    /**
     * Checks if squid is open
     * @return boolean where extended is true and not extended is false
     */
    public boolean isOpen() {
        return mSquidSolenoid.get() == DoubleSolenoid.Value.kForward;
    }

    /**
     * Checks if hatch is in the squid
     * @return true if there is a hatch in the squid if not return false
     */
    public boolean isDetectingHatch(){
        return mHatchDetector.get();
    }


    public void registerEnabledLoops(Looper enabledLooper){

    }

    @Override
    public void setDebug(boolean _debug) {
        debug = _debug;
    }

    /**
     * opens the squid
     */
    public void openSquid(){
        mSquidSolenoid.set(DoubleSolenoid.Value.kForward);

    }

    /**
     * closes the squid
     */
    public void closeSquid(){
        mSquidSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    /**
     * extends the squid
     */
    public void extendSquid() {
        mSquidExtender.set(DoubleSolenoid.Value.kForward);
    }

    /**
     * retracts the squid
     */
    public void retractSquid () {
        mSquidExtender.set(DoubleSolenoid.Value.kReverse);
    }

    /**
     * checks if the squid is extended
     * @return true if extended if not return false
     */
    public boolean isExtended() {
        return mSquidExtender.get() == DoubleSolenoid.Value.kForward;
    }

    /**
     * opens the squid if it detects a hatch panel
     */
    public void openSquidIfHatch() {
        if (isDetectingHatch()) {
            openSquid();
        } else if (isOpen()) {
            closeSquid();
        }
    }

    public void goToHatchMode(){
        //don't want to extend on mode switch
    }

    public void goToCargoMode(){
        retractSquid();
    }

    public void retractAll() {
        retractSquid();
    }
}

