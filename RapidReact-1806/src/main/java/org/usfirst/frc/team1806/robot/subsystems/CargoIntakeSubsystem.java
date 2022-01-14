package org.usfirst.frc.team1806.robot.subsystems;

//import jdk.internal.org.objectweb.asm.tree.InnerClassNode;
import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.RobotMap;
import org.usfirst.frc.team1806.robot.loop.Looper;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class CargoIntakeSubsystem implements Subsystem {
    boolean debug = false;
    //private LiftSubsystem liftSubsystem;
    private DoubleSolenoid barExtensionSolenoid;
    private DoubleSolenoid outerExtensionSolenoid;
    private IntakeSubsystem innerIntake;
    private IntakeSubsystem outerIntake;

    /**
     * Different power percentages for scoring a cargo
     */
    public enum ScoringPower {
        SLOW(.2),
        MEDIUM(.4),
        FAST(.6),
        IRRESPONSIBLE(.8),
        PLAID(1.0);


        Double power;

        /** Constructs a ScoringPower
         *
         * @param power this is the power value to use while using the Intake outwards to score the cargo
          (in percentages from 0.0 to 1.0)
         */
        ScoringPower(Double power) {
            this.power = power;
        }

        /**
         *
         * @return the power of scoring the cargo
         */
        Double getPower() {
            return power;
        }
    }

    private static CargoIntakeSubsystem mCargoIntakeSubsystem = new CargoIntakeSubsystem();
    public static CargoIntakeSubsystem getInstance(){
        return mCargoIntakeSubsystem;
    }

    private CargoIntakeSubsystem(){

        barExtensionSolenoid = new DoubleSolenoid(RobotMap.barIntakeExtend, RobotMap.barIntakeRetract);
        outerExtensionSolenoid = new DoubleSolenoid(RobotMap.outerIntakeExtend, RobotMap.outerIntakeRetract);
        innerIntake = new IntakeSubsystem(Constants.kInnerIntakingSpeed, RobotMap.leftInnerIntake, RobotMap.rightInnerIntake, false, false);
        outerIntake = new IntakeSubsystem(Constants.kOuterIntakingSpeed, RobotMap.leftOuterIntake, RobotMap.rightOuterIntake, false, true);
        //liftSubsystem = LiftSubsystem.getInstance();

    }


    public void writeToLog(){

    }

    public void outputToSmartDashboard(){

    }


    public void stop(){
        innerIntake.stop();
        outerIntake.stop();
    }

    public void zeroSensors(){

    }

    public void registerEnabledLoops(Looper enabledLooper){

    }

    @Override
    public void setDebug(boolean _debug) {
        debug = _debug;
    }

    /**
     * function to enable the intake system
     */
    public void intakeCargo(){
    innerIntake.intakeLeftSide(Constants.kInnerIntakingSpeed);
    innerIntake.intakeRightSide(Constants.kInnerIntakingSpeed);
    outerIntake.intakeLeftSide(Constants.kOuterIntakingSpeed);
    outerIntake.intakeRightSide(Constants.kOuterIntakingSpeed);
    }

    /**
     * Reverses the inner intake to shoot the ball.
     * @param power how much power should be used to shoot the ball.
     */
    public void scoreCargo(ScoringPower power){
    innerIntake.outtaking(power.getPower());
    outerIntake.stop();

    }

    public void extendBarIntake(){
        barExtensionSolenoid.set(DoubleSolenoid.Value.kForward);}

    public void retractBarIntake(){
        barExtensionSolenoid.set(DoubleSolenoid.Value.kReverse);}

    public void extendOuterIntake(){
        outerExtensionSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void retractOuterIntake(){
        outerExtensionSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void extendAllIntake(){
        extendBarIntake();
        extendOuterIntake();
    }

    public void retractAllIntake(){
        retractBarIntake();
        retractOuterIntake();
    }

    public boolean isExtended(){
        return barExtensionSolenoid.get() == DoubleSolenoid.Value.kForward && outerExtensionSolenoid.get() == DoubleSolenoid.Value.kForward;
    }

    /**
     * when HatchMode is enabled it retracts outer intake
     */
    public void goToHatchMode(){ retractAllIntake(); }

    public void goToCargoMode(){
        //TODO
    }

    public void retractAll() {
        retractAllIntake();
    }
}