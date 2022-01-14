package org.usfirst.frc.team1806.robot.subsystems;

import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotMap;
import org.usfirst.frc.team1806.robot.loop.Loop;
import org.usfirst.frc.team1806.robot.loop.Looper;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


/**
 * TODO: Dependency Injection, Remove references to snack manipulator, Make-reinstatiatable, make circular buffer more efficient, JavaDoc.
 */

public class IntakeSubsystem implements Subsystem{
	boolean debug = false;

	private TalonSRX leftOuterIntake, rightOuterIntake;
	private double mIntakingSpeed = 0;
	Timer timer, stopperTimer;
	double stopTime = .3; // Seconds to stop the intake for
	boolean hasStopped = false;
	PowerDistribution powerDistributionPanel;
	CircularBuffer intakeCircularBuffer;
	double circularBufferTotal = 0;
	int wantedSize = 30;
	double currentThreshold = 50;
	double baseLine = 3;
	double totalCurrent;
	private Loop mLooper = new Loop() {
		
		@Override
		public void onStop(double timestamp) {
			// TODO Auto-generated method stub
			
		}
		
		@Override
		public void onStart(double timestamp) {
			stopperTimer.reset();
			stopperTimer.start();
		}
		
		@Override
		public void onLoop(double timestamp) {
			// TODO Auto-generated method stub
			
		}
	};

	/**
	 *
	 * @param intakingSpeed The default intake speed
	 * @param rightCAN the id for the right motor TalonSRX
	 * @param leftCAN the id for the left TalonSRX
	 */
	public IntakeSubsystem(double intakingSpeed, int rightCAN, int leftCAN, boolean flipLeft, boolean flipRight) {
		leftOuterIntake = new TalonSRX(rightCAN);
		rightOuterIntake = new TalonSRX(leftCAN);
		intakeCircularBuffer = new CircularBuffer(wantedSize);
		timer = new Timer();
		stopperTimer = new Timer();
		rightOuterIntake.setInverted(flipRight);
		leftOuterIntake.setInverted(flipLeft);
		mIntakingSpeed = intakingSpeed;
	}
	@Override
	public void writeToLog() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void outputToSmartDashboard() {
		SmartDashboard.putNumber(Constants.kIntakeKey + "Total Current", circularBufferTotal);
		SmartDashboard.putNumber(Constants.kIntakeKey + "Average Current", circularBufferTotal / wantedSize);
		SmartDashboard.putBoolean(Constants.kIntakeKey + "Are we over Intake Threshold", circularBufferTotal / wantedSize >= currentThreshold);
	}

	@Override
	public void stop() {
		hasStopped = false;
		timer.reset();
		timer.stop();
		stopAllMotors();
	}

	@Override
	public void zeroSensors() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void registerEnabledLoops(Looper enabledLooper) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void setDebug(boolean _debug) {
		debug = _debug;
	}

	public void intaking(){
		circularBufferTotal = 0;
		//  double totalCurrent = Robot.powerDistributionPanel.getCurrent(5) + Robot.powerDistributionPanel.getCurrent(6);
		double totalCurrent = Robot.powerDistributionPanel.getCurrent(RobotMap.leftOuterIntake) + Robot.powerDistributionPanel.getCurrent(RobotMap.rightOuterIntake);
		intakeCircularBuffer.addFirst(totalCurrent);
		for(int i=0; i < wantedSize ; i++){
			circularBufferTotal += intakeCircularBuffer.get(i);
		}
		if(circularBufferTotal / wantedSize >= currentThreshold && !hasStopped){
			hasStopped = true;
			timer.reset();
			timer.start();
		}
		if(hasStopped && (timer.get() < stopTime)){
			System.out.println("stop time is doing it!");
			//SnackManipulatorSuperStructure.getInstance().intakeCube(0, 0);
		} else if(stopperTimer.get() % 1.1 > 1.0){
			//SnackManipulatorSuperStructure.getInstance().intakeCube(0, 0);
		} else if(hasStopped && timer.get() > stopTime) {
			timer.reset();
			timer.stop();
			hasStopped = false;
		} else{
			//SnackManipulatorSuperStructure.getInstance().intakeCube(1, 1);
		}
		SmartDashboard.putNumber("Total Intake Current", totalCurrent);
		SmartDashboard.putNumber("Average Intake Current", circularBufferTotal / wantedSize);
		SmartDashboard.putBoolean("Are we over Intake Threshold", circularBufferTotal / wantedSize >= currentThreshold);
	}
	public void intakeAtPower(double leftPower, double rightPower){
		leftOuterIntake.set(ControlMode.PercentOutput, leftPower);
		rightOuterIntake.set(ControlMode.PercentOutput, rightPower);
	}
	public void outtaking(double power){
		leftOuterIntake.set(ControlMode.PercentOutput, -power);
		rightOuterIntake.set(ControlMode.PercentOutput, -power);
	}
	public void stopAllMotors() {
		leftOuterIntake.set(ControlMode.PercentOutput,0);
		hasStopped = false;
		timer.reset();
		timer.stop();
		rightOuterIntake.set(ControlMode.PercentOutput, 0);
	}
	public void intakeRightSide(double power){
		rightOuterIntake.set(ControlMode.PercentOutput, power);
	}
	public void intakeLeftSide(double power){
		leftOuterIntake.set(ControlMode.PercentOutput, power);
	}


	public void goToHatchMode(){
		//nothing to do here, this only does cargo
	}

	public void goToCargoMode(){
		//nothing to do here, this only does cargo, this is always ready for cargo. Unless it's borked, but we can't code for that.
	}

	public void retractAll() {
		//extensions not controlled here
	}
}
