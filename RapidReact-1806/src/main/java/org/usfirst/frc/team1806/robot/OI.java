/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1806.robot;

import java.util.function.DoubleSupplier;
import java.util.function.Function;

import javax.lang.model.util.ElementScanner6;

import org.usfirst.frc.team1806.robot.game.Shot;
import org.usfirst.frc.team1806.robot.game.Shot.ShotDashboard;
import org.usfirst.frc.team1806.robot.subsystems.*;
import org.usfirst.frc.team1806.robot.subsystems.SuperStructure.SuperStructureStates;
import org.usfirst.frc.team1806.robot.util.CheesyDriveHelper;
import org.usfirst.frc.team1806.robot.util.Latch;
import org.usfirst.frc.team1806.robot.util.XboxController;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */

public class OI {
	private SendableChooser<DriverConrollerConfigs> controllerConfigChooser;

	protected static XboxController driverController = new XboxController(0, "Driver",
			Constants.kDriverControllerDefaultConfig);
	protected static XboxController operatorController = new XboxController(1, "Operator",
			Constants.kOperatorControllerDefaultConfig);
	protected static XboxController debugController = new XboxController(2, "Debug", Constants.kOperatorControllerDefaultConfig);

	private enum DriverConrollerConfigs{
		 kRetroGranTurismo(
			 new DoubleSupplier(){

			@Override
			public double getAsDouble() {
				//throttle
				return driverController.getConfigValues().getRightYMinimumOutput();
			} }, new DoubleSupplier(){

			@Override
			public double getAsDouble() {
				//wheel
				return driverController.getConfigValues().getLeftXMinimumOutput();
			} }),
		 kCallOfDuty(
			new DoubleSupplier(){

		   @Override
		   public double getAsDouble() {
			   //throttle
			   return driverController.getConfigValues().getLeftYMinimumOutput();
		   } }, new DoubleSupplier(){

		   @Override
		   public double getAsDouble() {
			   //wheel
			   return driverController.getConfigValues().getRightXMinimumOutput();
		   } }),
		 kForza(
			new DoubleSupplier(){

		   @Override
		   public double getAsDouble() {
			   //throttle
			   return driverController.getConfigValues().getTriggerMinimumOutput();
		   } }, new DoubleSupplier(){

		   @Override
		   public double getAsDouble() {
			   //wheel
			   return driverController.getConfigValues().getLeftXMinimumOutput();
		   } });

		 DoubleSupplier throttleDeadBandGetter, wheelDeadBandGetter;
		 private DriverConrollerConfigs(DoubleSupplier throttleDeadbandGetter, DoubleSupplier wheelDeadbandGetter)
		 {
			this.throttleDeadBandGetter = throttleDeadbandGetter;
			this.wheelDeadBandGetter = wheelDeadbandGetter;
		 }

		 public double getThrottleDeadBand(){
			 return throttleDeadBandGetter.getAsDouble();
		 }

		 public double getWheelDeadBand(){
			 return wheelDeadBandGetter.getAsDouble();
		 }

		 public CheesyDriveHelper getCheesyDriveHelper(){
			 return new CheesyDriveHelper(getWheelDeadBand(), getThrottleDeadBand());
		 }
	 }

	// snag some subsystem instances
	private DriveTrainSubsystem mDriveTrainSubsystem = DriveTrainSubsystem.getInstance();
	private SuperStructure mSuperStructure = SuperStructure.getInstance();
	private LEDStringSubsystem mLedStringSubsystem = LEDStringSubsystem.getInstance();

	// initialise controllers & ish
	private CheesyDriveHelper mCheesyDriveHelper;

	private DriverConrollerConfigs lastDriverControllerConfig;

	// Forza Config member vars
	double leftTriggerTimestamp = Double.MAX_VALUE;
	double rightTriggerTimestamp = Double.MAX_VALUE;

	public OI() {
		controllerConfigChooser = new SendableChooser<DriverConrollerConfigs>();
		controllerConfigChooser.addOption("Retro Gran Turismo", DriverConrollerConfigs.kRetroGranTurismo);
		controllerConfigChooser.addOption("Call Of Duty", DriverConrollerConfigs.kCallOfDuty);
		controllerConfigChooser.addOption("Forza", DriverConrollerConfigs.kForza);
		Robot.getMainDriverTab().add("Controller Config", controllerConfigChooser);

		mCheesyDriveHelper = DriverConrollerConfigs.kRetroGranTurismo.getCheesyDriveHelper();
		lastDriverControllerConfig = DriverConrollerConfigs.kRetroGranTurismo;
	}

	// start up button trackers
	private Latch autoInTeleOp = new Latch();
	private Boolean wasShift = false;
	private Boolean wasParkingBrake = false;

	

	public void runCommands() {
		double timestamp = Timer.getFPGATimestamp();

		boolean dashboardShot = debugController.getButtonA();
		//LED Controlls
		if(operatorController.getPOVUp()){
			mLedStringSubsystem.setRobotLEDModeGlitchy();
		}
		else if (operatorController.getPOVDown()){
			mLedStringSubsystem.setRobotLEDModeOff();
		}
		else if (operatorController.getPOVRight()){
			mLedStringSubsystem.setRobotLEDModeClimbComplete();
		}
		else{
			mLedStringSubsystem.setRobotLEDModeNormal();
		}

		// handle config switching
		DriverConrollerConfigs currentControllerConfig;
		if(controllerConfigChooser.getSelected() != null)
		{
			currentControllerConfig = controllerConfigChooser.getSelected();
			if (currentControllerConfig != lastDriverControllerConfig) {
				mCheesyDriveHelper = controllerConfigChooser.getSelected().getCheesyDriveHelper();
			}
			lastDriverControllerConfig = currentControllerConfig;
		}
		else
		{
			currentControllerConfig = DriverConrollerConfigs.kRetroGranTurismo;
		}



		// driver controls based on current config
		switch (currentControllerConfig) {
			case kCallOfDuty:{
				// buttons
				boolean quickTurn = driverController.getPOVLeft(); // Will Map to paddle
				boolean visionLineup = driverController.getPOVRight(); // Will map to paddle
				boolean shiftHighGear = driverController.getButtonRB();
				boolean shiftLowGear = driverController.getButtonLB();

				boolean intakeFront = driverController.getRightTriggerAsDigital();
				boolean intakeBack = driverController.getLeftTriggerAsDigital();
				boolean feedThroughFromFront = driverController.getPOVUp();
				boolean feedThroughFromBack = driverController.getPOVDown();
				// face buttons are all unused.

				// Superstructure stuff
				if (intakeFront) {
					mSuperStructure.wantIntakeFront();
				} else if (intakeBack) {
					mSuperStructure.wantIntakeBack();
				}

				else if (feedThroughFromFront){
					mSuperStructure.wantFeedFrontIntake();
				} else if (feedThroughFromBack) {
					mSuperStructure.wantFeedBackIntake();
				} else if (dashboardShot){
					mSuperStructure.wantPrepareShot(Shot.TheShotDashboard.getDashboardShot());
				}
				 else {
					mSuperStructure.stop();
				}

				// Handle shfting
				if (shiftHighGear) {
					mDriveTrainSubsystem.setHighGear(true);
				} else if (shiftLowGear) {
					mDriveTrainSubsystem.setHighGear(false);
				} 
				// decide throttle with triggers

				double throttle = driverController.getLeftJoyY();

				double turn = visionLineup ? 0.0:driverController.getRightJoyX(); // TODO: Add Vision Lineup code

				mDriveTrainSubsystem.setOpenLoop(
						mCheesyDriveHelper.cheesyDrive(throttle, turn, quickTurn, mDriveTrainSubsystem.isHighGear()));
			}
				break;
			case kForza: {
				// Tunable things
				double MIN_SPEED_TO_CONSIDER_MOVING = 1.0; // inches per second

				// buttons
				boolean quickTurn = driverController.getButtonA();
				boolean visionLineup = driverController.getButtonX();
				boolean shiftHighGear = driverController.getButtonY();
				boolean shiftLowGear = driverController.getButtonB();

				boolean intakeFront = driverController.getButtonRB();
				boolean intakeBack = driverController.getButtonLB();
				boolean feedThroughFromFront = driverController.getPOVUp();
				boolean feedThroughFromBack = driverController.getPOVDown();
				// POV Left and right are still unused, as is the right stick, and forward/back
				// buttons.

				// Superstructure stuff
				if (intakeFront) {
					mSuperStructure.wantIntakeFront();
				} else if (intakeBack) {
					mSuperStructure.wantIntakeBack();
				}

				else if (feedThroughFromFront){
					mSuperStructure.wantFeedFrontIntake();
				} else if (feedThroughFromBack) {
					mSuperStructure.wantFeedBackIntake();
				} else if (dashboardShot){
					mSuperStructure.wantPrepareShot(Shot.TheShotDashboard.getDashboardShot());
				} else {
					mSuperStructure.stop();
				}


				// Handle shfting
				if (shiftHighGear) {
					mDriveTrainSubsystem.setHighGear(true);
				} else if (shiftLowGear) {
					mDriveTrainSubsystem.setHighGear(false);
				}
				// decide throttle with triggers

				// know which trigger was pressed first if both
				if(leftTriggerTimestamp == Double.MAX_VALUE)
				{
					leftTriggerTimestamp = driverController.getLeftTrigger() > 0 ? timestamp : Double.MAX_VALUE;
				}
				else{
					if(driverController.getLeftTrigger() == 0) leftTriggerTimestamp = Double.MAX_VALUE;
				}
				
				if(rightTriggerTimestamp == Double.MAX_VALUE)
				{
					rightTriggerTimestamp = driverController.getRightTrigger() > 0 ? timestamp : Double.MAX_VALUE;
				}
				else{
					if(driverController.getRightTrigger() == 0) rightTriggerTimestamp = Double.MAX_VALUE;
				}
				
				

				double throttle = 0.0;
				if (leftTriggerTimestamp < rightTriggerTimestamp) {
					throttle = (-driverController.getLeftTrigger()) * (1.0 - driverController.getRightTrigger());
				} else if (rightTriggerTimestamp <= leftTriggerTimestamp) {
					throttle = driverController.getRightTrigger() * (1.0 - driverController.getLeftTrigger());
				}

				double turn = visionLineup ? 0.0:driverController.getLeftJoyX(); // TODO: Add Vision Lineup code

				mDriveTrainSubsystem.setOpenLoop(
						mCheesyDriveHelper.cheesyDrive(throttle, turn, quickTurn, mDriveTrainSubsystem.isHighGear()));
			}
				break;
			default:
			case kRetroGranTurismo: {
				// buttons
				boolean quickTurn = driverController.getPOVLeft(); // Will Map to paddle
				boolean visionLineup = driverController.getPOVRight(); // Will map to paddle
				boolean shiftHighGear = driverController.getButtonRB();
				boolean shiftLowGear = driverController.getButtonLB();

				boolean intakeFront = driverController.getRightTriggerAsDigital();
				boolean intakeBack = driverController.getLeftTriggerAsDigital();
				boolean feedThroughFromFront = driverController.getPOVUp();
				boolean feedThroughFromBack = driverController.getPOVDown();
				// face buttons are all unused.

				// Superstructure stuff
				if (intakeFront) {
					mSuperStructure.wantIntakeFront();
				} else if (intakeBack) {
					mSuperStructure.wantIntakeBack();
				}

				else if (feedThroughFromFront){
					mSuperStructure.wantFeedFrontIntake();
				} else if (feedThroughFromBack) {
					mSuperStructure.wantFeedBackIntake();
				} else if (dashboardShot){
					mSuperStructure.wantPrepareShot(Shot.TheShotDashboard.getDashboardShot());
				} else {
					mSuperStructure.stop();
				}


				// Handle shfting
				if (shiftHighGear) {
					mDriveTrainSubsystem.setHighGear(true);
				} else if (shiftLowGear) {
					mDriveTrainSubsystem.setHighGear(false);
				}
				// decide throttle with triggers

				double throttle = driverController.getRightJoyY();

				double turn = visionLineup ? 0.0:driverController.getLeftJoyX(); // TODO: Add Vision Lineup code

				mDriveTrainSubsystem.setOpenLoop(
						mCheesyDriveHelper.cheesyDrive(throttle, turn, quickTurn, mDriveTrainSubsystem.isHighGear()));
			}
				break;

		}

		return;
	}

	public void updateConfigs() {
		driverController.updateConfig();
		operatorController.updateConfig();
		debugController.updateConfig();
		if(controllerConfigChooser.getSelected() != null)
		{
			mCheesyDriveHelper = controllerConfigChooser.getSelected().getCheesyDriveHelper();
		}
		else
		{
			mCheesyDriveHelper = DriverConrollerConfigs.kRetroGranTurismo.getCheesyDriveHelper();
		}
	}
}