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
import org.usfirst.frc.team1806.robot.util.Rotation2d;
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
	protected static XboxController debugController = new XboxController(2, "Debug",
			Constants.kOperatorControllerDefaultConfig);
			

	private enum DriverConrollerConfigs {
		kRetroGranTurismo(
				new DoubleSupplier() {

					@Override
					public double getAsDouble() {
						// throttle
						return driverController.getConfigValues().getRightYMinimumOutput();
					}
				}, new DoubleSupplier() {

					@Override
					public double getAsDouble() {
						// wheel
						return driverController.getConfigValues().getLeftXMinimumOutput();
					}
				}),
		kCallOfDuty(
				new DoubleSupplier() {

					@Override
					public double getAsDouble() {
						// throttle
						return driverController.getConfigValues().getLeftYMinimumOutput();
					}
				}, new DoubleSupplier() {

					@Override
					public double getAsDouble() {
						// wheel
						return driverController.getConfigValues().getRightXMinimumOutput();
					}
				}),
		kForza(
				new DoubleSupplier() {

					@Override
					public double getAsDouble() {
						// throttle
						return driverController.getConfigValues().getTriggerMinimumOutput();
					}
				}, new DoubleSupplier() {

					@Override
					public double getAsDouble() {
						// wheel
						return driverController.getConfigValues().getLeftXMinimumOutput();
					}
				});

		DoubleSupplier throttleDeadBandGetter, wheelDeadBandGetter;

		private DriverConrollerConfigs(DoubleSupplier throttleDeadbandGetter, DoubleSupplier wheelDeadbandGetter) {
			this.throttleDeadBandGetter = throttleDeadbandGetter;
			this.wheelDeadBandGetter = wheelDeadbandGetter;
		}

		public double getThrottleDeadBand() {
			return throttleDeadBandGetter.getAsDouble();
		}

		public double getWheelDeadBand() {
			return wheelDeadBandGetter.getAsDouble();
		}

		public CheesyDriveHelper getCheesyDriveHelper() {
			return new CheesyDriveHelper(getWheelDeadBand(), getThrottleDeadBand());
		}
	}

	// snag some subsystem instances
	private DriveTrainSubsystem mDriveTrainSubsystem = DriveTrainSubsystem.getInstance();
	private SuperStructure mSuperStructure = SuperStructure.getInstance();
	//private LEDStringSubsystem mLedStringSubsystem = LEDStringSubsystem.getInstance();
	private LaunchBoxAngler mLaunchBoxAngler = LaunchBoxAngler.getInstance();

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


	public void runCommands() {
		double timestamp = Timer.getFPGATimestamp();

		boolean dashboardShot = debugController.getButtonA();
		//LED Controls
		/*if(operatorController.getPOVUp()){
			mLedStringSubsystem.setRobotLEDModeGlitchy();
		}
		else if (operatorController.getPOVDown()){
			mLedStringSubsystem.setRobotLEDModeOff();
		}*/
		/*
		if (operatorController.getPOVLeft()){
			mLedStringSubsystem.setRobotLEDModeClimbComplete();
		}
		else{
			mLedStringSubsystem.setRobotLEDModeNormal();
		}*/

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

		

		//Operator Controls
		boolean closeShot = operatorController.getButtonB();
		boolean closeFlipShot = operatorController.getButtonY();
		boolean lowGoalShot = operatorController.getButtonA();
		boolean lowGoalFlipShot = operatorController.getButtonX();
		boolean visionShot = operatorController.getRightTriggerAsDigital(); //driver may also be able to activate this

		boolean overwriteBallCountTo0 = operatorController.getPOVDown();
		boolean overwriteBallCountTo1 = operatorController.getPOVRight();
		boolean overwriteBallCountTo2 = operatorController.getPOVUp();
		double  anglerPower = operatorController.getLeftJoyY();

		boolean enableAngler = operatorController.getButtonStart();
		boolean zeroAngler = operatorController.getButtonBack();

		boolean loadConevyor = operatorController.getButtonRB();
		boolean reverseConveyor = operatorController.getButtonLB();



		//define all driver controls
		boolean quickTurn;
		boolean visionLineup;
		boolean shiftHighGear;
		boolean shiftLowGear;

		boolean intakeFront;
		boolean intakeBack;
		boolean feedThroughFromFront;
		boolean feedThroughFromBack;
		boolean confirmShot;

		double throttle;
		double turn;

		// driver controls based on current config
		switch (currentControllerConfig) {
			case kCallOfDuty:{
				// buttons
				quickTurn = driverController.getPOVLeft(); // Will Map to paddle
				visionLineup = driverController.getPOVRight(); // Will map to paddle
				shiftHighGear = driverController.getButtonRB();
				shiftLowGear = driverController.getButtonLB();

				intakeFront = driverController.getRightTriggerAsDigital();
				intakeBack = driverController.getLeftTriggerAsDigital();
				feedThroughFromFront = driverController.getPOVUp();
				feedThroughFromBack = driverController.getPOVDown();
				// face buttons are all unused.

				confirmShot = driverController.getButtonA();
				
				visionShot = visionShot || driverController.getButtonB();
				visionLineup = visionLineup || driverController.getButtonB();

				// decide throttle with triggers

				throttle = driverController.getLeftJoyY();

				turn = driverController.getRightJoyX();


			}
				break;
			case kForza: {
				// Tunable things
				//double MIN_SPEED_TO_CONSIDER_MOVING = 1.0; // inches per second

				// buttons
				quickTurn = driverController.getButtonA();
				visionLineup = driverController.getButtonX();
				shiftHighGear = driverController.getButtonY();
				shiftLowGear = driverController.getButtonB();

				intakeFront = driverController.getButtonLB();
				intakeBack = driverController.getButtonRB();
				feedThroughFromFront = driverController.getPOVUp();
				feedThroughFromBack = driverController.getPOVDown();
				// POV Left and right are still unused, as is the right stick, and forward/back
				// buttons.

				confirmShot =driverController.getPOVLeft();


				visionShot = visionShot || driverController.getPOVRight();
				visionLineup = visionLineup || driverController.getPOVRight();

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
				
				

				throttle = 0.0;
				if (leftTriggerTimestamp < rightTriggerTimestamp) {
					throttle = (-driverController.getLeftTrigger()) * (1.0 - driverController.getRightTrigger());
				} else if (rightTriggerTimestamp <= leftTriggerTimestamp) {
					throttle = driverController.getRightTrigger() * (1.0 - driverController.getLeftTrigger());
				}

				turn = driverController.getLeftJoyX();
			}
				break;
			default:
			case kRetroGranTurismo: {
				// buttons
				quickTurn = driverController.getPOVLeft(); // Will Map to paddle
				visionLineup = driverController.getPOVRight(); // Will map to paddle
				shiftHighGear = driverController.getButtonRB();
				shiftLowGear = driverController.getButtonLB();

				intakeFront = driverController.getRightTriggerAsDigital();
				intakeBack = driverController.getLeftTriggerAsDigital();
				feedThroughFromFront = driverController.getPOVUp();
				feedThroughFromBack = driverController.getPOVDown();

				confirmShot = driverController.getButtonA();

				visionShot = visionShot || driverController.getButtonB();
				visionLineup = visionLineup || driverController.getButtonB();

				throttle = driverController.getRightJoyY();

				turn = driverController.getLeftJoyX(); // TODO: Add Vision Lineup code
			}
				break;

		}



		//perform actions based on control values

		// Superstructure stuff
		mSuperStructure.wantConfirmLaunch(confirmShot);

		if(overwriteBallCountTo0){
			mSuperStructure.overwiteBallCount(0);
		}

		if(overwriteBallCountTo1){
			mSuperStructure.overwiteBallCount(1);
		}

		if(overwriteBallCountTo2){
			mSuperStructure.overwiteBallCount(2);
		}

		//TODO: Add vision shot
		if (intakeFront) {
			mSuperStructure.wantIntakeFront();
		} else if (intakeBack) {
			mSuperStructure.wantIntakeBack();
		}else if (feedThroughFromFront){
			mSuperStructure.wantFeedFrontIntake();
		} else if (feedThroughFromBack) {
			mSuperStructure.wantFeedBackIntake();
		} else if (dashboardShot){
			mSuperStructure.wantPrepareShot(Shot.TheShotDashboard.getDashboardShot());
		} else if (closeShot){
			mSuperStructure.wantPrepareShot(Shot.CLOSE_SHOT);
		} else if (lowGoalShot){
			mSuperStructure.wantPrepareShot(Shot.LOW_GOAL);
		} else if (closeFlipShot){
			mSuperStructure.wantPrepareShot(Shot.FLIPPED_CLOSE_SHOT);
		} else if (lowGoalFlipShot){
			mSuperStructure.wantPrepareShot(Shot.LOW_GOAL_FLIPPED);
		}else {
			mSuperStructure.stop();
		}

		if(loadConevyor){
			mSuperStructure.wantInnerBallPathIntake();
		}else if (reverseConveyor){
			mSuperStructure.wantInnerBallPathReverse();
		}else{
			mSuperStructure.wantInnerBallPathStop();
		}
		
		
		// Handle shfting
		if (shiftHighGear) {
			mDriveTrainSubsystem.setHighGear(true);
		} else if (shiftLowGear) {
			mDriveTrainSubsystem.setHighGear(false);
		}


		if(visionLineup){
			mDriveTrainSubsystem.setWantVisionTracking(throttle);
		}
		else{
			mDriveTrainSubsystem.setOpenLoop(mCheesyDriveHelper.cheesyDrive(throttle, turn, quickTurn, mDriveTrainSubsystem.isHighGear()));
		}

		if(driverController.getButtonStart()){
			mDriveTrainSubsystem.setGyroAngle(Rotation2d.fromDegrees(0.0));
		}


		mLaunchBoxAngler.runManualMode(anglerPower);

		if(enableAngler){
			mLaunchBoxAngler.reEnable();
		}
		if(zeroAngler){
			mLaunchBoxAngler.resetEncoder();
		}
		
		return;
	}

	public void updateConfigs() {
		driverController.updateConfig();
		operatorController.updateConfig();
		debugController.updateConfig();
		if (controllerConfigChooser.getSelected() != null) {
			mCheesyDriveHelper = controllerConfigChooser.getSelected().getCheesyDriveHelper();
		} else {
			mCheesyDriveHelper = DriverConrollerConfigs.kRetroGranTurismo.getCheesyDriveHelper();
		}
	}
}