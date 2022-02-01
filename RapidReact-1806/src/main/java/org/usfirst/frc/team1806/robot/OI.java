/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1806.robot;

import org.usfirst.frc.team1806.robot.subsystems.*;
import org.usfirst.frc.team1806.robot.util.CheesyDriveHelper;
import org.usfirst.frc.team1806.robot.util.Latch;
import org.usfirst.frc.team1806.robot.util.XboxController;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	//snag some subsystem instances
	private DriveTrainSubsystem mDriveTrainSubsystem = DriveTrainSubsystem.getInstance();
	private ElevatorSubsystem mElevatorSubsystem = ElevatorSubsystem.getInstance();

	//initialise controllers & ish
	private CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();
	private XboxController driverController = new XboxController(0, "Driver", Constants.kDriverControllerDefaultConfig);
	private XboxController operatorController = new XboxController(1, "Operator", Constants.kOperatorControllerDefaultConfig);
	private XboxController debugController = new XboxController(2, "Debug", null);
	//start up button trackers
	private Latch autoInTeleOp = new Latch();
	private Boolean wasSquidExtendButton = false;
	private Boolean wasSquidOpenButton = false;
	private Boolean wasChangeModeButton = false;
	private Boolean wasOuterIntakeButton = false;
	private Boolean wasShift = false;
	private Boolean wasParkingBrake = false;

	public void runCommands(){
		return;
	}

	public void updateConfigs(){
		driverController.updateConfig();
		operatorController.updateConfig();
		debugController.updateConfig();
	}
}