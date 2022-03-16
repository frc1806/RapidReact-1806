
package org.usfirst.frc.team1806.robot;

import edu.wpi.first.wpilibj.DutyCycle;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	

    ///////////// Analog Inputs
	public static int mStringPotentiometer = 0;
    ///////////// CAN ports 
	
	//Drive train CAN

	public static int leftLeader = 1;
	public static int leftFollowerA = 2;
	public static int leftFollowerB = 3;
	public static int rightLeader = 4;
	public static int rightFollowerA = 5;
	public static int rightFollowerB = 6;
	
	//drive encoder dios
	public static int leftDriveEncoder = 0;
	public static int leftDriveEncoderB = 1;
	public static int rightDriveEncoder = 2;
	public static int rightDriveEncoderB = 3;
	
	//Elevator
	public static int elevatorLeader = 10;
	public static int elevatorFollower = 11;

	public static int launchBoxAngler = 15;
	public static int launchBoxAngleQuadA = 22;
	public static int launchBoxAngleQuadB = 23;
	public static final int launchBoxAngleEncoder = 4;
	//Intake CAN Ports
	public static int frontIntake = 20;

	public static int rearIntake = 22;

	//Conveyor
	public static int frontRoller = 30;
	public static int rearRoller = 31;
	public static int lowerConveyor = 32;

	//Flywheel
	public static int upFlywheel = 42;
	public static int downFlywheel = 40;

	public static int upFlyWheelEncoderA = 5;
	public static int upFlywheelEncoderB = 6;
	public static int downFlywheelEncoderA = 7;
	public static int downFlywheelEncoderB = 8;





	////////// These are all of the solenoids for the robotri
	public static int module1Number = 0;

	//Intake
	public static int frontIntakeExtend = 0;//module 1
	public static int frontIntakeRetract = 1;//module 1
	public static int backIntakeExtend = 2;//module 1
	public static int backIntakeRetract = 3;//module 1

	//shifter6
	public static int shiftHigh = 4;
	public static int shiftLow = 5;




}
