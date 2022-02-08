
package org.usfirst.frc.team1806.robot;

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
	public static int leftDriveEncoder = 0;
	public static int leftDriveEncoderB = 1;
	public static int rightDriveEncoder = 2;
	public static int rightDriveEncoderB = 3;
	
	//Elevator
	public static int elevatorLeader = 10;
	public static int elevatorFollower = 11;

	//Intake CAN Ports
	public static int frontIntake = 20;

	public static int rearIntake = 22;

	//Conveyor
	public static int roller1 = 30;
	public static int roller2 = 31;
	public static int lowerConveyor = 32;

	//Flywheel
	public static int upFlywheel = 40;
	public static int downFlywheel = 42;


	////////// These are all of the solenoids for the robotri
	public static int module1Number = 0;
	public static int module2Number = 0;

	//Intake
	public static int frontIntakeExtend = 0;//module 1
	public static int frontIntakeRetract = 1;//module 1
	public static int backIntakeExtend = 2;//module 1
	public static int backIntakeRetract = 3;//module 1

	//shifter
	public static int shiftHigh = 4;
	public static int shiftLow = 5;

	///// DIOs

	public static int liftBottomLimit = 0;
	public static int liftHighLimit = 1;

	//HABClimber


	///////////// PDP PORTS



	//CANifier IDs
	public static int UpFlywheelCANifier = 1;
	public static int DownFlywheelCANifier = 2;


}
