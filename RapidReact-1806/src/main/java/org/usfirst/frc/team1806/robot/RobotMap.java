
package org.usfirst.frc.team1806.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    ///////////// CAN ports 
	
	//Drive train CAN

	public static int masterRight = 1;
	public static int rightA = 2;
	public static int masterLeft = 4;
	public static int leftA = 5;
	public static int leftB = 15;
	public static int rightB = 16;
	
	//Cube Elevator CANs

	public static int liftLead = 6;
	public static int liftFollow = 7;


	//Intake CAN Ports
	
	public static int leftInnerIntake = 11;
	public static int rightInnerIntake = 12;
	public static int leftOuterIntake = 13;
	public static int rightOuterIntake = 14;


	//Hatch Floor Pickup
	public static int tongueMotor = 42;




	////////// These are all of the solenoids for the robotri
	public static int module1Number = 0;
	public static int module2Number = 0;
	
	//Shifting
	public static int shiftHigh = 0; //module 2
	public static int shiftLow = 1;//module 2

	//Intake
	public static int barIntakeExtend = 5;//module 1
	public static int barIntakeRetract = 4;//module 1
	public static int outerIntakeExtend = 6;//module 1
	public static int outerIntakeRetract = 7;//module 1

	//Squid

	public static int squidOpenPort = 1;//module 1
	public static int squidClosePort = 0; //module 1
	public static int squidExtendForward = 3;//module 1
	public static int squidExtendBackward = 2;//module 1

	//BeaverTail

	//public static int beaverTailEjectExtend = 6;
	//public static int beaverTailEjectRetract = 7;

	//LiftStand
	//public static int liftStandUp = 6;
	//public static int liftLeanBack = 7;




	///// DIOs

	public static int liftBottomLimit = 0;
	public static int liftHighLimit = 1;
	public static int rearLeftTrigger = 4;
	public static int rearLeftResponse = 5;
	public static int rearRightTrigger = 8;
	public static int rearRightResponse = 9;
	public static int leftTrigger = 10;
	public static int leftResponse = 11;
	public static int rightTrigger = 12;
	public static int rightResponse = 2;
	public static int hatchDetector = 3;

	//HABClimber


	///////////// PDP PORTS



}
