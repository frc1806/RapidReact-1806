package org.usfirst.frc.team1806.robot;

import org.usfirst.frc.team1806.robot.util.Translation2d;
import org.usfirst.frc.team1806.robot.util.XboxControllerConfigValues;
import org.usfirst.frc.team1806.robot.util.PicoColorSensor.RawColor;

import edu.wpi.first.math.util.Units;

public class Constants {

        public final static boolean kIsCompBot = true;

    public static final XboxControllerConfigValues kDriverControllerDefaultConfig = XboxControllerConfigValues.Builder
            .create().withLeftXDeadzone(.12).withLeftXMinimumOutput(.04).withLeftXLinearity(.32).withLeftYDeadzone(.12)
            .withLeftYMinimumOutput(.04).withLeftYLinearity(.32).withRightXDeadzone(.12).withRightXMinimumOutput(.04)
            .withRightXLinearity(.32).withRightYDeadzone(.12).withRightYMinimumOutput(.04).withRightYLinearity(.32)
            .withTriggerDeadzone(.2).withTriggerMinimumOutput(.04).withTriggerLinearity(.32)
            .withTriggerAsDigitalDeadzone(.5).build();

    public static final XboxControllerConfigValues kOperatorControllerDefaultConfig = XboxControllerConfigValues.Builder
            .create().withLeftXDeadzone(.12).withLeftXMinimumOutput(.04).withLeftXLinearity(.32).withLeftYDeadzone(.12)
            .withLeftYMinimumOutput(.04).withLeftYLinearity(.32).withRightXDeadzone(.12).withRightXMinimumOutput(.04)
            .withRightXLinearity(.32).withRightYDeadzone(.12).withRightYMinimumOutput(.04).withRightYLinearity(.32)
            .withTriggerDeadzone(.2).withTriggerMinimumOutput(.04).withTriggerLinearity(.32)
            .withTriggerAsDigitalDeadzone(.5).build();

    public final static int kCoprocessorPort = 8806;

    public final static boolean enableAutoInTeleOp = false;
    public final static boolean enableDebugMode = true;
    public final static double kLooperDt = 0.005;
    public final static double kDriveWheelDiameterInches = 4;
    public final static double kTrackWidthInches = 27.5;
    public final static double kTrackScrubFactor = .978;

    public final static double kVisionExpectedCameraLag = 0.085;

    public final static double kStallTimeout = 2;
    public final static double kStallWaitPeriod = .3;
    public final static double kStallSpeed = 500;
    public final static double kStallPower = 1.8;


    public final static int kDriveTrainPIDSetTimeout = 30;
    public final static double kCountsPerInch = 168.359374;
    public final static double kDriveInchesPerCount = 1/kCountsPerInch;
    public final static int kDIODriveLeftEncoderA = 0;
    public final static int kDIODriveLeftEncoderB = 1;
    public final static int kDIODriveRightEncoderA = 2;
    public final static int kDIODriveRightEncoderB = 3;

    ///Motion
    public final static double kMinLookAhead = 9; // inches
    public final static double kMinLookAheadSpeed = 9.0; // inches per second
    public final static double kMaxLookAhead = 42; // inches
    public final static double kMaxLookAheadSpeed = 144.0; // inches per second
    public final static double kDeltaLookAhead = kMaxLookAhead - kMinLookAhead;
    public final static double kDeltaLookAheadSpeed = kMaxLookAheadSpeed - kMinLookAheadSpeed;

    public final static double kInertiaSteeringGain = 0.0; // angular velocity command is multiplied by this gain *
    // our speed
    // in inches per sec
    public final static double kSegmentCompletionTolerance = 0.5; // inches
    public final static double kPathFollowingMaxAccel = 144; // inches per second^2
    public final static double kPathFollowingMaxVel = 144; // inches per second
    public final static double kPathFollowingProfileKp = 1.15; //.99
    public final static double kPathFollowingProfileKi = 0.05; //.049
    public final static double kPathFollowingProfileKv = 0.0002; //0.000013
    public final static double kPathFollowingProfileKffv = 1.2; //1.2
    public final static double kPathFollowingProfileKffa = 0.05; //.05
    public final static double kPathFollowingGoalPosTolerance = 0.75;
    public final static double kPathFollowingGoalVelTolerance = 18.0;
    public final static double kPathStopSteeringDistance = 9.0; //2.25

    //
    // PID gains for drive velocity loop (HIGH GEAR)
    // Units: setpoint, error, and output are in inches per second
    public final static double kDriveHighGearVelocityKp =.0319767; //.23109; //.0004;//.08;//.16; //1.01; ;
    public final static double kDriveHighGearVelocityKi = 0.0;
    public final static double kDriveHighGearVelocityKd =  0.022; //.6125; //1.25; //7.8; //0.0001; //6.0/1500;
    public final static double kDriveHighGearVelocityKf = 0.00664;//.0175; //.035; //0.21; //.025;
    public static final double kDriveHighGearVelocityKs = 0.48391;
    public static final double kDriveHighGearVelcoityKv = 0.14387;
    public static final double kDriveHighGearVelocityKa = 0.26916;
    public final static int kDriveHighGearVelocityIZone = 0;
    public final static double kDriveHighGearVelocityRampRate = .1;
    public final static double kDriveHighGearNominalOutput = 0.25;
    public final static double kDriveHighGearMaxSetpoint = 12 * 13; //FPS

    // PID gains for drive velocity loop (HIGH GEAR)
    // Units: setpoint, error, and output are in counts per tenth of a second
    public final static double kDriveHighGearVelocityLowKp = .0004; // 1.2/1500;
    public final static double kDriveHighGearVelocityLowKi = 0.0000000; //0.0;
    public final static double kDriveHighGearVelocityLowKd = 0; //0.0001; //6.0/1500;
    //public final static double kDriveHighGearVelocityLowKf = 0.00000254;//0; //.025;
    public static final double kDriveHighGearVelocityLowKs = 0.0;
    public static final double kDriveHighGearVelcoityLowKv = 0.0;
    public static final double kDriveHighGearVelocityLowKa = 0.0;
    public final static int kDriveHighGearVelocityLowIZone = 0;
    public final static double kDriveHighGearVelocityLowRampRate = .1;
    public final static double kDriveHighGearLowNominalOutput = 0.25;
    public final static double kDriveHighGearLowMaxSetpoint = 10.5 * 12; //FPS

    // PID gains for drive velocity loop ***This should be high gear lollz sorry yall
    // This is typically used in the most in the TurnToPoint action
    // Units: setpoint, error, and output are in counts
    public final static double kDriveLowGearPositionKp = .1; //.008
    public final static double kDriveLowGearPositionKi = .00044;
    public final static double kDriveLowGearPositionKd = 0.1; //0.0035
    public final static double kDriveLowGearPositionKf = 0.000;
    public final static double kDriveLowGearPositionIZone = 1.5;
    public final static int kDriveLowGearMaxVelocity = 700; // Counts
    public final static int kDriveLowGearMaxAccel = 20; // Counts
    public final static double kDriveTurnMaxPower = 8;

    // PID gains for drive velocity loop ***This should be high gear lollz sorry yall
    // Units: setpoint, error, and output are in counts
    public final static double kDriveVisionTurnKp = 0;
    public final static double kDriveVisionTurnKi = 0;
    public final static double kDriveVisionTurnKd = .0;
    public final static double kDriveVisionTurnKf = 0.0;
    public final static int kDriveVisionTurnIZone = 4;


    public final static double kElevatorHoldPercentOutput = .045;
    // Encoder constants used by Elevator system
    public final static int kCreepModeLiftHeight = 13000;

    public final static double kElevatorInchesPerCountDefault = 1;
    public final static int kLiftPositionControlPIDSlot = 10;
    public final static int kLiftPositionPIDTimeout = 10;
    public final static double kElevatorPositionkP = 0.09;    
    public final static double kElevatorPositionkI =0.075; ///0.02;
    public final static double kElevatorPositionkD =0.0; ///0.015;
    public final static double kElevatorPositionkF =0.0; //1 / 2000;
    public final static int kElevatorPositionIZone = 25;// 800;
    public final static double kLiftPositionRampRate = 0;
    public final static int kBottomLimitTolerance = 50;
    public final static double kLiftBottomPivotHeight = 35.0;
    public final static double kLiftSlowDownHeight = 37.0;

    public final static Double kElevatorPositionTolerance = 0.25;
    public final static int kElevatorVelocityTolerance = 500;

    public final static double elevatorResetSpeed = .2;


    public final static double kElevatorHoldkPGain = .04;
    public final static double kBumpEncoderPosition = .5;

    public final static double kLiftTopLimitSwitchPosition = 12.0;

    //Compressor Control Constants
    public final static int kPressureSensorSamplingLoops = 60; //each loop is 1/200 of a second
    public final static double kPressureAverageMinimumToStart = 90; //PSI

    public final static int kBatteryVoltageSamplingLoops = 60; //each loop is 1/200 of a second
    public final static double kBatteryVoltageCompressorShutoffThreshold = 11.0; //volts
    public final static double kBatteryVoltageAbsoluteCompressorShutoffThreshold = 8.0; //volts

    public final static int kRobotDemandAmpsSamplingLoops = 60; // each loop is 1/200 of a second
    public final static double kAverageAmpDemandToShutOffCompressor = 200; //Amps
    public final static double kAbsoluteRobotCompressorShutOffAmps = 300; //Amps

    public final static double kExpectedCompressorMaxCurrentDraw = 40;
    public final static double kExpectedCompressorVoltageDrop = 0.3;


    //Intake Constants
    public final static double kInnerIntakingSpeed = .8;
    public final static double kOuterIntakingSpeed = .8;
    public final static double kIntakeSpeed = 1.0;
    public final static double kSweep = 1.0;

    //Battery State Of Charge
    public final static double kFullChargeBatteryCoulombCount = 25200;
    public final static double kBatteryFullChargeVoltage = 12.7;
    public final static double kBatteryDepletedVoltage = 11.8;
    public final static double kLowAmpLoad = 10;

    //Lift interference avoidence
    public final static double kLiftWaitForExtendIntake = 0.25;
    public final static int kMaxElevatorHeightToNeedToExtendIntake = 1000;
    public final static int kSafeElevatorHeightOffsetToNotHitIntake = 500; // total of this and the line above will be the setpoint

    //controls
    public final static double kTriggerThreshold = .2;

    //Output to Smart Dashboard Keys
    public final static String kCompressorKey = "Compressor ";
    public final static String kDriveTrainKey = "DriveTrain ";
    public final static String kIntakeKey = "Intake ";
    public final static String kLiftKey = "Elevator ";
    public final static String kRobotStateKey = "robot";
    

    //Left Flywheel
    public final static Double kLeftFlywheelKp = 0.0;
    public final static Double kLeftFlywheelKi = 0.0;
    public final static Double kLeftFlywheelKd = 0.0;
    public final static double kLeftFlywheelKf = 0.0;
    public final static Double kLeftFlywheelIzone = 0.0;
    public final static Double kLeftFlywheelConversionFactor = 0.0;
    //Right Flywheel
    public final static Double kRightFlywheelKp = 0.0;
    public final static Double kRightFlywheelKi = 0.0;
    public final static double kRightFlywheelKd = 0.0;
    public final static double kRightFlywheelKf = 0.0;
    public final static Double kRightFlywheelIzone = 0.0;
    public final static Double kRightFlywheelConversionFactor = 0.0;

    //Conveyor
    public final static Double kConveyorIntakingSpeed = 1.0;
    //public final static Double kLaunchingSpeed = NOT KNOWN YET

    
    public static Double kRPMToCounts = 1.0;

public static Double kTopFlywheelKp =0.0001;

public static Double kTopFlywheelKi =0.0;

public static Double kTopFlywheelKd =0.0;

public static Double kTopFlywheelKf =12.0/1800;

public static Double kTopFlywheelIzone =0.0;


public static Double kTopFlywheelKs =0.0;//0.16303;

public static Double kTopFlywheelKv =0.0; //1.563;
public static Double kTopFlywheelKa =0.0; //0.86814;

//Launchbox Angler PID
public static Double kLaunchBoxAnglerKp = 0.45/10;
public static Double kLaunchBoxAnglerKi = 0.0001;
public static Double kLaunchBoxAnglerKd = 0.0;
public static Double kLaunchBoxInchesToFreedom = 2.5;


//VISION
public static Double kCameraHeightMeters = Units.inchesToMeters(30);
public static Double kTargetHeightMeters = Units.inchesToMeters(72);
public static Double kCameraPitchRadians = Units.degreesToRadians(20);

//COLOR SENSOR
public static RawColor kBlueBallMaxValues = new RawColor(100, 100, 255, 255);
public static RawColor kBlueBallMinimumValues = new RawColor(0, 0, 150, 0);
public static RawColor kRedBallMaxValues = new RawColor(255, 100, 100, 255);
public static RawColor kRedBallMinimumValues = new RawColor(150, 0, 0, 0);


//Vision lineup
public static Double visionLineUpProportional = 1.0/20.0;

}
