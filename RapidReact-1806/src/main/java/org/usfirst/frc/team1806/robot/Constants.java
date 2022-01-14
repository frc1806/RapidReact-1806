package org.usfirst.frc.team1806.robot;

import org.usfirst.frc.team1806.robot.util.Translation2d;

public class Constants {

    public final static int kCoprocessorPort = 8806;

    public final static boolean enableAutoInTeleOp = false;
    public final static boolean enableDebugMode = true;
    public final static double kLooperDt = 0.005;
    public final static double kDriveWheelDiameterInches = 4;
    public final static double kTrackWidthInches = 14;
    public final static double kTrackScrubFactor = .920;

    public final static double kVisionExpectedCameraLag = 0.085;

    public final static double kStallTimeout = 2;
    public final static double kStallWaitPeriod = .3;
    public final static double kStallSpeed = 500;
    public final static double kStallPower = .15;


    public final static int kDriveTrainPIDSetTimeout = 30;
    public final static double kCountsPerInch = 0.575;
    public final static double kDriveInchesPerRevolution = 1/kCountsPerInch;

    ///Motion
    public final static double kMinLookAhead = 4; // inches
    public final static double kMinLookAheadSpeed = 3.0; // inches per second
    public final static double kMaxLookAhead = 42; // inches
    public final static double kMaxLookAheadSpeed = 144.0; // inches per second
    public final static double kDeltaLookAhead = kMaxLookAhead - kMinLookAhead;
    public final static double kDeltaLookAheadSpeed = kMaxLookAheadSpeed - kMinLookAheadSpeed;

    public final static double kInertiaSteeringGain = 0.0; // angular velocity command is multiplied by this gain *
    // our speed
    // in inches per sec
    public final static double kSegmentCompletionTolerance = 0.5; // inches
    public final static double kPathFollowingMaxAccel = 55; // inches per second^2
    public final static double kPathFollowingMaxVel = 132; // inches per second
    public final static double kPathFollowingProfileKp = 1.1; //.99
    public final static double kPathFollowingProfileKi = 0.049; //.049
    public final static double kPathFollowingProfileKv = 0.000013; //0.000013
    public final static double kPathFollowingProfileKffv = 1.2; //1.2
    public final static double kPathFollowingProfileKffa = 0.05; //.05
    public final static double kPathFollowingGoalPosTolerance = 0.3;
    public final static double kPathFollowingGoalVelTolerance = 18.0;
    public final static double kPathStopSteeringDistance = 6; //2.25

    //
    // PID gains for drive velocity loop (HIGH GEAR)
    // Units: setpoint, error, and output are in counts per tenth of a second
    public final static double kDriveHighGearVelocityKp = .001; //.0004;//.08;//.16; //1.01;
    public final static double kDriveHighGearVelocityKi = 0.00000001;
    public final static double kDriveHighGearVelocityKd =  0.002; //.6125; //1.25; //7.8; //0.0001; //6.0/1500;
    public final static double kDriveHighGearVelocityKf = 0.00004;//.0175; //.035; //0.21; //.025;
    public final static int kDriveHighGearVelocityIZone = 0;
    public final static double kDriveHighGearVelocityRampRate = .1;
    public final static double kDriveHighGearNominalOutput = 0.25;
    public final static double kDriveHighGearMaxSetpoint = 12 * 13; //FPS

    // PID gains for drive velocity loop (HIGH GEAR)
    // Units: setpoint, error, and output are in counts per tenth of a second
    public final static double kDriveHighGearVelocityLowKp = .0004; // 1.2/1500;
    public final static double kDriveHighGearVelocityLowKi = 0.0000000; //0.0;
    public final static double kDriveHighGearVelocityLowKd = 0; //0.0001; //6.0/1500;
    public final static double kDriveHighGearVelocityLowKf = 0.00000254;//0; //.025;
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
    public final static double kDriveTurnMaxPower = .6;

    // PID gains for drive velocity loop ***This should be high gear lollz sorry yall
    // Units: setpoint, error, and output are in counts
    public final static double kDriveVisionTurnKp = 0;
    public final static double kDriveVisionTurnKi = 0;
    public final static double kDriveVisionTurnKd = .0;
    public final static double kDriveVisionTurnKf = 0.0;
    public final static int kDriveVisionTurnIZone = 4;


    public final static double kLiftHoldPercentOutput = .025;
    // Encoder constants used by Rocket Elevator system
    public final static int kCreepModeLiftHeight = 13000;

    public final static int kLiftPositionControlPIDSlot = 10;
    public final static int kLiftPositionPIDTimeout = 10;
    public final static double kLiftPositionkP = .0016;
    public final static double kLiftPositionkI =0.0004; ///0.02;
    public final static double kLiftPositionkD =0.00015; ///0.015;
    public final static double kLiftPositionkF =1 / 60000; //1 / 2000;
    public final static int kLiftPositionIZone = 25;// 800;
    public final static double kLiftPositionRampRate = 0;
    public final static int kBottomLimitTolerance = 50;

    public final static int kLiftPositionTolerance = 100;
    public final static int kLiftVelocityTolerance = 500;

    public final static double liftSpeed = .2;


    public final static double kLiftHoldkPGain = .00005;
    public final static int kBumpEncoderPosition = 1000;

    public final static int kLiftTopLimitSwitchPosition = 500;

    public final static int kTeleOpHoldHeight = 1100;

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

    //Battery State Of Charge
    public final static double kFullChargeBatteryCoulombCount = 25200;
    public final static double kBatteryFullChargeVoltage = 12.7;
    public final static double kBatteryDepletedVoltage = 11.8;
    public final static double kLowAmpLoad = 10;

    //Accelerometer Constants
    public final static double habDropAccelerationThreshold = 1.0; //g force

    //Lift interference avoidence
    public final static double kLiftWaitForExtendIntake = 0.25;
    public final static int kMaxLiftHeightToNeedToExtendIntake = 1000;
    public final static int kSafeLiftHeightOffsetToNotHitIntake = 500; // total of this and the line above will be the setpoint

    //controls
    public final static double kTriggerThreshold = .2;


    //HAB Climb Constnats
    public final static double kHABClimbVelocityKp = 0.005;
    public final static double kHABClimbVelocityKi = 0.0;
    public final static double kHABClimbVelocityKd = 0.0;
    public final static double kHABClimbVelocityKf = 0.0;
    public final static double kHABClimbVelocityKiZone = 1;

    public final static double kHABClimbSyncKp = 1;
    public final static double kHABClimbSyncThrottleLetoffDistance = 0.000;
    public final static double kHABClimbSyncVelocityTolerance = 500;
    public final static double kHABClimbSyncPositionTolerance = 0.5;

    public final static double kHabClimbTargetSpeed = 2000;

    public final static double kLickKp = 0.04;
    public final static double kLickKi = 0.00;
    public final static double kLickKd = 0.00;
    public final static double kLickKf = 0.00;
    public final static int kLickKiZone = 10;

    public final static int kLickPositionTolerance = 20;
    public final static int kLickVelocityTolerance = 2000;

    //Output to Smart Dashboard Keys
    public final static String kCompressorKey = "Compressor ";
    public final static String kDriveTrainKey = "DriveTrain ";
    public final static String kIntakeKey = "Intake ";
    public final static String kLiftKey = "Lift ";
    public final static String kSquidKey = "Squid ";
    public final static String kRobotStateKey = "robot";

}
