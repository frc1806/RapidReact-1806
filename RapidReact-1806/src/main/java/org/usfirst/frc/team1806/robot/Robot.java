package org.usfirst.frc.team1806.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import org.usfirst.frc.team1806.robot.auto.*;
import org.usfirst.frc.team1806.robot.auto.modes.VisionMode;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeBase;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeExecuter;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeSelector;
import org.usfirst.frc.team1806.robot.loop.Looper;
import org.usfirst.frc.team1806.robot.path.motion.RobotStateEstimator;
import org.usfirst.frc.team1806.robot.subsystems.*;
import org.usfirst.frc.team1806.robot.util.CrashTracker;
import org.usfirst.frc.team1806.robot.util.DriveSignal;
import org.usfirst.frc.team1806.robot.util.RigidTransform2d;

import java.util.Arrays;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    private DriveTrainSubsystem mDrive = DriveTrainSubsystem.getInstance();
    private RobotState mRobotState = RobotState.getInstance();
    private AutoModeExecuter mAutoModeExecuter = null;
    private AutoModeExecuter mAutomatedSequenceExecuter = null;
    public static OI m_oi;
    public static PowerDistribution powerDistributionPanel;
    private String selectedModeName;
    private String lastSelectedModeName;
    private boolean bAutoModeStale = false;
    SendableChooser<edu.wpi.first.wpilibj2.command.Command> m_chooser = new SendableChooser<>();


    private static final SubsystemManager S_SubsystemManager = new SubsystemManager(
            Arrays.asList(DriveTrainSubsystem.getInstance(), SuperStructure.getInstance(), VisionSubsystem.getInstance(), LEDStringSubsystem.getInstance(), LaunchBoxAngler.getInstance())); ///TODO RE ADD IN SUBSYSTEMS

    private Looper mEnabledLooper = new Looper();
    private Looper mDisabledLooper = new Looper();

    public static SequenceState mSequenceState = SequenceState.VISION;
    public static AutoModeBase selectedAuto;
    public static boolean isBlue;
    public boolean arePathsInit = false;
    public static boolean needToPositionControlInTele = false;
    private boolean automatedSequenceEnabled = true;
    private boolean sequenceStarting = false;
    private boolean sequenceEnding = false;
    private boolean wasAutomatedSequenceEnabled = false;
    public static AutoModeBase currentSequence;

    //Global Dashboard tabs
    private static ShuffleboardTab competitionTab;

    public static ShuffleboardTab getMainDriverTab()
    {
      return competitionTab;
    }
    
    public enum AutoInTeleOp{
      AUTO_DISABLED,
      AUTO_INIT,
      AUTO_PERIODIC,
        CONCURRENT_WITH_DC;
    }
    public AutoInTeleOp autoInteleOpState = AutoInTeleOp.AUTO_DISABLED;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        selectedModeName = "";
        lastSelectedModeName = "";
        competitionTab = Shuffleboard.getTab("Main Competition Tab");
      m_oi = new OI();
      zeroAllSensors();
      //mDrive.setDebug(true);
      //adds in the iterative code to make the code run
      mEnabledLooper.register(RobotStateEstimator.getInstance());
      S_SubsystemManager.registerEnabledLoops(mEnabledLooper);

      powerDistributionPanel = new PowerDistribution();
      SmartDashboard.putData("Auto mode", m_chooser);
      mAutoModeExecuter = null;
      mAutomatedSequenceExecuter = null;
      mAutoModeExecuter = new AutoModeExecuter();
      mAutomatedSequenceExecuter = new AutoModeExecuter();
      //mAutoModeExecuter.setAutoMode(new QualMode()); TODO
      mDrive.setCoastMode();
      AutoModeSelector.registerDisabledLoop(mDisabledLooper);
      AutoModeSelector.initAutoModeSelector();
      needToPositionControlInTele = false;
      try {
        Thread.sleep(3000);
      } catch (InterruptedException e){
        System.out.println(e);
      }
      S_SubsystemManager.setUpDriverTab();
    }

    public enum SequenceState{
        VISION(new VisionMode());

        public AutoModeBase getAutoMode() {
            return autoMode;
        }
        public boolean isActive() {
            return bIsActive;
        }
        AutoModeBase autoMode;

        public void setbIsActive(boolean bIsActive) {
            this.bIsActive = bIsActive;
        }

        boolean bIsActive = false;
        SequenceState(AutoModeBase _autoMode) {
            autoMode = _autoMode;
        }
    }
    @Override
    public void disabledInit() {
      mDrive.setCoastMode();
      mEnabledLooper.stop();
        if(mAutoModeExecuter != null) {
            mAutoModeExecuter.stop();
            selectedAuto = AutoModeSelector.getSelectedAutoMode();
        }
        if(mAutomatedSequenceExecuter != null) {
            mAutomatedSequenceExecuter.stop();
        }
        autoInteleOpState = AutoInTeleOp.AUTO_DISABLED;
        mDisabledLooper.stop();
    }

    @Override
    public void disabledPeriodic() {
      if(DriverStation.isDSAttached() ){
        if(DriverStation.getAlliance() == DriverStation.Alliance.Blue){
          isBlue = true;
        } else if(DriverStation.getAlliance() == DriverStation.Alliance.Red){
          isBlue = false;
        }
        m_oi.updateConfigs();
      }

      allPeriodic();
      selectedModeName = SmartDashboard.getString(
                AutoModeSelector.SELECTED_AUTO_MODE_DASHBOARD_KEY,
                "org.usfirst.frc.team1806.robot.auto.modes.NothingAuto");
      if(!selectedModeName.equals(lastSelectedModeName) || bAutoModeStale){
          bAutoModeStale = false;
          selectedAuto = AutoModeSelector.getSelectedAutoMode();
      }

      autoInteleOpState = AutoInTeleOp.AUTO_DISABLED;
      S_SubsystemManager.stop();
      S_SubsystemManager.outputToSmartDashboard();

      lastSelectedModeName = selectedModeName;

    }


    @Override
    public void autonomousInit() {
      mDisabledLooper.stop();
      bAutoModeStale = true;
      try {
			zeroAllSensors();
			CrashTracker.logAutoInit();
            System.out.println("Auto star t timestamp: " + Timer.getFPGATimestamp());
            if (mAutoModeExecuter != null) {
                mAutoModeExecuter.stop();
            }
            mDrive.setHighGear(true);
        needToPositionControlInTele = false;
        mDrive.setBrakeMode();
        mEnabledLooper.start();
        mAutoModeExecuter.setAutoMode(selectedAuto);
        mAutoModeExecuter.start();
      } catch (Throwable t) {
        CrashTracker.logThrowableCrash(t);
        throw t;
      }

    }

    /**
     * This function is called periodically during autonomous.
     */
    boolean isAutoInterrupted = false;
    @Override
    public void autonomousPeriodic() {
      allPeriodic();
      CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
      mDisabledLooper.stop();
      mEnabledLooper.start();
        if(mAutoModeExecuter != null) {
            mAutoModeExecuter.stop();
            mAutoModeExecuter = null;
            mAutoModeExecuter = new AutoModeExecuter();
        }
        if(mAutomatedSequenceExecuter != null) {
            mAutomatedSequenceExecuter.stop();
            mAutomatedSequenceExecuter = null;
            mAutomatedSequenceExecuter = new AutoModeExecuter();
        }
      mDrive.setOpenLoop(DriveSignal.NEUTRAL);
      mDrive.setNeutralMode(false);
      //SquidSubsystem.getInstance().extendSquid();
      autoInteleOpState = AutoInTeleOp.AUTO_DISABLED;
    }


    @Override
    public void teleopPeriodic() {
      if(Constants.enableAutoInTeleOp){
        switch(autoInteleOpState){
          case AUTO_DISABLED:
            if(false){//m_oi.autoInTeleOpOn()){
              autoInteleOpState = AutoInTeleOp.AUTO_INIT;
            } else {
              runTeleOp();
            }
            break;
          case AUTO_INIT:
            selectedAuto = AutoModeSelector.getSelectedAutoMode();
            if(false){//m_oi.autoInTeleOpOn()){
              zeroAllSensors();
              if (mAutoModeExecuter != null) {
                mAutoModeExecuter.stop();

              }
              autonomousInit();
              //m_oi.autoRunCommands();
              autoInteleOpState = AutoInTeleOp.AUTO_PERIODIC;
            } else {
              autoInteleOpState = AutoInTeleOp.AUTO_DISABLED;
              teleopInit();
            }
            break;

          case AUTO_PERIODIC:
            if(false){//m_oi.autoInTeleOpOn()){
              autonomousPeriodic();
              //m_oi.autoRunCommands();
            } else {
              autoInteleOpState = AutoInTeleOp.AUTO_DISABLED;
              teleopInit();
            }
            break;
          default:
            runTeleOp();
            break;
        }

      }
      else {
          //update boolean trackers
          //automatedSequenceEnabled = m_oi.getAutomatedSequenceButton();
          sequenceStarting = automatedSequenceEnabled && !wasAutomatedSequenceEnabled;
          sequenceEnding = !automatedSequenceEnabled && wasAutomatedSequenceEnabled;

          if(automatedSequenceEnabled) {
              if(sequenceStarting) {
                  //mSequenceState = m_oi.getAutomatedSequenceMode();
                  mSequenceState.setbIsActive(true);
                  mAutoModeExecuter.setAutoMode(mSequenceState.getAutoMode());
                  mAutoModeExecuter.start();

              }
          }
          else{
              if(sequenceEnding) {
                  mAutoModeExecuter.stop();
                  mSequenceState.setbIsActive(false);
              }

          }
          runTeleOp();
          wasAutomatedSequenceEnabled = automatedSequenceEnabled;
      }
    }

    @Override
    public void testPeriodic() {
    }


    public void zeroAllSensors() {
//		System.out.println("Zeroing all Sensors..");
      S_SubsystemManager.zeroSensors();
      mRobotState.reset(Timer.getFPGATimestamp(), new RigidTransform2d());
//        System.out.print("All Sensors zeroed!");

}
    public synchronized void allPeriodic() {
      S_SubsystemManager.outputToSmartDashboard();
      mRobotState.outputToSmartDashboard();
      mEnabledLooper.outputToSmartDashboard();
      SmartDashboard.putString("Auto We Are Running", AutoModeSelector.returnNameOfSelectedAuto()==null?"Nothing":AutoModeSelector.returnNameOfSelectedAuto());
     //SmartDashboard.putNumber("PDP Total", powerDistributionPanel.getTotalCurrent());
    }
    private void runTeleOp(){
      CommandScheduler.getInstance().run();
      m_oi.runCommands();
      allPeriodic();
    }
  }
