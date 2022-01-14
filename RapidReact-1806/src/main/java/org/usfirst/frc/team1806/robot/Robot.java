package org.usfirst.frc.team1806.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team1806.robot.Vision.VisionServer;
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
    private CargoIntakeSubsystem mCargoIntakeSubsystem = CargoIntakeSubsystem.getInstance();
    private RobotState mRobotState = RobotState.getInstance();
    private VisionServer mVisionServer = VisionServer.getInstance();
    private AutoModeExecuter mAutoModeExecuter = null;
    private AutoModeExecuter mAutomatedSequenceExecuter = null;
    public static OI m_oi;
    public static PowerDistributionPanel powerDistributionPanel;
    private String selectedModeName;
    private String lastSelectedModeName;
    private boolean bAutoModeStale = false;
    SendableChooser<Command> m_chooser = new SendableChooser<>();


    private static final SubsystemManager S_SubsystemManager = new SubsystemManager(
            Arrays.asList(DriveTrainSubsystem.getInstance(), LiftSubsystem.getInstance(), CargoIntakeSubsystem.getInstance(), CompressorControlSubsystem.getInstance(), SquidSubsystem.getInstance())); ///TODO RE ADD IN SUBSYSTEMS

    private Looper mEnabledLooper = new Looper();

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


    public enum AutoInTeleOp{
      AUTO_DISABLED,
      AUTO_INIT,
      AUTO_PERIODIC,
        CONCURRENT_WITH_DC;
    }
    public AutoInTeleOp autoInteleOpState = AutoInTeleOp.AUTO_DISABLED;

    public enum GamePieceMode{
        HATCH_PANEL,
        CARGO
    }

    private static GamePieceMode GamePieceMode;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        Robot.setGamePieceMode(GamePieceMode.HATCH_PANEL);
        selectedModeName = "";
        lastSelectedModeName = "";
      m_oi = new OI();
      zeroAllSensors();
      //mDrive.setDebug(true);
      //adds in the iterative code to make the code run
      mEnabledLooper.register(RobotStateEstimator.getInstance());
      S_SubsystemManager.registerEnabledLoops(mEnabledLooper);

      powerDistributionPanel = new PowerDistributionPanel();
      SmartDashboard.putData("Auto mode", m_chooser);
      mAutoModeExecuter = null;
      mAutomatedSequenceExecuter = null;
      mAutoModeExecuter = new AutoModeExecuter();
      mAutomatedSequenceExecuter = new AutoModeExecuter();
      //mAutoModeExecuter.setAutoMode(new QualMode()); TODO
      mDrive.setCoastMode();
      AutoModeSelector.initAutoModeSelector();
      needToPositionControlInTele = false;
      try {
        Thread.sleep(3000);
      } catch (InterruptedException e){
        System.out.println(e);
      }
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
            AutoModeSelector.initAutoModeSelector();
            selectedAuto = AutoModeSelector.getSelectedAutoMode(selectedModeName);
        }
        if(mAutomatedSequenceExecuter != null) {
            mAutomatedSequenceExecuter.stop();
        }
        autoInteleOpState = AutoInTeleOp.AUTO_DISABLED;

      m_oi.resetAutoLatch();
    }

    @Override
    public void disabledPeriodic() {
      if(DriverStation.isDSAttached() ){
        if(DriverStation.getAlliance() == DriverStation.Alliance.Blue){
          isBlue = true;
        } else if(DriverStation.getAlliance() == DriverStation.Alliance.Red){
          isBlue = false;
        }
      }

      allPeriodic();
      selectedModeName = SmartDashboard.getString(
                AutoModeSelector.SELECTED_AUTO_MODE_DASHBOARD_KEY,
                "org.usfirst.frc.team1806.robot.auto.modes.NothingAuto");
      if(!selectedModeName.equals(lastSelectedModeName) || bAutoModeStale){
          bAutoModeStale = false;
          AutoModeSelector.initAutoModeSelector();
          selectedAuto = AutoModeSelector.getSelectedAutoMode(selectedModeName);
      }

      autoInteleOpState = AutoInTeleOp.AUTO_DISABLED;
      S_SubsystemManager.stop();
      S_SubsystemManager.outputToSmartDashboard();

      lastSelectedModeName = selectedModeName;

    }


    @Override
    public void autonomousInit() {
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
        mCargoIntakeSubsystem.retractOuterIntake();
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
      Scheduler.getInstance().run();
      if((m_oi.getDisableAutoButton() || mAutoModeExecuter.isStopped()) && !isAutoInterrupted) {
          isAutoInterrupted = true;
          teleopInit();
      }
      else if(isAutoInterrupted) {
          teleopPeriodic();
      }
    }

    @Override
    public void teleopInit() {
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
            if(m_oi.autoInTeleOpOn()){
              autoInteleOpState = AutoInTeleOp.AUTO_INIT;
            } else {
              runTeleOp();
            }
            break;
          case AUTO_INIT:
            selectedAuto = AutoModeSelector.getSelectedAutoMode(selectedModeName);
            if(m_oi.autoInTeleOpOn()){
              zeroAllSensors();
              if (mAutoModeExecuter != null) {
                mAutoModeExecuter.stop();

              }
              autonomousInit();
              m_oi.autoRunCommands();
              autoInteleOpState = AutoInTeleOp.AUTO_PERIODIC;
            } else {
              autoInteleOpState = AutoInTeleOp.AUTO_DISABLED;
              teleopInit();
            }
            break;

          case AUTO_PERIODIC:
            if(m_oi.autoInTeleOpOn()){
              autonomousPeriodic();
              m_oi.autoRunCommands();
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
          automatedSequenceEnabled = m_oi.getAutomatedSequenceButton();
          sequenceStarting = automatedSequenceEnabled && !wasAutomatedSequenceEnabled;
          sequenceEnding = !automatedSequenceEnabled && wasAutomatedSequenceEnabled;

          if(automatedSequenceEnabled) {
              if(sequenceStarting) {
                  mSequenceState = m_oi.getAutomatedSequenceMode();
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
      SmartDashboard.putString("Auto We Are Running", AutoModeSelector.returnNameOfSelectedAuto());
     //SmartDashboard.putNumber("PDP Total", powerDistributionPanel.getTotalCurrent());
    }
    private void runTeleOp(){
      Scheduler.getInstance().run();
      m_oi.runCommands();
      allPeriodic();
    }

    /**
     * Sets the global game piece mode, runs functions associated with mode change on subsystems.
     * @param mode the wanted mode
     */
    public static synchronized void setGamePieceMode(GamePieceMode mode){
        GamePieceMode = mode;

        switch(mode){
            case CARGO:
                S_SubsystemManager.goToCargoMode();
                break;
            case HATCH_PANEL:
                S_SubsystemManager.goToHatchMode();
                break;
            default:
                break;
        }
    }


    public static synchronized GamePieceMode getGamePieceMode(){
        return GamePieceMode;
    }

    public static synchronized void RetractAll(){
        S_SubsystemManager.retractAll();
    }
  }
