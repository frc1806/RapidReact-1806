package org.usfirst.frc.team1806.robot.subsystems;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.driver.AnalogPressureSensor;
import org.usfirst.frc.team1806.robot.loop.Loop;
import org.usfirst.frc.team1806.robot.loop.Looper;
import edu.wpi.first.wpilibj.Compressor;
import org.usfirst.frc.team1806.robot.util.SamplingFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CompressorControlSubsystem implements Subsystem {

    boolean debug = false;

    private static CompressorControlSubsystem compressorControlSubsystem = new CompressorControlSubsystem();

    public static CompressorControlSubsystem getInstance(){
        return compressorControlSubsystem;
    }

    Compressor compressor;

    private double currentTimeStamp;
    private double lastTimeStamp;
    private AnalogPressureSensor pressureSensor;
    private Boolean override;
    private Double batteryCoulombCount = 0.0;

    private Boolean wasCompressorRunning;
    private Boolean runCompressor;


    private Loop mLoop = new Loop() {

        @Override
        public synchronized void onStart(double timestamp) {
            lastTimeStamp = currentTimeStamp;
            currentTimeStamp = timestamp;
        }

        @Override
        public synchronized void onLoop(double timestamp) {
            // TODO Auto-generated method stub
            lastTimeStamp = currentTimeStamp;
            currentTimeStamp = timestamp;
            synchronized (CompressorControlSubsystem.this) {

                //IF statements in reverse order of priority

                runCompressor = false;
                if(pressureSamplingFilter.update(pressureSensor.getPressure()) < Constants.kPressureAverageMinimumToStart){
                    runCompressor = true;
                }

                if(voltageSamplingFilter.update(Robot.powerDistributionPanel.getVoltage()) < Constants.kBatteryVoltageCompressorShutoffThreshold){
                    runCompressor = false;
                }

                if(amperageSamplingFilter.update(Robot.powerDistributionPanel.getTotalCurrent()) > Constants.kAverageAmpDemandToShutOffCompressor) {
                    runCompressor = false;
                }

                if(override){
                    runCompressor = true;
                }

                if((wasCompressorRunning && Robot.powerDistributionPanel.getTotalCurrent() > Constants.kAbsoluteRobotCompressorShutOffAmps) ||
                        (!wasCompressorRunning && Robot.powerDistributionPanel.getTotalCurrent() > Constants.kAbsoluteRobotCompressorShutOffAmps - Constants.kExpectedCompressorMaxCurrentDraw)){
                    runCompressor = false;
                }

                if((wasCompressorRunning && Robot.powerDistributionPanel.getVoltage() < Constants.kBatteryVoltageAbsoluteCompressorShutoffThreshold) ||
                        (!wasCompressorRunning && Robot.powerDistributionPanel.getVoltage() < Constants.kBatteryVoltageAbsoluteCompressorShutoffThreshold + Constants.kExpectedCompressorVoltageDrop)) {
                    runCompressor = false;
                }

                compressor.setClosedLoopControl(runCompressor);
                /**
                 * mathematical function for figuring out battery charge
                 */
                if(Robot.powerDistributionPanel.getTotalCurrent()<Constants.kLowAmpLoad){
                    batteryCoulombCount = ((Robot.powerDistributionPanel.getVoltage() - Constants.kBatteryDepletedVoltage)/(Constants.kBatteryFullChargeVoltage -Constants.kBatteryDepletedVoltage))* Constants.kFullChargeBatteryCoulombCount;
                }
                else {
                    batteryCoulombCount -= Robot.powerDistributionPanel.getTotalCurrent() * (currentTimeStamp - lastTimeStamp);
                }

                }
            }

            @Override
        public synchronized void onStop(double timestamp){
                lastTimeStamp = currentTimeStamp;
                currentTimeStamp = timestamp;
            }


        };

    SamplingFilter pressureSamplingFilter;
    SamplingFilter voltageSamplingFilter;
    SamplingFilter amperageSamplingFilter;



    private CompressorControlSubsystem(){
        compressor = new Compressor(0);
        pressureSensor = new AnalogPressureSensor(0);//TODO put in robot map
        pressureSamplingFilter = new SamplingFilter(Constants.kPressureSensorSamplingLoops);
        voltageSamplingFilter = new SamplingFilter(Constants.kBatteryVoltageSamplingLoops);
        amperageSamplingFilter = new SamplingFilter(Constants.kRobotDemandAmpsSamplingLoops);
        batteryCoulombCount = Constants.kFullChargeBatteryCoulombCount;
        wasCompressorRunning =false;
        override = false;
    }
    public void writeToLog(){

    }

    public void outputToSmartDashboard(){
        if(debug) {
            SmartDashboard.putNumber(Constants.kCompressorKey + "STORAGE PRESSURE", pressureSensor.getPressure());
            SmartDashboard.putNumber(Constants.kCompressorKey + "Recent Average Pressure", pressureSamplingFilter.getCurrentAverage());
            SmartDashboard.putNumber(Constants.kCompressorKey + "Current", compressor.getCompressorCurrent());
            SmartDashboard.putNumber(Constants.kCompressorKey + "Recent Average Battery Voltage", voltageSamplingFilter.getCurrentAverage());
            SmartDashboard.putNumber(Constants.kCompressorKey + "Recent Average PDP Total Current", amperageSamplingFilter.getCurrentAverage());
            SmartDashboard.putBoolean(Constants.kCompressorKey + "Is Running?", compressor.getClosedLoopControl());
            SmartDashboard.putNumber(Constants.kCompressorKey + "Battery Charge", (batteryCoulombCount / Constants.kFullChargeBatteryCoulombCount) * 100);
        }
    }

    public void stop(){
        compressor.setClosedLoopControl(false);
    }

    public void zeroSensors(){
        //Can't zero an analog sensor, sorry.

    }

    public void registerEnabledLoops(Looper enabledLooper){
        enabledLooper.register(mLoop);
    }

    @Override
    public void setDebug(boolean _debug) {
        debug = _debug;
    }

    public void setOverride(boolean override){
        this.override = override;
    }



    public void goToHatchMode(){
        //nothing to do here unless we want to change how we manage air based on what mode we're in
    }

    public void goToCargoMode(){
        //nothing to do here unless we want to change how we manage air based on what mode we're in
    }

    public void retractAll() {
        //nothing to do here unless we want to change how we manage air based on what mode we're in
    }
}
