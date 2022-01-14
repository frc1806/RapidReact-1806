package org.usfirst.frc.team1806.robot.subsystems;

import java.util.List;

import org.usfirst.frc.team1806.robot.loop.Looper;

/**
 * Used to reset, start, stop, and update all subsystems at once
 */
public class SubsystemManager {

    private final List<Subsystem> mAllSubsystems;

    public SubsystemManager(List<Subsystem> allSubsystems) {
        mAllSubsystems = allSubsystems;
    }

    public void outputToSmartDashboard() {
        mAllSubsystems.forEach((s) -> s.outputToSmartDashboard());
    }

    public void writeToLog() {
        mAllSubsystems.forEach((s) -> s.writeToLog());
    }

    public void stop() {
        mAllSubsystems.forEach((s) -> s.stop());
    }

    public void zeroSensors() {
        mAllSubsystems.forEach((s) -> s.zeroSensors());
    }

    public void registerEnabledLoops(Looper enabledLooper) {
        mAllSubsystems.forEach((s) -> s.registerEnabledLoops(enabledLooper));
    }

    //2019 specific
    public void goToHatchMode() {mAllSubsystems.forEach((s) -> s.goToHatchMode());}

    public void goToCargoMode() {mAllSubsystems.forEach((s) -> s.goToCargoMode());}

    public void retractAll() { mAllSubsystems.forEach((s) -> s.retractAll());}
}