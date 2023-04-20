package org.usfirst.frc.team1806.robot.subsystems;

import org.usfirst.frc.team1806.robot.loop.Looper;

public interface Subsystem {
    public void writeToLog();

    public abstract void outputToSmartDashboard();

    public abstract void stop();

    public abstract void zeroSensors();

    public abstract void registerEnabledLoops(Looper enabledLooper);

    public abstract void setDebug(boolean _debug);

    public abstract void retractAll();

    public abstract void setupDriverTab();
}