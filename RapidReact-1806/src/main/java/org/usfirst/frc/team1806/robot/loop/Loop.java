package org.usfirst.frc.team1806.robot.loop;

public interface Loop {

    /**
     * The method run on loop startup.
     * @param timestamp current robot runtime in seconds
     */
    public void onStart(double timestamp);

    /**
     * The method run on normal loop operation, after start.
     * @param timestamp current robot runtime in seconds
     */
    public void onLoop(double timestamp);

    /**
     * The method run to stop the loop.
     * @param timestamp current robot runtime in seconds.
     */
    public void onStop(double timestamp);
}
