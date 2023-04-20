package org.usfirst.frc.team1806.robot.util;


public class Latch {
	/*
	 * The Latch command is used to swap
	 * between true and false
	 */
    private boolean mLastVal;
    private boolean mToggle = false;
    private boolean mTheThingToRun = false;
    
    public boolean update(boolean newVal) {
		if (mToggle && newVal) {  // Only execute once per Button push
			  mToggle = false;  // Prevents this section of code from being called again until the Button is released and re-pressed
			  if (mTheThingToRun) {  // Decide which way to set the motor this time through (or use this as a motor value instead)
				  mTheThingToRun= false;
			  } else {
			    mTheThingToRun = true;
			  }
			} else if(!newVal) { 
			    mToggle = true; // Button has been released, so this allows a re-press to activate the code above.
			}
		return mTheThingToRun;
    }
    public void resetLatch(){
    	mTheThingToRun = false;
    	mToggle = false;
    	mLastVal = false;
	}
    public boolean returnStatus(){
    	return mTheThingToRun;
    }
}