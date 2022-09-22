package org.usfirst.frc.team1806.robot.util;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Calendar;

import edu.wpi.first.wpilibj.Timer;

/*
 * DataLogger is used so we can place CSV files on a flashdrive
 * and do analysis on it later.
 */

public class DataLogger {
	
	// Creates fileName using RoboRio calender
	private Calendar mC = Calendar.getInstance();
	private String mFileName = "/U/" + String.valueOf(mC.getTime() + ".csv");
	private boolean mAlreadyRan = false;
	
	public DataLogger(){
		//replace spaces and colons to make filename acceptable
		mFileName = mFileName.replaceAll("\\s", "_");
		mFileName = mFileName.replace(":", "-");
	}
		
	public void addTimestamp(String[] thingsNeeded){
		// Adding timestamp happens at the beginning of each TeleOP cycle
		// Adds date along with other parameters
		 FileWriter fileWriter;
			try {
				fileWriter = new FileWriter(mFileName, true);
		      	BufferedWriter bufferedWriter = new BufferedWriter(fileWriter);
		 		bufferedWriter.write("\r\n" + "New Teleop Cycle Started" +  "\r\n");
		 		bufferedWriter.close();
			} catch (IOException e) {
				e.printStackTrace();
			}
	        
	}
	
	public void writeData(String data, Timer time){
        try {
        	
        	FileWriter fileWriter = new FileWriter(mFileName, true);
        	BufferedWriter bufferedWriter = new BufferedWriter(fileWriter);
            bufferedWriter.newLine();
            bufferedWriter.write(data +  "," + time.get() + "\r\n");
 
            // Always close files.
            bufferedWriter.close();
        }
        catch(IOException ex) {
        	if(!mAlreadyRan) {
                System.out.println(
                        "failed to write to file '"
                        + mFileName + "'");
        	}
        	mAlreadyRan = true;
        }	
	}
	
	public void writeNewTeleopCycle(Double[] thingsToWrite){
		//This method will write data every cycle of TeleOp
		try {
	       	 FileWriter fileWriter = new FileWriter(mFileName, true);
	       	 BufferedWriter bufferedWriter = new BufferedWriter(fileWriter);
	         StringBuilder sBuilder = new StringBuilder();
	         
	         for(double s : thingsToWrite) {
	        	 sBuilder.append(s);
	        	 sBuilder.append(",");
	         }
	         sBuilder.append("\n");
	         bufferedWriter.write(sBuilder.toString());
	         // Always close files.
	         bufferedWriter.close();
       }
       catch(IOException ex) {
           System.out.println(
               "failed to write to file '"
               + mFileName + "'");
           
       }	
		
	}
	
	
}