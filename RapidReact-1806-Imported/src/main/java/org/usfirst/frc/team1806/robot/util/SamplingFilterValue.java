package org.usfirst.frc.team1806.robot.util;

public class SamplingFilterValue {

    private Double currentValue;
    private Double runningTotal;

    /**
     * Stores the current value and running total
     * @param currentValue the sample at this point in time
     * @param runningTotal the running total at this point in time
     */
    public SamplingFilterValue(Double currentValue, Double runningTotal) {
        this.currentValue = currentValue;
        this.runningTotal = runningTotal;
    }

    /**
     *
     * @return The value at this point in time.
     */
    public Double getCurrentValue(){
        return currentValue;
    }

    /**
     *
     * @return The running total at this point in time.
     */
    public Double getRunningTotal(){
        return runningTotal;
    }
}