package org.usfirst.frc.team1806.robot.util;

import edu.wpi.first.util.CircularBuffer;

public class BisectedCircularBuffer extends CircularBuffer {

    int bisectionPoint;
    private int length;
    /**
     * A {@link CircularBuffer that can be cut into 2 pieces for comparing filtered values}
     * @param size The size of the {@link CircularBuffer}, that will contain all of the values for both halves.
     * @param bisectionPoint The index at which to split the Buffer, must be between 1 and size - 2.
     */
    public BisectedCircularBuffer(int size, int bisectionPoint) {
        super(size);
        length = size;
        this.bisectionPoint = bisectionPoint;
    }

    public double getMoreRecentAverage()
    {
        double runningTotal = 0.0;
        for(int i = bisectionPoint; i < length ; i++)
        {
            runningTotal += get(i);
        }
        return runningTotal / (length - bisectionPoint);
    }

    public double getOlderAverage()
    {
        double runningTotal = 0.0;
        for(int i = 0; i < bisectionPoint; i++)
        {
            runningTotal += get(i);
        }
        return runningTotal / bisectionPoint;
    }
    
}
