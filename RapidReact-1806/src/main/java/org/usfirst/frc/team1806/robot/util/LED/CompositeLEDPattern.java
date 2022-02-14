package org.usfirst.frc.team1806.robot.util.LED;

import edu.wpi.first.wpilibj.util.Color;

public class CompositeLEDPattern implements LEDPattern {

    LEDPatternSegment[] segments;
    int frameRate;
    int frameCounter;

    /**
     * Make a composotite LED pattern consisting of multiple LED pattern segments. Uses the frame rate from the first segement.
     * @param ledPatternSegments
     */
    public CompositeLEDPattern(LEDPatternSegment... ledPatternSegments) {
        segments = ledPatternSegments;
        frameRate = ledPatternSegments[0].getPattern().getFPS();
        frameCounter = 0;
    }

    @Override
    public Color getColorForPositionInString(int position) {
        int posCounter = 0;
        for(LEDPatternSegment segment: segments)
        {
            if( posCounter + segment.getLength() > position)
            {
                if(segment.getInvert())
                {
                    return segment.getPattern().getColorForPositionInString((segment.getLength() -1) - (position-posCounter));
                }
                return segment.getPattern().getColorForPositionInString(position-posCounter);
            }
            else
            {
                posCounter += segment.getLength();
            }
        }
        return Color.kBlack;
    }

    @Override
    public void updateAnimation() {
        for(LEDPatternSegment segment: segments)
        {
            segment.getPattern().updateAnimation();
        }

    }

    @Override
    public int getFPS() {
        
        return frameRate;
    }
}
