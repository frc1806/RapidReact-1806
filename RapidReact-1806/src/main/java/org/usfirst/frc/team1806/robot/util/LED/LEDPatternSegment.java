package org.usfirst.frc.team1806.robot.util.LED;

public class LEDPatternSegment {
    private int length;
    private LEDPattern pattern;
    private boolean invert;

    public LEDPatternSegment(int length, LEDPattern pattern, boolean invert) {
        this.length = length;
        this.pattern = pattern;
        this.invert = invert;
    }

    public int getLength() {
        return length;
    }

    public LEDPattern getPattern() {
        return pattern;
    }

    public boolean getInvert(){
        return invert;
    }
}
