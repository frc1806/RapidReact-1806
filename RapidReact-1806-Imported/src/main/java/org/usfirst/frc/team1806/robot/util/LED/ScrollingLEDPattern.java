package org.usfirst.frc.team1806.robot.util.LED;

import edu.wpi.first.wpilibj.util.Color;

import java.util.Arrays;
import java.util.List;

public class ScrollingLEDPattern implements LEDPattern {
    public static final ScrollingLEDPattern VISION_GREEN = new ScrollingLEDPattern(Arrays.asList(Color.kGreen), 0, 24);
    public static final ScrollingLEDPattern LIGHTS_OUT = new ScrollingLEDPattern(Arrays.asList(Color.kBlack), 0, 24);
    public static final ScrollingLEDPattern BLACK_AND_WHITE = new ScrollingLEDPattern(Arrays.asList(Color.kBlack, Color.kBlack, Color.kBlack, Color.kWhite, Color.kWhite, Color.kWhite), 1, 24);
    public static final ScrollingLEDPattern ECTOPLASM = new ScrollingLEDPattern(Arrays.asList(Color.kDarkGreen, Color.kLimeGreen, Color.kLimeGreen, Color.kDarkGreen, Color.kBlack, Color.kBlack, Color.kBlack, Color.kBlack), 2, 24);
    public static final ScrollingLEDPattern RED_ALLIANCE = GenerateRedAlliancePatternAtSpeed(1);
    public static final ScrollingLEDPattern BLUE_ALLIANCE = GenerateBlueAlliancePatternAtSpeed(1);
    public static final ScrollingLEDPattern NO_ALLIANCE = new ScrollingLEDPattern(Arrays.asList(new Color(1.0, 1.0, 1.0), new Color(1.0, 1.0, 1.0), new Color(1.0, 1.0, 1.0), new Color(0.9, 0.9, 0.9), new Color(1.0, 1.0, 1.0), new Color(0.9, 0.9, 0.9), new Color(1.0, 1.0, 1.0), new Color(0.9, 0.9, 0.9), new Color(1.0, 1.0, 1.0), new Color(0.9, 0.9, 0.9), new Color(0.9, 0.9, 0.9), new Color(0.9, 0.9, 0.9)), 1, 24);
    
    public static ScrollingLEDPattern GenerateRedAlliancePatternAtSpeed(int speed)
    {
        return new ScrollingLEDPattern(Arrays.asList(new Color(1.0, 0.0, 0.0), new Color(1.0, 0.0, 0.0), new Color(1.0, 0.0, 0.0), new Color(1.0, 0.2, 0.2), new Color(1.0, 0.0, 0.0),new Color(1.0, 0.2, 0.2), new Color(1.0, 0.0, 0.0),new Color(1.0, 0.2, 0.2), new Color(1.0, 0.0, 0.0), new Color(1.0, 0.2, 0.2), new Color(1.0, 0.2, 0.2), new Color(1.0, 0.2, 0.2)), speed, 24);
    }

    public static ScrollingLEDPattern GenerateBlueAlliancePatternAtSpeed(int speed)
    {
        return new ScrollingLEDPattern(Arrays.asList(new Color(0.0, 0.0, 1.0), new Color(0.0, 0.0, 1.0), new Color(0.0, 0.0, 1.0), new Color(0.2, 0.2, 1.0), new Color(0.0, 0.0, 1.0),new Color(0.2, 0.2, 1.0), new Color(0.0, 0.0, 1.0),new Color(0.2, 0.2, 1.0), new Color(0.0, 0.0, 1.0), new Color(0.2, 0.2, 1.0), new Color(0.2, 0.2, 1.0), new Color(0.2, 0.2, 1.0)), speed, 24);
    }

    public static ScrollingLEDPattern GenerateRedAllianceTracerPatternAtSpeed(int speed)
    {
        return new ScrollingLEDPattern(Arrays.asList(new Color(.1, 0.0, 0.0),new Color(.2, 0.0, 0.0), new Color(.4, 0.0, 0.0), new Color(.8, 0.0, 0.0), new Color(1.0, 0.0, 0.0), new Color(1.0, 0.0, 0.0), new Color(1.0, 0.0, 0.0), new Color(0.8, 0.0, 0.0), new Color(0.4, 0.0, 0.0), new Color(0.2, 0.0, 0.0), new Color(0.1, 0.0, 0.0), new Color(0.0, 0.0, 0.0), new Color(0.0, 0.0, 0.0), new Color(0.0, 0.0, 0.0), new Color(0.0, 0.0, 0.0), new Color(0.0, 0.0, 0.0), new Color(0.0, 0.0, 0.0), new Color(0.0, 0.0, 0.0), new Color(0.0, 0.0, 0.0), new Color(0.0, 0.0, 0.0), new Color(0.0, 0.0, 0.0), new Color(0.0, 0.0, 0.0)), speed, 24);
    }

    public static ScrollingLEDPattern GenerateBlueAllianceTracerPatternAtSpeed(int speed)
    {
        return new ScrollingLEDPattern(Arrays.asList(new Color(0.0, 0.0, 0.1),new Color(0.0, 0.0, 0.2), new Color(0.0, 0.0, 0.4), new Color(0.0, 0.0, 0.8), new Color(0.0, 0.0, 1.0), new Color(0.0, 0.0, 1.0), new Color(0.0, 0.0, 1.0), new Color(0.0, 0.0, 0.8), new Color(0.0, 0.0, 0.4), new Color(0.0, 0.0, 0.2), new Color(0.0, 0.0, 0.1), new Color(0.0, 0.0, 0.0), new Color(0.0, 0.0, 0.0), new Color(0.0, 0.0, 0.0), new Color(0.0, 0.0, 0.0), new Color(0.0, 0.0, 0.0), new Color(0.0, 0.0, 0.0), new Color(0.0, 0.0, 0.0), new Color(0.0, 0.0, 0.0), new Color(0.0, 0.0, 0.0), new Color(0.0, 0.0, 0.0), new Color(0.0, 0.0, 0.0)), speed, 24);
    }

    private List<Color> pattern;
    private int animationStepAmount;
    private int currentAnimationStep;
    private int FPS;

    public ScrollingLEDPattern(List<Color> pattern, int animationStepAmount, int FPS) {
        this.pattern = pattern;
        this.animationStepAmount = animationStepAmount;
        this.FPS = FPS;
    }

    /**
     *{@inheritDoc}
     */
    public Color getColorForPositionInString(int position){
        return pattern.get(Math.abs(position-currentAnimationStep) % pattern.size());
    }

    /**
     *{@inheritDoc}
     */
    public void updateAnimation(){
        currentAnimationStep += animationStepAmount;
    }

    /**
     *{@inheritDoc}
     */
    public int getFPS(){
        return FPS;
    }
}
