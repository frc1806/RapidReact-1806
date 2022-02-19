package org.usfirst.frc.team1806.robot.util.LED;

import edu.wpi.first.wpilibj.util.Color;

import java.util.Arrays;
import java.util.List;

public class RGBWaveLEDPattern implements LEDPattern {

    private double speed;
    private int fps;
    private int size;
    private int positionCounter;

    public RGBWaveLEDPattern(double speed, int fps, int size){
        this.speed = speed;
        this.fps = fps;
        this.size = size;
    }

    @Override
    public Color getColorForPositionInString(int position) {

        // Calculate the hue - hue is easier for rainbows because the color
                  
        // shape is a circle so only one value needs to precess
                  
        final var hue = ((((positionCounter + position) % size )* 180 / size)) % 180;
                  
        // Set the value
                  
        return Color.fromHSV(hue, 255, 128);
    }

    @Override
    public void updateAnimation() {
        positionCounter += (speed * size) / (fps);
    }

    @Override
    public int getFPS() {
        // TODO Auto-generated method stub
        return fps;
    }
}