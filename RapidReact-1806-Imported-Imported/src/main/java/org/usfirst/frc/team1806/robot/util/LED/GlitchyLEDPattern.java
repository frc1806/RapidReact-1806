package org.usfirst.frc.team1806.robot.util.LED;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

import java.util.ArrayList;
import java.util.Random;
import java.util.function.IntConsumer;

/**
 * This class starts with a given pattern and corrupts it every 100 loops, and adds a 1 frame corruption if conditions are met.
 */
public class GlitchyLEDPattern implements LEDPattern {

    private static final int INTS_FOR_FUN_SIZE = 64;
    private static final int RANDOM_GLITCH_EFFECT_CHOCIES = 14;

    private Random mRandom;
    private int mAnimationCounter;
    private AddressableLEDBuffer mLEDBuffer;
    private ArrayList<Integer> mIntsForFun;
    private int garbageAccumulator;
    private int scrollSpeed;
    private int scrollAccumulator;

    public GlitchyLEDPattern(LEDPattern startingPoint, int size, int startingScrollSpeed){
        mRandom = new Random(System.currentTimeMillis());
        mLEDBuffer = new AddressableLEDBuffer(size);
        scrollSpeed = startingScrollSpeed;
        mIntsForFun = new ArrayList<Integer>(INTS_FOR_FUN_SIZE);
        //Init an internal buffer to the current state of what the starting point would push to the string.
        for(int i = 0; i < mLEDBuffer.getLength(); i++){
            mLEDBuffer.setLED(i, startingPoint.getColorForPositionInString(i));
        }
        mAnimationCounter = 0;
         mRandom.ints(INTS_FOR_FUN_SIZE).forEach(new IntConsumer() {

            @Override
            public void accept(int value) {
                mIntsForFun.add(value); 
                garbageAccumulator += value;
            }
            
        });;
    }


    /**
     *{@inheritDoc}
     */
    @Override
    public Color getColorForPositionInString(int position) {
        int effectivePosition = Math.abs(position + scrollAccumulator) % mLEDBuffer.getLength();
        if(!mIntsForFun.contains(effectivePosition) || !mIntsForFun.contains(mAnimationCounter) || garbageAccumulator % (effectivePosition +1) < mLEDBuffer.getLength() * .05) {
            return mLEDBuffer.getLED(effectivePosition);
        }
        else{
            return getGlitchEffectedColor(effectivePosition);
        }

    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void updateAnimation() {
        //choose your corruption
        mAnimationCounter++;
        scrollAccumulator+= scrollSpeed;
        garbageAccumulator+= mAnimationCounter;
        if(mAnimationCounter % 16 == 0)
        {
            if(mRandom.nextDouble()  < 0.1)
            {
                //DO A RARE EFFECT
                switch(Math.abs(mRandom.nextInt()) % 26){
                    default:
                    case 0:
                    case 1:
                        break; //or not...
                    case 2:
                        for(int i = 0; i < mLEDBuffer.getLength(); i++)
                        {
                            mLEDBuffer.setLED(i, ScrollingLEDPattern.LIGHTS_OUT.getColorForPositionInString(i));
                        }
                        break;
                    case 3:
                    for(int i = 0; i < mLEDBuffer.getLength(); i++)
                    {
                        mLEDBuffer.setLED(i, ScrollingLEDPattern.RED_ALLIANCE.getColorForPositionInString(i));
                    }
                    break;
                    case 4:
                    for(int i = 0; i < mLEDBuffer.getLength(); i++)
                    {
                        mLEDBuffer.setLED(i, ScrollingLEDPattern.BLUE_ALLIANCE.getColorForPositionInString(i));
                    }
                    break;
                    case 5:
                    for(int i = 0; i < mLEDBuffer.getLength(); i++)
                    {
                        mLEDBuffer.setLED(i, ScrollingLEDPattern.VISION_GREEN.getColorForPositionInString(i));
                    }
                    break;
                    case 6:
                    for(int i = 0; i < mLEDBuffer.getLength(); i++)
                    {
                        mLEDBuffer.setLED(i, ScrollingLEDPattern.NO_ALLIANCE.getColorForPositionInString(i));
                    }
                    break;
                    case 7:
                    for(int i = 0; i < mLEDBuffer.getLength(); i++)
                    {
                        mLEDBuffer.setLED(i, ScrollingLEDPattern.ECTOPLASM.getColorForPositionInString(i));
                    }
                    break;
                    case 8:
                    for(int i = 0; i < mLEDBuffer.getLength(); i++)
                    {
                        mLEDBuffer.setLED(i, Color.kAquamarine);
                    }
                    break;
                    case 9:
                    for(int i = 0; i < mLEDBuffer.getLength(); i++)
                    {
                        mLEDBuffer.setLED(i, Color.kGold);
                    }
                    break;
                    case 10:
                    for(int i = 0; i < mLEDBuffer.getLength(); i++)
                    {
                        mLEDBuffer.setLED(i, Color.kRed);
                    }
                    break;
                    case 11:
                    for(int i = 0; i < mLEDBuffer.getLength(); i++)
                    {
                        mLEDBuffer.setLED(i, Color.kDarkGreen);
                    }
                    break;
                    case 12:
                    for(int i = 0; i < mLEDBuffer.getLength(); i++)
                    {
                        mLEDBuffer.setLED(i, Color.kBlue);
                    }
                    break;
                    case 13:
                    for(int i = 0; i < mLEDBuffer.getLength(); i++)
                    {  
                        //broken dumb rainbow... still glitchy, leaving it
                        mLEDBuffer.setLED(i, Color.fromHSV((int)((double) i / mLEDBuffer.getLength()) * 180 , 255, 255));
                    }
                    break;
                    case 15:
                    for(int i = 0; i < mLEDBuffer.getLength(); i++)
                    {
                        mLEDBuffer.setLED(i, Color.kBlueViolet);
                    }
                    break;
                    case 16:
                    for (var i = 0; i < mLEDBuffer.getLength(); i++) {

                        // Calculate the hue - hue is easier for rainbows because the color
                  
                        // shape is a circle so only one value needs to precess
                  
                        final var hue = ((i * 180 / mLEDBuffer.getLength())) % 180;
                  
                        // Set the value
                  
                        mLEDBuffer.setHSV(i, hue, 255, 128);
                  
                      }
                    break;
                }
            }
        }
        if(mAnimationCounter % 8 == 0 ){
            garbageAccumulator += mRandom.nextInt();
            switch(Math.abs(mRandom.nextInt()) % 26){
                default:
                case 1:
                case 2:
                case 5:
                    break; // do nothing. That's a glitchy thing, right?
                case 6:
                    //replace ~20% of the LEDs with random colors
                    for(int i = 0; i < mLEDBuffer.getLength() * .2; i++){
                        mLEDBuffer.setLED(Math.abs(mRandom.nextInt() % mLEDBuffer.getLength()), generateRandomColor());
                    }
                    break;
                case 7:
                    //replace ~20% of the LEDs with glitch colors
                    for(int i = 0; i < mLEDBuffer.getLength() * .2; i++){
                        int afflictedPosition = Math.abs(mRandom.nextInt() % mLEDBuffer.getLength());
                        mLEDBuffer.setLED(afflictedPosition , getGlitchEffectedColor(afflictedPosition));
                    }
                    break;
                case 8:
                case 4:
                    //Random shift
                    for(int i = 0; i < mLEDBuffer.getLength(); i++){
                        mLEDBuffer.setLED(i, mLEDBuffer.getLED((i + Math.abs(mIntsForFun.get(3))) % mLEDBuffer.getLength()));
                    }
                    break;
                case 9:
                case 3:
                    //flip buffer
                    for(int i = 0; i < mLEDBuffer.getLength(); i++){
                        mLEDBuffer.setLED(i, mLEDBuffer.getLED((mLEDBuffer.getLength()-1) - i));
                    }
                case 10:
                    //apply random color to random first percent of the buffer.
                    for(int i = 0; i < mLEDBuffer.getLength() * Math.abs(mRandom.nextDouble()); i++){
                        mLEDBuffer.setLED(i, generateRandomColor());
                    }
                    break;
                case 11:
                    //replace random percent of the buffer with glitches from front.
                    for(int i = 0; i < mLEDBuffer.getLength() * mRandom.nextDouble(); i++){
                        mLEDBuffer.setLED(i , getGlitchEffectedColor(i));
                    }
                    break;
                case 12:
                    //apply random color to random last percent of the buffer.
                    for(int i = mLEDBuffer.getLength() -1 ; i > mLEDBuffer.getLength() * Math.abs(mRandom.nextDouble()); i--){
                        mLEDBuffer.setLED(i, generateRandomColor());
                    }
                    break;
                case 13:
                    //replace random percent of the buffer with glitches from back.
                    for(int i = mLEDBuffer.getLength() - 1; i < mLEDBuffer.getLength() * mRandom.nextDouble(); i--){
                        mLEDBuffer.setLED(i , getGlitchEffectedColor(i));
                    }
                    break;
                case 15:
                    scrollSpeed = scrollSpeed*2;
                    break;
                case 16:
                    scrollSpeed = scrollSpeed* -1;
                    break;
                case 14:    
                case 17:
                    scrollSpeed = 1;
                    break;
                case 18:
                    scrollSpeed = 5;
                    break;
                case 19:
                    scrollSpeed = 10;
                    break;
                case 20:
                    scrollSpeed = scrollSpeed * 3;
                    break;
                case 21:
                    scrollSpeed = 0;
                    break;
                case 22:
                case 23:
                case 24:
                case 25:
                    break; //another do nothing block
            }
        }
    }

    private Color generateRandomColor(){
        return new Color(mRandom.nextDouble(), mRandom.nextDouble(), mRandom.nextDouble());
    }

    private Color getGlitchEffectedColor(int position){
        int effectivePosition = Math.abs(position % mLEDBuffer.getLength());
        Color originalColor = mLEDBuffer.getLED(effectivePosition);
        switch(Math.abs(mIntsForFun.get(Math.abs(mRandom.nextInt() % INTS_FOR_FUN_SIZE))) % RANDOM_GLITCH_EFFECT_CHOCIES){
            default:
            case 0:
                return Color.kLimeGreen; //green
            case 1:
                return Color.kBlack; //black
            case 2:
                return Color.kRed; //red
            case 3:
                return Color.kAquamarine; // A common glitch color in old NES games, when SMB3 glitches out, the screen goes aquamarine a lot.
            case 4:
                return Color.kWhite; //rwhite
            case 5:
                //new random color
                return generateRandomColor();
            case 6:
                //invert color
                return new Color (1.0-originalColor.red, 1.0-originalColor.green, 1.0-originalColor.blue);
            case 7:
                //flip red and green
                return new Color (originalColor.green, originalColor.red, originalColor.blue);
            case 8:
                //flip red and blue
                return  new Color(originalColor.blue, originalColor.green, originalColor.red);
            case 9:
                //flip blue and green
                return new Color(originalColor.red, originalColor.blue, originalColor.green);
            case 10:
                //left shift color values
                return new Color(originalColor.green, originalColor.blue, originalColor.red);
            case 11:
                //grab a random pixel out of the saved buffer
                return mLEDBuffer.getLED(Math.abs(mRandom.nextInt() % mLEDBuffer.getLength()));
            case 12:
                //shift a random amount in the buffer
                return mLEDBuffer.getLED((effectivePosition + Math.abs(mIntsForFun.get(2))) % mLEDBuffer.getLength());
            case 13:
                return Color.kDarkGray;

        }
    }

    /**
     * {@inheritDoc}
     */
    public int getFPS(){
        return 24;
    }
}
