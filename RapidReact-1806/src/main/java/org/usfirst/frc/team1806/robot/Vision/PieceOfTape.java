package org.usfirst.frc.team1806.robot.Vision;

import org.opencv.core.Point;

public class PieceOfTape {

    public enum TapeType{
        UNKNOWN,
        LEFT,
        RIGHT
    }

    Point mOuter = new Point(0,0);
    Point mTop = new Point(0,0);



    Point mInner = new Point(0,0);
    
    private double leftLineDistance;
    private double rightLineDistance;

    public TapeType mTapeType = TapeType.UNKNOWN;

    public PieceOfTape(Point left, Point top, Point right){
     leftLineDistance = Math.sqrt(Math.pow(top.x-left.x, 2)+ Math.pow(top.y - left.y, 2));
     //origin top left
     if(left.y > right.y) {
         mTapeType = TapeType.LEFT;
         mOuter = left;
         mTop = top;
         mInner = right;
     }
     else if (left.y < right.y){
         mTapeType = TapeType.RIGHT;
         mOuter = right;
         mTop = top;
         mInner = left;
     }
     else {
         mTapeType = TapeType.UNKNOWN;
         //System.out.println("Tape unknown");
     }
     if(mTapeType != TapeType.UNKNOWN){
         double distance = getDistance();
         if(distance > 150 || distance < 10){
             mTapeType = TapeType.UNKNOWN;
         }
     }
    }

    public Point getmOuter() {
        return mOuter;
    }

    public Point getmTop() {
        return mTop;
    }

    public Point getmInner() {
        return mInner;
    }

    public double getOuterToTopHeight(){
        return mOuter.y - mTop.y;
    }

    public double getDistance(){
        double height = getOuterToTopHeight();
        return 3875.46567547210361226462* Math.pow(height,-0.97957530188343977517);
    }


}
