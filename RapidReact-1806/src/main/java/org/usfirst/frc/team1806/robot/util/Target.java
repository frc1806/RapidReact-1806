package org.usfirst.frc.team1806.robot.util;

import com.google.gson.JsonObject;
import org.usfirst.frc.team1806.robot.Vision.PieceOfTape;
import com.google.gson.JsonObject;
import com.google.gson.JsonPrimitive;

public class Target {
    private double distance;
    private double robotToTarget;
    private double targetHeadingOffset; // Always 0 if target doesn't have a heading i.e. Cargo

    public PieceOfTape getLeftTarget() {
        return leftTarget;
    }

    public PieceOfTape getRightTarget() {
        return rightTarget;
    }

    private PieceOfTape leftTarget, rightTarget;

    public double getRobotToTarget() {return robotToTarget;}

    public double getTargetHeadingOffset() {return targetHeadingOffset;}

    public double getDistance() {
        return distance;
    }

    public Target(JsonObject targetObject){
        distance = targetObject.get("distance").getAsDouble();
        robotToTarget = targetObject.get("robot_to_target").getAsDouble();
        targetHeadingOffset = targetObject.get("target_heading_offset").getAsDouble();
    }

    public JsonObject getTargetJson(){
        JsonObject targetsObject = new JsonObject();
        targetsObject.add("distance", new JsonPrimitive(getDistance()));
        targetsObject.add("robot_to_target", new JsonPrimitive(getRobotToTarget()));
        targetsObject.add("target_heading_offset", new JsonPrimitive(getTargetHeadingOffset()));
        return targetsObject;
    }

    public int getMiddle() {
        return (int) ((leftTarget.getmTop().x + rightTarget.getmTop().x) / 2);
    }



    public RigidTransform2d getTargetPosition(RigidTransform2d robotPosition){
        double angleForConversion = robotPosition.getRotation().getDegrees() + robotToTarget;
        double targetX = distance * Math.cos(Math.toRadians(angleForConversion));
        double targetY = distance * Math.sin(Math.toRadians(angleForConversion));
        double targetHeading = robotPosition.getRotation().getDegrees() + robotToTarget + targetHeadingOffset ;
        return new RigidTransform2d(new Translation2d(targetX,targetY), new Rotation2d().fromDegrees(targetHeading));
    }

    public String toString(){
        return "distance: " + distance + ", Robot to target:" + robotToTarget + ", targetHeadingOffset:" + targetHeadingOffset + ",leftHeight:" + leftTarget.getOuterToTopHeight() + ", rightHeight" + rightTarget.getOuterToTopHeight();
    }

}
