package org.usfirst.frc.team1806.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotState;
import org.usfirst.frc.team1806.robot.Vision.VisionServer;

import java.util.ArrayList;

public class TargetHelper {


    public static RigidTransform2d generateTemporaryVisionPose() {
        double dist = SmartDashboard.getNumber("Vdistance", 0);
        double angle = SmartDashboard.getNumber("Vangle", 0);

        double x = dist * Math.sin(angle);
        double y = dist * Math.cos(angle);

        return new RigidTransform2d(new Translation2d(x,y), Rotation2d.fromDegrees(angle));
    }

    public static Target getClosestTargetToRobot(){
        VisionServer visServer = VisionServer.getInstance();
        ArrayList<Target> targets =  visServer.getTargets();
        double targetsTimestamp = visServer.getTargetsTimestamp();
        if(targets != null) {
            if (!targets.isEmpty()) {
                RigidTransform2d closestBayPose = new RigidTransform2d(new Translation2d(10000, 10000), Rotation2d.fromDegrees(0.0));
                RigidTransform2d latestFieldToVehicle = RobotState.getInstance().getLatestFieldToVehicle().getValue();
                Target closestTarget = null;
                for (Target target : targets) {
                    RigidTransform2d robotPose = RobotState.getInstance().getFieldToVehicle(targetsTimestamp - Constants.kVisionExpectedCameraLag);
                    double goalHeading = robotPose.getRotation().getDegrees() - target.getTargetHeadingOffset();
                    RigidTransform2d xCorrectedRobotPose = interpolateAlongLine(robotPose, -3.75, robotPose.getRotation().getRadians(), robotPose.getRotation().getRadians());
                    RigidTransform2d correctedRobotPose = interpolateAlongLine(xCorrectedRobotPose, -8.5, robotPose.getRotation().getRadians() + Math.toRadians(90), Rotation2d.fromRadians(robotPose.getRotation().getRadians()).getRadians());
                    RigidTransform2d proposedBayPose = interpolateAlongLine(correctedRobotPose, target.getDistance(), Math.toRadians(-target.getRobotToTarget() + robotPose.getRotation().getDegrees()), Math.toRadians(-target.getTargetHeadingOffset() + robotPose.getRotation().getDegrees()));
                    if (closestBayPose == null || proposedBayPose.getTranslation().subtract(latestFieldToVehicle.getTranslation()).norm() < closestBayPose.getTranslation().subtract(latestFieldToVehicle.getTranslation()).norm()) {
                        closestBayPose = proposedBayPose;
                        closestTarget = target;
                    }
                }
                if (closestTarget != null) {
                    System.out.println("Robot to target" + closestTarget.getRobotToTarget());
                    System.out.println("Target heading offset" + closestTarget.getTargetHeadingOffset());
                    System.out.println("Distance" + closestTarget.getDistance());
                }

                return closestTarget;
            }
        }
        return null;
    }


    public static RigidTransform2d generateBayVisionPoseFromODO() {
        VisionServer visServer = VisionServer.getInstance();
        ArrayList<Target> targets =  visServer.getTargets();
        double targetsTimestamp = visServer.getTargetsTimestamp();
        if(targets != null){
            if (!targets.isEmpty()){
                RigidTransform2d closestBayPose = new RigidTransform2d(new Translation2d(10000, 10000), Rotation2d.fromDegrees(0.0));
                RigidTransform2d latestFieldToVehicle = RobotState.getInstance().getLatestFieldToVehicle().getValue();
                Target closestTarget = null;
                for(Target target: targets){
                    RigidTransform2d robotPose = RobotState.getInstance().getFieldToVehicle(targetsTimestamp - Constants.kVisionExpectedCameraLag);
                    double goalHeading = robotPose.getRotation().getDegrees() - target.getTargetHeadingOffset();
                    RigidTransform2d xCorrectedRobotPose = interpolateAlongLine(robotPose, -3.75, robotPose.getRotation().getRadians(), robotPose.getRotation().getRadians());
                    RigidTransform2d correctedRobotPose = interpolateAlongLine(xCorrectedRobotPose, -8.5, robotPose.getRotation().getRadians() + Math.toRadians(90), Rotation2d.fromRadians(robotPose.getRotation().getRadians()).getRadians());
                    RigidTransform2d proposedBayPose = interpolateAlongLine(correctedRobotPose, target.getDistance(), Math.toRadians(-target.getRobotToTarget()+ robotPose.getRotation().getDegrees()), Math.toRadians(-target.getTargetHeadingOffset() + robotPose.getRotation().getDegrees()));
                    if(closestBayPose == null || proposedBayPose.getTranslation().subtract(latestFieldToVehicle.getTranslation()).norm() < closestBayPose.getTranslation().subtract(latestFieldToVehicle.getTranslation()).norm()){
                        closestBayPose = proposedBayPose;
                        closestTarget = target;
                    }
                }
                if(closestTarget != null){
                    System.out.println("Robot to target" + closestTarget.getRobotToTarget());
                    System.out.println("Target heading offset" + closestTarget.getTargetHeadingOffset());
                    System.out.println("Distance" + closestTarget.getDistance());
                }

                return closestBayPose;
            }
        }
//        Target goalTarget = targets.get(0);
//        if(true) {
//            for(int i = 1; i < targets.size(); i++){
//                if(targets.get(i).getDistance() < goalTarget.getDistance()){
//                    goalTarget = targets.get(i);
//                }
//            }
//        }
//        else {
//            for(int i = 1; i < targets.size(); i++){
//                if(targets.get(i).getMiddle() > goalTarget.getMiddle()){
//                    goalTarget = targets.get(i);
//                }
//            }
//        }
//        VisionMode.mAngle = goalTarget.getTargetHeadingOffset();
//        RigidTransform2d bayPose = new RigidTransform2d();
//        if(targets.size() != 0) {
//            RigidTransform2d robotPose = RobotState.getInstance().getFieldToVehicle(targetsTimestamp - Constants.kVisionExpectedCameraLag);
//            double goalHeading = robotPose.getRotation().getDegrees() - goalTarget.getTargetHeadingOffset();
//            RigidTransform2d xCorrectedRobotPose = interpolateAlongLine(robotPose, -2, robotPose.getRotation().getRadians(), robotPose.getRotation().getRadians());
//            RigidTransform2d correctedRobotPose = interpolateAlongLine(xCorrectedRobotPose, -8.5, robotPose.getRotation().getRadians() + Math.toRadians(90), Rotation2d.fromRadians(robotPose.getRotation().getRadians()).getRadians());
//            bayPose = interpolateAlongLine(correctedRobotPose, goalTarget.getDistance(), Math.toRadians(-goalTarget.getRobotToTarget()+ robotPose.getRotation().getDegrees()), Math.toRadians(-goalTarget.getTargetHeadingOffset() + robotPose.getRotation().getDegrees()));
//        }

        return null;
    }

    public Translation2d interpolateAlongLine(Translation2d point, double adjust, double heading) {
        double x = 0;
        double y = 0;
        x = point.x() + adjust * Math.cos(heading);
        y = point.y() + adjust * Math.sin(heading);
/*        }
        else if (heading >= Math.toRadians(90) && heading < Math.toRadians(180) ){
            x = point.getTranslation().x() + adjust * -Math.cos(heading);
            y = point.getTranslation().y() + adjust * Math.sin(heading);
        }
        else if ((heading >= Math.toRadians(180) && heading < Math.toRadians(270)) || (heading>= Math.toRadians(-180) && heading < Math.toRadians(-90))){
            x = point.getTranslation().x() + adjust * -Math.cos(heading);
            y = point.getTranslation().y() + adjust * -Math.sin(heading);
        }
        else if ((heading >= Math.toRadians(270) && heading < Math.toRadians(360)) || (heading>= Math.toRadians(-90) && heading < 0)){
            x = point.getTranslation().x() + adjust * -Math.cos(heading);
            y = point.getTranslation().y() + adjust * Math.sin(heading);
        }
        else{
            System.out.println("AAAAAAAAAAAAAAAAAAAAAAAAAAA VISION PATH.java : interpolateAlongLing : Angle didn't fit into defined quadrants");
        }
*/        return new Translation2d(x,y);

    }
    public static RigidTransform2d interpolateAlongLine(RigidTransform2d point, double adjust, double heading, double heading2) {
        double x = 0;
        double y = 0;
        x = point.getTranslation().x() + adjust * Math.cos(heading);
        y = point.getTranslation().y() + adjust * Math.sin(heading);
/*        }
        else if (heading >= Math.toRadians(90) && heading < Math.toRadians(180) ){
            x = point.getTranslation().x() + adjust * -Math.cos(heading);
            y = point.getTranslation().y() + adjust * Math.sin(heading);
        }
        else if ((heading >= Math.toRadians(180) && heading < Math.toRadians(270)) || (heading>= Math.toRadians(-180) && heading < Math.toRadians(-90))){
            x = point.getTranslation().x() + adjust * -Math.cos(heading);
            y = point.getTranslation().y() + adjust * -Math.sin(heading);
        }
        else if ((heading >= Math.toRadians(270) && heading < Math.toRadians(360)) || (heading>= Math.toRadians(-90) && heading < 0)){
            x = point.getTranslation().x() + adjust * -Math.cos(heading);
            y = point.getTranslation().y() + adjust * Math.sin(heading);
        }
        else{
            System.out.println("AAAAAAAAAAAAAAAAAAAAAAAAAAA VISION PATH.java : interpolateAlongLing : Angle didn't fit into defined quadrants");
        }
*/

        return new RigidTransform2d(new Translation2d(x,y), Rotation2d.fromRadians(heading2));

    }
}
