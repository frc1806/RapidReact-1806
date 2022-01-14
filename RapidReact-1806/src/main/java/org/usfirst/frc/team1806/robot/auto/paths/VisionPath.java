package org.usfirst.frc.team1806.robot.auto.paths;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotState;
import org.usfirst.frc.team1806.robot.Vision.VisionServer;
import org.usfirst.frc.team1806.robot.auto.modes.VisionMode;
import org.usfirst.frc.team1806.robot.path.Path;
import org.usfirst.frc.team1806.robot.path.PathBuilder;
import org.usfirst.frc.team1806.robot.path.PathContainer;
import org.usfirst.frc.team1806.robot.subsystems.DriveTrainSubsystem;
import org.usfirst.frc.team1806.robot.util.RigidTransform2d;
import org.usfirst.frc.team1806.robot.util.Rotation2d;
import org.usfirst.frc.team1806.robot.util.Target;
import org.usfirst.frc.team1806.robot.util.Translation2d;

import java.util.ArrayList;

public class VisionPath implements PathContainer {
    public enum BayLocation {
        CARGO_SHIP_SIDE_1_AUDIANCE(new RigidTransform2d(new Translation2d(0,0), Rotation2d.fromDegrees(0))),
        CARGO_SHIP_SIDE_2_AUDIANCE(new RigidTransform2d(new Translation2d(), new Rotation2d())),
        CARGO_SHIP_SIDE_3_AUDIANCE(new RigidTransform2d(new Translation2d(), new Rotation2d())),
        CARGO_SHIP_SIDE_1_NONAUDIANCE(new RigidTransform2d(new Translation2d(), new Rotation2d())),
        CARGO_SHIP_SIDE_2_NONAUDIANCE(new RigidTransform2d(new Translation2d(), new Rotation2d())),
        CARGO_SHIP_SIDE_3_NONAUDIANCE(new RigidTransform2d(new Translation2d(), new Rotation2d())),
        CARGO_SHIP_FRONT_AUDIANCE(new RigidTransform2d(new Translation2d(), new Rotation2d())),
        CARGO_SHIP_FRONT_NONAUDIANCE(new RigidTransform2d(new Translation2d(), new Rotation2d()));
        public RigidTransform2d getBayCoordinates() {
            return bayCoordinates;
        }

        private RigidTransform2d bayCoordinates;
        BayLocation(RigidTransform2d bayCoordinates) {
            this.bayCoordinates = bayCoordinates;
        }
    }

    BayLocation trackedBay = BayLocation.CARGO_SHIP_FRONT_AUDIANCE;
    RigidTransform2d odometry;
    public final int speed = 90;
    private VisionServer mVisionServer = VisionServer.getInstance();
    ArrayList<Target> targets;
    public RigidTransform2d bayyPose;
    public Double targetsTimestamp;
    public VisionPath(RigidTransform2d odo) {
        odometry = odo;
    }
    @Override
    public Path buildPath() {
        double fps = 1;
        ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();
        if(false) {
            RigidTransform2d roboPose = generateTemporaryVisionPose();
            sWaypoints.add(new PathBuilder.Waypoint(roboPose.getTranslation().x(), roboPose.getTranslation().y(), 0, speed));
            sWaypoints.add(new PathBuilder.Waypoint(0, roboPose.getTranslation().y()- 4, 0, speed));
            sWaypoints.add(new PathBuilder.Waypoint(0, 20, 0, speed));
            sWaypoints.add(new PathBuilder.Waypoint(0, 18, 0, speed));
        }
        else {
            targets = mVisionServer.getTargets();

            //System.out.println("SSS target info" + targets.get(0).getDistance());
            targetsTimestamp = mVisionServer.getTargetsTimestamp();
            if(Timer.getFPGATimestamp() - targetsTimestamp < 1.000){
                RigidTransform2d roboPose = RobotState.getInstance().getLatestFieldToVehicle().getValue();
                System.out.println("Field to vehicle: (" + roboPose.getTranslation().x() + ", " + roboPose.getTranslation().y() + ")");
                //RigidTransform2d bayyPose = new RigidTransform2d(new Translation2d(roboPose.getTranslation().x() - odometry.getTranslation().x(), roboPose.getTranslation().y() + odometry.getTranslation().y()), Rotation2d.fromDegrees(roboPose.getRotation().getDegrees() + odometry.getRotation().getDegrees()));
                bayyPose = generateBayVisionPoseFromODO();
                double robotEndDistanceFromTarget = -13.75;
                double robotDistanceToEnd = -interpolateAlongLine(bayyPose.getTranslation(), robotEndDistanceFromTarget, bayyPose.getRotation().getRadians()).subtract(roboPose.getTranslation()).norm();
                double secondFromEndDistance = Math.min((robotDistanceToEnd+robotEndDistanceFromTarget) * 0.70, robotEndDistanceFromTarget - 4);
                double firstFromEndDistance = Math.min((robotDistanceToEnd+robotEndDistanceFromTarget) * 0.30, robotEndDistanceFromTarget - 2);
                DriveTrainSubsystem driveTrain = DriveTrainSubsystem.getInstance();
                double averageSpeed = (driveTrain.getLeftVelocityInchesPerSec() + driveTrain.getRightVelocityInchesPerSec()) / 2;
                sWaypoints.add(new PathBuilder.Waypoint(interpolateAlongLine(roboPose.getTranslation(), 0, roboPose.getRotation().getRadians()), 0, Math.max(10, averageSpeed)));
                if(robotDistanceToEnd > 40){

                    //sWaypoints.add(new PathBuilder.Waypoint(interpolateAlongLine(roboPose.getTranslation(), 2, roboPose.getRotation().getRadians()), 0, Math.max(10, averageSpeed)));
                    Translation2d nonSideInterpolSecondPoint = interpolateAlongLine(bayyPose.getTranslation(), secondFromEndDistance,  bayyPose.getRotation().getRadians());
                    sWaypoints.add(new PathBuilder.Waypoint(interpolateAlongLine(nonSideInterpolSecondPoint,roboPose.getTranslation().subtract(nonSideInterpolSecondPoint).norm()*0.25 , roboPose.getRotation().getRadians()),Math.abs(firstFromEndDistance - robotEndDistanceFromTarget) *0.25, speed));

                }
                sWaypoints.add(new PathBuilder.Waypoint(interpolateAlongLine(bayyPose.getTranslation(), firstFromEndDistance,  bayyPose.getRotation().getRadians()),Math.abs(firstFromEndDistance - robotEndDistanceFromTarget) *0.5, speed));
                //sWaypoints.add(new PathBuilder.Waypoint(interpolateAlongLine(interpolateAlongLine(bayyPose.getTranslation(), -27, bayyPose.getRotation().getRadians()), 3, roboPose.getRotation().getRadians()), 0, speed));
                //sWaypoints.add(new PathBuilder.Waypoint(interpolateAlongLine(bayyPose, -26, bayyPose.getRotation().getRadians()), 0, speed));
                //sWaypoints.add(new PathBuilder.Waypoint(interpolateAlongLine(bayyPose, -23, bayyPose.getRotation().getRadians()), 0, speed));
                //sWaypoints.add(new PathBuilder.Waypoint(interpolateAlongLine(bayyPose, -21, bayyPose.getRotation().getRadians()), 0, speed));
                //sWaypoints.add(new PathBuilder.Waypoint(interpolateAlongLine(bayyPose, -19, bayyPose.getRotation().getRadians()), 0, speed));
                //sWaypoints.add(new PathBuilder.Waypoint(interpolateAlongLine(bayyPose.getTranslation(), -13.5, bayyPose.getRotation().getRadians()), 0, speed));
                sWaypoints.add(new PathBuilder.Waypoint(interpolateAlongLine(bayyPose.getTranslation(), robotEndDistanceFromTarget, bayyPose.getRotation().getRadians()), 0, speed));
            }

        }
        return PathBuilder.buildPathFromWaypoints(sWaypoints);

    }
    public RigidTransform2d generateTemporaryVisionPose() {
        double dist = SmartDashboard.getNumber("Vdistance", 0);
        double angle = SmartDashboard.getNumber("Vangle", 0);

        double x = dist * Math.sin(angle);
        double y = dist * Math.cos(angle);

        return new RigidTransform2d(new Translation2d(x,y), Rotation2d.fromDegrees(angle));
    }
    public RigidTransform2d generateBayVisionPoseFromODO() {
        if(targets != null){
            if (!targets.isEmpty()){
                RigidTransform2d closestBayPose = new RigidTransform2d(new Translation2d(10000, 10000), Rotation2d.fromDegrees(0.0));
                RigidTransform2d latestFieldToVehicle = RobotState.getInstance().getLatestFieldToVehicle().getValue();
                Target closestTarget = null;
                for(Target target: targets){
                    RigidTransform2d robotPose = RobotState.getInstance().getFieldToVehicle(targetsTimestamp - Constants.kVisionExpectedCameraLag);
                    double goalHeading = robotPose.getRotation().getDegrees() - target.getTargetHeadingOffset();
                    RigidTransform2d xCorrectedRobotPose = interpolateAlongLine(robotPose, -5.5, robotPose.getRotation().getRadians(), robotPose.getRotation().getRadians());
                    RigidTransform2d correctedRobotPose = interpolateAlongLine(xCorrectedRobotPose, -8.5, robotPose.getRotation().getRadians() + Math.toRadians(90), Rotation2d.fromRadians(robotPose.getRotation().getRadians()).getRadians());
                    RigidTransform2d proposedBayPose = interpolateAlongLine(correctedRobotPose, target.getDistance(), Math.toRadians(-target.getRobotToTarget()+ robotPose.getRotation().getDegrees()), Math.toRadians(-target.getTargetHeadingOffset() + robotPose.getRotation().getDegrees()));
                    if(closestBayPose == null || proposedBayPose.getTranslation().subtract(latestFieldToVehicle.getTranslation()).norm() < closestBayPose.getTranslation().subtract(latestFieldToVehicle.getTranslation()).norm()){
                        closestBayPose = proposedBayPose;
                        closestTarget = target;
                    }
                }
                if(closestTarget != null){
                    System.out.println("IT'S RUNNING");
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
    public RigidTransform2d interpolateAlongLine(RigidTransform2d point, double adjust, double heading, double heading2) {
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
    @Override
    public RigidTransform2d getStartPose() {
        return RobotState.getInstance().getLatestFieldToVehicle().getValue();
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
