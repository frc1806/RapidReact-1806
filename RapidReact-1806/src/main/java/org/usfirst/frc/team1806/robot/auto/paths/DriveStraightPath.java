package org.usfirst.frc.team1806.robot.auto.paths;

import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team1806.robot.RobotState;
import org.usfirst.frc.team1806.robot.path.Path;
import org.usfirst.frc.team1806.robot.path.PathBuilder;
import org.usfirst.frc.team1806.robot.path.PathContainer;
import org.usfirst.frc.team1806.robot.util.RigidTransform2d;
import org.usfirst.frc.team1806.robot.util.Rotation2d;
import org.usfirst.frc.team1806.robot.util.Translation2d;

import java.util.ArrayList;

public class DriveStraightPath implements PathContainer {
    private double dist;
    private double speed;

    private RigidTransform2d pose = new RigidTransform2d();
    public DriveStraightPath(double _dist, double _speed) {
        dist = _dist;
        speed = _speed;
    }


    @Override
    public Path buildPath() {
        ArrayList< PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();
        sWaypoints.add(new PathBuilder.Waypoint(pose.getTranslation().x(), pose.getTranslation().y(), 0, 0));
        RigidTransform2d p1 = interpolateAlongLine(pose, dist, pose.getRotation().getRadians(), pose.getRotation().getRadians());
        sWaypoints.add(new PathBuilder.Waypoint(p1.getTranslation().x(), p1.getTranslation().y(), 0, speed));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public RigidTransform2d getStartPose() {
        pose = RobotState.getInstance().getPredictedFieldToVehicle(Timer.getFPGATimestamp());
        return pose;
    }

    @Override
    public boolean isReversed() {
        return dist <= 0;
    }
    public Translation2d interpolateAlongLine(Translation2d point, double adjust, double heading) {
        double x = 0;
        double y = 0;

        x = point.x() + adjust * Math.cos(heading);
        y = point.y() + adjust * Math.sin(heading);

        return new Translation2d(x,y);
    }
    public RigidTransform2d interpolateAlongLine(RigidTransform2d point, double adjust, double heading, double heading2) {
        double x = 0;
        double y = 0;

        x = point.getTranslation().x() + adjust * Math.cos(heading);
        y = point.getTranslation().y() + adjust * Math.sin(heading);

        return new RigidTransform2d(new Translation2d(x,y), Rotation2d.fromRadians(heading2));

    }
}
