package org.usfirst.frc.team1806.robot.auto.paths;

import org.usfirst.frc.team1806.robot.RobotState;
import org.usfirst.frc.team1806.robot.path.Path;
import org.usfirst.frc.team1806.robot.path.PathBuilder;
import org.usfirst.frc.team1806.robot.path.PathContainer;
import org.usfirst.frc.team1806.robot.util.RigidTransform2d;
import org.usfirst.frc.team1806.robot.util.Translation2d;

import java.util.ArrayList;

public class PIDTestPath implements PathContainer {
    @Override
    public Path buildPath() {
        ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();
        double pathSpeed = 70;
        sWaypoints.add(new PathBuilder.Waypoint(115,116,0,0));
        sWaypoints.add(new PathBuilder.Waypoint(116,116,0,pathSpeed));
        sWaypoints.add(new PathBuilder.Waypoint(150,100,10,pathSpeed));
        sWaypoints.add(new PathBuilder.Waypoint(180,100,10,pathSpeed));
        sWaypoints.add(new PathBuilder.Waypoint(200,80,10,pathSpeed));
        sWaypoints.add(new PathBuilder.Waypoint(220,80,0,pathSpeed));
        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(115, 116), RobotState.getInstance().getLatestFieldToVehicle().getValue().getRotation());
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
