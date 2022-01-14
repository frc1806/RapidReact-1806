package org.usfirst.frc.team1806.robot.auto.paths;

import org.usfirst.frc.team1806.robot.RobotState;
import org.usfirst.frc.team1806.robot.path.Path;
import org.usfirst.frc.team1806.robot.path.PathBuilder;
import org.usfirst.frc.team1806.robot.path.PathContainer;
import org.usfirst.frc.team1806.robot.util.RigidTransform2d;
import org.usfirst.frc.team1806.robot.util.Translation2d;

import java.util.ArrayList;

public class WhipFromRocketRight implements PathContainer {
    @Override
    public Path buildPath() {
        ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();
        sWaypoints.add(new PathBuilder.Waypoint(190,31,0,0));
        sWaypoints.add(new PathBuilder.Waypoint(180,36,7,60));
        sWaypoints.add(new PathBuilder.Waypoint(182,60,0,60));


        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(190, 31), RobotState.getInstance().getLatestFieldToVehicle().getValue().getRotation());
    }

    @Override
    public boolean isReversed() {
        return true;
    }

}