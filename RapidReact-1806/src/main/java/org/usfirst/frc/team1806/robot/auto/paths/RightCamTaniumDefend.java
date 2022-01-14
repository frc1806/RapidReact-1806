package org.usfirst.frc.team1806.robot.auto.paths;

import org.usfirst.frc.team1806.robot.path.Path;
import org.usfirst.frc.team1806.robot.path.PathBuilder;
import org.usfirst.frc.team1806.robot.path.PathContainer;
import org.usfirst.frc.team1806.robot.util.RigidTransform2d;
import org.usfirst.frc.team1806.robot.util.Rotation2d;
import org.usfirst.frc.team1806.robot.util.Translation2d;

import java.util.ArrayList;

public class RightCamTaniumDefend implements PathContainer {

    @Override
    public Path buildPath() {
        ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();
        sWaypoints.add(new PathBuilder.Waypoint(66, 115, 0, 0));
        sWaypoints.add(new PathBuilder.Waypoint(67, 115, 0, 10));
        sWaypoints.add(new PathBuilder.Waypoint(116, 115, 0, 20));
        sWaypoints.add(new PathBuilder.Waypoint(196, 95, 0, 60));
        sWaypoints.add(new PathBuilder.Waypoint(274, 80, 0, 60));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override

    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(66, 115), Rotation2d.fromDegrees(0.0));
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
// WAYPOINT_DATA: [{"position":{"x":66,"y":209},"speed":0,"radius":0,"comment":""},{"position":{"x":67,"y":209},"speed":10,"radius":0,"comment":""},{"position":{"x":116,"y":209},"speed":20,"radius":0,"comment":""},{"position":{"x":196,"y":229},"speed":60,"radius":0,"comment":""},{"position":{"x":274,"y":240},"speed":60,"radius":0,"comment":""}]
// IS_REVERSED: false
// FILE_NAME: UntitledPath