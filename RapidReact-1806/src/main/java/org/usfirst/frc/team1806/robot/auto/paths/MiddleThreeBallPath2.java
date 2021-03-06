package org.usfirst.frc.team1806.robot.auto.paths;

import java.util.ArrayList;

import org.usfirst.frc.team1806.robot.path.Path;
import org.usfirst.frc.team1806.robot.path.PathContainer;
import org.usfirst.frc.team1806.robot.path.PathBuilder;
import org.usfirst.frc.team1806.robot.path.PathBuilder.Waypoint;
import org.usfirst.frc.team1806.robot.util.RigidTransform2d;
import org.usfirst.frc.team1806.robot.util.Rotation2d;
import org.usfirst.frc.team1806.robot.util.Translation2d;

public class MiddleThreeBallPath2 implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(225,72,0,60));
        sWaypoints.add(new Waypoint(120,110,50,60));
        sWaypoints.add(new Waypoint(60,60,0,60));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(225, 72), Rotation2d.fromDegrees(160.0)); 
    }

    @Override
    public boolean isReversed() {
        return true; 
    }
	// WAYPOINT_DATA: [{"position":{"x":225,"y":72},"speed":60,"radius":0,"comment":""},{"position":{"x":120,"y":110},"speed":60,"radius":50,"comment":""},{"position":{"x":60,"y":60},"speed":60,"radius":0,"comment":""}]
	// IS_REVERSED: true
	// FILE_NAME: MiddleThreeBall2
}