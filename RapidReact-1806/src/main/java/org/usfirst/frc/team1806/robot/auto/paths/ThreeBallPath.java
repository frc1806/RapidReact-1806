package org.usfirst.frc.team1806.robot.auto.paths;

import java.util.ArrayList;

import org.usfirst.frc.team1806.robot.path.Path;
import org.usfirst.frc.team1806.robot.path.PathBuilder;
import org.usfirst.frc.team1806.robot.path.PathContainer;
import org.usfirst.frc.team1806.robot.path.PathBuilder.Waypoint;
import org.usfirst.frc.team1806.robot.util.RigidTransform2d;
import org.usfirst.frc.team1806.robot.util.Rotation2d;
import org.usfirst.frc.team1806.robot.util.Translation2d;

public class ThreeBallPath implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(305,37,0,0));
        sWaypoints.add(new Waypoint(275,100,30,60));
        sWaypoints.add(new Waypoint(225,85,0,60));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(305, 37), Rotation2d.fromDegrees(180.0)); 
    }

    @Override
    public boolean isReversed() {
        return true; 
    }
	// WAYPOINT_DATA: [{"position":{"x":305,"y":37},"speed":0,"radius":0,"comment":""},{"position":{"x":275,"y":100},"speed":60,"radius":30,"comment":""},{"position":{"x":225,"y":85},"speed":60,"radius":0,"comment":""}]
	// IS_REVERSED: true
	// FILE_NAME: ThreeBallPath
}