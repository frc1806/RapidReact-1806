package org.usfirst.frc.team1806.robot.auto.paths;

import java.util.ArrayList;

import org.usfirst.frc.team1806.robot.path.Path;
import org.usfirst.frc.team1806.robot.path.PathContainer;
import org.usfirst.frc.team1806.robot.path.PathBuilder;
import org.usfirst.frc.team1806.robot.path.PathBuilder.Waypoint;
import org.usfirst.frc.team1806.robot.util.RigidTransform2d;
import org.usfirst.frc.team1806.robot.util.Rotation2d;
import org.usfirst.frc.team1806.robot.util.Translation2d;

public class TwoBallClose2 implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(215,230,0,0));
        sWaypoints.add(new Waypoint(245,190,15,60));
        sWaypoints.add(new Waypoint(270,180,0,60));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(215, 230), Rotation2d.fromDegrees(230.0)); 
    }

    @Override
    public boolean isReversed() {
        return true; 
    }
	// WAYPOINT_DATA: [{"position":{"x":215,"y":230},"speed":0,"radius":0,"comment":""},{"position":{"x":245,"y":190},"speed":60,"radius":30,"comment":""},{"position":{"x":270,"y":180},"speed":60,"radius":0,"comment":""}]
	// IS_REVERSED: true
	// FILE_NAME: ThreeBallClose2
}