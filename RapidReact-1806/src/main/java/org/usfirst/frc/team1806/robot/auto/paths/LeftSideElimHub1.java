package org.usfirst.frc.team1806.robot.auto.paths;

import java.util.ArrayList;

import org.usfirst.frc.team1806.robot.path.Path;
import org.usfirst.frc.team1806.robot.path.PathContainer;
import org.usfirst.frc.team1806.robot.path.PathBuilder;
import org.usfirst.frc.team1806.robot.path.PathBuilder.Waypoint;
import org.usfirst.frc.team1806.robot.util.RigidTransform2d;
import org.usfirst.frc.team1806.robot.util.Rotation2d;
import org.usfirst.frc.team1806.robot.util.Translation2d;

public class LeftSideElimHub1 implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(215,233,0,0)); //Starts from SixBallPath1
        sWaypoints.add(new Waypoint(200,242,5,30));
        sWaypoints.add(new Waypoint(180,280,10,30));
        sWaypoints.add(new Waypoint(205,286,0,30)); //Angling for maximum pain
        sWaypoints.add(new Waypoint(215,285,0,10)); //Slow down for ball

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(215, 233), Rotation2d.fromDegrees(180.0)); 
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
	// WAYPOINT_DATA: [{"position":{"x":215,"y":233},"speed":0,"radius":0,"comment":"Starts from SixBallPath1"},{"position":{"x":200,"y":242},"speed":30,"radius":5,"comment":""},{"position":{"x":180,"y":280},"speed":30,"radius":10,"comment":""},{"position":{"x":205,"y":286},"speed":30,"radius":0,"comment":"Angling for maximum pain"},{"position":{"x":215,"y":285},"speed":10,"radius":0,"comment":"Slow down for ball"}]
	// IS_REVERSED: false
	// FILE_NAME: LeftSideElimHub1
}