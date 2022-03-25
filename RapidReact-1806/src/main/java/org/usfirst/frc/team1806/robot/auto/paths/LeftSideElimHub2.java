package org.usfirst.frc.team1806.robot.auto.paths;

import java.util.ArrayList;

import org.usfirst.frc.team1806.robot.path.Path;
import org.usfirst.frc.team1806.robot.path.PathContainer;
import org.usfirst.frc.team1806.robot.path.PathBuilder;
import org.usfirst.frc.team1806.robot.path.PathBuilder.Waypoint;
import org.usfirst.frc.team1806.robot.util.RigidTransform2d;
import org.usfirst.frc.team1806.robot.util.Rotation2d;
import org.usfirst.frc.team1806.robot.util.Translation2d;

public class LeftSideElimHub2 implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(215,285,0,0)); //Starts from LeftSideElimHub1
        sWaypoints.add(new Waypoint(180,286,10,30));
        sWaypoints.add(new Waypoint(160,260,10,30));
        sWaypoints.add(new Waypoint(155,200,10,30)); //Angling for maximum pain
        sWaypoints.add(new Waypoint(175,150,0,10)); //slow for ball

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(215, 285), Rotation2d.fromDegrees(180.0)); 
    }

    @Override
    public boolean isReversed() {
        return true; 
    }
	// WAYPOINT_DATA: [{"position":{"x":215,"y":285},"speed":0,"radius":0,"comment":"Starts from LeftSideElimHub1"},{"position":{"x":180,"y":286},"speed":30,"radius":10,"comment":""},{"position":{"x":160,"y":260},"speed":30,"radius":10,"comment":""},{"position":{"x":155,"y":200},"speed":30,"radius":10,"comment":"Angling for maximum pain"},{"position":{"x":175,"y":150},"speed":10,"radius":0,"comment":"slow for ball"}]
	// IS_REVERSED: true
	// FILE_NAME: LeftSideElimHub2
}