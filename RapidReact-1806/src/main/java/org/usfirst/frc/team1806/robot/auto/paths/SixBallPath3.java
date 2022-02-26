package org.usfirst.frc.team1806.robot.auto.paths;

import java.util.ArrayList;

import org.usfirst.frc.team1806.robot.path.Path;
import org.usfirst.frc.team1806.robot.path.PathContainer;
import org.usfirst.frc.team1806.robot.path.PathBuilder;
import org.usfirst.frc.team1806.robot.path.PathBuilder.Waypoint;
import org.usfirst.frc.team1806.robot.util.RigidTransform2d;
import org.usfirst.frc.team1806.robot.util.Rotation2d;
import org.usfirst.frc.team1806.robot.util.Translation2d;

public class SixBallPath3 implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(190,150,0,60));
        sWaypoints.add(new Waypoint(190,85,10,60));
        sWaypoints.add(new Waypoint(230,20,10,60));
        sWaypoints.add(new Waypoint(280,20,0,60));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(190, 150), Rotation2d.fromDegrees(180.0)); 
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
	// WAYPOINT_DATA: [{"position":{"x":190,"y":150},"speed":60,"radius":0,"comment":""},{"position":{"x":190,"y":85},"speed":60,"radius":10,"comment":""},{"position":{"x":230,"y":20},"speed":60,"radius":10,"comment":""},{"position":{"x":280,"y":20},"speed":60,"radius":0,"comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: SixBallPath3
}