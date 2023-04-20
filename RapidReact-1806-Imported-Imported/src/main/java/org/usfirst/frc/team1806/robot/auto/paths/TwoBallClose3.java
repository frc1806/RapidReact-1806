package org.usfirst.frc.team1806.robot.auto.paths;

import java.util.ArrayList;

import org.usfirst.frc.team1806.robot.path.Path;
import org.usfirst.frc.team1806.robot.path.PathContainer;
import org.usfirst.frc.team1806.robot.path.PathBuilder;
import org.usfirst.frc.team1806.robot.path.PathBuilder.Waypoint;
import org.usfirst.frc.team1806.robot.util.RigidTransform2d;
import org.usfirst.frc.team1806.robot.util.Rotation2d;
import org.usfirst.frc.team1806.robot.util.Translation2d;

public class TwoBallClose3 implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(270,180,0,60));
        sWaypoints.add(new Waypoint(180,200,90,60));
        sWaypoints.add(new Waypoint(60,65,0,60));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(270, 180), Rotation2d.fromDegrees(15.0)); 
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
	// WAYPOINT_DATA: [{"position":{"x":270,"y":180},"speed":60,"radius":0,"comment":""},{"position":{"x":180,"y":200},"speed":60,"radius":90,"comment":""},{"position":{"x":60,"y":65},"speed":60,"radius":0,"comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: ThreeBallClose3
}