package org.usfirst.frc.team1806.robot.auto.paths;

import java.util.ArrayList;

import org.usfirst.frc.team1806.robot.path.Path;
import org.usfirst.frc.team1806.robot.path.PathContainer;
import org.usfirst.frc.team1806.robot.path.PathBuilder;
import org.usfirst.frc.team1806.robot.path.PathBuilder.Waypoint;
import org.usfirst.frc.team1806.robot.util.RigidTransform2d;
import org.usfirst.frc.team1806.robot.util.Rotation2d;
import org.usfirst.frc.team1806.robot.util.Translation2d;

public class Right3Ball2 implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(300,36,0,0));
        sWaypoints.add(new Waypoint(250,55,5,25));
        sWaypoints.add(new Waypoint(222,67,0,25));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(300, 36), Rotation2d.fromDegrees(-21.9)); 
    }

    @Override
    public boolean isReversed() {
        return true; 
    }
	// WAYPOINT_DATA: [{"position":{"x":300,"y":36},"speed":0,"radius":0,"comment":""},{"position":{"x":250,"y":55},"speed":30,"radius":0,"comment":""},{"position":{"x":222,"y":67},"speed":10,"radius":0,"comment":""}]
	// IS_REVERSED: false
	// FILE_NAME: Right3Ball2
}