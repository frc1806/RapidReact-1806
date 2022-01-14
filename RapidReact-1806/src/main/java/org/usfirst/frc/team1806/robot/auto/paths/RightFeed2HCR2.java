package org.usfirst.frc.team1806.robot.auto.paths;

import org.usfirst.frc.team1806.robot.path.Path;
import edu.wpi.first.wpilibj.DriverStation;
import org.usfirst.frc.team1806.robot.path.PathBuilder;
import org.usfirst.frc.team1806.robot.path.PathBuilder.Waypoint;
import org.usfirst.frc.team1806.robot.path.PathContainer;
import org.usfirst.frc.team1806.robot.util.RigidTransform2d;
import org.usfirst.frc.team1806.robot.util.Rotation2d;
import org.usfirst.frc.team1806.robot.util.Translation2d;

import java.util.ArrayList;



public class RightFeed2HCR2 implements PathContainer{


    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();

        sWaypoints.add(new Waypoint(19,27,0,0));
        sWaypoints.add(new Waypoint(89,27,20,115));
        sWaypoints.add(new Waypoint(139,47,20,115));
        sWaypoints.add(new Waypoint(235,88,20,115));
        sWaypoints.add(new Waypoint(284,88,0,115));


        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(19, 27), Rotation2d.fromDegrees(180));
    }


    @Override
    public boolean isReversed() {
        return true;
    }
    // WAYPOINT_DATA: [{"position":{"x":19,"y":27},"speed":0,"radius":0,"comment":""},{"position":{"x":100,"y":50},"speed":80,"radius":10,"comment":""},{"position":{"x":150,"y":90},"speed":80,"radius":10,"comment":""},{"position":{"x":150,"y":160},"speed":60,"radius":0,"comment":""}]
    // IS_REVERSED: false
    // FILE_NAME: UntitledPath

}