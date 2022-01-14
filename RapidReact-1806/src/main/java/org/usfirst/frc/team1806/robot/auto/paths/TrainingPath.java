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



public class TrainingPath implements PathContainer{

    /*
    I completely redid this, you guys need to make your path classes implement path container
                -Dillon
     */
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();

        sWaypoints.add(new Waypoint(40,45,0,0));
        sWaypoints.add(new Waypoint(70,60,5,60));
        sWaypoints.add(new Waypoint(140,79,20,60));
        sWaypoints.add(new Waypoint(220,79,15,60));
        sWaypoints.add(new Waypoint(275,75,13,60));
        sWaypoints.add(new Waypoint(276,90,0,55));
        sWaypoints.add(new Waypoint(284,110,0,60));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(40, 45), Rotation2d.fromDegrees(0));
    }


    @Override
    public boolean isReversed() {
        return false;
    }
    // WAYPOINT_DATA: [{"position":{"x":50,"y":50},"speed":0,"radius":0,"comment":""},{"position":{"x":100,"y":50},"speed":80,"radius":10,"comment":""},{"position":{"x":150,"y":90},"speed":80,"radius":10,"comment":""},{"position":{"x":150,"y":160},"speed":60,"radius":0,"comment":""}]
    // IS_REVERSED: false
    // FILE_NAME: UntitledPath

}
