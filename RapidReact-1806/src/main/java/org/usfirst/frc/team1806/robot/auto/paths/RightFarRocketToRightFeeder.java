package org.usfirst.frc.team1806.robot.auto.paths;

import org.usfirst.frc.team1806.robot.auto.GeneralPathAdapter;
import org.usfirst.frc.team1806.robot.path.Path;
import org.usfirst.frc.team1806.robot.path.PathBuilder;
import org.usfirst.frc.team1806.robot.path.PathBuilder.Waypoint;
import org.usfirst.frc.team1806.robot.path.PathContainer;
import org.usfirst.frc.team1806.robot.util.RigidTransform2d;
import org.usfirst.frc.team1806.robot.util.Rotation2d;
import org.usfirst.frc.team1806.robot.util.Translation2d;

import java.util.ArrayList;

public class RightFarRocketToRightFeeder implements PathContainer {

    @Override
    public Path buildPath() {

        return GeneralPathAdapter.getInstance().getRightFarRocketToRightFeeder();
    }
    @Override

    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(290, 44), Rotation2d.fromDegrees(0.0));
    }

    @Override
    public boolean isReversed() {
        return false;
    }
    // WAYPOINT_DATA: [{"position":{"x":290,"y":44},"speed":0,"radius":0,"comment":""},{"position":{"x":280,"y":38},"speed":60,"radius":5,"comment":""},{"position":{"x":230,"y":75},"speed":110,"radius":20,"comment":""},{"position":{"x":120,"y":30},"speed":110,"radius":20,"comment":""},{"position":{"x":60,"y":30},"speed":110,"radius":0,"comment":""}]
    // IS_REVERSED: false
    // FILE_NAME: UntitledPath
}
