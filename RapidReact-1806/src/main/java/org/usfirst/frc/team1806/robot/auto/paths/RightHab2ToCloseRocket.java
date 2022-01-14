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

public class RightHab2ToCloseRocket implements PathContainer {

    @Override
    public Path buildPath() {

        return GeneralPathAdapter.getInstance().getRightHab2ToCloseRocket();
    }
    @Override

    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(115, 115), Rotation2d.fromDegrees(0.0));
    }

    @Override
    public boolean isReversed() {
        return false;
    }
    // WAYPOINT_DATA: [{"position":{"x":115,"y":115},"speed":0,"radius":0,"comment":""},{"position":{"x":140,"y":100},"speed":100,"radius":10,"comment":""},{"position":{"x":140,"y":50},"speed":100,"radius":10,"comment":""},{"position":{"x":160,"y":40},"speed":100,"radius":0,"comment":""}]
    // IS_REVERSED: false
    // FILE_NAME: RightSendToCloseRocketVis
}
