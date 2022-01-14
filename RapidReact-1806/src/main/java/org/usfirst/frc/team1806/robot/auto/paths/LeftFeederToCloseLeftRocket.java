package org.usfirst.frc.team1806.robot.auto.paths;

import org.usfirst.frc.team1806.robot.auto.GeneralPathAdapter;
import org.usfirst.frc.team1806.robot.path.Path;
import org.usfirst.frc.team1806.robot.path.PathContainer;
import org.usfirst.frc.team1806.robot.util.RigidTransform2d;
import org.usfirst.frc.team1806.robot.util.Rotation2d;
import org.usfirst.frc.team1806.robot.util.Translation2d;

public class LeftFeederToCloseLeftRocket implements PathContainer {
    @Override
    public Path buildPath() {

        return GeneralPathAdapter.getInstance().getLeftFeederToCloseLeftRocket();
    }
    @Override

    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(18, 290), Rotation2d.fromDegrees(0.0));
    }

    @Override
    public boolean isReversed() {
        return true;
    }
    // WAYPOINT_DATA: [{"position":{"x":18,"y":30},"speed":0,"radius":0,"comment":""},{"position":{"x":150,"y":30},"speed":60,"radius":5,"comment":""}]
    // IS_REVERSED: true
    // FILE_NAME: UntitledPath
}
