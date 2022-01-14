package org.usfirst.frc.team1806.robot.auto.paths;

import org.usfirst.frc.team1806.robot.RobotState;
import org.usfirst.frc.team1806.robot.auto.GeneralPathAdapter;
import org.usfirst.frc.team1806.robot.path.Path;
import org.usfirst.frc.team1806.robot.path.PathContainer;
import org.usfirst.frc.team1806.robot.util.RigidTransform2d;
import org.usfirst.frc.team1806.robot.util.Translation2d;

public class LeftSideHAB1ToCloseHatchRocket implements PathContainer {

    @Override
    public Path buildPath() {
        return GeneralPathAdapter.getInstance().getLeftSideHAB1ToCloseHatchRocket();
    }
    @Override

    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(66, 205), RobotState.getInstance().getLatestFieldToVehicle().getValue().getRotation());
    }

    @Override
    public boolean isReversed() {
        return false;
    }
    // WAYPOINT_DATA: [{"position":{"x":67,"y":115},"speed":0,"radius":0,"comment":""},{"position":{"x":125,"y":115},"speed":60,"radius":10,"comment":""},{"position":{"x":160,"y":65},"speed":60,"radius":0,"comment":""}]
    // IS_REVERSED: false
    // FILE_NAME: UntitledPath
}
