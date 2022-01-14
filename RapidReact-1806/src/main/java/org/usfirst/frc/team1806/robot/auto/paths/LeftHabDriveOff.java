package org.usfirst.frc.team1806.robot.auto.paths;

import org.usfirst.frc.team1806.robot.auto.GeneralPathAdapter;
import org.usfirst.frc.team1806.robot.path.Path;
import org.usfirst.frc.team1806.robot.path.PathContainer;
import org.usfirst.frc.team1806.robot.util.RigidTransform2d;
import org.usfirst.frc.team1806.robot.util.Rotation2d;
import org.usfirst.frc.team1806.robot.util.Translation2d;


public class LeftHabDriveOff implements PathContainer {
    @Override
    public Path buildPath() {
        return GeneralPathAdapter.getInstance().getLeftHabDriveOff();
    }

    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(66, 205), Rotation2d.fromDegrees(0));
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
