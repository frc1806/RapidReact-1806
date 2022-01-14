package org.usfirst.frc.team1806.robot.auto.paths;

import org.usfirst.frc.team1806.robot.auto.GeneralPathAdapter;
import org.usfirst.frc.team1806.robot.path.Path;
import org.usfirst.frc.team1806.robot.path.PathBuilder;
import org.usfirst.frc.team1806.robot.path.PathContainer;
import org.usfirst.frc.team1806.robot.util.RigidTransform2d;
import org.usfirst.frc.team1806.robot.util.Rotation2d;
import org.usfirst.frc.team1806.robot.util.Translation2d;

import java.util.ArrayList;


public class RightHabDriveOff implements PathContainer {
    @Override
    public Path buildPath() {
        return GeneralPathAdapter.getInstance().getRightHabDriveOff();
    }

    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(66, 115), Rotation2d.fromDegrees(0));
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
