package org.usfirst.frc.team1806.robot.auto;

import org.usfirst.frc.team1806.robot.path.Path;

public interface PathAdapter {
    void initPaths();

    Path getRightHab2DriveOff();

    Path getRightFarRocketToRightFeeder();

    Path getRightFeederToCloseRightRocket();

    Path getRightHab2ToCloseRocket();

    Path getRightHab2ToFarRocket();

    Path getRightHabDriveOff();

    Path getRightSideCloseHatchRocketToFeeder();

    Path getRightSideHAB1ToCloseHatchRocket();

    Path getRightSideHAB1ToCloseHatchRocketNoVision();


    Path getLeftHab2DriveOff();

    Path getLeftFarRocketToLeftFeeder();

    Path getLeftFeederToCloseLeftRocket();

    Path getLeftHab2ToCloseRocket();

    Path getLeftHab2ToFarRocket();

    Path getLeftHabDriveOff();

    Path getLeftSideCloseHatchRocketToFeeder();

    Path getLeftSideHAB1ToCloseHatchRocket();

    Path getLeftSideHAB1ToCloseHatchRocketNoVision();
}
