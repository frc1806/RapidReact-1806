package org.usfirst.frc.team1806.robot.auto;

import org.usfirst.frc.team1806.robot.path.Path;
import org.usfirst.frc.team1806.robot.path.PathBuilder;

import java.util.ArrayList;

public class GeneralPathAdapter implements PathAdapter {

    static GeneralPathAdapter pathAdapter = new GeneralPathAdapter();
    //general vars for radiusing
    static final double kLargeRadius = 45;
    static final double kModerateRadius = 30;
    static final double kNominalRadius = 20;
    static final double kSmallRadius = 10;
    static final double kSpeed = 90;

    Path mRightHab2DriveOff;
    Path mRightRocketToRightFeeder;
    Path mRightFeederToCloseRightRocket;
    Path mRightHab2ToCloseRocket;
    Path mRightHab2ToFarRocket;
    Path mRightHabDriveOff;
    Path mRightSideCloseHatchRocketToFeeder;
    Path mRightSideHAB1ToCloseHatchRocket;
    Path mRightSideHAB1ToCloseHatchRocketNoVision;

    Path mLeftHab2DriveOff;
    Path mLeftFarRocketToLeftFeeder;
    Path mLeftFeederToCloseLeftRocket;
    Path mLeftHab2ToCloseRocket;
    Path mLeftHab2ToFarRocket;
    Path mLeftHabDriveOff;
    Path mLeftSideCloseHatchRocketToFeeder;
    Path mLeftSideHAB1ToCloseHatchRocket;
    Path mLeftSideHAB1ToCloseHatchRocketNoVision;

    private GeneralPathAdapter(){
    }

    public static GeneralPathAdapter getInstance(){
        return pathAdapter;
    }

    @Override
    public void initPaths() {

    }
}
