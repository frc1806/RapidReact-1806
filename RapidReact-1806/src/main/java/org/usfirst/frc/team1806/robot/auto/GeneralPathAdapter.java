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
        getRightHab2DriveOff();
        getRightHab2ToCloseRocket();
        getRightHab2ToFarRocket();
        getRightHabDriveOff();
        getRightSideCloseHatchRocketToFeeder();
        getRightSideHAB1ToCloseHatchRocket();
        getRightSideHAB1ToCloseHatchRocketNoVision();

        getLeftHab2DriveOff();
        getLeftHab2ToCloseRocket();
        getLeftHab2ToFarRocket();
        getLeftHabDriveOff();
        getLeftSideCloseHatchRocketToFeeder();
        getLeftSideHAB1ToCloseHatchRocket();
        getLeftSideHAB1ToCloseHatchRocketNoVision();
    }

    public static GeneralPathAdapter getInstance(){
        return pathAdapter;
    }

    @Override
    public Path getRightHab2DriveOff() {
        if(mRightHab2DriveOff == null){
            ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();
            sWaypoints.add(new PathBuilder.Waypoint(18,115,0,0));
            sWaypoints.add(new PathBuilder.Waypoint(63,115,0,125));
            sWaypoints.add(new PathBuilder.Waypoint(71,115,0,20));
            sWaypoints.add(new PathBuilder.Waypoint(83,115,0,125));
            sWaypoints.add(new PathBuilder.Waypoint(115,115,0,125));
            mRightHab2DriveOff = PathBuilder.buildPathFromWaypoints(sWaypoints);
            return mRightHab2DriveOff;
        }
        else{
            return mRightHab2DriveOff;
        }
    }

    @Override
    public Path getRightHab2ToCloseRocket() {
        if(mRightHab2ToCloseRocket == null){
            ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();
            sWaypoints.add(new PathBuilder.Waypoint(115,115,0,100));
            sWaypoints.add(new PathBuilder.Waypoint(140,100,10,100));
            sWaypoints.add(new PathBuilder.Waypoint(140,50,10,100));
            sWaypoints.add(new PathBuilder.Waypoint(160,40,0,100));
            mRightHab2ToCloseRocket = PathBuilder.buildPathFromWaypoints(sWaypoints);
            return mRightHab2ToCloseRocket;
        }
        else{
            return mRightHab2ToCloseRocket;
        }
    }

    @Override
    public Path getRightHab2ToFarRocket() {
        if(mRightHab2ToFarRocket == null){
            ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();
            sWaypoints.add(new PathBuilder.Waypoint(110,115,0,120));
            sWaypoints.add(new PathBuilder.Waypoint(135,115,15,120));
            sWaypoints.add(new PathBuilder.Waypoint(273,30,15,120));
            sWaypoints.add(new PathBuilder.Waypoint(285,40,0,120));
            mRightHab2ToFarRocket = PathBuilder.buildPathFromWaypoints(sWaypoints);
            return mRightHab2ToFarRocket;
        }
        else{
            return mRightHab2ToFarRocket;
        }
    }

    @Override
    public Path getRightHabDriveOff() {
        if(mRightHabDriveOff == null){
            ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();
            sWaypoints.add(new PathBuilder.Waypoint(66,115,0,0));
            sWaypoints.add(new PathBuilder.Waypoint(67,115,0,10));
            sWaypoints.add(new PathBuilder.Waypoint(116,115,0,20));
            mRightHabDriveOff = PathBuilder.buildPathFromWaypoints(sWaypoints);
            return mRightHabDriveOff;
        }
        else{
            return mRightHabDriveOff;
        }
    }

    @Override
    public Path getRightSideCloseHatchRocketToFeeder() {
        if(mRightSideCloseHatchRocketToFeeder == null){
            ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();
            sWaypoints.add(new PathBuilder.Waypoint(190,30,0,0));
            sWaypoints.add(new PathBuilder.Waypoint(60,30,0,60));
            mRightSideCloseHatchRocketToFeeder = PathBuilder.buildPathFromWaypoints(sWaypoints);
            return mRightSideCloseHatchRocketToFeeder;
        }
        else{
            return mRightSideCloseHatchRocketToFeeder;
        }
    }

    @Override
    public Path getRightSideHAB1ToCloseHatchRocket() {
        if(mRightSideHAB1ToCloseHatchRocket == null){
            ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();
            sWaypoints.add(new PathBuilder.Waypoint(66,115,0,0));
            sWaypoints.add(new PathBuilder.Waypoint(67,115,0,10));
            sWaypoints.add(new PathBuilder.Waypoint(116,115,0,20));
            sWaypoints.add(new PathBuilder.Waypoint(136,74,10,50));
            sWaypoints.add(new PathBuilder.Waypoint(166,50,0,80));
            mRightSideHAB1ToCloseHatchRocket = PathBuilder.buildPathFromWaypoints(sWaypoints);
            return mRightSideHAB1ToCloseHatchRocket;
        }
        else{
            return mRightSideHAB1ToCloseHatchRocket;
        }
    }

    @Override
    public Path getRightSideHAB1ToCloseHatchRocketNoVision() {
        if(mRightSideHAB1ToCloseHatchRocketNoVision == null){
            ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();
            sWaypoints.add(new PathBuilder.Waypoint(116,118,0,0));
            sWaypoints.add(new PathBuilder.Waypoint(117,118,0,60));
            sWaypoints.add(new PathBuilder.Waypoint(136,100,6,60));
            sWaypoints.add(new PathBuilder.Waypoint(156,47,6,50));
            sWaypoints.add(new PathBuilder.Waypoint(176,37,0,50));
            mRightSideHAB1ToCloseHatchRocketNoVision = PathBuilder.buildPathFromWaypoints(sWaypoints);
            return mRightSideHAB1ToCloseHatchRocketNoVision;
        }
        else{
            return mRightSideHAB1ToCloseHatchRocketNoVision;
        }
    }

    @Override
    public Path getLeftHab2DriveOff() {

        if(mLeftHab2DriveOff == null){
            ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();
            sWaypoints.add(new PathBuilder.Waypoint(18,205,0,0));
            sWaypoints.add(new PathBuilder.Waypoint(63,205,0,125));
            sWaypoints.add(new PathBuilder.Waypoint(71,205,0,20));
            sWaypoints.add(new PathBuilder.Waypoint(83,205,0,125));
            sWaypoints.add(new PathBuilder.Waypoint(115,205,0,125));
            mLeftHab2DriveOff = PathBuilder.buildPathFromWaypoints(sWaypoints);
            return mLeftHab2DriveOff;
        }
        else{
            return mLeftHab2DriveOff;
        }
    }

    @Override
    public Path getLeftHab2ToCloseRocket() {
        if(mLeftHab2ToCloseRocket == null){
            ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();
            sWaypoints.add(new PathBuilder.Waypoint(115,205,0,100));
            sWaypoints.add(new PathBuilder.Waypoint(140,220,10,100));
            sWaypoints.add(new PathBuilder.Waypoint(140,270,10,100));
            sWaypoints.add(new PathBuilder.Waypoint(160,280,0,100));
            mLeftHab2ToCloseRocket = PathBuilder.buildPathFromWaypoints(sWaypoints);
            return mLeftHab2ToCloseRocket;
        }
        else{
            return mLeftHab2ToCloseRocket;
        }
    }

    @Override
    public Path getLeftHab2ToFarRocket() {
        if(mLeftHab2ToFarRocket == null){
            ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();
            sWaypoints.add(new PathBuilder.Waypoint(110,205,0,120));
            sWaypoints.add(new PathBuilder.Waypoint(135,205,15,120));
            sWaypoints.add(new PathBuilder.Waypoint(273,290,15,120));
            sWaypoints.add(new PathBuilder.Waypoint(285,280,0,120));
            mLeftHab2ToFarRocket = PathBuilder.buildPathFromWaypoints(sWaypoints);
            return mLeftHab2ToFarRocket;
        }
        else{
            return mLeftHab2ToFarRocket;
        }
    }

    @Override
    public Path getLeftHabDriveOff() {
        if(mLeftHabDriveOff == null){
            ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();
            sWaypoints.add(new PathBuilder.Waypoint(66,205,0,0));
            sWaypoints.add(new PathBuilder.Waypoint(67,205,0,10));
            sWaypoints.add(new PathBuilder.Waypoint(116,205,0,20));
            mLeftHabDriveOff = PathBuilder.buildPathFromWaypoints(sWaypoints);
            return mLeftHabDriveOff;
        }
        else{
            return mLeftHabDriveOff;
        }
    }

    @Override
    public Path getLeftSideCloseHatchRocketToFeeder() {
        if(mLeftSideCloseHatchRocketToFeeder == null){
            ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();
            sWaypoints.add(new PathBuilder.Waypoint(190,290,0,0));
            sWaypoints.add(new PathBuilder.Waypoint(60,290,0,60));
            mLeftSideCloseHatchRocketToFeeder = PathBuilder.buildPathFromWaypoints(sWaypoints);
            return mLeftSideCloseHatchRocketToFeeder;
        }
        else{
            return mLeftSideCloseHatchRocketToFeeder;
        }
    }

    @Override
    public Path getLeftSideHAB1ToCloseHatchRocket() {
        if(mLeftSideHAB1ToCloseHatchRocket == null){
            ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();
            sWaypoints.add(new PathBuilder.Waypoint(66,205,0,0));
            sWaypoints.add(new PathBuilder.Waypoint(67,205,0,10));
            sWaypoints.add(new PathBuilder.Waypoint(116,205,0,20));
            sWaypoints.add(new PathBuilder.Waypoint(136,246,10,50));
            sWaypoints.add(new PathBuilder.Waypoint(166,270,0,80));
            mLeftSideHAB1ToCloseHatchRocket = PathBuilder.buildPathFromWaypoints(sWaypoints);
            return mLeftSideHAB1ToCloseHatchRocket;
        }
        else{
            return mLeftSideHAB1ToCloseHatchRocket;
        }
    }

    @Override
    public Path getRightFeederToCloseRightRocket() {
        if(mRightFeederToCloseRightRocket == null){
            ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();
            sWaypoints.add(new PathBuilder.Waypoint(18,30,0,0));
            sWaypoints.add(new PathBuilder.Waypoint(150,30,5,110));
            mRightFeederToCloseRightRocket = PathBuilder.buildPathFromWaypoints(sWaypoints);
            return mRightFeederToCloseRightRocket;
        }
        else{
            return mRightFeederToCloseRightRocket;
        }
    }

    @Override
    public Path getLeftFeederToCloseLeftRocket() {
        if(mLeftFeederToCloseLeftRocket == null){
            ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();
            sWaypoints.add(new PathBuilder.Waypoint(18,290,0,0));
            sWaypoints.add(new PathBuilder.Waypoint(150,290,5,110));
            mLeftFeederToCloseLeftRocket = PathBuilder.buildPathFromWaypoints(sWaypoints);
            return mLeftFeederToCloseLeftRocket;
        }
        else{
            return mLeftFeederToCloseLeftRocket;
        }
    }

    @Override
    public Path getRightFarRocketToRightFeeder() {
        if(mRightRocketToRightFeeder == null){
            ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();
            sWaypoints.add(new PathBuilder.Waypoint(290,44,0,0));
            sWaypoints.add(new PathBuilder.Waypoint(280,38,5,120));
            sWaypoints.add(new PathBuilder.Waypoint(230,75,20,120));
            sWaypoints.add(new PathBuilder.Waypoint(120,30,20,120));
            sWaypoints.add(new PathBuilder.Waypoint(90,30,20,120));
            sWaypoints.add(new PathBuilder.Waypoint(60,30,0,60));
            mRightRocketToRightFeeder = PathBuilder.buildPathFromWaypoints(sWaypoints);
            return mRightRocketToRightFeeder;
        }
        else{
            return mRightRocketToRightFeeder;
        }
    }

    @Override
    public Path getLeftFarRocketToLeftFeeder() {
        if(mLeftFarRocketToLeftFeeder == null){
            ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();
            sWaypoints.add(new PathBuilder.Waypoint(290,276,0,0));
            sWaypoints.add(new PathBuilder.Waypoint(280,282,5,120));
            sWaypoints.add(new PathBuilder.Waypoint(230,245,20,120));
            sWaypoints.add(new PathBuilder.Waypoint(120,290,20,120));
            sWaypoints.add(new PathBuilder.Waypoint(90,290,20,120));
            sWaypoints.add(new PathBuilder.Waypoint(60,290,0,60));
            mLeftFarRocketToLeftFeeder = PathBuilder.buildPathFromWaypoints(sWaypoints);
            return mLeftFarRocketToLeftFeeder;
        }
        else{
            return mLeftFarRocketToLeftFeeder;
        }
    }

    @Override
    public Path getLeftSideHAB1ToCloseHatchRocketNoVision() {
        if(mLeftSideHAB1ToCloseHatchRocketNoVision == null){
            ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();
            sWaypoints.add(new PathBuilder.Waypoint(116,202,0,0));
            sWaypoints.add(new PathBuilder.Waypoint(117,202,0,60));
            sWaypoints.add(new PathBuilder.Waypoint(136,202,6,60));
            sWaypoints.add(new PathBuilder.Waypoint(156,273,6,50));
            sWaypoints.add(new PathBuilder.Waypoint(176,283,0,50));
            mLeftSideHAB1ToCloseHatchRocketNoVision = PathBuilder.buildPathFromWaypoints(sWaypoints);
            return mLeftSideHAB1ToCloseHatchRocketNoVision;
        }
        else{
            return mLeftSideHAB1ToCloseHatchRocketNoVision;
        }
    }

    @Override
    public void initPaths() {

    }
}
