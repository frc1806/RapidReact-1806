package org.usfirst.frc.team1806.robot.auto.modes;
import org.usfirst.frc.team1806.robot.RobotState;
import org.usfirst.frc.team1806.robot.auto.actions.DriveToStall;
import org.usfirst.frc.team1806.robot.auto.actions.actionUtil.*;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeBase;
import org.usfirst.frc.team1806.robot.auto.modes.modesUtil.AutoModeEndedException;
import org.usfirst.frc.team1806.robot.subsystems.DriveTrainSubsystem;

public class VisionMode extends AutoModeBase {
    public static double mAngle = -1000;
    boolean done = false;
    @Override
    protected void routine() throws AutoModeEndedException {
        //TODO: Rewrite
        done = false;
        int testX = 72;
        int testY = 12;
        double testAngle = 0;
        DriveTrainSubsystem.getInstance().stop();
        //VisionPath visPath = new VisionPath(new RigidTransform2d(new Translation2d(testX, testY), Rotation2d.fromRadians(testAngle)));
        runAction(new SwitchToLowPID());
        //runAction(new DrivePathAction(visPath));
        System.out.println("heading after path " + RobotState.getInstance().getLatestFieldToVehicle().getValue().getRotation().getDegrees());
       // runAction(new TurnTowardsPoint(visPath.bayyPose.getTranslation()));
        System.out.println("heading after TTP " + RobotState.getInstance().getLatestFieldToVehicle().getValue().getRotation().getDegrees());
        runAction(new DriveToStall());
        done = true;
       // runAction(new WaitAction(15));

    }

    public boolean getIsDone(){
        return done;
    }
}
