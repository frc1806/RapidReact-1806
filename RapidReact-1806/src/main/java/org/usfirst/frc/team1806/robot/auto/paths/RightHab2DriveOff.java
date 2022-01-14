package org.usfirst.frc.team1806.robot.auto.paths;
import java.util.ArrayList;

import org.usfirst.frc.team1806.robot.auto.GeneralPathAdapter;
import org.usfirst.frc.team1806.robot.path.PathBuilder.Waypoint;
import org.usfirst.frc.team1806.robot.path.Path;
import org.usfirst.frc.team1806.robot.util.RigidTransform2d;
import org.usfirst.frc.team1806.robot.path.PathContainer;
import org.usfirst.frc.team1806.robot.path.PathBuilder;
import org.usfirst.frc.team1806.robot.util.Rotation2d;
import org.usfirst.frc.team1806.robot.util.Translation2d;

public class RightHab2DriveOff implements PathContainer {

        @Override
        public Path buildPath() {
            return GeneralPathAdapter.getInstance().getRightHab2DriveOff();
        }
        @Override

        public RigidTransform2d getStartPose() {
            return new RigidTransform2d(new Translation2d(18, 115), Rotation2d.fromDegrees(180.0));
        }

        @Override
        public boolean isReversed() {
            return true;
        }
        // WAYPOINT_DATA: [{"position":{"x":18,"y":115},"speed":0,"radius":0,"comment":""},{"position":{"x":45,"y":115},"speed":60,"radius":0,"comment":""},{"position":{"x":70,"y":115},"speed":20,"radius":0,"comment":""},{"position":{"x":83,"y":115},"speed":60,"radius":0,"comment":""},{"position":{"x":115,"y":115},"speed":20,"radius":0,"comment":""}]
        // IS_REVERSED: false
        // FILE_NAME: UntitledPath
}
