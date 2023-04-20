package org.usfirst.frc.team1806.robot.auto.paths;

import java.util.ArrayList;

import org.usfirst.frc.team1806.robot.RobotState;
import org.usfirst.frc.team1806.robot.path.Path;
import org.usfirst.frc.team1806.robot.path.PathBuilder;
import org.usfirst.frc.team1806.robot.path.PathContainer;
import org.usfirst.frc.team1806.robot.path.PathBuilder.Waypoint;
import org.usfirst.frc.team1806.robot.util.RigidTransform2d;
import org.usfirst.frc.team1806.robot.util.Rotation2d;
import org.usfirst.frc.team1806.robot.util.Translation2d;

public class UpOneFootRR implements PathContainer{
	/**
	 * This class allows us to on the fly move up one foot without having to create a seperate path for a foot
	 */
	int startX, startY, distanceDriven;
	boolean isReversed;
	public UpOneFootRR(int startX, int startY, int distanceDriven, boolean isReversed) {
		this.startX = startX;
		this.startY = startY;
		this.distanceDriven = distanceDriven;
		this.isReversed = isReversed;
	}
	@Override
	public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(startX,startY,0,120));
        sWaypoints.add(new Waypoint(startX + distanceDriven,startY,0,80));
        return PathBuilder.buildPathFromWaypoints(sWaypoints);
	}

	@Override
	public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(startX, startY), Rotation2d.fromDegrees(RobotState.getInstance().getLatestFieldToVehicle().getValue().getRotation().getDegrees())); 
	}

	@Override
	public boolean isReversed() {
		// TODO Auto-generated method stub
		return isReversed;
	}

}
