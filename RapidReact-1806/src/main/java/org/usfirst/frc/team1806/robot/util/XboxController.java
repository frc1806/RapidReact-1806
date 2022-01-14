package org.usfirst.frc.team1806.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;

/*
 * The XboxController class is a wrapper of the joystick class,
 * auto applies deadzones and makes nice methods for us
 */
public class XboxController extends Joystick {
	private static DriverStation mDS;
	private final int mPort;
	public Timer rumbleTimer;

	// defaults the deadzone to .15 if a value is not passed in as a parameter
	private double mJoystickDeadzoneValue = .15;
	private double mTriggerDeadzoneValue = .05;
	
	public XboxController(int port) {
		super(port);
		mDS = DriverStation.getInstance();
		mPort = port;
	}

	public XboxController(int port, double joystickdeadzone, double triggerdeadzone) {
		super(port);
		mDS = DriverStation.getInstance();
		mPort = port;
		mJoystickDeadzoneValue = joystickdeadzone;
		mTriggerDeadzoneValue = triggerdeadzone;
	}

	public double getRawAxis(final int axis) {
		return mDS.getStickAxis(mPort, axis);
	}

	public boolean getRawButton(final int button) {
		return ((0x1 << (button - 1)) & mDS.getStickButtons(mPort)) != 0;
	}

	public boolean isPressed(final int button) {
		return getRawButton(button);
	}

	public double getRightTrigger() {
			return getRawAxis(3);
	}

	public double getLeftTrigger() {
			return getRawAxis(2);
	}

	public double getRightJoyX() {
			return getRawAxis(4);
	}

	public double getRightJoyY() {
			return -getRawAxis(5);
	}

	public double getLeftJoyX() {
			return getRawAxis(0);
	}

	public double getLeftJoyY() {
			if(Math.abs(getRawAxis(1)) < .2) {
				return 0;
			} else {
				return -getRawAxis(1);
			}
	}

	public boolean getButtonA() {
		return getRawButton(1);
	}

	public boolean getButtonB() {
		return getRawButton(2);
	}

	public boolean getButtonX() {
		return getRawButton(3);
	}

	public boolean getButtonY() {
		return getRawButton(4);
	}

	public boolean getButtonBack() {
		return getRawButton(7);
	}

	public boolean getButtonStart() {
		return getRawButton(8);
	}

	public boolean getButtonRB() {
		return getRawButton(6);
	}

	public boolean getButtonLB() {
		return getRawButton(5);
	}

	public boolean getButtonLS() {
		return getRawButton(9);
	}

	public boolean getButtonRS() {
		return getRawButton(10);
	}
	
	public boolean getPOVUp() {
		//TODO, Fix these POV things? Not sure if they work too well.
		return (getPOV() < 45 || getPOV() > 315) && getPOV() != -1;
	}

	public boolean getPOVLeft() {
		return getPOV() > 215 && getPOV() <= 315;
	}

	public boolean getPOVDown() {
		return getPOV() > 135 && getPOV() <= 215;
	}

	public boolean getPOVRight() {

		return getPOV() > 45 && getPOV() <= 135;
	}

	/**
	 * Makes the controller rumble.
	 * @param l The left rumble value.
	 * @param r The right rumble value.
	 */
	public void rumble(double l, double r) {
		setRumble(RumbleType.kLeftRumble, l);
		setRumble(RumbleType.kRightRumble, r);
	}

	/**
	 * Makes the controller rumble for X seconds.
	 * @param l The left rumble value.
	 * @param r The right rumble value.
	 * @param seconds How long the controller should rumble.
	 */
	public void rumble(double l, double r, double seconds) {
		rumble(l, r);
		rumbleTimer = new Timer(seconds, false, new TimerUser() {
			public void timer() {
				rumble(0, 0);
			}
			public void timerStop() {
				rumbleTimer = null;
			}
		}).start();
	}
}