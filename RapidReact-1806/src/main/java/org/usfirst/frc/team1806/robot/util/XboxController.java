package org.usfirst.frc.team1806.robot.util;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/*
 * The XboxController class is a wrapper of the joystick class,
 * auto applies deadzones and makes nice methods for us
 */
public class XboxController extends Joystick {
	private static Map<String, Object> AXIS_DISPLAY_BAR_PROPS = new HashMap<>();

    static {
        AXIS_DISPLAY_BAR_PROPS.put("Min", -1.0d);
        AXIS_DISPLAY_BAR_PROPS.put("Max", 1.0d);
        AXIS_DISPLAY_BAR_PROPS.put("Center", 0.0d);
    }

	private final int mPort;
	public Timer rumbleTimer;
	private String mName;
	private ShuffleboardTab mControllerConfigTab;
	private XboxControllerConfigDashboard mControllerConfigDashboard;
	private XboxControllerConfigValues mConfigValues;
	Supplier<double[]> valueSupplier = new Supplier(){

		@Override
		public Object get() {
			// TODO Auto-generated method stub
			return null;
		}

	};
	
	public XboxController(int port, String name, XboxControllerConfigValues defaultConfig) {
		super(port);
		mConfigValues = defaultConfig;
		mName = name;
		DriverStation.getInstance();
		mPort = port;
		mControllerConfigTab = Shuffleboard.getTab(mName.concat(" Controller Configuration"));
		mControllerConfigDashboard = new XboxControllerConfigDashboard(mControllerConfigTab, defaultConfig);
		updateConfig();
		configValueDisplay();
	}

	public double getRawAxis(final int axis) {
		return DriverStation.getStickAxis(mPort, axis);
	}

	public double applyConfig(double value, double deadZone, double minimumOutput, double linearity)
	{
		if(Math.abs(value) < deadZone)
		{
			return 0.0;
		}
		//Apply Deadzone
		if(deadZone != 0)
		{
			value =(value>0?1:-1) * ((Math.abs(value) - deadZone) * (1.0 / (1.0 - deadZone)));
		}
		//Apply Linearity this doesn't need any absolute value to handle so long as the non-linear part is an odd power.
		value = (value * linearity) + ( Math.pow(value, 5.0) * (1.0 - linearity));
		//Apply Minimum output
		if(Math.abs(value) > 0)
		{
			value = (value>0?1:-1) * (Math.abs(value) * (1.0-minimumOutput) + minimumOutput);
		}
		return value;
		
	}

	public boolean getRawButton(final int button) {
		return ((0x1 << (button - 1)) & DriverStation.getStickButtons(mPort)) != 0;
	}

	public boolean isPressed(final int button) {
		return getRawButton(button);
	}

	public double getRightTrigger() {
			return applyConfig(getRawAxis(3), mConfigValues.getTriggerDeadzone(), mConfigValues.getTriggerMinimumOutput(), mConfigValues.getTriggerLinearity());
	}

	public double getLeftTrigger() {
			return applyConfig(getRawAxis(2), mConfigValues.getTriggerDeadzone(), mConfigValues.getTriggerMinimumOutput(), mConfigValues.getTriggerLinearity());
	}

	public double getRightJoyX() {
			return applyConfig(getRawAxis(4), mConfigValues.getRightXDeadzone(), mConfigValues.getRightXMinimumOutput(), mConfigValues.getRightXLinearity());
	}

	public double getRightJoyY() {
			return applyConfig(-getRawAxis(5), mConfigValues.getRightYDeadzone(), mConfigValues.getRightYMinimumOutput(), mConfigValues.getRightYLinearity());
	}

	public double getLeftJoyX() {
			return applyConfig(getRawAxis(0), mConfigValues.getLeftXDeadzone(), mConfigValues.getLeftXMinimumOutput(), mConfigValues.getLeftXLinearity());
	}

	public double getLeftJoyY() {
			return applyConfig(-getRawAxis(1), mConfigValues.getLeftYDeadzone(), mConfigValues.getLeftYMinimumOutput(), mConfigValues.getLeftYLinearity());
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

	public boolean getLeftTriggerAsDigital(){
		return getRawAxis(2) > mConfigValues.getTriggerAsDigitalDeadzone();
	}

	public boolean getRightTriggerAsDigital(){
		return getRawAxis(3) > mConfigValues.getTriggerAsDigitalDeadzone();
	}

	public void updateConfig()
	{
		if(DriverStation.isDSAttached())
		{
			mConfigValues = mControllerConfigDashboard.getUpdatedConfigValues();
		}
	}

	public void configValueDisplay()
	{
		
		mControllerConfigTab.addNumber("Left X", new DoubleSupplier() {

			@Override
			public double getAsDouble() {
				return getLeftJoyX();
			}
		}).withWidget(BuiltInWidgets.kNumberBar).withProperties(AXIS_DISPLAY_BAR_PROPS);

		mControllerConfigTab.addNumber("Left Y", new DoubleSupplier() {

			@Override
			public double getAsDouble() {
				return getLeftJoyY();
			}
		}).withWidget(BuiltInWidgets.kNumberBar).withProperties(AXIS_DISPLAY_BAR_PROPS);

		mControllerConfigTab.addNumber("Right X", new DoubleSupplier() {

			@Override
			public double getAsDouble() {
				return getRightJoyX();
			}
		}).withWidget(BuiltInWidgets.kNumberBar).withProperties(AXIS_DISPLAY_BAR_PROPS);

		mControllerConfigTab.addNumber("Right Y", new DoubleSupplier() {

			@Override
			public double getAsDouble() {
				return getRightJoyY();
			}
		}).withWidget(BuiltInWidgets.kNumberBar).withProperties(AXIS_DISPLAY_BAR_PROPS);

		mControllerConfigTab.addNumber("Left Trigger", new DoubleSupplier() {

			@Override
			public double getAsDouble() {
				return getLeftTrigger();
			}
		}).withWidget(BuiltInWidgets.kNumberBar).withProperties(AXIS_DISPLAY_BAR_PROPS);

		mControllerConfigTab.addNumber("Right Trigger", new DoubleSupplier() {

			@Override
			public double getAsDouble() {
				return getRightTrigger();
			}
		}).withWidget(BuiltInWidgets.kNumberBar).withProperties(AXIS_DISPLAY_BAR_PROPS);
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