package org.usfirst.frc.team1806.robot.util;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;

public class XboxControllerConfigDashboard {

    private static final XboxControllerConfigValues NETWORK_TABLE_FAILURE_FALLBACK_VALUES = XboxControllerConfigValues.Builder
            .create().withLeftXDeadzone(.12).withLeftXMinimumOutput(.04).withLeftXLinearity(.32).withLeftYDeadzone(.12)
            .withLeftYMinimumOutput(.04).withLeftYLinearity(.32).withRightXDeadzone(.12).withRightXMinimumOutput(.04)
            .withRightXLinearity(.32).withRightYDeadzone(.12).withRightYMinimumOutput(.04).withRightYLinearity(.32)
            .withTriggerDeadzone(.2).withTriggerMinimumOutput(.04).withTriggerLinearity(.32)
            .withTriggerAsDigitalDeadzone(.5).build();

    private static Map<String, Object> CONTROLLER_CONFIG_SLIDER_PROPS = new HashMap<>();

    static {
        CONTROLLER_CONFIG_SLIDER_PROPS.put("Min", 0.0d);
        CONTROLLER_CONFIG_SLIDER_PROPS.put("Max", 1.0d);
        CONTROLLER_CONFIG_SLIDER_PROPS.put("Block increment", 0.01d);
    }
    ShuffleboardTab controllerConfigTab;
    private NetworkTableEntry leftXDeadzone;
    private NetworkTableEntry leftXMinimumOutput;
    private NetworkTableEntry leftXLinearity;
    private NetworkTableEntry leftYDeadzone;
    private NetworkTableEntry leftYMinimumOutput;
    private NetworkTableEntry leftYLinearity;
    private NetworkTableEntry rightXDeadzone;
    private NetworkTableEntry rightXMinimumOutput;
    private NetworkTableEntry rightXLinearity;
    private NetworkTableEntry rightYDeadzone;
    private NetworkTableEntry rightYMinimumOutput;
    private NetworkTableEntry rightYLinearity;
    private NetworkTableEntry triggerDeadzone;
    private NetworkTableEntry triggerMinimumOutput;
    private NetworkTableEntry triggerLinearity;
    private NetworkTableEntry triggerAsDigitalDeadzone;

    public XboxControllerConfigDashboard(ShuffleboardTab configTab, XboxControllerConfigValues configValues) {
        if(configValues == null)
        {
            configValues = NETWORK_TABLE_FAILURE_FALLBACK_VALUES;
        }
        this.controllerConfigTab = configTab;
        this.leftXDeadzone = controllerConfigTab.addPersistent("Left X Deadzone", configValues.getLeftXDeadzone())
                .withWidget(BuiltInWidgets.kNumberSlider).withProperties(CONTROLLER_CONFIG_SLIDER_PROPS).withSize(2, 1).withPosition(0, 0).getEntry();
        this.leftXMinimumOutput = controllerConfigTab
                .addPersistent("Left X Minimum Output", configValues.getLeftXMinimumOutput())
                .withWidget(BuiltInWidgets.kNumberSlider).withProperties(CONTROLLER_CONFIG_SLIDER_PROPS).withSize(2, 1).withPosition(0, 1).getEntry();
        this.leftXLinearity = controllerConfigTab.addPersistent("Left X Linearity", configValues.getLeftXLinearity())
                .withWidget(BuiltInWidgets.kNumberSlider).withProperties(CONTROLLER_CONFIG_SLIDER_PROPS).withSize(2, 1).withPosition(0, 2).getEntry();
        this.leftYDeadzone = controllerConfigTab.addPersistent("Left Y Deadzone", configValues.getLeftYDeadzone())
                .withWidget(BuiltInWidgets.kNumberSlider).withProperties(CONTROLLER_CONFIG_SLIDER_PROPS).withSize(2, 1).withPosition(2, 0).getEntry();
        this.leftYMinimumOutput = controllerConfigTab
                .addPersistent("Left Y Minimum Output", configValues.getLeftYMinimumOutput())
                .withWidget(BuiltInWidgets.kNumberSlider).withProperties(CONTROLLER_CONFIG_SLIDER_PROPS).withSize(2, 1).withPosition(2, 1).getEntry();
        this.leftYLinearity = controllerConfigTab.addPersistent("Left Y Linearity", configValues.getLeftYLinearity())
                .withWidget(BuiltInWidgets.kNumberSlider).withProperties(CONTROLLER_CONFIG_SLIDER_PROPS).withSize(2, 1).withPosition(2, 2).getEntry();
        this.rightXDeadzone = controllerConfigTab.addPersistent("Right X Deadzone", configValues.getRightXDeadzone())
                .withWidget(BuiltInWidgets.kNumberSlider).withProperties(CONTROLLER_CONFIG_SLIDER_PROPS).withSize(2, 1).withPosition(4, 0).getEntry();
        this.rightXMinimumOutput = controllerConfigTab
                .addPersistent("Right X Minimum Output", configValues.getRightXMinimumOutput())
                .withWidget(BuiltInWidgets.kNumberSlider).withProperties(CONTROLLER_CONFIG_SLIDER_PROPS).withSize(2, 1).withPosition(4, 1).getEntry();
        this.rightXLinearity = controllerConfigTab.addPersistent("Right X Linearity", configValues.getRightXLinearity())
                .withWidget(BuiltInWidgets.kNumberSlider).withProperties(CONTROLLER_CONFIG_SLIDER_PROPS).withSize(2, 1).withPosition(4, 2).getEntry();
        this.rightYDeadzone = controllerConfigTab.addPersistent("Right Y Deadzone", configValues.getRightYDeadzone())
                .withWidget(BuiltInWidgets.kNumberSlider).withProperties(CONTROLLER_CONFIG_SLIDER_PROPS).withSize(2, 1).withPosition(6, 0).getEntry();
        this.rightYMinimumOutput = controllerConfigTab
                .addPersistent("Right Y Minimum Output", configValues.getRightYMinimumOutput())
                .withWidget(BuiltInWidgets.kNumberSlider).withProperties(CONTROLLER_CONFIG_SLIDER_PROPS).withSize(2, 1).withPosition(6, 1).getEntry();
        this.rightYLinearity = controllerConfigTab.addPersistent("Right Y Linearity", configValues.getRightYLinearity())
                .withWidget(BuiltInWidgets.kNumberSlider).withProperties(CONTROLLER_CONFIG_SLIDER_PROPS).withSize(2, 1).withPosition(6, 2).getEntry();
        this.triggerDeadzone = controllerConfigTab.addPersistent("Trigger Deadzone", configValues.getTriggerDeadzone())
                .withWidget(BuiltInWidgets.kNumberSlider).withProperties(CONTROLLER_CONFIG_SLIDER_PROPS).withSize(2, 1).withPosition(8, 0).getEntry();
        this.triggerMinimumOutput = controllerConfigTab
                .addPersistent("Trigger Minimum Output", configValues.getTriggerMinimumOutput())
                .withWidget(BuiltInWidgets.kNumberSlider).withProperties(CONTROLLER_CONFIG_SLIDER_PROPS).withSize(2, 1).withPosition(8, 1).getEntry();
        this.triggerLinearity = controllerConfigTab
                .addPersistent("Trigger Linearity", configValues.getTriggerLinearity())
                .withWidget(BuiltInWidgets.kNumberSlider).withProperties(CONTROLLER_CONFIG_SLIDER_PROPS).withSize(2, 1).withPosition(8, 2).getEntry();
        this.triggerAsDigitalDeadzone = controllerConfigTab
                .addPersistent("Trigger As Digital Deadzone", configValues.getTriggerAsDigitalDeadzone())
                .withWidget(BuiltInWidgets.kNumberSlider).withProperties(CONTROLLER_CONFIG_SLIDER_PROPS).withSize(2, 1).withPosition(8, 3).getEntry();
    }

    public XboxControllerConfigValues getUpdatedConfigValues() {

        return XboxControllerConfigValues.Builder.create()
                .withLeftXDeadzone(leftXDeadzone.getDouble(NETWORK_TABLE_FAILURE_FALLBACK_VALUES.getLeftXDeadzone()))
                .withLeftXMinimumOutput(
                        leftXMinimumOutput.getDouble(NETWORK_TABLE_FAILURE_FALLBACK_VALUES.getLeftXMinimumOutput()))
                .withLeftXLinearity(leftXLinearity.getDouble(NETWORK_TABLE_FAILURE_FALLBACK_VALUES.getLeftXLinearity()))
                .withLeftYDeadzone(leftYDeadzone.getDouble(NETWORK_TABLE_FAILURE_FALLBACK_VALUES.getLeftYDeadzone()))
                .withLeftYMinimumOutput(
                        leftYMinimumOutput.getDouble(NETWORK_TABLE_FAILURE_FALLBACK_VALUES.getLeftYMinimumOutput()))
                .withLeftYLinearity(leftYLinearity.getDouble(NETWORK_TABLE_FAILURE_FALLBACK_VALUES.getLeftYLinearity()))
                .withRightXDeadzone(rightXDeadzone.getDouble(NETWORK_TABLE_FAILURE_FALLBACK_VALUES.getRightXDeadzone()))
                .withRightXMinimumOutput(
                        rightXMinimumOutput.getDouble(NETWORK_TABLE_FAILURE_FALLBACK_VALUES.getRightXMinimumOutput()))
                .withRightXLinearity(
                        rightXLinearity.getDouble(NETWORK_TABLE_FAILURE_FALLBACK_VALUES.getRightXLinearity()))
                .withRightYDeadzone(rightYDeadzone.getDouble(NETWORK_TABLE_FAILURE_FALLBACK_VALUES.getRightYDeadzone()))
                .withRightYMinimumOutput(
                        rightYMinimumOutput.getDouble(NETWORK_TABLE_FAILURE_FALLBACK_VALUES.getRightYMinimumOutput()))
                .withRightYLinearity(
                        rightYLinearity.getDouble(NETWORK_TABLE_FAILURE_FALLBACK_VALUES.getRightYLinearity()))
                .withTriggerDeadzone(
                        triggerDeadzone.getDouble(NETWORK_TABLE_FAILURE_FALLBACK_VALUES.getTriggerDeadzone()))
                .withTriggerMinimumOutput(
                        triggerMinimumOutput.getDouble(NETWORK_TABLE_FAILURE_FALLBACK_VALUES.getTriggerMinimumOutput()))
                .withTriggerLinearity(
                        triggerLinearity.getDouble(NETWORK_TABLE_FAILURE_FALLBACK_VALUES.getTriggerLinearity()))
                .withTriggerAsDigitalDeadzone(triggerAsDigitalDeadzone
                        .getDouble(NETWORK_TABLE_FAILURE_FALLBACK_VALUES.getTriggerAsDigitalDeadzone()))
                .build();
    }
}
