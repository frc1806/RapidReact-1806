package org.usfirst.frc.team1806.robot.auto.modes.modesUtil;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.reflections.Reflections;
import org.usfirst.frc.team1806.robot.auto.modes.DummyMode;
import org.usfirst.frc.team1806.robot.loop.Loop;
import org.usfirst.frc.team1806.robot.loop.Looper;

import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.Set;

/**
 * Class that allows a user to select which autonomous mode to execute from shuffleboard.
 */
public class AutoModeSelector {

    public static final String AUTO_OPTIONS_DASHBOARD_KEY = "Auto Options";
    public static final String SELECTED_AUTO_MODE_DASHBOARD_KEY = "Selected Auto Mode";
    public static final String AUTO_MODES_PACKAGE = "org.usfirst.frc.team1806.robot.auto.modes";
    private static SendableChooser<String> AUTO_CHOOSER = new SendableChooser<>();

    /**
     * Uses reflection to get every auto mode in the defined auto modes package. The idea being to remove a step from the process of adding an autonomous mode.
     */
    public static void initAutoModeSelector() {
        Reflections reflections = new Reflections(AUTO_MODES_PACKAGE);
        Set<Class<? extends AutoModeBase>> modes = reflections.getSubTypesOf(AutoModeBase.class);
        for (Class<?> mode : modes) {
            AUTO_CHOOSER.addOption(mode.getSimpleName(), mode.getName());
        }
        SmartDashboard.putData(AUTO_OPTIONS_DASHBOARD_KEY, AUTO_CHOOSER);
    }


    /**
     * Checks the returned automode against every mode in the defined auto modes package. If there is a problem instantiating it, or the mode selected doesn't exist, will return a default auto.
     */
    public static AutoModeBase getSelectedAutoMode() {

        String selectedModeName = AUTO_CHOOSER.getSelected();
        Reflections reflections = new Reflections(AUTO_MODES_PACKAGE);
        Set<Class<? extends AutoModeBase>> modes = reflections.getSubTypesOf(AutoModeBase.class);

        for (Class<?> mode : modes) {
            if (mode != null && selectedModeName != null) {
                if (selectedModeName.equals(mode.getName())) {
                    try {
                        return (AutoModeBase) mode.getConstructor().newInstance();
                    } catch (InstantiationException e) {
                        e.printStackTrace();
                        return fallBackToDefaultAuto(selectedModeName);
                    } catch (IllegalAccessException e) {
                        e.printStackTrace();
                        return fallBackToDefaultAuto(selectedModeName);
                    } catch (IllegalArgumentException e) {
                        e.printStackTrace();
                        return fallBackToDefaultAuto(selectedModeName);
                    } catch (InvocationTargetException e) {
                        e.printStackTrace();
                        return fallBackToDefaultAuto(selectedModeName);
                    } catch (NoSuchMethodException e) {
                        e.printStackTrace();
                        return fallBackToDefaultAuto(selectedModeName);
                    } catch (SecurityException e) {
                        e.printStackTrace();
                        return fallBackToDefaultAuto(selectedModeName);
                    } finally {
                    }
                }
            }
        }

        return fallBackToDefaultAuto(selectedModeName);
    }

    public static AutoModeBase fallBackToDefaultAuto(String wantedAutoMode) {
        DriverStation.reportError("Failed to select auto mode: " + wantedAutoMode, false);
        return new DummyMode();
    }

    public static String returnNameOfSelectedAuto() {
        return AUTO_CHOOSER.getSelected();
    }

    public static void registerDisabledLoop(Looper in) {
        in.register(new Loop() {

            @Override
            public void onStart(double timestamp) {

            }

            @Override
            public void onLoop(double timestamp) {
                if (AUTO_CHOOSER.getSelected() != null) {
                    SmartDashboard.putString(SELECTED_AUTO_MODE_DASHBOARD_KEY, AUTO_CHOOSER.getSelected());
                }

            }

            @Override
            public void onStop(double timestamp) {

            }
        });
    }
}