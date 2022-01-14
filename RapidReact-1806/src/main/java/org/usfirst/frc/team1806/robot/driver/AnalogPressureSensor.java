package org.usfirst.frc.team1806.robot.driver;
import edu.wpi.first.wpilibj.AnalogInput;

public class AnalogPressureSensor {
    AnalogInput pressureSensor;

    public AnalogPressureSensor(int analogChannel){
        pressureSensor = new AnalogInput(analogChannel);
    }

    public double getPressure(){
        return 250*(pressureSensor.getVoltage() / 5.0) - 25;
    }


}
