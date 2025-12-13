/*package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.*;
public class TestBenchColor {
    NormalizedColorSensor colorSensor;

    public enum DetectedColor {
        RED,
        BLUE,
        YELLOW,
        UNKNOWN
    }

    public void init(HardwareMap hwMap) {
        colorSensor = hwMap.get(NormalizedColorSensor.class, "sensor_color_distance");
        colorSensor.setGain(4);
    }

    public class DectectedColor getDetectedColor( Telemetry telemetry) {
        NormalizedRGBA colors = colorSensor.getNormalizedColors(); //return 4 values

        float normRed, normGreen, normBlue;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;

        telemetry.addData("red" , normRed);
        telemetry.addData("green" , normGreen);
        telemetry.addData("blue" , normBlue);

        // TODO add if statements for specific colors added
        /*
        red, green, blue
        RED = >.35, <.3, <.3
        YELLOW = >.5, >.9, <.6
        BLUE =


        return DetectedColor.UNKNOWN;
    }
}
*/
