package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.LEDlights;

public class TestBenchColorTestNewThingy {
    public NormalizedColorSensor colorSensor;
    private LEDlights lights;

    public enum DetectedColor {
      UNKNOWN
    }

    public void init(HardwareMap hwMap) {
        colorSensor = hwMap.get(NormalizedColorSensor.class, "sensor_color_distance");
        colorSensor.setGain(3);
        lights = new LEDlights(hwMap, "light_strip");
    }

    public TestBenchColorTestNewThingy.DetectedColor getDetectedColor(Telemetry telemetry) {

        NormalizedRGBA colors = colorSensor.getNormalizedColors(); //return 4 values
        telemetry.addData("red", colors.red);
        telemetry.addData("green", colors.green);
        telemetry.addData("blue", colors.blue);


        // TODO add if statements for specific colors added
        /*
                     red, green, blue
            RED    = >.35, <.30, <.30
            YELLOW = >.50, >.90, <.60
            GREEN  = <.036, <.13, <.09
            PURPLE = <.22, <.19, <.42
         */

        return DetectedColor.UNKNOWN;
    }

    boolean isGreen = false;

    public boolean isItGreen(Telemetry telemetry) {

        NormalizedRGBA colors = colorSensor.getNormalizedColors(); //return 4 values


        // TODO add if statements for specific colors added
        /*
                     red, green, blue
            RED    = >.35, <.30, <.30
            YELLOW = >.50, >.90, <.60
            GREEN  = <.036, <.13, <.09
            PURPLE = <.22, <.19, <.42
         */

        if (IsColorGreen(colors)) {
            isGreen = true;
            lights.setColor(LEDlights.green);

        } else if (IsColorNotGreen(colors)) {
            isGreen = false;
        }


        telemetry.addData("is it green", isGreen);
        return IsColorGreen(colors);
    }

    private boolean IsColorGreen(NormalizedRGBA colors) {
        return IsNotVeryRed(colors) &&
                (IsVeryGreen(colors) && IsColorGreenBlue(colors));
    }

    private boolean IsNotVeryRed(NormalizedRGBA colors) {
        return colors.red < colors.green;
    }

    private boolean IsVeryGreen(NormalizedRGBA colors) {
        return colors.green > colors.blue;
    }

    private boolean IsColorGreenBlue(NormalizedRGBA colors) {
        return colors.green > 0.09 && colors.blue > 0.06;
    }

    private boolean IsColorNotGreen(NormalizedRGBA colors) {
        return IsVeryRed(colors) &&
                (IsNotVeryGreen(colors) || IsColorNotGreenBlue(colors));
    }

    private boolean IsVeryRed(NormalizedRGBA colors) {
        return colors.red < 0.02;
    }

    private boolean IsNotVeryGreen(NormalizedRGBA colors) {
        return colors.green < .14;
    }

    private boolean IsColorNotGreenBlue(NormalizedRGBA colors) {
        return colors.green < 0.14 && colors.blue < 0.1;
    }

    boolean isPurple = false;

    public boolean isItPurple(Telemetry telemetry) {

        NormalizedRGBA colors = colorSensor.getNormalizedColors(); //return 4 values


        // TODO add if statements for specific colors added
        /*
                     red, green, blue
            RED    = >.35, <.30, <.30
            YELLOW = >.50, >.90, <.60
            GREEN  = <.036, <.13, <.09
            PURPLE = <.22, <.19, <.42
         */

        if (IsColorPurple(colors)) {
            isPurple = true;
            lights.setColor(LEDlights.violet);
        } else if (IsColorNotPurple(colors)) {
            isPurple = false;
        }

        telemetry.addData("is it purple", isPurple);
        return IsColorPurple(colors);
    }

    private boolean IsColorPurple(NormalizedRGBA colors) {
        return IsNotVeryRed(colors) &&
                (IsVeryBlue(colors) && IsKindaGreenRed(colors));
    }

    private boolean IsColorNotPurple(NormalizedRGBA colors) {
        return IsVeryRed(colors) &&
                (IsNotVeryBlue(colors) || IsKindaNotGreenRed(colors));
    }

    private boolean IsVeryBlue(NormalizedRGBA colors) {
        return colors.blue > colors.green &&
                colors.blue > colors.red;
    }

    private boolean IsNotVeryBlue(NormalizedRGBA colors) {
        return colors.green > colors.blue ||
                colors.red > colors.blue;
    }

    private boolean IsKindaGreenRed(NormalizedRGBA colors) {
        return colors.green > 0.1 && colors.red > 0.02;
    }

    private boolean IsKindaNotGreenRed(NormalizedRGBA colors) {
        return colors.green < 0.1 && colors.red < 0.02;
    }

    boolean isBlack = false;

    public boolean isItBlack(Telemetry telemetry) {

        NormalizedRGBA colors = colorSensor.getNormalizedColors(); //return 4 values

        // TODO add if statements for specific colors added
        /*
                     red, green, blue
            RED    = >.35, <.30, <.30
            YELLOW = >.50, >.90, <.60
            GREEN  = <.036, <.13, <.09
            PURPLE = <.22, <.19, <.42
         */

        if (IsColorBlack(colors)) {
            isBlack = true;
            lights.setColor(LEDlights.black);
        } else {
            isBlack = false;
        }

        telemetry.addData("is it black", isBlack);
        return IsColorBlack(colors);
    }

    private boolean IsColorBlack(NormalizedRGBA colors) {
        return colors.green < 0.009 && colors.red < 0.009 && colors.blue < 0.009;
    }

    private boolean IsColorNotBlack(NormalizedRGBA colors) {
        return (IsColorGreen(colors) || IsColorPurple(colors));
    }
}