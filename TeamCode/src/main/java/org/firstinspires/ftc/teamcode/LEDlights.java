package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;

public class LEDlights {

    private Servo lights;

    public static double yellowColor = 0.695;
    public static double redFireColor = 0.335;
    public static double blueRayColor = 0.270;

    public LEDlights(HardwareMap hardwareMap, String deviceName) {
        lights = hardwareMap.get(Servo.class, deviceName);
    }

    public void setColor(double value) {
        lights.setPosition(value);
    }
}
