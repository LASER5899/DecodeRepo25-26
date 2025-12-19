package org.firstinspires.ftc.teamcode.tuning.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@Config
public class RobotConstants {
    public static double TARGET_RPM =  1000;
    public static double kP = 0.0005;
    public static double kF = 0.0005;

    public static double holdSeconds = 5;

    public static double maxAccel = 500;
    //public static PIDCoefficients TURNING_PID = new PIDCoefficients();
    // other constants
}
