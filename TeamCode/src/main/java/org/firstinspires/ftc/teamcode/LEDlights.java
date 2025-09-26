package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;

public class LEDlights {

    private Servo lights;

    public static double rainbow_Rainbow = 0.220;
    public static double rainbow_Party = 0.230;
    public static double rainbow_Ocean = 0.235;
    public static double rainbow_Lava = 0.243;
    public static double rainbow_Forest = 0.249;
    public static double rainbow_Glitter = 0.253;
    public static double confetti = 0.260;
    public static double shot_Red = 0.265;
    public static double shot_Blue = 0.270;
    public static double shot_White = 0.275;
    public static double sinelon_Rainbow = 0.280;
    public static double sinelon_Party = 0.287;
    public static double sinelon_Ocean = 0.292;
    public static double sinelon_Lava = 0.300;
    public static double sinelon_Forest = 0.305;
    public static double bpm_Rainbow = 0.31;
    public static double bpm_Party = 0.315;
    public static double bpm_Ocean = 0.321;
    public static double bpm_Lava = 0.328;
    public static double bpm_Forest = 0.332;
    public static double fire_Medium = 0.335;
    public static double fire_Large = 0.340;
    public static double twinkles_Rainbow = 0.347;
    public static double twinkles_Party = 0.357;
    public static double twinkles_Ocean = 0.359;
    public static double twinkles_Lava = 0.364;
    public static double twinkles_Forest = 0.370;
    public static double colorwave_Rainbow = 0.376;
    public static double colorwave_Party = 0.381;
    public static double colorwave_Ocean = 0.387;
    public static double colorwave_Lava = 0.393;
    public static double colorwave_Forest = 0.399;
    public static double larson_Red = 0.403;
    public static double larson_Gray = 0.410;
    public static double lightchase_Red = 0.415;
    public static double lightchase_Blue = 0.420;
    public static double lightchase_Grey = 0.426;
    public static double heartbeat_Red = 0.430;
    public static double heartbeat_Blue = 0.437;
    public static double heartbeat_White = 0.446;
    public static double heartbeat_Gray = 0.450;
    public static double breath_Red = 0.454;
    public static double breath_Blue = 0.459;
    public static double breath_Grey = 0.467;
    public static double strobe_Red = 0.470;
    public static double strobe_Blue = 0.477;
    public static double strobe_Gold = 0.481;
    public static double strobe_White = 0.487;
    public static double black_Blend_Col1 = 0.492;
    public static double larson_Scanner_Col1 = 0.500;
    public static double light_Chase_Col1 = 0.504;
    public static double heartbeat_Slow_Col1 = 0.509;
    public static double heartbeat_Medium_Col1 = 0.515;
    public static double heartbeat_Fast_Col1 = 0.520;
    public static double breath_Slow_Col1 = 0.526;
    public static double breath_Fast_Col1 = 0.531;
    public static double shot_Col1 = 0.537;
    public static double strobe_Col1 = 0.544;
    public static double black_Blend_Col2 = 0.549;
    public static double larson_Scanner_Col2 = 0.553;
    public static double light_Chase_Col2 = 0.559;
    public static double heartbeat_Slow_Col2 = 0.564;
    public static double heartbeat_Medium_Col2 = 0.571;
    public static double heartbeat_Fast_Col2 = 0.575;
    public static double breath_Slow_Col2 = 0.580;
    public static double breath_Fast_Col2 = 0.587;
    public static double shot_Col2 = 0.592;
    public static double strobe_Col2 = 0.600;
    public static double sparkle_Col1on2 = 0.607;
    public static double sparkle_Col2on1 = 0.613;
    public static double gradient_Col1on2 = 0.617;
    public static double bpm_Col1on2 = 0.620;
    public static double end_Blend_Col1on2 = 0.628;
    public static double end_Blend_Col2on1 = 0.632;
    public static double no_Blend_Col1and2 = 0.637;
    public static double twinkles_Col1and2 = 0.641;
    public static double color_Waves_Col1and2 = 0.648;
    public static double sinelon_Col1and2 = 0.654;
    public static double hot_Pink = 0.66;
    public static double red = 0.666;
    public static double red_Orange = 0.670;
    public static double orange = 0.678;
    public static double gold = 0.681;
    public static double yellow = 0.693;
    public static double yellow_Green = 0.698;
    public static double lime = 0.703;
    public static double dark_Green = 0.709;
    public static double green = 0.715;
    public static double blue_Green = 0.720;
    public static double aqua = 0.725;
    public static double sky_Blue = 0.730;
    public static double dark_Blue = 0.736;
    public static double blue = 0.742;
    public static double blue_Violet = 0.749;
    public static double violet = 0.755;
    public static double white_Gray = 0.760;
    public static double medium_Gray = 0.765;
    public static double dark_Gray = 0.772;
    public static double black = 0.780;


    public LEDlights(HardwareMap hardwareMap, String deviceName) {
        lights = hardwareMap.get(Servo.class, deviceName);
    }

    public void setColor(double value) {
        lights.setPosition(value);
    }
}
