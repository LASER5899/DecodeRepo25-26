package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.LEDlights;
import org.firstinspires.ftc.teamcode.old.redo.RobotHardware;


@TeleOp(name="Test Color Sensor 1", group="Linear OpMode")
//@Disabled
public class ColorSensorTestNewThingy extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    private LEDlights lights;
    @Override
    public void runOpMode() {
        TestBenchColorTestNewThingy bench = new TestBenchColorTestNewThingy();
        RobotHardware robotHardware = new RobotHardware();
        bench.init(hardwareMap);
        lights = new LEDlights(hardwareMap, "light_strip");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        boolean prevGA = false;
        boolean prevGB = false;
        int gain = 5;

        boolean isGreen = false;
        boolean isPurple = false;
        boolean isBlack = true;

        while (opModeIsActive()) {

            bench.getDetectedColor(telemetry);
            lights.setColor(LEDlights.black);
            if (bench.isItGreen(telemetry)) {
                lights.setColor(LEDlights.green);
            } else if (bench.isItPurple(telemetry)) {
                lights.setColor(LEDlights.violet);
            } else if (bench.isItBlack(telemetry)){
                lights.setColor(LEDlights.black);
            }
            telemetry.addData("gain:", gain);
            telemetry.update();

            if (gamepad1.a && !prevGA) { //
                gain = gain + 1;
            }
            if (gamepad1.b && !prevGB) { // &&
                gain = gain - 1;
            }
            bench.colorSensor.setGain(gain);

            prevGA = gamepad1.a;
            prevGB = gamepad1.b;

            if (!opModeIsActive()) {
                break;
            }
        }
    }
}