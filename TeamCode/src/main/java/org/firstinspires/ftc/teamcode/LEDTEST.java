package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="LEDTEST", group="Linear OpMode")
//@Disabled
public class LEDTEST extends LinearOpMode {

    private Servo lights;

    @Override
    public void runOpMode() {

        lights = hardwareMap.get(Servo.class, "light_strip");
        double lightVal = 0.000;
        boolean prevAButton = false;
        boolean prevBButton = false;

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a && !prevAButton) {
                lightVal += 0.001;
            } else if (gamepad1.b && !prevBButton) {
                lightVal -= 0.001;
            }

            if (gamepad1.x) {
                lightVal += 0.001;
            } else if (gamepad1.y) {
                lightVal -= 0.001;
            }

            lights.setPosition(lightVal);

            prevAButton = gamepad1.a;
            prevBButton = gamepad1.b;

            telemetry.addData("light value", lightVal);
            telemetry.update();
        }
    }
}