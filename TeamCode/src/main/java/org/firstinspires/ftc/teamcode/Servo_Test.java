package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="servotestah", group="Linear OpMode")
//@Disabled
public class Servo_Test extends LinearOpMode {

    private Servo servo;

    @Override
    public void runOpMode() {

        servo = hardwareMap.get(Servo.class, "servo");
        double servoVal = 0.0;

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a && servoVal < 1.0)
                servoVal += 0.005;

            if (gamepad1.b && servoVal > 0.0 )
                servoVal -= 0.005;

            servo.setPosition(servoVal);

            telemetry.addData("servo value", servoVal);
            telemetry.update();

            sleep(100);
        }
    }
}