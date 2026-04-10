package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.Vision;
import org.firstinspires.ftc.teamcode.Vision.*;
@TeleOp(name= "Vision Test Environment")//, group="Linear OpMode")
public class VisionTestEnvironment extends LinearOpMode {

    public Vision camera = new Vision();




    @Override
    public void runOpMode() {
        WebcamName cam1 = hardwareMap.get(WebcamName.class, "Camera1");

        camera.setTarget(Target.blue);

        waitForStart();
        camera.aprilTagSetUp(cam1);
        telemetry.addLine("Entering");
        while(opModeIsActive()){
                telemetry.addLine("RUN RUN RUDOLF");
            telemetry.addData("Range",camera.centerDistanceCM());
            telemetry.update();

        }

    }
}
