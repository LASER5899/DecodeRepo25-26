package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.Vision;
import org.firstinspires.ftc.teamcode.Vision.*;
@Autonomous(name="Vision Test Environment", group="Linear OpMode")
public class VisionTestEnvironment extends LinearOpMode {
   // public VisionTestEnvironment(HardwareMap map){
   //     this.hardwareMap=map;
   // }

    public Vision camera = new Vision();




    @Override
    public void runOpMode(){

        WebcamName cam1 = hardwareMap.get(WebcamName.class, "Camera1");
        //camera.setCamera("Camera1");
        camera.setTarget(Target.blue);
        Vision.DevModeOn();
        waitForStart();
        camera.aprilTagSetUp(cam1);

        while(opModeIsActive()){
            camera.scanForTarget();
            telemetry.addData("xDistance",camera.getxDistance());
            telemetry.addData("yDistance",camera.getyDistance());
            telemetry.addData("zDistance",camera.getzDistance());
            telemetry.addData("range",camera.getRange());

            telemetry.addData("pitch",camera.getPitch());
            telemetry.addData("yaw",camera.getxDistance());
            telemetry.addData("yeet",camera.getxDistance());
            telemetry.addData("xDistance",camera.getxDistance());
            telemetry.addData("xDistance",camera.getxDistance());
            telemetry.addData("horizontal Angle",camera.horizAngleReturn());
            telemetry.addData("center Distance",camera.centerDistanceReturn());
            telemetry.addData("Vertical angle ",camera.vertAngleReturn());
            telemetry.update();
        }

    }

    
}
