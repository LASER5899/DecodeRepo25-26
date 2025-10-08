package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Vision;
import org.firstinspires.ftc.teamcode.Vision.*;

public class VisionTestEnvironment extends LinearOpMode {

    public Vision camera = new Vision();





    @Override
    public void runOpMode(){
        camera.setCamera("Camera 1");
        camera.setTarget(Target.blue);

        camera.aprilTagSetUp();
        while(opModeIsActive()){
            Vision.toggleDevModeOn();
            camera.scanForTarget();
        }

    }

    
}
