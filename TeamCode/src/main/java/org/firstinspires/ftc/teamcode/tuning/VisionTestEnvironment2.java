package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.Vision;
import org.firstinspires.ftc.teamcode.Vision.*;
@TeleOp(name="2nd Vision Test Environment", group="Linear OpMode")
public class VisionTestEnvironment2 extends LinearOpMode {

    public Vision camera = new Vision();
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;

    double graceMargin = 0.1;



    @Override
    public void runOpMode() {
        WebcamName cam1 = hardwareMap.get(WebcamName.class, "Camera1");
        //camera.setCamera("Camera1");
        camera.setTarget(Target.blue);
        //Vision.DevModeOn();
        waitForStart();
        camera.aprilTagSetUp(cam1);
        boolean alignValue = false;
        double alignVal=10000;
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        double turnSpeed = 0.1;
        double originValue=0;
        while (opModeIsActive()) {
            ///.addData("Pattern", camera.scanForPattern());
            ///Lots of Telemetry :)
            telemetry.addData("AlignVal",alignVal);
            telemetry.addData("x button: ",gamepad1.x);

            //Actual Code
            if (alignValue == false && gamepad1.x) {
                alignValue = true;
                originValue=0;

            }
            if (!gamepad1.x && alignValue) {

                alignVal = camera.alignmentValue();
                if (!(alignVal < graceMargin&& alignVal > -graceMargin) && !gamepad1.x) {


                    alignVal = camera.alignmentValue();
                    if(originValue==0){
                        originValue=alignVal;
                    }
                    if (!(alignVal == -10000)) {

                        if (alignVal < 0) {
                            //turn left
                            leftFrontDrive.setPower(-1 * turnSpeed);
                            leftBackDrive.setPower(-1 * turnSpeed);
                            rightFrontDrive.setPower(1 * turnSpeed);
                            rightBackDrive.setPower(1 * turnSpeed);

                            telemetry.addData("turning: ","left");

                        } else if (alignVal > 0) {
                            //turnright
                            leftFrontDrive.setPower(1 * turnSpeed);
                            leftBackDrive.setPower(1 * turnSpeed);
                            rightFrontDrive.setPower(-1 * turnSpeed);
                            rightBackDrive.setPower(-1 * turnSpeed);
                            telemetry.addData("turning: ","right");

                        }
                    }
                }else {
                    alignValue = false;
                    leftFrontDrive.setPower(0);
                    leftBackDrive.setPower(0);
                    rightFrontDrive.setPower(0);
                    rightFrontDrive.setPower(0);
                    telemetry.addData("turning: ","none");

                }

            }
            telemetry.update();
        }

    }
}
