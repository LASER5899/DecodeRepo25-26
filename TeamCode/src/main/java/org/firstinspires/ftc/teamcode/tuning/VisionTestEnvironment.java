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
@TeleOp(name= "Vision Test Environment")//, group="Linear OpMode")
public class VisionTestEnvironment extends LinearOpMode {

    public Vision camera = new Vision();
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;

    double graceMargin = 0.1;



    @Override
    public void runOpMode() {
        WebcamName cam1 = hardwareMap.get(WebcamName.class, "Camera1");

        camera.setTarget(Target.blue);

        waitForStart();
        camera.aprilTagSetUp(cam1);

        double alignVal=10000;
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double turnSpeed = 0.1;
        double originValue=0;

        boolean reached = false;
        boolean turnCodeOn = false;



        while (opModeIsActive()) {
            ///.addData("Pattern", camera.scanForPattern());
            ///Lots of Telemetry :)
            telemetry.addData("AlignVal",camera.alignmentValue());
            telemetry.addData("originVal", originValue);
            telemetry.addData("x button: ",gamepad1.x);
            telemetry.addData("reached ",reached);


            //Actual Code
            if(gamepad1.y){
                reached = true;
                turnCodeOn = false;
            }
            if (!turnCodeOn && gamepad1.x&& !(camera.alignmentValue() == -10000)) {
                turnCodeOn = true;
                originValue = 0;
                reached = false;

            }
            if(reached){
                leftFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightFrontDrive.setPower(0);
                rightBackDrive.setPower(0);
                telemetry.addData("turning: ","none");

            }
            alignVal = camera.alignmentValue();
            if(originValue==0){
                if(alignVal>0) {
                    originValue = 1;
                }else if(alignVal<0){
                    originValue = -1;
                }
            }
            if(turnCodeOn&&(alignVal*originValue<0)){
                reached=true;
                turnCodeOn = false;
                leftFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightFrontDrive.setPower(0);
                rightBackDrive.setPower(0);
                telemetry.addData("turning: ","none");
            }
            if (!gamepad1.x && turnCodeOn&&!reached) {


                /*if(Math.abs(alignVal)>70){
                    turnSpeed = 1;
                }else{
                    turnSpeed = 0.8/67*(Math.abs(alignVal)-70)+1;
                }*/

                //below here we need to add when its ok.
                ///if ((originValue*alignVal)>0) {


                    if (!(alignVal == -10000)) {


                        if ((originValue<0)&&(alignVal < 0)) {
                            //turn left
                            leftFrontDrive.setPower(-1 * turnSpeed);
                            leftBackDrive.setPower(-1 * turnSpeed);
                            rightFrontDrive.setPower(1 * turnSpeed);
                            rightBackDrive.setPower(1 * turnSpeed);

                            telemetry.addData("turning: ","left");

                        } else if ((originValue>0)&&(alignVal > 0)) {
                            //turnright

                            leftFrontDrive.setPower(1 * turnSpeed);
                            leftBackDrive.setPower(1 * turnSpeed);
                            rightFrontDrive.setPower(-1 * turnSpeed);
                            rightBackDrive.setPower(-1 * turnSpeed);
                            telemetry.addData("turning: ","right");

                        } else {
                            reached = true;
                            turnCodeOn = false;
                            leftFrontDrive.setPower(0);
                            leftBackDrive.setPower(0);
                            rightFrontDrive.setPower(0);
                            rightBackDrive.setPower(0);
                            telemetry.addData("turning: ","none");
                        }
                    }
                /*}else {
                    turnCodeOn = false;
                    leftFrontDrive.setPower(0);
                    leftBackDrive.setPower(0);
                    rightFrontDrive.setPower(0);
                    rightFrontDrive.setPower(0);
                    telemetry.addData("turning: ","none");


                }*/

            }
            telemetry.update();
        }
    }
}
