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
@TeleOp(name="Vision Test Environment", group="Linear OpMode")
public class VisionTestEnvironment extends LinearOpMode {

    public Vision camera = new Vision();

    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;

    private DcMotor outtake_motor;

    private Servo intakeServo;
    private Servo transferServo;
    private Servo flickServo;
    double alignVal;
    @Override
    public void runOpMode() {
        WebcamName cam1 = hardwareMap.get(WebcamName.class, "Camera1");
        Boolean alignValue=false;
        double turnSpeed =0.1;
        //camera.setCamera("Camera1");
        camera.setTarget(Target.blue);
        //Vision.DevModeOn();
        waitForStart();
        camera.aprilTagSetUp(cam1);

        final int CYCLE_MS = 50;

        double speed = 1.0;   // used to manage half speed, defaults to full speed
        boolean invertDir = false;  // used for inverted direction prompts
        int invDir = 1;    // used to activate inverted direction
        boolean keyA = false, keyB = false;    // used for toggle keys

        double C_LATERAL, C_AXIAL, C_YAW;
        boolean C_HALF_SPEED, C_INV_DIR, C_INTAKE, C_TRANSFER_PA, C_TRANSFER_PB, C_TRANSFER_PC, C_FLICK;

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "right_back_drive");

        outtake_motor   = hardwareMap.get(DcMotor.class, "outtake_drive");
        //outtake_motor.setPower(-1.0);

        intakeServo   = hardwareMap.get(Servo.class, "intake_servo");
        transferServo = hardwareMap.get(Servo.class, "transfer_servo");
        flickServo    = hardwareMap.get(Servo.class, "flick_servo");
        double tranferPosNeutral = 0.2 * (16.0/22.0);
        double tranferPosA = 0.0 * (16.0/24.0); // gear ratio
        double tranferPosB = 0.4 * (16.0/24.0);
        double tranferPosC = 0.8 * (16.0/24.0);

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive axial.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick axial and observe the direction the wheels turn.
        // Reverse the direction (flip axial <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot axial when you push the left joystick axial.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");

        telemetry.update();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // KEYBINDS
            /*
             * 1) Axial:    Driving axial and backward               Left-joystick axial/Backward
             * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
             * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
             */
            C_AXIAL       = gamepad1.left_stick_y;
            C_LATERAL     = gamepad1.left_stick_x;
            C_YAW         = gamepad1.right_stick_x;
            C_HALF_SPEED  = gamepad1.a;
            C_INV_DIR     = gamepad1.b;
            C_INTAKE      = gamepad2.x;
            C_FLICK       = gamepad2.y;
            C_TRANSFER_PA = gamepad2.dpad_left;
            C_TRANSFER_PB = gamepad2.dpad_up;
            C_TRANSFER_PC = gamepad2.dpad_right;

            double max;

            // POV Mode uses left joystick to go axial & strafe, and right joystick to yaw.
            double axial   = -C_AXIAL;  // Note: pushing stick axial gives negative value
            double lateral =  C_LATERAL;
            double yaw     =  C_YAW;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double rightBackPower  = axial + lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run axial.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above. -_-
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.dpad_left ? 1.0 : 0.0;  // left gamepad
            leftBackPower   = gamepad1.dpad_down ? 1.0 : 0.0;  // down gamepad
            rightFrontPower = gamepad1.dpad_up ? 1.0 : 0.0;  // up gamepad
            rightBackPower  = gamepad1.dpad_right ? 1.0 : 0.0;  // right gamepad
            */

            // HALF SPEED CONTROLS
            if (C_HALF_SPEED) {
                if (keyA == false) {
                    keyA = !keyA;
                    switch ((int)(speed * 10)) {
                        case 10 : speed = 0.5; break;
                        case 5 : speed = 1.0; break;
                    }
                }
            } else {
                keyA = false;
            }

            // INVERTED DIRECTION CONTROLS
            if (C_INV_DIR) {
                if (keyB == false) {
                    keyB = !keyB;
                    invertDir = !invertDir;
                    if (invertDir) {
                        invDir = -1;
                    } else {
                        invDir = 1;
                    }
                }
            } else {
                keyB = false;
            }

            if (C_INTAKE) {
                intakeServo.setPosition(0.0);
            } else {
                intakeServo.setPosition(0.5);
            }

            if (C_TRANSFER_PA) {
                transferServo.setPosition(tranferPosA);
            } else if (C_TRANSFER_PB) {
                transferServo.setPosition(tranferPosB);
            } else if (C_TRANSFER_PC) {
                transferServo.setPosition(tranferPosC);
            } else {
                if (flickServo.getPosition()<= 0.05){
                    transferServo.setPosition(tranferPosNeutral);
                }
            }
            if (C_FLICK) {
               // flickServo.setPosition(0.0);
            } else {
              //  flickServo.setPosition(0.3);
            }
            if(alignValue=false){//&&gamepad1.x){
                alignValue=true;
            }
            if(alignValue){//!gamepad1.x&&alignValue){
                alignVal=camera.alignmentValue();
                if(!(alignVal==0.0)&&!gamepad1.x){
                    alignVal =camera.alignmentValue();
                    if(alignVal==-10000){

                    }else if(alignVal<0){
                        //turn left
                        leftFrontDrive.setPower(-1*turnSpeed);
                        leftBackDrive.setPower(-1*turnSpeed);
                        rightFrontDrive.setPower(1*turnSpeed);
                        rightFrontDrive.setPower(1*turnSpeed);

                    } else if(alignVal>0){
                        //turnright
                        leftFrontDrive.setPower(1*turnSpeed);
                        leftBackDrive.setPower(1*turnSpeed);
                        rightFrontDrive.setPower(-1*turnSpeed);
                        rightFrontDrive.setPower(-1*turnSpeed);
                    }


                }else {
                    alignValue=false;
                    leftFrontDrive.setPower(0);
                    leftBackDrive.setPower(0);
                    rightFrontDrive.setPower(0);
                    rightFrontDrive.setPower(0);
                }
            }
            // if some button and pA empty
            // move servo to pA
            // do we want it?
            // set pA color to whatever color color sensor has
            // intake
            // move servo home
            // no?
            // spit it out
            // if some other button and pB empty
            // move servo to pB
            // do we want it?
            // set pA color to whatever color color sensor has
            // intake
            // move servo home
            // no?
            // spit it out
            // if some other button and pC empty
            // move servo to pC
            // do we want it?
            // set pA color to whatever color color sensor has
            // intake
            // move servo home
            // no?
            // spit it out

            // if some other button
            // find an empty slot
            // move servo
            // intake
            // move servo home

            // if outtake green button
            // does A have green?
            // outtake A
            // does B have green?
            // outtake B
            // does C have green?
            // outtake C
            // else
            // flash lights red
            // if outtake purple button
            // same sequence but for purple

            // Send calculated power to wheels

            leftFrontDrive.setPower(leftFrontPower * speed * invDir);
            rightFrontDrive.setPower(rightFrontPower * speed * invDir);
            leftBackDrive.setPower(leftBackPower * speed * invDir);
            rightBackDrive.setPower(rightBackPower * speed * invDir);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower * speed * invDir, rightFrontPower * speed * invDir);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower * speed * invDir, rightBackPower * speed * invDir);
            telemetry.addData("Speed", "%4.2f", speed);
            telemetry.addData("Invert Direction", "%1b", invertDir);
            telemetry.update();

            sleep(CYCLE_MS);

        }
    }
}
    /*
    public Vision camera = new Vision();




    @Override
    public void runOpMode(){

        WebcamName cam1 = hardwareMap.get(WebcamName.class, "Camera1");
        //camera.setCamera("Camera1");
        camera.setTarget(Target.blue);
        //Vision.DevModeOn();
        waitForStart();
        camera.aprilTagSetUp(cam1);

        while(opModeIsActive()){
            telemetry.addData("horiz: ",camera.horizAngle());
            telemetry.addData("vert: ",camera.vertAngle());
            telemetry.addData("distance in cm: ",camera.centerDistanceCM());
            telemetry.addData("bearing: ",camera.getBearing());
            telemetry.addData("align: ",camera.align());
            telemetry.update();
        }

    }*/


    

