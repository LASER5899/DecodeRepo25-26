package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Classes.LEDlights;

@TeleOp(name="LASER Main Teleop", group="Linear OpMode")
//@Disabled
public class OLD_LASER_Teleop extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private DcMotor slideVertical;
    private DcMotor slideHorizontal;
    private DcMotor wristMotor;
    private Servo   outtakeServo;
    private Servo   intakeServo;
    private Servo   blockPushServo;
    private LEDlights lights;

    @Override
    public void runOpMode() {

        final int CYCLE_MS = 50;

        double speed = 1.0;   // used to manage half speed, defaults to full speed
        boolean invertDir = false;  // used for inverted direction prompts
        int invDir = 1;    // used to activate inverted direction
        boolean keyA = false, keyB = false;    // used for toggle keys

        double C_LATERAL, C_AXIAL, C_YAW, C_HORIZ_SLIDE, C_HORIZ_SLIDE_RESET;
        boolean C_HALF_SPEED, C_INV_DIR, C_OUT_SERVO,
                C_IN_SERVO_TRANSF, C_INTAKE, C_SPIT,
                C_VERT_SLIDE_UP = false, PREV_C_VERT_SLIDE_UP = false,
                C_VERT_SLIDE_DWN = false, PREV_C_VERT_SLIDE_DWN = false,
                C_BPUSHSERVO, C_BPUSHSERVO_LOAD, C_VERT_SLIDE_RESET;

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "right_back_drive");

        lights = new LEDlights(hardwareMap, "light_strip");
        intakeServo    = hardwareMap.get(Servo.class, "intake_servo");
        blockPushServo = hardwareMap.get(Servo.class, "block_push_servo");
        double bPushServoSet;
        outtakeServo   = hardwareMap.get(Servo.class, "outtake_servo");
        final double OUT_SERVO_DOWN_POS = 0.7;
        final double OUT_SERVO_UP_POS   = 0.06;
        final double SERVO_SPEED        = -20;
        double outtakeServoPosition     = outtakeServo.getPosition();

        slideHorizontal = hardwareMap.get(DcMotor.class, "horizontal_slide");
        slideHorizontal.setDirection(DcMotor.Direction.REVERSE);
        slideHorizontal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideHorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideHorizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /*slideHorizontal.setTargetPosition(0);
        slideHorizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideHorizontal.setPower(1.0);
        int hSlidePos = 0; */

        slideVertical = hardwareMap.get(DcMotor.class, "vertical_slide");
        slideVertical.setDirection(DcMotor.Direction.FORWARD);
        slideVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideVertical.setTargetPosition(0);
        slideVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideVertical.setPower(1.0);
        int vSlideMotorState = 0;

        wristMotor = hardwareMap.get(DcMotor.class, "wrist_drive");
        wristMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wristMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wristMotor.setTargetPosition(0);
        wristMotor.setPower(0.5);
        wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double targetTime = 0.0;

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
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
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // KEYBINDS
            /*
             * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
             * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
             * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
             */
            C_AXIAL               = gamepad1.left_stick_y;
            C_LATERAL             = gamepad1.left_stick_x;
            C_YAW                 = gamepad1.right_stick_x;
            C_HALF_SPEED          = gamepad1.a;
            C_INV_DIR             = gamepad1.b;
            C_BPUSHSERVO          = gamepad1.right_bumper;
            C_BPUSHSERVO_LOAD     = gamepad1.left_bumper;
            PREV_C_VERT_SLIDE_UP  = C_VERT_SLIDE_UP;
            C_VERT_SLIDE_UP       = gamepad2.right_bumper;
            PREV_C_VERT_SLIDE_DWN = C_VERT_SLIDE_DWN;
            C_VERT_SLIDE_DWN      = gamepad2.left_bumper;
            C_VERT_SLIDE_RESET    = gamepad2.dpad_down;
            C_HORIZ_SLIDE         = gamepad2.left_stick_y;
            C_HORIZ_SLIDE_RESET   = gamepad2.right_trigger + gamepad2.left_trigger;
            C_OUT_SERVO           = gamepad2.b;
            C_IN_SERVO_TRANSF     = gamepad2.y;
            C_SPIT                = gamepad2.x;
            C_INTAKE              = gamepad2.a;







            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -C_AXIAL;  // Note: pushing stick forward gives negative value
            double lateral = -C_LATERAL;
            double yaw     =  C_YAW;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = -axial + lateral + yaw;
            double rightFrontPower = -axial - lateral - yaw;
            double leftBackPower   = -axial - lateral + yaw;
            double rightBackPower  =  axial - lateral + yaw;

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
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
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

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower * speed * invDir);
            rightFrontDrive.setPower(rightFrontPower * speed * invDir);
            leftBackDrive.setPower(leftBackPower * speed * invDir);
            rightBackDrive.setPower(rightBackPower * speed * invDir);

            // VERTICAL SLIDE CONTROLS
            // 157 = 1 INCH
            switch (vSlideMotorState) {
                case 0:
                    if (C_VERT_SLIDE_UP && !PREV_C_VERT_SLIDE_UP) {vSlideMotorState = 1;}
                    slideVertical.setPower(1.0);
                    slideVertical.setTargetPosition(78);
                    break;
                case 1:
                    if (C_VERT_SLIDE_UP && !PREV_C_VERT_SLIDE_UP) {vSlideMotorState = 2;}
                    else if (C_VERT_SLIDE_DWN && !PREV_C_VERT_SLIDE_DWN) {vSlideMotorState = 0;}
                    slideVertical.setPower(1.0);
                    slideVertical.setTargetPosition(2669);
                    break;
                case 2:
                    if (C_VERT_SLIDE_DWN && !PREV_C_VERT_SLIDE_DWN) {vSlideMotorState = 1;}
                    slideVertical.setPower(0.7);
                    slideVertical.setTargetPosition(5024);
                    break;
            }
            while (C_VERT_SLIDE_RESET && opModeIsActive()) {
                slideVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slideVertical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slideVertical.setPower(-0.5);
                wristMotor.setPower(0.2);
                wristMotor.setTargetPosition(130);
                vSlideMotorState = 0;
                sleep(500);
                C_VERT_SLIDE_RESET = gamepad2.dpad_down;
            }
            slideVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // HORIZONTAL SLIDE CONTROLS
            /*
            if (hSlidePos < 0) {
                hSlidePos = 0;
                slideHorizontal.setTargetPosition(hSlidePos);
            } else if (hSlidePos > 1000) {
                hSlidePos = 1000;
                slideHorizontal.setTargetPosition(hSlidePos);
            } else {
                hSlidePos += (int)(C_HORIZ_SLIDE * 100);
                slideHorizontal.setTargetPosition(hSlidePos);
            } */
            if (C_HORIZ_SLIDE > 0 && slideHorizontal.getCurrentPosition() < 0) {
                slideHorizontal.setPower(C_HORIZ_SLIDE);
            } else if (C_HORIZ_SLIDE < 0 && slideHorizontal.getCurrentPosition() > -1700) {
                slideHorizontal.setPower(C_HORIZ_SLIDE);
            } else {
                slideHorizontal.setPower(0);
            }
            while (C_HORIZ_SLIDE_RESET >= 1.0 && opModeIsActive()) {
                slideHorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slideHorizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slideHorizontal.setPower(0.5);
                sleep(500);
                C_HORIZ_SLIDE_RESET = gamepad2.right_trigger + gamepad2.left_trigger;
            }
            if (slideHorizontal.getCurrentPosition() > -200) {
                lights.setColor(LEDlights.yellowColor);
            } else if (slideHorizontal.getCurrentPosition() < -1500) {
                lights.setColor(LEDlights.redFireColor);
            } else {
                lights.setColor(LEDlights.blueRayColor);
            }

            // OUTTAKE SERVO CONTROLS
            if (C_OUT_SERVO) {
                outtakeServoPosition += SERVO_SPEED;
                if (outtakeServoPosition <= OUT_SERVO_UP_POS) {
                    outtakeServoPosition = OUT_SERVO_UP_POS;
                }
            } else {
                outtakeServoPosition -= SERVO_SPEED;
                if (outtakeServoPosition >= OUT_SERVO_DOWN_POS) {
                    outtakeServoPosition = OUT_SERVO_DOWN_POS;
                }
            }
            outtakeServo.setPosition(outtakeServoPosition);

            // INTAKE SERVO / TRANSFER CONTROLS
            if (C_IN_SERVO_TRANSF && !C_INTAKE) {
                intakeServo.setPosition(1.0); //pick up or transfer
            } else if (C_SPIT) {
                intakeServo.setPosition(0.0); //spit out
            } else {
                intakeServo.setPosition(0.5); //not moving
            }

            // BLOCK PUSH SERVO CONTROLS
            if (C_BPUSHSERVO) {
                blockPushServo.setPosition(0.8);
            } else if (C_BPUSHSERVO_LOAD) {
                blockPushServo.setPosition(1.0);
            } else {
                blockPushServo.setPosition(0.0);
            }

            // INTAKE WRIST CONTROLS
            if (C_INTAKE || C_SPIT) {
                wristMotor.setPower(0.5);
                wristMotor.setTargetPosition(683);
                if (C_INTAKE) {
                    intakeServo.setPosition(1.0);
                }
            } else if (slideVertical.isBusy() && slideVertical.getCurrentPosition() < 1500 && slideVertical.getCurrentPosition() > 150) {
                wristMotor.setPower(0.2);
                wristMotor.setTargetPosition(130); //avoid
            } else if (slideHorizontal.getCurrentPosition() < -300) {
                wristMotor.setPower(0.5);
                wristMotor.setTargetPosition(400); //idle
            } else {
                wristMotor.setPower(0.7);
                wristMotor.setTargetPosition(15); //in
                if (!C_IN_SERVO_TRANSF && !C_SPIT) {
                    intakeServo.setPosition(0.5);
                }
            }
            /*
            if (C_INTAKE) {
                wristMotor.setTargetPosition(100);
                intakeServo.setPosition(1.0);
                wristMotor.setPower(0.4);
                if (wristMotor.getCurrentPosition() > wristMotor.getTargetPosition()) {
                    wristMotor.setPower(0.0);
                }
            } else {
                wristMotor.setTargetPosition(0);
                intakeServo.setPosition(0.5);
                wristMotor.setPower(0.5);
                if (wristMotor.getCurrentPosition() < wristMotor.getTargetPosition()) {
                    wristMotor.setPower(0.0);
                }
            }
            */

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time:" + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower * speed * invDir, rightFrontPower * speed * invDir);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower * speed * invDir, rightBackPower * speed * invDir);
            telemetry.addData("Speed", "%4.2f", speed);
            telemetry.addData("Invert Direction", "%1b", invertDir);
            telemetry.addData("Servo Position", "%4.2f", outtakeServoPosition);
            telemetry.addData("Servo Position", "%4.2f", outtakeServo.getPosition());
            telemetry.addData("Wrist Position", "%4.2f", (double)wristMotor.getCurrentPosition()); //??
            telemetry.addData("Wrist Power", "%4.2f", wristMotor.getPower()); //??
            telemetry.addData("Slide Level", vSlideMotorState);
            telemetry.addData("vSlidePos", slideVertical.getCurrentPosition());
            telemetry.addData("hSlidePower", C_HORIZ_SLIDE);
            //telemetry.addData("hSlidePos", hSlidePos);
            telemetry.addData("hSlidePos", slideHorizontal.getCurrentPosition());

            telemetry.update();

            sleep(CYCLE_MS);

            if (!opModeIsActive()) {
                slideVertical.setTargetPosition(0);
                wristMotor.setPower(0.2);
                wristMotor.setTargetPosition(0);
            }
        }
    }
}