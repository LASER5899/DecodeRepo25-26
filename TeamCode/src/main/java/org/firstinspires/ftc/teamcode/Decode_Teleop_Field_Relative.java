package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Decode_Teleop_Field_Relative", group="Linear OpMode")
public class Decode_Teleop_Field_Relative extends LinearOpMode {


    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;

    private DcMotor outtake_motor;

    private CRServo intakeServo;
    private Servo transferServo;
    private Servo flickServo;

    private IMU imu;

    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        leftFrontDrive.setPower(frontLeftPower / maxPower);
        rightFrontDrive.setPower(frontRightPower / maxPower);
        leftBackDrive.setPower(backLeftPower / maxPower);
        rightBackDrive.setPower(backRightPower / maxPower);
    }

    @Override
    public void runOpMode() {

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
        double outtakeMotorPower = -0.5;
        boolean prevG2A = false;
        boolean prevG2B = false;

        intakeServo   = hardwareMap.get(CRServo.class, "intake_servo");
        transferServo = hardwareMap.get(Servo.class, "transfer_servo");
        flickServo    = hardwareMap.get(Servo.class, "flick_servo");
        double tranferPosA = 0.68;
        double tranferPosB = 0.61;
        double tranferPosC = 0.535;
        double tranferPosCOut = 0.647;
        double tranferPosAOut = 0.575;
        double tranferPosBOut = 0.497;

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

        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

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

            // If you press the A button, then you reset the Yaw to be zero from the way
            // the robot is currently pointing
            if (gamepad1.x) {
                imu.resetYaw();
            }
            // If you press the left bumper, you get a drive from the point of view of the robot
            // (much like driving an RC vehicle)
            if (gamepad1.right_bumper) {
                drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            } else {
                driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
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
/*
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
*/
            if (C_INTAKE) {
                intakeServo.setPower(-1.0);
            } else {
                intakeServo.setPower(0.0);
            }

            //if (flickServo.getPosition() >= 2.95){
            if (C_TRANSFER_PA && !gamepad2.left_bumper) {
                transferServo.setPosition(tranferPosA);
            } else if (C_TRANSFER_PB && !gamepad2.left_bumper) {
                transferServo.setPosition(tranferPosB);
            } else if (C_TRANSFER_PC && !gamepad2.left_bumper) {
                transferServo.setPosition(tranferPosC);
            } else if (C_TRANSFER_PA && gamepad2.left_bumper){
                transferServo.setPosition(tranferPosAOut);
            } else if (C_TRANSFER_PB && gamepad2.left_bumper){
                transferServo.setPosition(tranferPosBOut);
            } else if (C_TRANSFER_PC && gamepad2.left_bumper){
                transferServo.setPosition(tranferPosCOut);
            } else {
                transferServo.setPosition(tranferPosAOut);
            }
            //}

            if (C_FLICK && gamepad2.left_bumper) {
                flickServo.setPosition(0.0);
            } else {
                flickServo.setPosition(0.3);
            }

            if (gamepad2.a && !prevG2A) {
                outtakeMotorPower -= 0.1;
            } else if (gamepad2.b && !prevG2B) {
                outtakeMotorPower += 0.1;
            }
            outtake_motor.setPower(outtakeMotorPower);
            prevG2A = gamepad2.a;
            prevG2B = gamepad2.b;

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


            // Show the elapsed game time and wheel power.
            //telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower * speed * invDir, rightFrontPower * speed * invDir);
            //telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower * speed * invDir, rightBackPower * speed * invDir);
            telemetry.addData("Speed", "%4.2f", speed);
            telemetry.addData("Invert Direction", "%1b", invertDir);
            telemetry.addData("flickServo", flickServo.getPosition());
            telemetry.addData("outtakePower", outtakeMotorPower);
            telemetry.update();

            sleep(CYCLE_MS);

        }
    }
}
