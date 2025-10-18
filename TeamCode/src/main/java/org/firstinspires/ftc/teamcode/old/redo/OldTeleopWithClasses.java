package org.firstinspires.ftc.teamcode.old.redo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="second innovate", group="Linear OpMode")
public class OldTeleopWithClasses extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private RobotHardware robot = new RobotHardware();

    private VerticalSlide slideVertical;

    @Override
    public void runOpMode() {

        final int CYCLE_MS = 50;

        robot.init(hardwareMap);

        double speed = 1.0;   // half/full speed toggle
        boolean invertDir = false;
        int invDir = 1;
        boolean keyA = false, keyB = false;

        double C_LATERAL, C_AXIAL, C_YAW, C_HORIZ_SLIDE, C_HORIZ_SLIDE_RESET;
        boolean C_HALF_SPEED, C_INV_DIR, C_OUT_SERVO,
                C_IN_SERVO_TRANSF, C_INTAKE, C_SPIT,
                C_VERT_SLIDE_UP = false, PREV_C_VERT_SLIDE_UP = false,
                C_VERT_SLIDE_DWN = false, PREV_C_VERT_SLIDE_DWN = false,
                C_BPUSHSERVO, C_BPUSHSERVO_LOAD, C_VERT_SLIDE_RESET;

        double outtakeServoPosition = robot.outtakeServo.getPosition();
        //double outtakeServoPosition = 0;
        int vSlideMotorState = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // GAMEPAD INPUTS
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

            double axial   = -C_AXIAL;
            double lateral = -C_LATERAL;
            double yaw     =  C_YAW;

            double leftFrontPower  = -axial + lateral + yaw;
            double rightFrontPower = -axial - lateral - yaw;
            double leftBackPower   = -axial - lateral + yaw;
            double rightBackPower  =  axial - lateral + yaw;

            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // HALF SPEED TOGGLE
            if (C_HALF_SPEED) {
                if (!keyA) {
                    keyA = true;
                    speed = (speed == 1.0) ? 0.5 : 1.0;
                }
            } else keyA = false;

            // INVERTED DRIVE TOGGLE
            if (C_INV_DIR) {
                if (!keyB) {
                    keyB = true;
                    invertDir = !invertDir;
                    invDir = invertDir ? -1 : 1;
                }
            } else keyB = false;

            // DRIVE
            robot.leftFrontDrive.setPower(leftFrontPower * speed * invDir);
            robot.rightFrontDrive.setPower(rightFrontPower * speed * invDir);
            robot.leftBackDrive.setPower(leftBackPower * speed * invDir);
            robot.rightBackDrive.setPower(rightBackPower * speed * invDir);

            // VERTICAL SLIDE STATE MACHINE
            robot.slideVertical.update(gamepad2.right_bumper, gamepad2.left_bumper, gamepad2.dpad_down);

            /*while (C_VERT_SLIDE_RESET && opModeIsActive()) {
                robot.slideVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.slideVertical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.slideVertical.setPower(-0.5);
                robot.wristMotor.setPower(0.2);
                robot.wristMotor.setTargetPosition(130);
                vSlideMotorState = 0;
                sleep(500);
                C_VERT_SLIDE_RESET = gamepad2.dpad_down;
            }*/
            robot.slideVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // HORIZONTAL SLIDE
            if (C_HORIZ_SLIDE > 0 && robot.slideHorizontal.getCurrentPosition() < 0) {
                robot.slideHorizontal.setPower(C_HORIZ_SLIDE);
            } else if (C_HORIZ_SLIDE < 0 && robot.slideHorizontal.getCurrentPosition() > -1700) {
                robot.slideHorizontal.setPower(C_HORIZ_SLIDE);
            } else {
                robot.slideHorizontal.setPower(0);
            }
            while (C_HORIZ_SLIDE_RESET >= 1.0 && opModeIsActive()) {
                robot.slideHorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.slideHorizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.slideHorizontal.setPower(0.5);
                sleep(500);
                C_HORIZ_SLIDE_RESET = gamepad2.right_trigger + gamepad2.left_trigger;
            }

            // OUTTAKE SERVO
            if (C_OUT_SERVO) {
                outtakeServoPosition += -20;
                if (outtakeServoPosition <= RobotHardware.OUT_SERVO_UP_POS)
                    outtakeServoPosition = RobotHardware.OUT_SERVO_UP_POS;
            } else {
                outtakeServoPosition -= -20;
                if (outtakeServoPosition >= RobotHardware.OUT_SERVO_DOWN_POS)
                    outtakeServoPosition = RobotHardware.OUT_SERVO_DOWN_POS;
            }
            robot.outtakeServo.setPosition(outtakeServoPosition);

            // INTAKE / TRANSFER
            if (C_IN_SERVO_TRANSF && !C_INTAKE) robot.intakeServo.setPosition(1.0);
            else if (C_SPIT) robot.intakeServo.setPosition(0.0);
            else robot.intakeServo.setPosition(0.5);

            // BLOCK PUSH SERVO
            if (C_BPUSHSERVO) robot.blockPushServo.setPosition(0.8);
            else if (C_BPUSHSERVO_LOAD) robot.blockPushServo.setPosition(1.0);
            else robot.blockPushServo.setPosition(0.0);

            // WRIST
            if (C_INTAKE || C_SPIT) {
                robot.wristMotor.setPower(0.5);
                robot.wristMotor.setTargetPosition(683);
                if (C_INTAKE) robot.intakeServo.setPosition(1.0);
            } else if (robot.slideVertical.isBusy() && robot.slideVertical.getCurrentPosition() < 1500 && robot.slideVertical.getCurrentPosition() > 150) {
                robot.wristMotor.setPower(0.2);
                robot.wristMotor.setTargetPosition(130);
            } else if (robot.slideHorizontal.getCurrentPosition() < -300) {
                robot.wristMotor.setPower(0.5);
                robot.wristMotor.setTargetPosition(400);
            } else {
                robot.wristMotor.setPower(0.7);
                robot.wristMotor.setTargetPosition(15);
                if (!C_IN_SERVO_TRANSF && !C_SPIT) robot.intakeServo.setPosition(0.5);
            }

            // TELEMETRY
            telemetry.addData("Status", "Run Time:" + runtime.toString());
            telemetry.addData("Front L/R", "%4.2f / %4.2f", leftFrontPower * speed * invDir, rightFrontPower * speed * invDir);
            telemetry.addData("Back  L/R", "%4.2f / %4.2f", leftBackPower * speed * invDir, rightBackPower * speed * invDir);
            telemetry.addData("Speed", "%4.2f", speed);
            telemetry.addData("Invert Direction", "%1b", invertDir);
            telemetry.addData("Outtake Servo Pos", "%4.2f", outtakeServoPosition);
            telemetry.addData("Wrist Pos", robot.wristMotor.getCurrentPosition());
            telemetry.addData("Slide Level", vSlideMotorState);
            telemetry.addData("vSlidePos", robot.slideVertical.getCurrentPosition());
            telemetry.addData("hSlidePower", C_HORIZ_SLIDE);
            telemetry.addData("hSlidePos", robot.slideHorizontal.getCurrentPosition());
            telemetry.update();

            sleep(CYCLE_MS);

            if (!opModeIsActive()) {
                robot.slideVertical.setTargetPosition(0);
                robot.wristMotor.setPower(0.2);
                robot.wristMotor.setTargetPosition(0);
            }
        }
    }
}
