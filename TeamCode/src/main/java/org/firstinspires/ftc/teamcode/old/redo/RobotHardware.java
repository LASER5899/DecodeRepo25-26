package org.firstinspires.ftc.teamcode.old.redo;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.teamcode.LEDlights;
import org.firstinspires.ftc.teamcode.old.redo.VerticalSlide;
import com.qualcomm.robotcore.hardware.DcMotor;
// THIS IS BASED ON THE OLD TELEOP

public class RobotHardware {
    public DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;
    public DcMotor slideHorizontal, wristMotor;
    public VerticalSlide slideVertical;
    public Servo outtakeServo, intakeServo, blockPushServo;
    public LEDlights lights;

    public static final double OUT_SERVO_DOWN_POS = 0.7;
    public static final double OUT_SERVO_UP_POS   = 0.06;
    public static final double SERVO_SPEED        = -20;

    public void init(HardwareMap hwMap) {
        // map motors and servos
        leftFrontDrive  = hwMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive   = hwMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hwMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive  = hwMap.get(DcMotor.class, "right_back_drive");
        slideHorizontal = hwMap.get(DcMotor.class, "horizontal_slide");
        wristMotor      = hwMap.get(DcMotor.class, "wrist_drive");
        intakeServo     = hwMap.get(Servo.class, "intake_servo");
        blockPushServo  = hwMap.get(Servo.class, "block_push_servo");
        outtakeServo    = hwMap.get(Servo.class, "outtake_servo");
        slideVertical   = new VerticalSlide(hwMap, "vertical_slide");
        lights          = new LEDlights(hwMap, "light_strip");

        // set directions / behaviors / run modes
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideHorizontal.setDirection(DcMotor.Direction.REVERSE);
        slideHorizontal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideHorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideHorizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideVertical.setDirection(DcMotor.Direction.FORWARD);
        slideVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideVertical.setTargetPosition(0);
        slideVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideVertical.setPower(1.0);

        wristMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wristMotor.setTargetPosition(0);
        wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
