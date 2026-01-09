package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.teamcode.shooter.ShooterControl;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.config.Config;

@TeleOp(name="Shooter Test Teleop", group="Linear OpMode")
public class Shooter_Test_Teleop extends LinearOpMode {

    VoltageSensor battery;


    private ShooterControl shooter;

    private DcMotor outtake_motor;

    private Servo transferServo;
    private Servo flickServo;

    @Override
    public void runOpMode() {

        final int CYCLE_MS = 50;

        battery = hardwareMap.voltageSensor.iterator().next();




        boolean isEnter = false;

        double C_LATERAL, C_AXIAL, C_YAW;
        double correctPow;
        boolean C_HALF_SPEED, C_INV_DIR, C_INTAKE, C_TRANSFER_PA, C_TRANSFER_PB, C_TRANSFER_PC, C_FLICK = false, C_MOVE_LEFT, C_MOVE_RIGHT;

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        shooter = new ShooterControl(hardwareMap);
        //outtake_motor = hardwareMap.get(DcMotor.class, "outtake_drive");
        //double outtakeMotorPower = -0.75;

        transferServo = hardwareMap.get(Servo.class, "transfer_servo");
        flickServo = hardwareMap.get(Servo.class, "flick_servo");

        double tranferPosCOut = 0.647;
        double tranferPosAOut = 0.575;
        double tranferPosBOut = 0.497;
        double postrack = tranferPosCOut;


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        transferServo.setPosition(tranferPosCOut);

        waitForStart();

        while (opModeIsActive()) {
            shooter.setBatteryVoltage(battery.getVoltage());
            shooter.setTargetRPM(940);
            shooter.flywheelHold();

            // KEYBINDS
            /*
             * 1) Axial:    Driving axial and backward               Left-joystick axial/Backward
             * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
             * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
             */
            C_AXIAL = gamepad1.left_stick_y;
            C_LATERAL = gamepad1.left_stick_x;
            C_YAW = gamepad1.right_stick_x;
            C_HALF_SPEED = gamepad1.a;
            C_INV_DIR = gamepad1.b;
            C_INTAKE = gamepad2.x;
            C_FLICK = gamepad2.y;
            C_MOVE_LEFT = gamepad2.dpad_left;
            C_MOVE_RIGHT = gamepad2.dpad_right;

            double max;

            // POV Mode uses left joystick to go axial & strafe, and right joystick to yaw.
            double axial = -C_AXIAL;  // Note: pushing stick axial gives negative value
            double lateral = C_LATERAL;
            double yaw = C_YAW;




            if (gamepad2.y) {
                flickServo.setPosition(0.0);
                isEnter = true;
            } else {
                flickServo.setPosition(0.3);
            }

            if (gamepad2.dpad_left){
                transferServo.setPosition(tranferPosCOut);
                postrack = tranferPosCOut;
            }

            if (gamepad2.dpad_right && postrack == tranferPosCOut){
                transferServo.setPosition(tranferPosAOut);
                postrack = tranferPosAOut;
            }
            else if (gamepad2.dpad_right && postrack == tranferPosAOut){
                transferServo.setPosition(tranferPosBOut);
                postrack = tranferPosBOut;
            }








            telemetry.addData("flickServo", flickServo.getPosition());
            //telemetry.addData("outtakePower", outtakeMotorPower);
            telemetry.addData("isEnter", isEnter);
            telemetry.addData("gamepad 2 x", gamepad2.x);
            telemetry.update();

            sleep(CYCLE_MS);

        }
    }
}