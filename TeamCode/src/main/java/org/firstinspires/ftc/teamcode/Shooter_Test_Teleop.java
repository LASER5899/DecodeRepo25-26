package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.old.redo.VerticalSlide;
import org.firstinspires.ftc.teamcode.shooter.ShooterControl;
import org.firstinspires.ftc.teamcode.tuning.shooter.RobotConstants;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Shooter Test Teleop", group="Linear OpMode")
public class Shooter_Test_Teleop extends LinearOpMode {
    private DcMotorEx flywheel;
    VoltageSensor battery;
    private ShooterControl shooter;
    private Servo transferServo;
    private Servo flickServo;
    private boolean toggle = true;

    public enum outtakeState {
        out1, out2, out3, kick, down, rest
    }
    private Shooter_Test_Teleop.outtakeState state = Shooter_Test_Teleop.outtakeState.down;

    @Override
    public void runOpMode() {

        final int CYCLE_MS = 10; //default is 50

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();

        shooter = new ShooterControl(hardwareMap);
        flywheel = hardwareMap.get(DcMotorEx.class, "outtake_drive");
        flywheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setDirection(DcMotorEx.Direction.REVERSE);

        transferServo = hardwareMap.get(Servo.class, "transfer_servo");
        flickServo = hardwareMap.get(Servo.class, "flick_servo");

        double transferPosCOut = 0.647;
        double transferPosAOut = 0.575;
        double transferPosBOut = 0.497;
        double presentVoltage;
        double postrack = transferPosAOut;

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        transferServo.setPosition(transferPosAOut);

        waitForStart();

        while (opModeIsActive()) {
            presentVoltage = battery.getVoltage();
            shooter.setBatteryVoltage(presentVoltage);

            shooter.setTargetRPM(RobotConstants.TARGET_RPM);

            shooter.setKf(RobotConstants.kF);
            shooter.setKp(RobotConstants.kP);
            shooter.setKi(RobotConstants.kI);
            shooter.setKd(RobotConstants.kD);

            shooter.setMaxAccel(RobotConstants.maxAccel);

            shooter.flywheelHold();



            /*if(gamepad2.a) {
                switch (state) {
                    case out1:
                        transferServo.setPosition(transferPosCOut);
                        postrack = transferPosCOut;
                        state = Shooter_Test_Teleop.outtakeState.kick;
                        break;

                    case out2:
                        transferServo.setPosition(transferPosAOut);
                        postrack = transferPosAOut;
                        break;

                    case out3:
                        transferServo.setPosition(transferPosBOut);
                        postrack = transferPosBOut;
                        state = Shooter_Test_Teleop.outtakeState.kick;
                        break;

                    case kick:
                        flickServo.setPosition(0.0);
                        state = Shooter_Test_Teleop.outtakeState.down;
                        break;

                    case down:
                        flickServo.setPosition(0.3);
                        if (postrack == transferPosCOut) {
                            state = Shooter_Test_Teleop.outtakeState.out2;
                        }
                        if (postrack == transferPosAOut) {
                            state = Shooter_Test_Teleop.outtakeState.out3;
                        }
                        if (postrack == transferPosBOut) {
                            state = Shooter_Test_Teleop.outtakeState.rest;
                        }
                        break;

                    case rest:
                        transferServo.setPosition(transferPosCOut);
                        break;
                }
            }*/


            if (gamepad2.y) {
                flickServo.setPosition(0.0);

            } else {
                flickServo.setPosition(0.3);
            }


            if (gamepad2.dpad_right && postrack == transferPosAOut){
                transferServo.setPosition(transferPosBOut);
                postrack = transferPosBOut;
            }

            else if (gamepad2.dpad_right && postrack == transferPosBOut){
                transferServo.setPosition(transferPosCOut);
                postrack = transferPosCOut;
            }
            else if (gamepad2.dpad_right && postrack == transferPosCOut){
                transferServo.setPosition(transferPosAOut);
                postrack = transferPosAOut;
            }

            //telemetry.addData("Status", "Run Time:" + runtime.toString());
            telemetry.addData("actualRPM", shooter.getActualRPM());  // smoothedRPM
            telemetry.addData("rawRPM", shooter.getRawRPM());        // currentRPM
            telemetry.addData("targetRPM", shooter.getTargetRPM());  // what you command
            telemetry.addData("rampingRPM", shooter.getRampingRPM());
            telemetry.addData("power", shooter.getPower());
            telemetry.addData("pos", postrack);



            telemetry.update();

            //sleep(CYCLE_MS); TODO: might need this line
            sleep(10);

        }
    }
}