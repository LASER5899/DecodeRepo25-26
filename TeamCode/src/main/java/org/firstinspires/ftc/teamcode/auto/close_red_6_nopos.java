package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.shooter.ShooterControl;
@Config
@Autonomous(name = "close red 6 nopos", group = "Autonomous")
//@Disabled
//psuedocode
/*
55 in fwd
turn 45deg clock
shoot 3
125 deg countclock
fwd 35in
intake on
fwd 10 in and run intake seq
back 32 in
135 deg clock
shoot 3
back 15 in
*/
public class close_red_6_nopos extends LinearOpMode {

    double a = 0.575;
    double b = 0.497;
    double c = 0.647;

    private ShooterControl flywheel;

    VoltageSensor battery;

    //mechanism instantiation

    public class intakeServo {
        private Servo intake;
        public intakeServo(HardwareMap hwMap) {
            intake = hardwareMap.get(Servo.class, "intake_servo");
        }

        public class Intaking implements Action {
            private boolean started = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setPosition(0.0);
                return false; // true reruns action
            }
        }
        public Action intaking(){
            return new Intaking();
        }

        public class StopIntaking implements Action {
            private boolean started = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setPosition(0.5);
                return false; // true reruns action
            }
        }
        public Action stopIntaking(){
            return new StopIntaking();
        }
    }

    public class transferServo {
        private Servo transfer;
        private final ElapsedTime timer = new ElapsedTime();
        private final double move_time = 0.5;
        public transferServo(HardwareMap hwMap) {
            transfer = hardwareMap.get(Servo.class, "transfer_servo");
        }

        public class ToA implements Action {
            private boolean started = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!started) {
                    transfer.setPosition(a);
                    timer.reset();
                    started = true;
                }
                return timer.seconds() < move_time; // true reruns action
            }
        }
        public Action toA(){
            return new ToA();
        }

        public class ToB implements Action {
            private boolean started = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!started) {
                    transfer.setPosition(b);
                    timer.reset();
                    started = true;
                }
                return timer.seconds() < move_time; // true reruns action
            }
        }
        public Action toB(){
            return new ToB();
        }

        public class ToC implements Action {
            private boolean started = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!started) {
                    transfer.setPosition(c);
                    timer.reset();
                    started = true;
                }
                return timer.seconds() < move_time; // true reruns action
            }
        }
        public Action toC(){
            return new ToC();
        }
    }

    public class flickServo {
        private final ElapsedTime timer = new ElapsedTime();
        private final double move_time = 0.5;
        private boolean started = false;
        private Servo flicker;
        public flickServo(HardwareMap hwMap) {
            flicker = hardwareMap.get(Servo.class, "flick_servo");
        }

        public class Kick implements Action {
            private boolean started = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!started) {
                    flicker.setPosition(0.0);
                    timer.reset();
                    started = true;
                }
                return timer.seconds() < move_time; // true reruns action
            }
        }
        public Action kick(){
            return new Kick();
        }

        public class GoBack implements Action {
            private boolean started = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!started) {
                    flicker.setPosition(0.3);
                    timer.reset();
                    started = true;

                }
                return timer.seconds() < move_time; // true reruns action
            }
        }
        public Action goBack(){
            return new GoBack();
        }
    }

    public class outtakeMotor {
        private DcMotorEx shooter;
        double power = 0;
        private final ElapsedTime timer = new ElapsedTime();
        double dt = timer.seconds();
        double maxStep = 2;
        private boolean started = false;

        public outtakeMotor(HardwareMap hwMap) {
            shooter = hwMap.get(DcMotorEx.class, "outtake_drive");
            shooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            shooter.setDirection(DcMotorEx.Direction.REVERSE);
        }

        public class FireUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                    power += dt * maxStep;
                    shooter.setPower(power);
                    timer.reset();
                return !(shooter.getPower() > 0.7); // true reruns action
            }
        }
        public Action fireUp(){
            return new FireUp();
        }

        public class SetPower implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                shooter.setPower(0.7);
                return false; // true reruns action
            }
        }
        public Action setPower(){
            return new SetPower();
        }

        public class Hold implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                flywheel.setTargetRPM(940);
                return false; // true reruns action
            }
        }
        public Action hold(){
            return new Hold();
        }

        public class Stop implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                shooter.setPower(0);
                return false; // true reruns action
            }
        }
        public Action stop(){
            return new Stop();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        battery = hardwareMap.voltageSensor.iterator().next();

        flywheel = new ShooterControl(hardwareMap);

        Pose2d initPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);
        outtakeMotor shooter = new outtakeMotor(hardwareMap);
        transferServo transfer = new transferServo(hardwareMap);
        intakeServo intake = new intakeServo(hardwareMap);
        flickServo flicker = new flickServo(hardwareMap);

        TrajectoryActionBuilder one = drive.actionBuilder(initPose)
                .strafeToConstantHeading(new Vector2d(0, 55), new TranslationalVelConstraint(10))
                .turn(Math.toRadians(-45)); //counterclockwise by default

        TrajectoryActionBuilder two = drive.actionBuilder(initPose)
                .turn(Math.toRadians(135)) //counterclockwise by default
                .strafeToConstantHeading(new Vector2d(0, 35), new TranslationalVelConstraint(10));

        TrajectoryActionBuilder three = drive.actionBuilder(initPose)
                .strafeToConstantHeading(new Vector2d(0, 10), new TranslationalVelConstraint(10));

        TrajectoryActionBuilder four = drive.actionBuilder(initPose)
                .strafeToConstantHeading(new Vector2d(0, -32), new TranslationalVelConstraint(10))
                .turn(Math.toRadians(-135)); //counterclockwise by default

        TrajectoryActionBuilder five = drive.actionBuilder(initPose)
                .strafeToConstantHeading(new Vector2d(0, -15), new TranslationalVelConstraint(10));



        // actions that need to happen on init


        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        shooter.fireUp(),
                        shooter.hold(),
                        one.build(),

                        transfer.toA(),
                        flicker.kick(),
                        flicker.goBack(),
                        transfer.toB(),
                        flicker.kick(),
                        flicker.goBack(),
                        transfer.toC(),
                        flicker.kick(),
                        flicker.goBack(),

                        two.build(),
                        new ParallelAction( //TODO: the transfer timer should be longer for intaking than for outtaking
                            intake.intaking(),
                            three.build(),
                            new SequentialAction(
                                transfer.toA(),
                                transfer.toB(),
                                transfer.toC()
                            )
                        ),
                        intake.stopIntaking(),
                        four.build(),

                        transfer.toA(),
                        flicker.kick(),
                        flicker.goBack(),
                        transfer.toB(),
                        flicker.kick(),
                        flicker.goBack(),
                        transfer.toC(),
                        flicker.kick(),
                        flicker.goBack(),

                        five.build(),
                        shooter.stop()

                )

        );


    }
}