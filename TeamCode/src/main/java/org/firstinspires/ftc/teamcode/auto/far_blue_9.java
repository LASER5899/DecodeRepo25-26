package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.shooter.ShooterControl;
@Config
@Autonomous(name = "far blue 9", group = "Autonomous")
//@Disabled
//psuedocode
/*

 */
public class far_blue_9 extends LinearOpMode{

    // if odometry is not properly tuned or constantly being retuned:
    // you MIGHT find it useful to change these values and use multiples of them instead of direct number
    // keep in mind that this may not work well
    // i.e. if something is wrong with acceleration/deceleration, two lengths may not be equal to 2 * (one length)
    double quarter = 90; // "90 degrees" / right angle turn
    double tile = 24; // "24 inches" / one tile

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
                dt = timer.seconds();
                power += dt * maxStep;
                power = Range.clip(power,0,1);
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


    //begin code
    @Override
    public void runOpMode() throws InterruptedException {

        battery = hardwareMap.voltageSensor.iterator().next();

        flywheel = new ShooterControl(hardwareMap);

        Pose2d pose0 = new Pose2d(0, 0, Math.toRadians(0));
        Pose2d pose2 = new Pose2d(-3, 0, Math.toRadians(30));
        Pose2d pose3 = new Pose2d(-26, 15, Math.toRadians(270));
        Pose2d pose4 = new Pose2d(-26, 40, Math.toRadians(270));
        Pose2d pose5 = new Pose2d(-3, 0, Math.toRadians(30));
        Pose2d pose6 = new Pose2d(-50, 15, Math.toRadians(270));
        Pose2d pose7 = new Pose2d(-50, 40, Math.toRadians(270));
        Pose2d pose8 = new Pose2d(-3, 0, Math.toRadians(30));
        MecanumDrive drive = new MecanumDrive(hardwareMap, pose0);
        outtakeMotor shooter = new outtakeMotor(hardwareMap);
        transferServo transfer = new transferServo(hardwareMap);
        intakeServo intake = new intakeServo(hardwareMap);
        flickServo flicker = new flickServo(hardwareMap);

        TrajectoryActionBuilder test = drive.actionBuilder(pose0)
                .strafeToConstantHeading(new Vector2d(-3, 0), new TranslationalVelConstraint(50))
                .turnTo(Math.toRadians(30))
                .strafeToConstantHeading(new Vector2d(-26, -15), new TranslationalVelConstraint(50)) //counterclockwise by default
                .turnTo(Math.toRadians(270))
                .strafeToConstantHeading(new Vector2d(-26, -40), new TranslationalVelConstraint(15))
                .strafeToConstantHeading(new Vector2d(-3, 0), new TranslationalVelConstraint(50))
                .turnTo(Math.toRadians(30))
                .strafeToConstantHeading(new Vector2d(-50, -15), new TranslationalVelConstraint(50))
                .turnTo(Math.toRadians(270))
                .strafeToConstantHeading(new Vector2d(-50, -40), new TranslationalVelConstraint(15))
                .strafeToLinearHeading(new Vector2d(-3, 0), Math.toRadians(30), new TranslationalVelConstraint(50))
                .strafeToConstantHeading(new Vector2d(-15, -15), new TranslationalVelConstraint(50));

        TrajectoryActionBuilder one = drive.actionBuilder(pose0)
                .strafeToConstantHeading(new Vector2d(-3, 0), new TranslationalVelConstraint(50))
                .turnTo(Math.toRadians(30));

        TrajectoryActionBuilder two = drive.actionBuilder(pose2)
                .strafeToConstantHeading(new Vector2d(-26, -15), new TranslationalVelConstraint(10)) //counterclockwise by default
                .turnTo(Math.toRadians(270))
                .strafeToConstantHeading(new Vector2d(-26, -40), new TranslationalVelConstraint(10));


        TrajectoryActionBuilder three = drive.actionBuilder(pose3)
                .strafeToConstantHeading(new Vector2d(-26, -40), new TranslationalVelConstraint(10));

        TrajectoryActionBuilder four = drive.actionBuilder(pose4)
                .strafeToConstantHeading(new Vector2d(-3, 0), new TranslationalVelConstraint(10))
                .turnTo(Math.toRadians(30));

        TrajectoryActionBuilder five = drive.actionBuilder(pose5)
                .strafeToConstantHeading(new Vector2d(-50, -15), new TranslationalVelConstraint(10))
                .turnTo(Math.toRadians(270));

        TrajectoryActionBuilder six = drive.actionBuilder(pose6)
                .strafeToConstantHeading(new Vector2d(-50, -40), new TranslationalVelConstraint(10));

        TrajectoryActionBuilder seven = drive.actionBuilder(pose7)
                .strafeToConstantHeading(new Vector2d(-3, 0), new TranslationalVelConstraint(10))
                .turnTo(Math.toRadians(30));

        TrajectoryActionBuilder eight = drive.actionBuilder(pose8)
                .strafeToConstantHeading(new Vector2d(-15, -15), new TranslationalVelConstraint(10));

        // actions that need to happen on init



        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        //shooter.fireUp(),
                        //shooter.hold(),
                        //one.build(),

                        /*transfer.toA(),
                        flicker.kick(),
                        flicker.goBack(),
                        transfer.toB(),
                        flicker.kick(),
                        flicker.goBack(),
                        transfer.toC(),
                        flicker.kick(),
                        flicker.goBack(),*/

                        test.build()

                        //one.build(),
                        //two.build(),
                        //three.build(),
                        //four.build(),
                        //five.build(),
                        //six.build(),
                        //seven.build(),
                        //eight.build()
                        /*new ParallelAction( //TODO: the transfer timer should be longer for intaking than for outtaking
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

                        shooter.stop()*/

                )

        );


    }
}

