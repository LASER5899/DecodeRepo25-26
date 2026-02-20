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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision;
import org.firstinspires.ftc.teamcode.classes.Transfer_Values;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.shooter.ShooterControl;
import org.firstinspires.ftc.teamcode.tuning.shooter.RobotConstants;

@Config
@Autonomous(name = "!!!!sixsevennnn new", group = "Autonomous")
//psuedocode
/*

 */
public class cb6_inorder_new extends LinearOpMode{

    // if odometry is not properly tuned or constantly being retuned:
    // you MIGHT find it useful to change these values and use multiples of them instead of direct number
    // keep in mind that this may not work well
    // i.e. if something is wrong with acceleration/deceleration, two lengths may not be equal to 2 * (one length)
    double quarter = 90; // "90 degrees" / right angle turn
    double tile = 24; // "24 inches" / one tile

    private Transfer_Values transferValues;
    private VoltageSensor battery;


    double bIn = 0.07;//0.07;
    double cOut = 0.105;//0.100;
    double aIn = 0.14;//0.145;
    double bOut = 0.175;//0.175;
    double cIn = 0.21;//0.21;
    double aOut = 0.250;//0.240;
    double rest = 0.0875;//0.4;

    private ShooterControl flywheel;

    //VoltageSensor battery;

    //mechanism instantiation

    public String sequ = "PPG";

    public class intakeServo {
        private CRServo intake;
        public intakeServo(HardwareMap hwMap) {
            intake = hardwareMap.get(CRServo.class, "intake_servo");
        }

        public class Intaking implements Action {
            private boolean started = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setPower(-1.0);
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
                intake.setPower(0.0);
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
        private final double move_time = 1.2;
        public transferServo(HardwareMap hwMap) {
            transfer = hardwareMap.get(Servo.class, "transfer_servo");
        }

        public class ShootSet1 implements Action {
            private boolean started = false;
            private String sequence;
            private final ElapsedTime t1 = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!started) {
                    t1.reset();
                    if (sequence == null) {
                        sequence = "GPP"; // fallback
                    }
                    flickServo flicker = new flickServo(hardwareMap);
                    // ---- Shot 1 position ----
                    if ("GPP".equals(sequ)) {
                        transfer.setPosition(aOut);
                    } else if ("PGP".equals(sequ)) {
                        transfer.setPosition(bOut);
                    } else if ("PPG".equals(sequ)) {
                        transfer.setPosition(bOut);   // P
                    } else {
                        transfer.setPosition(aOut);
                    }
                    timer.reset();
                    started = true;
                }
                return false; // true reruns action
            }
        }
        //public Action shootSet1(String sequence){ return new ShootSet1(sequence); }
        public Action shootSet1(){ return new ShootSet1(); }

        public class ShootSet2 implements Action {
            private boolean started = false;
            private String sequence;
            private final ElapsedTime t1 = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!started) {
                    t1.reset();
                    if (sequence == null) {
                        sequence = "GPP"; // fallback
                    }
                    flickServo flicker = new flickServo(hardwareMap);

                    // ---- Shot 2 position ----
                    if ("GPP".equals(sequ)) {
                        transfer.setPosition(bOut);
                    } else if ("PGP".equals(sequ)) {
                        transfer.setPosition(aOut);
                    } else if ("PPG".equals(sequ)) {
                        transfer.setPosition(cOut);   // P (second purple uses the other pos)
                    } else {
                        transfer.setPosition(bOut);
                    }

                    timer.reset();
                    started = true;
                }
                return t1.seconds() < 1.7; // true reruns action
            }
        }
        //public Action shootSet1(String sequence){ return new ShootSet1(sequence); }
        public Action shootSet2(){ return new ShootSet1(); }

        public class ShootSet3 implements Action {
            private boolean started = false;
            private String sequence;
            private final ElapsedTime t2 = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!started) {
                    t2.reset();
                    if (sequence == null) {
                        sequence = "GPP"; // fallback
                    }
                    flickServo flicker = new flickServo(hardwareMap);

                    // ---- Shot 3 position ----
                    if ("GPP".equals(sequ)) {
                        transfer.setPosition(cOut);
                    } else if ("PGP".equals(sequ)) {
                        transfer.setPosition(cOut);
                    } else if ("PPG".equals(sequ)) {
                        transfer.setPosition(aOut);   // G (your standardized green position)
                    } else {
                        transfer.setPosition(cOut);
                    }
                    sleep(1200);
                    Actions.runBlocking(new SequentialAction(flicker.kick(), flicker.goBack()));

                    timer.reset();
                    started = true;
                }
                return t2.seconds() < 1.7; // true reruns action
            }
        }
        //public Action shootSet1(String sequence){ return new ShootSet1(sequence); }
        public Action shootSet3(){ return new ShootSet1(); }
        public class ToAOut implements Action {
            private boolean started = false;

            private final ElapsedTime t3 = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!started) {
                    transfer.setPosition(aOut);
                    timer.reset();
                    started = true;
                }
                return t3.seconds() < 1.7; // true reruns action
            }
        }
        public Action toAOut(){
            return new ToAOut();
        }

        public class ToBOut implements Action {
            private boolean started = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!started) {
                    transfer.setPosition(bOut);
                    timer.reset();
                    started = true;
                }
                return timer.seconds() < move_time; // true reruns action
            }
        }
        public Action toBOut(){
            return new ToBOut();
        }

        public class ToCOut implements Action {
            private boolean started = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!started) {
                    transfer.setPosition(cOut);
                    timer.reset();
                    started = true;
                }
                return timer.seconds() < move_time; // true reruns action
            }
        }
        public Action toCOut(){
            return new ToCOut();
        }

        public class ToAIn implements Action {
            private boolean started = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!started) {
                    transfer.setPosition(aIn);
                    timer.reset();
                    started = true;
                }
                return timer.seconds() < move_time; // true reruns action
            }
        }
        public Action toAIn(){
            return new ToAIn();
        }

        public class ToBIn implements Action {
            private boolean started = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!started) {
                    transfer.setPosition(bIn);
                    timer.reset();
                    started = true;
                }
                return timer.seconds() < move_time; // true reruns action
            }
        }
        public Action toBIn(){
            return new ToBIn();
        }

        public class ToCIn implements Action {
            private boolean started = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!started) {
                    transfer.setPosition(cIn);
                    timer.reset();
                    started = true;
                }
                return timer.seconds() < move_time + 1; // true reruns action
            }
        }
        public Action toCIn(){
            return new ToCIn();
        }

        public class ToNeutral implements Action {
            private boolean started = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!started) {
                    transfer.setPosition(rest);
                    timer.reset();
                    started = true;
                }
                return timer.seconds() < move_time; // true reruns action
            }
        }
        public Action toNeutral(){
            return new ToNeutral();
        }
    }

    public class flickServo {
        private final ElapsedTime timer = new ElapsedTime();
        private final double move_time = 0.1;
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
                    timer.reset();
                    //flicker.setPosition(0.0);
                    started = true;
                }
                flicker.setPosition(0.0);
                return timer.seconds() <= 0.4; // true reruns action
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
                    timer.reset();
                    started = true;

                }
                flicker.setPosition(0.3);
                return timer.seconds() <= 0.4; // true reruns action
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

            private boolean started = false;
            private final ElapsedTime t = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!started){t.reset(); started=true;}
                double dt = t.seconds();
                power += dt * 100;
                power = Range.clip(power, 0, 0.8);
                shooter.setPower(power);
                t.reset();
                return power < 0.7; // true reruns action
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
                flywheel.setBatteryVoltage(battery.getVoltage());
                flywheel.setvoltCorr(1.5);
                flywheel.setKf(0.00083);
                flywheel.setKp(0.009);
                flywheel.setKi(0);
                flywheel.setKd(0.0009);
                flywheel.setTargetRPM(820);
                flywheel.flywheelHold();
                return true; // true reruns action
            }
        }
        public Action hold(){
            return new Hold();
        }

        public class Stop implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //shooter.setPower(0);
                flywheel.setTargetRPM(0);
                flywheel.flywheelHold();
                return false; // true reruns action
            }
        }
        public Action stop(){
            return new Stop();
        }
    }

    public class camera {

        public Vision camera = new Vision();

        private final ElapsedTime timercam = new ElapsedTime();

        public String sequence;

        public camera(HardwareMap hwMap) {
            WebcamName cam1 = hardwareMap.get(WebcamName.class, "Camera1");
            camera.aprilTagSetUp(cam1);
            camera.setTarget(Vision.Target.blue);
        }

        public class ScanObelisk implements Action {
            private boolean startedcam = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (!startedcam) {
                    timercam.reset();
                    startedcam = true;
                }

                String sequence = camera.scanForPattern();   // ONLY CALL ONCE
                if (sequence == null) sequence = "none";

                telemetry.addData("cam", "%s", sequence);
                telemetry.addData("t(ms)", "%.0f", timercam.milliseconds());
                telemetry.update();

                sequ = sequence;   // latch valid

                if ("none".equals(sequence) && timercam.milliseconds() < 5000) {
                    return true; // keep scanning
                }

                if ("none".equals(sequence)) {
                    sequ = "GPP";  // timeout fallback
                    return false;
                }

                return false;
            }
        }
        public Action scanObelisk(){
            return new ScanObelisk();
        }
    }


    //begin code
    @Override
    public void runOpMode() throws InterruptedException {

        battery = hardwareMap.voltageSensor.iterator().next();

        flywheel = new ShooterControl(hardwareMap);

        Pose2d pose0 = new Pose2d(0, 0, Math.toRadians(0));
        Pose2d pose1 = new Pose2d(45, 20, Math.toRadians(-20));
        Pose2d pose2 = new Pose2d(45, 20, Math.toRadians(50));
        Pose2d pose3 = new Pose2d(49, 10, Math.toRadians(-90));
        Pose2d pose4 = new Pose2d(48, -17, Math.toRadians(-90));
        Pose2d pose5 = new Pose2d(45, 20, Math.toRadians(50));
        Pose2d pose6 = new Pose2d(73, 10, Math.toRadians(-90));
        Pose2d pose7 = new Pose2d(73, -20, Math.toRadians(-90));
        Pose2d pose8 = new Pose2d(45, 20, Math.toRadians(50));
        MecanumDrive drive = new MecanumDrive(hardwareMap, pose0);
        outtakeMotor shooter = new outtakeMotor(hardwareMap);
        transferServo transfer = new transferServo(hardwareMap);
        intakeServo intake = new intakeServo(hardwareMap);
        flickServo flicker = new flickServo(hardwareMap);
        camera beginTs = new camera(hardwareMap);

        TrajectoryActionBuilder test = drive.actionBuilder(pose0)
                .strafeToConstantHeading(new Vector2d(-3, 0), new TranslationalVelConstraint(50))
                .turnTo(Math.toRadians(30))
                .strafeToConstantHeading(new Vector2d(-26, -15), new TranslationalVelConstraint(50)) //counterclockwise by default
                .turnTo(Math.toRadians(270))
                .strafeToConstantHeading(new Vector2d(-26, -40), new TranslationalVelConstraint(10))
                .strafeToConstantHeading(new Vector2d(-3, 0), new TranslationalVelConstraint(50))
                .turnTo(Math.toRadians(30))
                .strafeToConstantHeading(new Vector2d(-50, -15), new TranslationalVelConstraint(50))
                .turnTo(Math.toRadians(270))
                .strafeToConstantHeading(new Vector2d(-50, -40), new TranslationalVelConstraint(10))
                .strafeToLinearHeading(new Vector2d(-3, 0), Math.toRadians(30), new TranslationalVelConstraint(50))
                .strafeToConstantHeading(new Vector2d(-15, -15), new TranslationalVelConstraint(50));

        TrajectoryActionBuilder one = drive.actionBuilder(pose0)
                .strafeToLinearHeading(new Vector2d(45, 20), Math.toRadians(-20));//, new TranslationalVelConstraint(10));

        TrajectoryActionBuilder two = drive.actionBuilder(pose1)
                .turnTo(Math.toRadians(50));

        TrajectoryActionBuilder three = drive.actionBuilder(pose2)
                .strafeToLinearHeading(new Vector2d(49, 10), Math.toRadians(-90));//, new TranslationalVelConstraint(10)); //counterclockwise by default

        TrajectoryActionBuilder four = drive.actionBuilder(pose3)
                .strafeToConstantHeading(new Vector2d(48, -17), new TranslationalVelConstraint(15));

        TrajectoryActionBuilder five = drive.actionBuilder(pose4)
                .strafeToConstantHeading(new Vector2d(45, 20))//, new TranslationalVelConstraint(10))
                .turnTo(Math.toRadians(50));

        TrajectoryActionBuilder six = drive.actionBuilder(pose5)
                .strafeToConstantHeading(new Vector2d(73, 10))//, new TranslationalVelConstraint(10))
                .turnTo(Math.toRadians(-90));

        TrajectoryActionBuilder seven = drive.actionBuilder(pose6)
                .strafeToConstantHeading(new Vector2d(73, -20), new TranslationalVelConstraint(15));

        TrajectoryActionBuilder eight = drive.actionBuilder(pose7)
                .strafeToConstantHeading(new Vector2d(45, 20))//, new TranslationalVelConstraint(10))
                .turnTo(Math.toRadians(50));

        TrajectoryActionBuilder nine = drive.actionBuilder(pose8)
                .strafeToConstantHeading(new Vector2d(50, 5));//, new TranslationalVelConstraint(10));

        // actions that need to happen on init



        waitForStart();

        if (isStopRequested()) return;



        Actions.runBlocking(
                new SequentialAction(

                        shooter.fireUp(),
                        new ParallelAction(
                                shooter.hold(),
                                intake.intaking(),
                                transfer.toAOut(),
                                new SequentialAction(

                                        one.build(),

                                        beginTs.scanObelisk(),

                                        two.build(),

                                        transfer.shootSet1(),
                                        flicker.kick(),
                                        flicker.goBack(),
                                        transfer.shootSet2(),
                                        flicker.kick(),
                                        flicker.goBack(),
                                        transfer.shootSet3(),
                                        flicker.kick(),
                                        flicker.goBack(),

                                        three.build(),

                                        new ParallelAction( //TODO: the transfer timer should be longer for intaking than for outtaking
                                                four.build(),
                                                new SequentialAction(
                                                        transfer.toAIn(),
                                                        transfer.toBIn(),
                                                        transfer.toCIn(),
                                                        transfer.toNeutral()
                                                )
                                        ),
                                        five.build(),

                                        transfer.shootSet1(),
                                        flicker.kick(),
                                        flicker.goBack(),
                                        transfer.shootSet2(),
                                        flicker.kick(),
                                        flicker.goBack(),
                                        transfer.shootSet3(),
                                        flicker.kick(),
                                        flicker.goBack(),

                                        /*
                                        five.build(),

                                        new ParallelAction( //TODO: the transfer timer should be longer for intaking than for outtaking
                                                six.build(),
                                                new SequentialAction(
                                                        transfer.toAOut(),
                                                        transfer.toBOut(),
                                                        transfer.toCOut(),
                                                        transfer.toNeutral()
                                                )
                                        ),
                                        seven.build(),

                                        transfer.toAIn(),
                                        flicker.kick(),
                                        flicker.goBack(),
                                        transfer.toBIn(),
                                        flicker.kick(),
                                        flicker.goBack(),
                                        transfer.toCIn(),
                                        flicker.kick(),
                                        flicker.goBack(),
                                        */
                                        nine.build(),

                                        shooter.stop(),
                                        intake.stopIntaking()
                                )
                        )
                )
        );
    }
}

