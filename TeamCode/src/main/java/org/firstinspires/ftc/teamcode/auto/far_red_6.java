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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.shooter.ShooterControl;
import org.firstinspires.ftc.teamcode.classes.Transfer_Values;

@Config
@Autonomous(name = "far red 6", group = "Autonomous")
//@Disabled
//psuedocode
/*

 */
public class far_red_6 extends LinearOpMode{

    // if odometry is not properly tuned or constantly being retuned:
    // you MIGHT find it useful to change these values and use multiples of them instead of direct number
    // keep in mind that this may not work well
    // i.e. if something is wrong with acceleration/deceleration, two lengths may not be equal to 2 * (one length)
    double quarter = 90; // "90 degrees" / right angle turn
    double tile = 24; // "24 inches" / one tile

    private Transfer_Values transferValues;
    private VoltageSensor battery;

    double bIn = 0.07;//0.07;
    double cOut = 0.104;//0.105;!!!!!!!!!!11
    double aIn = 0.14;//0.145;
    double bOut = 0.175;//0.175;
    double cIn = 0.21;//0.21;
    double aOut = 0.250;//0.240;
    double rest = 0.23;//0.4;
    double kick = 0.0;
    double back = 0.3;

    private ShooterControl flywheel;

    //VoltageSensor battery;

    //mechanism instantiation

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
        private final ElapsedTime timeraout = new ElapsedTime();
        private final ElapsedTime timerbout = new ElapsedTime();
        private final ElapsedTime timercout = new ElapsedTime();
        private final ElapsedTime timerain = new ElapsedTime();
        private final ElapsedTime timerbin = new ElapsedTime();
        private final ElapsedTime timercin = new ElapsedTime();
        private final ElapsedTime timerrest = new ElapsedTime();
        private final ElapsedTime timeryay = new ElapsedTime();
        private final double move_time = 1.2;
        private final double move_time_in = 1.7;
        public transferServo(HardwareMap hwMap) {
            transfer = hardwareMap.get(Servo.class, "transfer_servo");
        }

        public class ToAOut implements Action {
            private boolean started = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!started) {
                    transfer.setPosition(aOut);
                    timeraout.reset();
                    started = true;
                }
                return timeraout.seconds() < move_time; // true reruns action
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
                    timerbout.reset();
                    started = true;
                }
                return timerbout.seconds() < move_time; // true reruns action
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
                    timercout.reset();
                    started = true;
                }
                return timercout.seconds() < move_time; // true reruns action
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
                    timerain.reset();
                    started = true;
                }
                return timerain.seconds() < move_time_in; // true reruns action
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
                    timerbin.reset();
                    started = true;
                }
                return timerbin.seconds() < move_time_in; // true reruns action
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
                    timercin.reset();
                    started = true;
                }
                return timercin.seconds() < move_time_in; // true reruns action
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
                    timerrest.reset();
                    started = true;
                }
                return timerrest.seconds() < move_time; // true reruns action
            }
        }
        public Action toNeutral(){
            return new ToNeutral();
        }

        public class TurnTransf implements Action {
            private boolean started = false;
            private final double pos;
            private final double sec;

            public TurnTransf(double pos, double sec) {
                this.pos = pos;
                this.sec = sec;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!started) {
                    transfer.setPosition(pos);
                    timeryay.reset();
                    started = true;
                }
                if(!(timeryay.seconds() < sec)){timeryay.reset(); return false;}
                else{return true;}
                //return timeryay.seconds() < sec; // true reruns action
            }
        }
        public Action turnTransf(double pos, double sec){
            return new TurnTransf(pos,sec);
        }
    }

    public class flickServo {
        private final ElapsedTime timerA = new ElapsedTime();
        private final ElapsedTime timerB = new ElapsedTime();
        private final ElapsedTime timerslay = new ElapsedTime();
        private final double move_time = 0.15;
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
                    //timer.reset();
                    flicker.setPosition(0.0);
                    timerA.reset();
                    started = true;
                }
                //flicker.setPosition(0.0);
                return timerA.seconds() < 0.5; // true reruns action
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
                    //timer.reset();
                    //started = true;
                    flicker.setPosition(0.3);
                    timerB.reset();
                    started = true;

                }
                //flicker.setPosition(0.3);
                return timerB.seconds() < 0.5; // true reruns action
            }
        }
        public Action goBack(){
            return new GoBack();
        }

        public class moveKicker implements Action {
            private boolean started = false;
            private final double pos;
            //private final double sec;

            public moveKicker(double pos) {
                this.pos = pos;
                //this.sec = sec;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!started) {
                    flicker.setPosition(pos);
                    timerslay.reset();
                    started = true;
                }
                if(!(timerslay.seconds() < 1)){timerslay.reset(); return false;}
                else{return true;}
                //return timeryay.seconds() < sec; // true reruns action
            }
        }
        public Action moveKicker(double pos){
            return new moveKicker(pos);
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
                flywheel.setvoltCorr(1);
                flywheel.setKf(0.00083);
                flywheel.setKp(0.009);
                flywheel.setKi(0);
                flywheel.setKd(0.0009);
                flywheel.setTargetRPM(1030);
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

    //begin code
    @Override
    public void runOpMode() throws InterruptedException {

        battery = hardwareMap.voltageSensor.iterator().next();

        flywheel = new ShooterControl(hardwareMap);

        Pose2d pose0 = new Pose2d(0, 0, Math.toRadians(0));
        Pose2d pose2 = new Pose2d(-3, 2, Math.toRadians(-20));
        Pose2d pose3 = new Pose2d(-26, 12, Math.toRadians(-262));
        Pose2d pose4 = new Pose2d(-26, 47, Math.toRadians(-262));
        //Pose2d pose5 = new Pose2d(-3, 3, Math.toRadians(-20));
        //Pose2d pose6 = new Pose2d(-48, 12, Math.toRadians(-270));
        //Pose2d pose7 = new Pose2d(-48, 45, Math.toRadians(-270));
        Pose2d pose8 = new Pose2d(-3, 3, Math.toRadians(-20));
        MecanumDrive drive = new MecanumDrive(hardwareMap, pose0);
        outtakeMotor shooter = new outtakeMotor(hardwareMap);
        transferServo transfer = new transferServo(hardwareMap);
        intakeServo intake = new intakeServo(hardwareMap);
        flickServo flicker = new flickServo(hardwareMap);

        /*TrajectoryActionBuilder test = drive.actionBuilder(pose0)
                .strafeToConstantHeading(new Vector2d(-3, 0), new TranslationalVelConstraint(50))
                .turnTo(Math.toRadians(40))
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
        */
        TrajectoryActionBuilder one = drive.actionBuilder(pose0)
                //.strafeToConstantHeading(new Vector2d(-3, 0))//, new TranslationalVelConstraint(50))
                .strafeToConstantHeading(new Vector2d(-3, 2))//, new TranslationalVelConstraint(50))
                .turnTo(Math.toRadians(-35));

        TrajectoryActionBuilder two = drive.actionBuilder(pose2)
                .strafeToLinearHeading(new Vector2d(-26, 12), Math.toRadians(-262));//285//, new TranslationalVelConstraint(10)); //counterclockwise by default

        TrajectoryActionBuilder three = drive.actionBuilder(pose3)
                //.strafeToConstantHeading(new Vector2d(-26, 45), new TranslationalVelConstraint(10));
                .strafeToConstantHeading(new Vector2d(-26, 47), new TranslationalVelConstraint(10));

        TrajectoryActionBuilder four = drive.actionBuilder(pose4)
                .strafeToConstantHeading(new Vector2d(-3, 3))//, new TranslationalVelConstraint(10))
                .turnTo(Math.toRadians(-35));

        /*TrajectoryActionBuilder five = drive.actionBuilder(pose5)
                .strafeToConstantHeading(new Vector2d(-55, 12));//, new TranslationalVelConstraint(10))
                //.turnTo(Math.toRadians(-295);

        TrajectoryActionBuilder six = drive.actionBuilder(pose6)
                .strafeToConstantHeading(new Vector2d(-48, 45), new TranslationalVelConstraint(10));

        TrajectoryActionBuilder seven = drive.actionBuilder(pose7)
                .strafeToConstantHeading(new Vector2d(-3, 0))//, new TranslationalVelConstraint(10))
                .turnTo(Math.toRadians(-20));*/

        TrajectoryActionBuilder eight = drive.actionBuilder(pose8)
                .strafeToConstantHeading(new Vector2d(-15, 15));//, new TranslationalVelConstraint(10));

        // actions that need to happen on init



        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
            /*new SequentialAction(
                shooter.fireUp(),
                new ParallelAction(
                    shooter.hold(),
                    intake.intaking(),
                    new SequentialAction(
                        one.build(),
                        transfer.toAOut(),
                        flicker.kick(),
                        flicker.goBack(),
                        transfer.toBOut(),
                        flicker.kick(),
                        flicker.goBack(),
                        transfer.toCOut(),
                        flicker.kick(),
                        flicker.goBack(),
                        two.build(),
                        new ParallelAction( //TODO: the transfer timer should be longer for intaking than for outtaking
                            three.build(),
                            new SequentialAction(
                                transfer.toBIn(),
                                transfer.toAIn(),
                                transfer.toCIn(),
                                transfer.toNeutral()
                            )
                        ),
                        four.build(),
                        transfer.toAOut(),
                        flicker.kick(),
                        flicker.goBack(),
                        transfer.toBOut(),
                        flicker.kick(),
                        flicker.goBack(),
                        transfer.toCOut(),
                        flicker.kick(),
                        flicker.goBack(),
                        eight.build(),
                        shooter.stop(),
                        intake.stopIntaking()
                    )
                )
                )
            );*/
            new SequentialAction(
                shooter.fireUp(),
                new ParallelAction(
                    shooter.hold(),
                    intake.intaking(),
                    new SequentialAction(
                        one.build(),
                        transfer.turnTransf(aOut+0.005, 1.2), //TODO: THIS +0.005 IS NOT TEST
                        flicker.kick(),
                        flicker.goBack(),
                        transfer.turnTransf(bOut-0.005, 1.2), //TODO: THIS -0.005 IS NOT TEST
                        flicker.kick(),
                        flicker.goBack(),
                        transfer.turnTransf(cOut-0.005, 1.2), //TODO: THIS -0.005 IS NOT TEST
                        flicker.kick(),
                        flicker.goBack(),
                        two.build(),
                        new ParallelAction(
                            three.build(),
                            new SequentialAction(
                                transfer.turnTransf(bIn, 1.7),
                                transfer.turnTransf(aIn, 1.7),
                                transfer.turnTransf(cIn, 1.7)
                            )
                        ),
                        new ParallelAction(
                                transfer.turnTransf(rest, 1),
                                four.build()
                                ),
                        transfer.turnTransf(aOut+0.005, 1.2), //TODO: THIS +0.005 IS NOT TEST
                        flicker.kick(),
                        flicker.goBack(),
                        transfer.turnTransf(bOut-0.005, 1.2), //TODO: THIS -0.005 IS NOT TEST
                        flicker.kick(),
                        flicker.goBack(),
                        transfer.turnTransf(cOut - 0.005, 1.2),
                        flicker.kick(),
                        flicker.goBack(),
                        eight.build(),
                        shooter.stop(),
                        intake.stopIntaking()
                    )
                )
                )
            );
    }
}

