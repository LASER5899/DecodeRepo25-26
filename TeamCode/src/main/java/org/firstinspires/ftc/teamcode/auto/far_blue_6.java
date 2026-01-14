package org.firstinspires.ftc.teamcode.auto;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


//this is theoretical if everything is perfectly tuned, but using 90 degrees and 24 inches
@Config
@Autonomous(name = "far blue 6", group = "Autonomous")
@Disabled
//psuedocode
/*
start against wall
turn cc 30 deg (less?)
shoot 3
22 in fwd
125 deg cc
13 in fwd
intake on
10 in fwd + intake sequ (pos a,b,c)
23 in back
125 deg clockwise
22 in back
shoot 3
12 in fwd
 */
public class far_blue_6 extends LinearOpMode{

    // if odometry is not properly tuned or constantly being retuned:
    // you MIGHT find it useful to change these values and use multiples of them instead of direct number
    // keep in mind that this may not work well
    // i.e. if something is wrong with acceleration/deceleration, two lengths may not be equal to 2 * (one length)
    double quarter = 90; // "90 degrees" / right angle turn
    double tile = 24; // "24 inches" / one tile

    double a = 0.575;
    double b = 0.497;
    double c = 0.647;

    //mechanism instantiation




    public class slideVertical {

        private DcMotorEx lift;

        public slideVertical(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "vertical_slide"); //config?
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotor.Direction.FORWARD);
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public class LowLift implements Action{
            //checks if lift motor has been powered on
            private boolean initialized = false;
            //actions formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                //powers on motor if not on
                if (!initialized){
                    lift.setPower(1);
                    initialized = true;
                }
                //checks lift's current position
                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 2512) {
                    //true causes the action to return
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setTargetPosition(2512);
                    return true;
                } else {
                    //false stops action rerun
                    lift.setPower(0);
                    return false;
                }
            }
        }

        public Action lowLift(){
            return new LowLift();
        }
        public class FinalLift implements Action{
            //checks if lift motor has been powered on
            private boolean initialized = false;
            //actions formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                //powers on motor if not on
                if (!initialized){
                    lift.setPower(1); //og 1
                    initialized = true;
                }
                //checks lift's current position
                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 2850) {
                    //true causes the action to return
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setTargetPosition(2850);
                    return true;
                } else {
                    //false stops action rerun
                    lift.setPower(0);
                    return false;
                }
            }
        }

        public Action finalLift(){
            return new FinalLift();
        }


        public class HighLift implements Action{
            //checks if lift motor has been powered on
            private boolean initialized = false;
            //actions formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                //powers on motor if not on
                if (!initialized){
                    lift.setPower(1); //og 1
                    initialized = true;
                }
                //checks lift's current position
                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 2800) {
                    //true causes the action to return
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setTargetPosition(2800);
                    return true;
                } else {
                    //false stops action rerun
                    lift.setPower(0);
                    return false;
                }
            }
        }

        public Action highLift(){
            return new HighLift();
        }

        public class StartHighLift implements Action{
            //checks if lift motor has been powered on
            private boolean initialized = false;
            //actions formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                //powers on motor if not on
                if (!initialized){
                    lift.setPower(1); //og 1
                    initialized = true;
                }
                //checks lift's current position
                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 1000) {
                    //true causes the action to return
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setTargetPosition(1000);
                    return true;
                } else {
                    //false stops action rerun
                    lift.setPower(0);
                    return false;
                }
            }
        }

        public Action startHighLift(){
            return new StartHighLift();
        }
        public class StartLiftDown implements Action{
            //checks if lift motor has been powered on
            private boolean initialized = false;
            //actions formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                //powers on motor if not on
                if (!initialized){
                    lift.setPower(1);
                    initialized = true;
                }
                //checks lift's current position
                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 1200) {
                    //true causes the action to return
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setTargetPosition(1200);
                    return true;
                } else {
                    //false stops action rerun

                    return false;
                }
            }

        }

        public Action startLiftDown(){
            return new StartLiftDown();
        }



        public class LiftDown implements Action{
            //checks if lift motor has been powered on
            private boolean initialized = false;
            //actions formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                //powers on motor if not on
                if (!initialized){
                    lift.setPower(1);
                    initialized = true;
                }
                //checks lift's current position
                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 0) { //50
                    //true causes the action to return
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setTargetPosition(0);
                    return true;
                } else {
                    //false stops action rerun
                    //TODO: turn off motor when done
                    lift.setPower(0);
                    return false;
                }
            }

        }

        public Action liftDown(){
            return new LiftDown();
        }
    }



    public class slideHorizontal{
        private DcMotorEx extend;

        public slideHorizontal(HardwareMap hardwareMap) {
            extend = hardwareMap.get(DcMotorEx.class, "horizontal_slide"); //config?
            extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            extend.setDirection(DcMotor.Direction.REVERSE);
            extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public class HSlideOut implements Action {
            //checks if lift motor has been powered on
            private boolean initialized = false;

            //actions formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //powers on motor if not on
                if (!initialized) {
                    extend.setPower(1);
                    initialized = true;
                }
                //checks lift's current position
                double pos = extend.getCurrentPosition();
                packet.put("extendPos", pos);
                if (pos < 500) { //50
                    //true causes the action to return
                    extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    extend.setTargetPosition(500);
                    return true;
                } else {
                    //false stops action rerun
                    //TODO: turn off motor when done
                    extend.setPower(0);
                    return false;
                }
            }
        }
        public Action hSlideOut(){
            return new slideHorizontal.HSlideOut();
        }

        public class HSlideIn implements Action {
            //checks if lift motor has been powered on
            private boolean initialized = false;

            //actions formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //powers on motor if not on
                if (!initialized) {
                    extend.setPower(1);
                    initialized = true;
                }
                //checks lift's current position
                double pos = extend.getCurrentPosition();
                packet.put("extendPos", pos);
                if (pos > 0) { //50
                    //true causes the action to return
                    extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    extend.setTargetPosition(0);
                    return true;
                } else {
                    //false stops action rerun
                    //TODO: turn off motor when done
                    extend.setPower(0);
                    return false;
                }
            }

            public Action hSlideIn() {
                return new slideHorizontal.HSlideIn();
            }
        }


    }

    public class wristDrive{
        private DcMotorEx wrist;

        public wristDrive(HardwareMap hardwareMap) {
            wrist = hardwareMap.get(DcMotorEx.class, "wrist_drive"); //NOT SURE WHAT IT IS IN CONFIG
            wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            wrist.setDirection(DcMotor.Direction.REVERSE);
            wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        public class IntakeAvoid implements Action {
            //checks if lift motor has been powered on
            private boolean initialized = false;

            //actions formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //powers on motor if not on
                if (!initialized) {
                    wrist.setPower(1);
                    initialized = true;
                }
                //checks lift's current position
                double pos = wrist.getCurrentPosition();
                packet.put("wristPos", pos);
                if (pos != 130) { //50
                    //true causes the action to return
                    wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    wrist.setTargetPosition(130);
                    return true;
                } else {
                    //false stops action rerun
                    //TODO: turn off motor when done
                    wrist.setPower(0);
                    return false;
                }
            }

            public Action intakeAvoid() {
                return new wristDrive.IntakeAvoid();
            }
        }

        public class IntakeTransfer implements Action { // this is also intake completely in
            //checks if lift motor has been powered on
            private boolean initialized = false;

            //actions formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //powers on motor if not on
                if (!initialized) {
                    wrist.setPower(1);
                    initialized = true;
                }
                //checks lift's current position
                double pos = wrist.getCurrentPosition();
                packet.put("wristPos", pos);
                if (pos != 15) { //50
                    //true causes the action to return
                    wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    wrist.setTargetPosition(15);
                    return true;
                } else {
                    //false stops action rerun
                    //TODO: turn off motor when done
                    wrist.setPower(0);
                    return false;
                }
            }

            public Action intakeTransfer() {
                return new wristDrive.IntakeTransfer();
            }
        }

        public class IntakeIdle implements Action { // this is also intake completely in
            //checks if lift motor has been powered on
            private boolean initialized = false;

            //actions formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //powers on motor if not on
                if (!initialized) {
                    wrist.setPower(1);
                    initialized = true;
                }
                //checks lift's current position
                double pos = wrist.getCurrentPosition();
                packet.put("wristPos", pos);
                if (pos != 400) { //50
                    //true causes the action to return
                    wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    wrist.setTargetPosition(400);
                    return true;
                } else {
                    //false stops action rerun
                    //TODO: turn off motor when done
                    wrist.setPower(0);
                    return false;
                }
            }

            public Action intakeIdle() {
                return new wristDrive.IntakeIdle();
            }
        }


    }

    public class intakeServo{
        private Servo spin;

        public intakeServo(HardwareMap hardwaremap){
            spin = hardwareMap.get(Servo.class, "intake_servo"); //NOT SURE WHAT IT IS IN CONFIG

        }
        public class IntakeServoStop implements Action { // this is also intake completely in
            //checks if lift motor has been powered on
            private boolean initialized = false;

            //actions formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //powers on motor if not on
                if (!initialized) {
                    spin.setPosition(0.5);
                    initialized = true;
                }
                //checks lift's current position
                double pos = spin.getPosition();
                packet.put("liftPos", pos);
                if (pos != 0.5) { //50
                    //true causes the action to return
                    spin.setPosition(0.5);

                    return true;
                } else {
                    //false stops action rerun
                    //TODO: turn off motor when done
                    spin.setPosition(0.5);
                    return false;
                }
            }

        }
        public Action intakeServoStop() {
            return new intakeServo.IntakeServoStop();
        }


        public class IntakeServoPickUp implements Action { // this is also intake completely in
            //checks if lift motor has been powered on
            private boolean initialized = false;

            //actions formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //powers on motor if not on
                if (!initialized) {
                    spin.setPosition(1);
                    initialized = true;
                }
                //checks lift's current position
                double pos = spin.getPosition();
                packet.put("liftPos", pos);
                if (pos != 1) { //50
                    //true causes the action to return
                    spin.setPosition(1);

                    return true;
                } else {
                    //false stops action rerun
                    //TODO: turn off motor when done
                    spin.setPosition(0.5);
                    return false;
                }
            }

        }
        public Action intakeServoPickUp() {
            return new intakeServo.IntakeServoPickUp();
        }

    }


    public class outtakeServo{
        private Servo rotateOut;

        public outtakeServo(HardwareMap hardwaremap){
            rotateOut = hardwareMap.get(Servo.class, "outtake_servo"); //NOT SURE WHAT IT IS IN CONFIG
            rotateOut.setPosition(0.70);
            //initpos 0.70
            //endpos 0.06
        }


    }




    //begin code
    @Override
    public void runOpMode() throws InterruptedException {
        //Pose2d initPose = new Pose2d(0, -72, Math.toRadians(270)); put this back in
        Pose2d backingPose = new Pose2d(0, -50, Math.toRadians(270));
        Pose2d initPose = new Pose2d(0, -72, Math.toRadians(270));
        Pose2d sub = new Pose2d(0, -41, Math.toRadians(270));
        Pose2d endZone = new Pose2d(32, -73.5, Math.toRadians(90));
        Pose2d sub2 = new Pose2d(-5, -41.5, Math.toRadians(270));
        Pose2d finale = new Pose2d(32, -74, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);
        slideVertical lift = new slideVertical(hardwareMap);

        TrajectoryActionBuilder toSub = drive.actionBuilder(initPose)
                .strafeToConstantHeading(new Vector2d(0, -41), new TranslationalVelConstraint(20)); //28 for 2, 1  for 14?
        TrajectoryActionBuilder back = drive.actionBuilder(sub)
                .strafeToConstantHeading(new Vector2d(0, -50)); //28 for 2, 1  for 14?

        TrajectoryActionBuilder push3 = drive.actionBuilder(backingPose)
                // PART 1: score 1st spec & prepare
                //.splineToSplineHeading(new Pose2d(30, -40, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(30, -40, Math.toRadians(90)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(30, -26), Math.toRadians(90))
                //PART 2a: 1st push
                .splineToConstantHeading(new Vector2d(39, -26), Math.toRadians(270)) //6 // position
                .splineToConstantHeading(new Vector2d(39, -57), Math.toRadians(90), new TranslationalVelConstraint(35), new ProfileAccelConstraint(-10,10)) //7 // push
                .splineToConstantHeading(new Vector2d(39, -26), Math.toRadians(90), new TranslationalVelConstraint(35)) //8 // back to samp area
                //PART 2b: 2nd push
                .splineToConstantHeading(new Vector2d(49.8, -26), Math.toRadians(270), new TranslationalVelConstraint(20)) //9 // position
                .splineToConstantHeading(new Vector2d(49.8, -57), Math.toRadians(270)) //10 // push
                //PART 3: pick up specimen
                .splineToConstantHeading(new Vector2d(43, -65), Math.toRadians(180)) //14 // quarter circle 1
                .splineToConstantHeading(new Vector2d(38, -55), Math.toRadians(270), new TranslationalVelConstraint(30)) //15 // quarter circle 2
                .splineToConstantHeading(new Vector2d(32, -74), Math.toRadians(90), new TranslationalVelConstraint(5))
                .splineToConstantHeading(new Vector2d(32, -71) , Math.toRadians(90), new TranslationalVelConstraint(10.0));




        TrajectoryActionBuilder score3 = drive.actionBuilder(endZone)
                //.strafetoeHeading(new Pose2d(-10, -60, Math.toRadians(180)), Math.toRadians(90)) //TODO: fix
                .strafeToLinearHeading(new Vector2d(-5, -42), Math.toRadians(270));

        //.splineToSplineHeading(new Pose2d(-10, -60, Math.toRadians(180)), Math.toRadians(90)) //TODO: fix
        //.splineToSplineHeading(new Pose2d(-5, -42, Math.toRadians(270)), Math.toRadians(270));

        TrajectoryActionBuilder homeStretch = drive.actionBuilder(sub2)
                //.splineToSplineHeading(new Pose2d(0, 60, Math.toRadians(180)), Math.toRadians(270), new TranslationalVelConstraint(10)) //TODO: fix
                //.splineToSplineHeading(new Pose2d(32, -71, Math.toRadians(90)) , Math.toRadians(90), new TranslationalVelConstraint(10.0));
                //.strafeToConstantHeading(new Vector2d(20, -71))
                .strafeToLinearHeading(new Vector2d(32, -69), Math.toRadians(90))
                //.strafeToLinearHeading(new Vector2d(-5, -42), Math.toRadians(270));
                //.strafeToLinearHeading(new Vector2d(32, -67), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(32, -74), new TranslationalVelConstraint(10))
                .strafeToConstantHeading(new Vector2d(32, -71), new TranslationalVelConstraint(15));

        TrajectoryActionBuilder done = drive.actionBuilder(finale)
                .strafeToConstantHeading(new Vector2d(32, -71))
                //.strafeToConstantHeading(new Vector2d(20, -71))
                .strafeToLinearHeading(new Vector2d(-5, -42), Math.toRadians(270));




        // actions that need to happen on init; for instance, a claw tightening.




        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(




                )

        );



    }
}
