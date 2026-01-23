package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.shooter.ShooterControl;
import org.firstinspires.ftc.teamcode.tuning.shooter.RobotConstants;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@TeleOp(name="Red_Teleop_Flywheel_Alignment", group="Linear OpMode")
public class Red_Teleop_Flywheel_Alignment extends LinearOpMode {
    public enum outtakeState {
        out1, out2, out3, kick, down, rest
    }



    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;

    private DcMotor outtake_motor;

    private Servo intakeServo;
    private Servo transferServo;
    private Servo flickServo;

    private DcMotorEx flywheel;
    //VoltageSensor battery;
    private ShooterControl shooter;
    public Vision camera = new Vision();
    ElapsedTime stateTimer = new ElapsedTime();
    boolean shooting = false;

    private outtakeState state = outtakeState.down;

    boolean pressNow = false;
    boolean pressPrev = false;


    @Override
    public void runOpMode() {

        WebcamName cam1 = hardwareMap.get(WebcamName.class, "Camera1");
        camera.setTarget(Vision.Target.red);
        camera.aprilTagSetUp(cam1);

        double presentVoltage;

        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();

        ElapsedTime timer = new ElapsedTime();

        shooter = new ShooterControl(hardwareMap);
        flywheel = hardwareMap.get(DcMotorEx.class, "outtake_drive");
        flywheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setDirection(DcMotorEx.Direction.REVERSE);

        final int CYCLE_MS = 50;

        double speed = 1.0;   // used to manage half speed, defaults to full speed
        boolean invertDir = false;  // used for inverted direction prompts
        int invDir = 1;    // used to activate inverted direction
        boolean keyA = false, keyB = false;    // used for toggle keys

        double postiion0 = 2.95;
        double position1 = 0.68;
        double position2 = 0.647;
        double position3 = 0.61;
        double position4 = 0.575;
        double position5 = 0.535;
        double position6 = 0.497;


        double distFromGoal = 0;
        // total line 80
        // front wheels @ edge - 900 rpm / distance 80
        // front wheels @ 10, 880 rpm / distance 70
        // front wheels @ 20, 860 rpm /
        // front wheels @ 33, 840 rpm question mark

        boolean isEnter = false;
        double setFlickPos = 1;

        double C_LATERAL, C_AXIAL, C_YAW;
        boolean C_HALF_SPEED, C_INV_DIR, SET_RPM, C_INTAKE, C_TRANSFER_PA, C_TRANSFER_PB, C_TRANSFER_PC, C_FLICK = false, C_MOVE_LEFT, C_MOVE_RIGHT;

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        outtake_motor = hardwareMap.get(DcMotor.class, "outtake_drive");
        double outtakeMotorPower = -0.75;
        boolean prevG2A = false;
        boolean prevG2B = false;

        intakeServo = hardwareMap.get(Servo.class, "intake_servo");
        transferServo = hardwareMap.get(Servo.class, "transfer_servo");
        flickServo = hardwareMap.get(Servo.class, "flick_servo");
        double rest = 2.95;
        double tranferPosAIn = 0.68;
        double tranferPosBIn = 0.61;
        double tranferPosCIn = 0.535;
        double tranferPosCOut = 0.647;
        double tranferPosAOut = 0.575;
        double tranferPosBOut = 0.497;
        double i = 0;

        double aIn = 0.68;
        double bIn = 0.61;
        double cIn = 0.535;
        double cOut = 0.647;
        double aOut = 0.575;
        double bOut = 0.497;


        double postrack = cOut;

        boolean spinningUp = true;
        double flywheelAccel = 180;
        double targRPM = 0;



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

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        double leftFrontPower = 0;
        double rightFrontPower = 0;
        double leftBackPower = 0;
        double rightBackPower = 0;
        while (opModeIsActive()) {



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
            SET_RPM = gamepad2.dpad_up;

            double max;

            // POV Mode uses left joystick to go axial & strafe, and right joystick to yaw.
            double axial = -C_AXIAL;  // Note: pushing stick axial gives negative value
            double lateral = C_LATERAL;
            double yaw = C_YAW;


            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            leftFrontPower = axial + lateral + yaw;
            rightFrontPower = axial - lateral - yaw;
            rightBackPower = axial + lateral - yaw;
            leftBackPower = axial - lateral + yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }




            // HALF SPEED CONTROLS
            if (C_HALF_SPEED) {
                if (keyA == false) {
                    keyA = !keyA;
                    switch ((int) (speed * 10)) {
                        case 10:
                            speed = 0.5;
                            break;
                        case 5:
                            speed = 1.0;
                            break;
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


            pressNow = gamepad2.a;




            distFromGoal = camera.centerDistanceCM();

            presentVoltage = battery.getVoltage();
            shooter.setBatteryVoltage(presentVoltage);

            if(spinningUp){
                double dt = timer.seconds();
                targRPM += dt * flywheelAccel;
                targRPM = Range.clip(targRPM, 0, 940);
                if (targRPM >= 920){spinningUp =false;}
            }
            //if(!spinningUp && (distFromGoal != -1)){targRPM = (0.560459*distFromGoal)+760.13857;}
            if(!spinningUp && (distFromGoal >= 140) && (distFromGoal <= 270)){
                shooter.setKf(0.00965);
                targRPM = (0.546172*distFromGoal)+722.4006;
            }
            else if(!spinningUp && (distFromGoal > 320)){
                shooter.setKf(0.0105);
                targRPM = (targRPM); //TODO: need equation for this
            }
            else if(!spinningUp && (distFromGoal < 140) && (distFromGoal != -1)){
                shooter.setKf(0.00965);
                targRPM = (0.909091*distFromGoal)+703.18182;//TODO: only made this equation with two points; need equation for this
            }

            /*NEW DATA
            lower ish end
            865 / 263
            860 / 243
            850 / 225
            830 / 213
            830 / 202
            825 / 188
            820 / 174
            810 / 162

            805 / 147
            825 / 134
            815 / 123


            NEW LINE
            NEW KF
            0.0105
            384 - 1055+
            */

            targRPM = Range.clip(targRPM, 0, 945);

            shooter.setTargetRPM(targRPM);


            /*if (!spinningUp && SET_RPM) {
                if (distFromGoal >= (245)) { // ish
                    targRPM = 940;
                } //lowkenuinely the rpm for >= 90 is a total guess
                if (distFromGoal >= (220) && distFromGoal < (245)) { //230
                    targRPM = 900;
                }
                if (distFromGoal >= (200) && distFromGoal < (220)) { //212
                    targRPM = 880;
                }
                if (distFromGoal >= (165) && distFromGoal < (200)) { //175
                    targRPM = 860;
                }
                if (distFromGoal < (165) && distFromGoal != -1) { //158
                    targRPM = 840;
                }
            }*/




            // total line 80
            // front wheels @ edge - 900 rpm / distance 80
            // front wheels @ 10, 880 rpm / distance 70
            // front wheels @ 20, 860 rpm / distance 60
            // front wheels @ 33, 840 rpm question mark / distance 47

            shooter.setKf(RobotConstants.kF);
            shooter.setKp(RobotConstants.kP);
            shooter.setKi(RobotConstants.kI);
            shooter.setKd(RobotConstants.kD);

            shooter.setMaxAccel(RobotConstants.maxAccel);

            shooter.flywheelHold();





            if(pressNow && !pressPrev){shooting = !shooting;} //if they're opposite i.e. it's changed

            if (shooting){
                state = outtakeState.out1;
                stateTimer.reset();
            }else{ //stop the sequence
                state = outtakeState.rest;
                flickServo.setPosition(0.3);
            }

            pressPrev = pressNow;

            if(shooting) {
                switch (state) {
                    case out1:
                        transferServo.setPosition(cOut);
                        postrack = cOut;
                        if (stateTimer.seconds() > 1){
                            state = outtakeState.kick;
                            stateTimer.reset();
                        }
                        break;

                    case out2:
                        transferServo.setPosition(aOut);
                        postrack = aOut;
                        if (stateTimer.seconds() > 1){
                            state = outtakeState.kick;
                            stateTimer.reset();
                        }
                        break;

                    case out3:
                        transferServo.setPosition(bOut);
                        postrack = bOut;
                        if (stateTimer.seconds() > 1){
                            state = outtakeState.kick;
                            stateTimer.reset();
                        }
                        break;

                    case kick:
                        flickServo.setPosition(0.0);
                        if (stateTimer.seconds() > 1){
                            state = outtakeState.down;
                            stateTimer.reset();
                        }
                        break;



                    case down:
                        flickServo.setPosition(0.3);
                        if (stateTimer.seconds() > 1) {
                            if (postrack == cOut) {
                                state = outtakeState.out2;
                            } else if (postrack == aOut) {
                                state = outtakeState.out3;
                            } else if (postrack == bOut) {
                                state = outtakeState.rest;
                                shooting = false;
                            }
                            break;
                        }

                    case rest:
                        transferServo.setPosition(cOut);
                        shooting = false;
                        break;
                }
            }

            if (gamepad2.x) {
                intakeServo.setPosition(0.0);
                isEnter = true;
            } else {
                intakeServo.setPosition(0.5);
            }

            if (C_MOVE_LEFT && !gamepad2.left_bumper && !gamepad2.right_bumper) {
                transferServo.setPosition(tranferPosAIn);
            } else if (C_MOVE_RIGHT && !gamepad2.left_bumper && !gamepad2.right_bumper) {
                transferServo.setPosition(tranferPosAOut);
            } else if (C_MOVE_LEFT && gamepad2.left_bumper && !gamepad2.right_bumper) {
                transferServo.setPosition(tranferPosBIn);
            } else if (C_MOVE_RIGHT && gamepad2.left_bumper && !gamepad2.right_bumper) {
                transferServo.setPosition(tranferPosBOut);
            } else if (C_MOVE_LEFT && !gamepad2.left_bumper && gamepad2.right_bumper) {
                transferServo.setPosition(tranferPosCIn);
            } else if (C_MOVE_RIGHT && !gamepad2.left_bumper && gamepad2.right_bumper) {
                transferServo.setPosition(tranferPosCOut);
            }


            if (gamepad2.y) {
                flickServo.setPosition(0.0);
                isEnter = true;
            } else {
                flickServo.setPosition(0.3);
            }


            // Send calculated power to wheels

            leftFrontDrive.setPower(leftFrontPower * speed * invDir);
            rightFrontDrive.setPower(rightFrontPower * speed * invDir);
            leftBackDrive.setPower(leftBackPower * speed * invDir);
            rightBackDrive.setPower(rightBackPower * speed * invDir);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Distance from Center of Red Goal (cm): ", camera.centerDistanceCM());
            telemetry.addData("target RPM", targRPM);
            telemetry.addData("flywheel measured velocity", flywheel.getVelocity());

            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower * speed * invDir, rightFrontPower * speed * invDir);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower * speed * invDir, rightBackPower * speed * invDir);
            telemetry.addData("Speed", "%4.2f", speed);
            telemetry.addData("Invert Direction", "%1b", invertDir);

            telemetry.update();

            sleep(CYCLE_MS);

        }
    }
}