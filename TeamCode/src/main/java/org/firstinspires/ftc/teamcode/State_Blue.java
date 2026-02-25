package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.shooter.ShooterControl;
import org.firstinspires.ftc.teamcode.tuning.shooter.RobotConstants;
import org.firstinspires.ftc.teamcode.classes.Transfer_Values;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="!state blue 67aaaaaa", group="Linear OpMode")
public class State_Blue extends LinearOpMode {

    boolean pdiddyNow, pdiddyPrev, ganttChart;
    private MecanumDrive drive;
    private double heady;
    private IMU imu;
    public enum outtakeState {
        ABC, BCA, CAB, REST
    }
    int shootStep = 0;



    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;

    private DcMotor outtake_motor;

    private CRServo intakeServo;
    private Servo transferServo;
    private Servo flickServo;

    private DcMotorEx flywheel;
    //VoltageSensor battery;
    private ShooterControl shooter;
    private Transfer_Values transferValues;
    public Vision camera = new Vision();
    ElapsedTime stateTimer = new ElapsedTime();
    boolean shooting, manualFlywheel, transfFailsafe = false;

    private outtakeState state = outtakeState.REST;

    boolean pressNowShoot, pressPrevShoot,pressNowTransf, pressPrevTransf, pressNowManualShoot, pressPrevManualShoot, patternDetected = false;

    String pattern;
    String shootOrder;


    private void driveFieldRelative(double forward, double right, double rotate, double head) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);
        //double head = drive.localizer.getPose().heading.toDouble();
        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                head);

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        leftFrontDrive.setPower(frontLeftPower  / maxPower);
        rightFrontDrive.setPower(frontRightPower / maxPower);
        leftBackDrive.setPower(backLeftPower  / maxPower);
        rightBackDrive.setPower(backRightPower / maxPower);
    }

    @Override
    public void runOpMode() {



        WebcamName cam1 = hardwareMap.get(WebcamName.class, "Camera1");
        camera.setTarget(Vision.Target.blue);
        camera.aprilTagSetUp(cam1);

        double presentVoltage;

        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();

        ElapsedTime timer = new ElapsedTime();

        shooter = new ShooterControl(hardwareMap);
        transferValues = new Transfer_Values();
        flywheel = hardwareMap.get(DcMotorEx.class, "outtake_drive");
        flywheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setDirection(DcMotorEx.Direction.REVERSE);

        final int CYCLE_MS = 50;

        double speed = 1.0;   // used to manage half speed, defaults to full speed
        boolean invertDir = false;  // used for inverted direction prompts
        int invDir = 1;    // used to activate inverted direction
        boolean keyA = false, keyB = false;    // used for toggle keys


        double distFromGoal = 0;
        // total line 80
        // front wheels @ edge - 900 rpm / distance 80
        // front wheels @ 10, 880 rpm / distance 70
        // front wheels @ 20, 860 rpm /
        // front wheels @ 33, 840 rpm question mark

        boolean isEnter = false;
        double setFlickPos = 1;

        double C_LATERAL, C_AXIAL, C_YAW;
        boolean C_HALF_SPEED, C_INV_DIR, SET_RPM, HEADING_FREEZE, C_INTAKE, C_TRANSFER_PA, C_TRANSFER_PB, C_TRANSFER_PC, SET_POWER, AUTO_SHOOT, C_SPIT, C_MOVE_REST, C_FLICK, TRANSFER_MANUAL = false, C_MOVE_LEFT, C_MOVE_RIGHT;

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        outtake_motor = hardwareMap.get(DcMotor.class, "outtake_drive");

        double outtakeMotorPower = 0.75;
        boolean prevG2A = false;
        boolean prevG2B = false;

        intakeServo = hardwareMap.get(CRServo.class, "intake_servo");
        transferServo = hardwareMap.get(Servo.class, "transfer_servo");
        flickServo = hardwareMap.get(Servo.class, "flick_servo");
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));


        double aIn = transferValues.aIn;
        double bIn = transferValues.bIn;
        double cIn = transferValues.cIn;
        double aOut = transferValues.aOut;
        double bOut = transferValues.bOut;
        double cOut = transferValues.cOut;
        double rest = transferValues.rest;


        double postrack = cOut;
        double postrackreset = 0;

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
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        boolean reached = false;
        boolean turnCodeOn = false;
        double alignVal=10000;
        double turnSpeed = 0.15;
        double originValue=0;

        int timesOut = 0;
        int timesNeeded =3;

        transferServo.setPosition(cOut);

        waitForStart();

        // run until the end of the match (driver presses STOP)
        double leftFrontPower = 0;
        double rightFrontPower = 0;
        double leftBackPower = 0;
        double rightBackPower = 0;
        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            heady = drive.localizer.getPose().heading.toDouble();



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
            C_SPIT = gamepad2.a;
            C_FLICK = gamepad2.y;
            C_MOVE_LEFT = gamepad2.dpad_left;
            C_MOVE_RIGHT = gamepad2.dpad_right;
            SET_POWER = (gamepad2.left_trigger > 0); //gamepad2 a and b are up and down
            AUTO_SHOOT = (gamepad2.right_trigger > 0);
            C_MOVE_REST = gamepad2.dpad_up;
            TRANSFER_MANUAL =  gamepad2.dpad_down;

            HEADING_FREEZE = gamepad1.right_bumper;

            // If you press the A button, then you reset the Yaw to be zero from the way
            // the robot is currently pointing
            if (gamepad1.x) {
                drive.localizer.setPose(new Pose2d(0, 0, Math.toRadians(0)));
            }
            // If you press the left bumper, you get a drive from the point of view of the robot
            // (much like driving an RC vehicle)
            pdiddyNow = HEADING_FREEZE;



            /*double max;

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
            */




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

            if(pdiddyNow && !pdiddyPrev){ganttChart = !ganttChart;}



            if (ganttChart) {
                //drive.updatePoseEstimate();
                driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, Math.toRadians(25), heady);
                if (((Math.abs(drive.localizer.getPose().heading.toDouble()+ Math.toRadians(25)) < Math.toRadians(3))|| (Math.abs(drive.localizer.getPose().heading.toDouble()- Math.toRadians(25))< Math.toRadians(3)))){
                    ganttChart = false;
                }
                //=driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);}
            } /*else {
                drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }*/

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

            String sequence = camera.scanForPattern();
            if (!patternDetected && !(sequence.equals("none"))){
                pattern = sequence;
                patternDetected = true;
            }


            //
            //TODO: change!!!!!!
            pressNowManualShoot = SET_POWER;
            pressNowShoot = AUTO_SHOOT;
            pressNowTransf = TRANSFER_MANUAL;
            distFromGoal = camera.centerDistanceCM();
            presentVoltage = battery.getVoltage();
            shooter.setBatteryVoltage(presentVoltage);


            if(gamepad1.y){
                reached = true;
                turnCodeOn = false;
            }
            if (!turnCodeOn && gamepad1.x&& !(camera.alignmentValue() == -10000)) {
                turnCodeOn = true;
                originValue = 0;
                reached = false;

            }
            if(reached){
                leftFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightFrontDrive.setPower(0);
                rightBackDrive.setPower(0);
                telemetry.addData("turning: ","none");
                reached = false;
                turnCodeOn = false;

            }
            alignVal = camera.alignmentValue();
            if(originValue==0){
                if(alignVal>0) {
                    originValue = 1;
                }else if(alignVal<0){
                    originValue = -1;
                }
            }
            if(turnCodeOn&&(alignVal*originValue<0)){
                reached=true;
                turnCodeOn = false;
                leftFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightFrontDrive.setPower(0);
                rightBackDrive.setPower(0);
                telemetry.addData("turning: ","none");
            }
            if (!gamepad1.x && turnCodeOn&&!reached) {
                //below here we need to add when its ok.
                ///if ((originValue*alignVal)>0) {
                if(timesOut>=timesNeeded){
                    reached = true;
                }
                if (!(alignVal == -10000)) {
                    if ((originValue<0)&&(alignVal < 0)) {
                        //turn left
                        leftFrontDrive.setPower(-1 * turnSpeed);
                        leftBackDrive.setPower(-1 * turnSpeed);
                        rightFrontDrive.setPower(1 * turnSpeed);
                        rightBackDrive.setPower(1 * turnSpeed);
                        timesOut = 0;
                        telemetry.addData("turning: ","left");
                    } else if ((originValue>0)&&(alignVal > 0)) {
                        //turn right
                        leftFrontDrive.setPower(1 * turnSpeed);
                        leftBackDrive.setPower(1 * turnSpeed);
                        rightFrontDrive.setPower(-1 * turnSpeed);
                        rightBackDrive.setPower(-1 * turnSpeed);
                        timesOut = 0;
                        telemetry.addData("turning: ","right");
                    } else {timesOut++;}
                }
            }

            if(spinningUp){
                double dt = timer.seconds();
                targRPM += dt * flywheelAccel;
                targRPM = Range.clip(targRPM, 0, 1050);
                shooter.setKf(RobotConstants.kF);
                if (targRPM >= 920){spinningUp =false; shooter.setKf(RobotConstants.kF);}
            }
            if(!spinningUp && (distFromGoal >= 140) && (distFromGoal <= 270)){
                targRPM = (0.546172*distFromGoal)+722.4006;
                //targRPM = 1050;

                shooter.setKf(0.0028);
                shooter.setvoltCorr(1.5);
            }
            else if(!spinningUp && (distFromGoal > 320)){
                targRPM = 1070;
                shooter.setvoltCorr(2);
                shooter.setKf(0.00285);
            }
            else if(!spinningUp && (distFromGoal < 140) && (distFromGoal != -1)){
                targRPM = (0.909091*distFromGoal)+703.18182;
                shooter.setvoltCorr(1.5);
            }

            //targRPM = 1050;
            //shooter.setKf(0.00285);




            targRPM = Range.clip(targRPM, 0, 1050);
            shooter.setMaxAccel(RobotConstants.maxAccel);

            shooter.setKp(RobotConstants.kP);
            shooter.setKi(RobotConstants.kI);
            shooter.setKd(RobotConstants.kD);

            if (!manualFlywheel) {
                shooter.setTargetRPM(targRPM);
                shooter.flywheelHold();
            }


            if(pressNowShoot && !pressPrevShoot){
                shooting = !shooting;
                if (shooting){
                    shootStep = 0;
                    if (   (pattern.equals("PPG") && postrack==cOut) || (pattern.equals("GPP") && postrack==aOut) || (pattern.equals("PGP") && postrack==bOut) ){
                        shootOrder = "ABC";
                        state = outtakeState.ABC;
                    }
                    else if (   (pattern.equals("PPG") && postrack==aOut) || (pattern.equals("GPP") && postrack==bOut) || (pattern.equals("PGP") && postrack==cOut) ){
                        shootOrder = "BCA";
                        state = outtakeState.BCA;
                    }
                    else if (   (pattern.equals("PPG") && postrack==bOut) || (pattern.equals("GPP") && postrack==cOut) || (pattern.equals("PGP") && postrack==aOut) ){
                        shootOrder = "CAB";
                        state = outtakeState.CAB;
                    }
                }else{ state = outtakeState.REST; flickServo.setPosition(0.3); }
                //MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
                stateTimer.reset();
            }
            if(pressNowManualShoot && !pressPrevManualShoot){manualFlywheel = !manualFlywheel;}

            if(shooting) {
                switch (state) {
                    case ABC:
                        switch (shootStep){
                            case 0: transferServo.setPosition(aOut); postrack = aOut; if (stateTimer.seconds() > 1) {stateTimer.reset(); postrack = aOut; shootStep++; } break;
                            case 1: flickServo.setPosition(0.0); if (stateTimer.seconds() > 0.5) {stateTimer.reset(); shootStep++; } break;
                            case 2: flickServo.setPosition(0.3); if (stateTimer.seconds() > 0.5) {stateTimer.reset(); shootStep++; } break;

                            case 3: transferServo.setPosition(bOut); postrack = bOut; if (stateTimer.seconds() > 1) {stateTimer.reset(); postrack = bOut; shootStep++; } break;
                            case 4: flickServo.setPosition(0.0); if (stateTimer.seconds() > 0.5) {stateTimer.reset(); shootStep++; } break;
                            case 5: flickServo.setPosition(0.3); if (stateTimer.seconds() > 0.5) {stateTimer.reset(); shootStep++; } break;

                            case 6: transferServo.setPosition(cOut); postrack = cOut; if (stateTimer.seconds() > 1) {stateTimer.reset(); postrack = cOut; shootStep++; } break;
                            case 7: flickServo.setPosition(0.0); if (stateTimer.seconds() > 0.5) {stateTimer.reset(); shootStep++; } break;
                            case 8: flickServo.setPosition(0.3); if (stateTimer.seconds() > 0.5) {stateTimer.reset(); state = outtakeState.REST; } break;
                        }
                        break;

                    case BCA:
                        switch (shootStep){
                            case 0: transferServo.setPosition(bOut); postrack = bOut; if (stateTimer.seconds() > 1) {stateTimer.reset(); postrack = bOut; shootStep++; } break;
                            case 1: flickServo.setPosition(0.0); if (stateTimer.seconds() > 0.5) {stateTimer.reset(); shootStep++; } break;
                            case 2: flickServo.setPosition(0.3); if (stateTimer.seconds() > 0.5) {stateTimer.reset(); shootStep++; } break;

                            case 3: transferServo.setPosition(cOut); postrack = cOut; if (stateTimer.seconds() > 1) {stateTimer.reset(); postrack = cOut; shootStep++; } break;
                            case 4: flickServo.setPosition(0.0); if (stateTimer.seconds() > 0.5) {stateTimer.reset(); shootStep++; } break;
                            case 5: flickServo.setPosition(0.3); if (stateTimer.seconds() > 0.5) {stateTimer.reset(); shootStep++; } break;

                            case 6: transferServo.setPosition(aOut); postrack = aOut; if (stateTimer.seconds() > 1) {stateTimer.reset(); postrack = aOut; shootStep++; } break;
                            case 7: flickServo.setPosition(0.0); if (stateTimer.seconds() > 0.5) {stateTimer.reset(); shootStep++; } break;
                            case 8: flickServo.setPosition(0.3); if (stateTimer.seconds() > 0.5) {stateTimer.reset(); state = outtakeState.REST; } break;
                        }
                        break;

                    case CAB:
                        switch (shootStep){
                            case 0: transferServo.setPosition(cOut); postrack = cOut; if (stateTimer.seconds() > 1) {stateTimer.reset(); postrack = cOut; shootStep++; } break;
                            case 1: flickServo.setPosition(0.0); if (stateTimer.seconds() > 0.5) {stateTimer.reset(); shootStep++; } break;
                            case 2: flickServo.setPosition(0.3); if (stateTimer.seconds() > 0.5) {stateTimer.reset(); shootStep++; } break;

                            case 3: transferServo.setPosition(aOut); postrack = aOut; if (stateTimer.seconds() > 1) {stateTimer.reset(); postrack = aOut; shootStep++; } break;
                            case 4: flickServo.setPosition(0.0); if (stateTimer.seconds() > 0.5) {stateTimer.reset(); shootStep++; } break;
                            case 5: flickServo.setPosition(0.3); if (stateTimer.seconds() > 0.5) {stateTimer.reset(); shootStep++; } break;

                            case 6: transferServo.setPosition(bOut); postrack = bOut; if (stateTimer.seconds() > 1) {stateTimer.reset(); postrack = bOut; shootStep++; } break;
                            case 7: flickServo.setPosition(0.0); if (stateTimer.seconds() > 0.5) {stateTimer.reset(); shootStep++; } break;
                            case 8: flickServo.setPosition(0.3); if (stateTimer.seconds() > 0.5) {stateTimer.reset(); state = outtakeState.REST; } break;
                        }
                        break;
                    case REST:
                        shootStep = 0;
                        shooting = false;
                        stateTimer.reset();
                }
            }

            //transfer failsafe

            if(pressNowTransf && !pressPrevTransf){
                if (transfFailsafe){
                    cOut = postrackreset;
                    postrack = cOut;
                    transferServo.setPosition(cOut);
                    //TODO: THIS IS THER CORRECT OPMORE
                    bIn = postrack - (transferValues.cOut - transferValues.bIn);
                    aIn = postrack + (transferValues.aIn - transferValues.cOut);
                    bOut = postrack + (transferValues.bOut - transferValues.cOut);
                    cIn = postrack + (transferValues.cIn - transferValues.cOut);
                    aOut = postrack + (transferValues.aOut - transferValues.cOut);
                    rest = postrack + (transferValues.rest - transferValues.cOut);
                }
                transfFailsafe = !transfFailsafe;
                }

            if(transfFailsafe){
                //GOAL: get cOut to the top, calibrate the rest based on that
                if (gamepad2.right_bumper && postrack <= 0.995){postrackreset += 0.005;}
                if (gamepad2.left_bumper && postrack >= 0.005){postrackreset -= 0.005;}
                transferServo.setPosition(postrack);
                telemetry.addData("postrack", postrack);
            }


            if (C_INTAKE) {intakeServo.setPower(-1.0);}
            else if (C_SPIT){intakeServo.setPower(1.0);}
            else {intakeServo.setPower(0.0);}

            //manual but non-failsafe controls for transfer + kicker + outtake

            if (!shooting && !transfFailsafe) {
                if (C_MOVE_LEFT && !gamepad2.left_bumper && !gamepad2.right_bumper) {
                    transferServo.setPosition(aIn);
                    postrack = aIn;
                } else if (C_MOVE_RIGHT && !gamepad2.left_bumper && !gamepad2.right_bumper) {
                    transferServo.setPosition(aOut);//aout
                    postrack = aOut;
                } else if (C_MOVE_LEFT && gamepad2.left_bumper && !gamepad2.right_bumper) {
                    transferServo.setPosition(bIn);
                    postrack = bIn;
                } else if (C_MOVE_RIGHT && gamepad2.left_bumper && !gamepad2.right_bumper) {
                    transferServo.setPosition(bOut); //bOut
                    postrack = bOut;
                } else if (C_MOVE_LEFT && !gamepad2.left_bumper && gamepad2.right_bumper) {
                    transferServo.setPosition(cIn);
                    postrack = cIn;
                } else if (C_MOVE_RIGHT && !gamepad2.left_bumper && gamepad2.right_bumper) {
                    transferServo.setPosition(cOut);
                    postrack = cOut;
                }else if (C_MOVE_REST && !gamepad2.left_bumper && !gamepad2.right_bumper){
                    transferServo.setPosition(rest);
                }

                if (C_FLICK) {flickServo.setPosition(0.0);}
                else {flickServo.setPosition(0.3);}
            }

            if(manualFlywheel) {
                if (gamepad2.a && !prevG2A) {
                    outtakeMotorPower -= 0.05;
                } else if (gamepad2.b && !prevG2B) {
                    outtakeMotorPower += 0.05;
                }
                outtake_motor.setPower(outtakeMotorPower);
                prevG2A = gamepad2.a;
                prevG2B = gamepad2.b;
            }


            if(!turnCodeOn && !HEADING_FREEZE) {
                /*leftFrontDrive.setPower(leftFrontPower * speed * invDir);
                rightFrontDrive.setPower(rightFrontPower * speed * invDir);
                leftBackDrive.setPower(leftBackPower * speed * invDir);
                rightBackDrive.setPower(rightBackPower * speed * invDir);*/
                //drive(-
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

                leftFrontDrive.setPower(leftFrontPower  * speed);
                rightFrontDrive.setPower(rightFrontPower * speed);
                leftBackDrive.setPower(leftBackPower  * speed);
                rightBackDrive.setPower(rightBackPower * speed);
            }
            pressPrevShoot = pressNowShoot;
            pressPrevTransf = pressNowTransf;
            pressPrevManualShoot = pressNowManualShoot;
            pdiddyPrev = pdiddyNow;

            // Show the elapsed game time and wheel power.
            telemetry.addData("free schlep", "%4.2f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("diddy", "%4.2f", Math.abs((imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)+ 20)));
            telemetry.addData("eepstein", "%4.2f", drive.localizer.getPose().heading.toDouble()+ Math.toRadians(20));
            telemetry.addData("Distance from Center of Red Goal (cm): ", camera.centerDistanceCM());
            telemetry.addData("Target RPM", targRPM);
            telemetry.addData("Flywheel measured velocity", flywheel.getVelocity());

            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower * speed * invDir, rightFrontPower * speed * invDir);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower * speed * invDir, rightBackPower * speed * invDir);
            telemetry.addData("Speed", "%4.2f", speed);
            telemetry.addData("Invert Direction", "%1b", invertDir);

            telemetry.update();

            sleep(CYCLE_MS);

        }
    }
}