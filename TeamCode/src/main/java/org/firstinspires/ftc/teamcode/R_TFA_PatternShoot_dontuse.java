package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.shooter.ShooterControl;
import org.firstinspires.ftc.teamcode.tuning.shooter.RobotConstants;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.classes.Transfer_Values;

@Disabled
@TeleOp(name="R_TFA_PatternShoot_dontuse", group="Linear OpMode")
public class R_TFA_PatternShoot_dontuse extends LinearOpMode {
    public enum outtakeState {
        outC, outA, outB, ABC, BCA, CAB, kick, down, REST
    }
    int shootStep =0;



    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;

    private DcMotor outtake_motor;

    //private Servo intakeServo;
    private CRServo intakeServo;
    private Servo transferServo;
    private Servo flickServo;

    private DcMotorEx flywheel;
    //VoltageSensor battery;
    private ShooterControl shooter;
    private Transfer_Values transferValues;
    //private Transfer_Values transferValues;
    public Vision camera = new Vision();
    ElapsedTime stateTimer = new ElapsedTime();
    boolean shooting = false;

    private outtakeState state = outtakeState.down;

    boolean pressNowShoot = false;
    boolean pressPrevShoot = false;
    boolean patternDetected = false;

    String pattern = "PPG";
    String shootOrder = "ABC";


    @Override
    public void runOpMode() {

        WebcamName cam1 = hardwareMap.get(WebcamName.class, "Camera1");
        camera.setTarget(Vision.Target.red);
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
        boolean C_HALF_SPEED, C_INV_DIR, AUTO_SHOT, OUTTAKE_MANUAL, TRANSFER_MANUAL, C_INTAKE, C_SPIT, C_TRANSFER_PA, C_TRANSFER_PB, C_TRANSFER_PC, C_FLICK = false, C_MOVE_LEFT, C_MOVE_RIGHT;

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

        //intakeServo = hardwareMap.get(Servo.class, "intake_servo");
        intakeServo = hardwareMap.get(CRServo.class, "intake_servo");
        transferServo = hardwareMap.get(Servo.class, "transfer_servo");
        flickServo = hardwareMap.get(Servo.class, "flick_servo");


        double aIn = transferValues.aIn;
        double bIn = transferValues.bIn;
        double cIn = transferValues.cIn;
        double aOut = transferValues.aOut;
        double bOut = transferValues.bOut;
        double cOut = transferValues.cOut;
        double rest = transferValues.rest;


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

        boolean reached = false;
        boolean turnCodeOn = false;
        double alignVal=10000;
        double turnSpeed = 0.15;
        double originValue=0;

        int timesOut = 0;
        int timesNeeded =3;

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

            //TODO: CHECK WITH ALLIE ON THE FOLLOWING FOUR
            C_SPIT = gamepad2.b;
            AUTO_SHOT = gamepad2.a;
            OUTTAKE_MANUAL =  gamepad2.a; //TODO: add
            TRANSFER_MANUAL =  gamepad2.a; //TODO: add


            //SET_RPM = gamepad2.dpad_up;
            //failsafes we need: outtake manual, intake flip power, transfer reset

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

            String sequence = camera.scanForPattern();
            if (!patternDetected && !(sequence.equals("none"))){
                pattern = sequence;
                patternDetected = true;
            }

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


                /*if(Math.abs(alignVal)>70){
                    turnSpeed = 1;
                }else{
                    turnSpeed = 0.8/67*(Math.abs(alignVal)-70)+1;
                }*/

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
                        //turnright

                        leftFrontDrive.setPower(1 * turnSpeed);
                        leftBackDrive.setPower(1 * turnSpeed);
                        rightFrontDrive.setPower(-1 * turnSpeed);
                        rightBackDrive.setPower(-1 * turnSpeed);
                        timesOut = 0;
                        telemetry.addData("turning: ","right");

                    } else {

                        timesOut++;
                    }
                }
                /*}else {
                    turnCodeOn = false;
                    leftFrontDrive.setPower(0);
                    leftBackDrive.setPower(0);
                    rightFrontDrive.setPower(0);
                    rightFrontDrive.setPower(0);
                    telemetry.addData("turning: ","none");


                }*/

            }


            pressNowShoot = AUTO_SHOT;




            distFromGoal = camera.centerDistanceCM();

            presentVoltage = battery.getVoltage();
            shooter.setBatteryVoltage(presentVoltage);

            if(spinningUp){
                double dt = timer.seconds();
                targRPM += dt * flywheelAccel;
                targRPM = Range.clip(targRPM, 0, 980);
                if (targRPM >= 920){spinningUp =false;}
            }
            //if(!spinningUp && (distFromGoal != -1)){targRPM = (0.560459*distFromGoal)+760.13857;}
            if(!spinningUp && (distFromGoal >= 140) && (distFromGoal <= 270)){
                //shooter.setKf(0.00965);
                targRPM = (0.546172*distFromGoal)+722.4006;
            }
            else if(!spinningUp && (distFromGoal > 320)){
                //shooter.setKf(0.0105);
                targRPM = (980); //TODO: need equation for this
            }
            else if(!spinningUp && (distFromGoal < 140) && (distFromGoal != -1)){
                //shooter.setKf(0.00965);
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

            targRPM = Range.clip(targRPM, 0, 980);

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





            if(pressNowShoot && !pressPrevShoot){shooting = !shooting; timer.reset();} //if they're opposite i.e. it's changed

            if (shooting){
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
                //stateTimer.reset();
            }else{ //stop the sequence
                state = outtakeState.REST;
                flickServo.setPosition(0.3);
            }

            pressPrevShoot = pressNowShoot;

            if(shooting) {
                switch (state) {
                    case ABC:
                        switch (shootStep){
                        case 0: transferServo.setPosition(aOut); if (stateTimer.seconds() > 0.5) {stateTimer.reset(); postrack = aOut; shootStep++; } break;
                        case 1: flickServo.setPosition(0.0); if (stateTimer.seconds() > 0.5) {stateTimer.reset(); shootStep++; } break;
                        case 2: flickServo.setPosition(0.3); if (stateTimer.seconds() > 0.5) {stateTimer.reset(); shootStep++; } break;

                        case 3: transferServo.setPosition(bOut); if (stateTimer.seconds() > 0.5) {stateTimer.reset(); postrack = bOut; shootStep++; } break;
                        case 4: flickServo.setPosition(0.0); if (stateTimer.seconds() > 0.5) {stateTimer.reset(); shootStep++; } break;
                        case 5: flickServo.setPosition(0.3); if (stateTimer.seconds() > 0.5) {stateTimer.reset(); shootStep++; } break;

                        case 6: transferServo.setPosition(cOut); if (stateTimer.seconds() > 0.5) {stateTimer.reset(); postrack = cOut; shootStep++; } break;
                        case 7: flickServo.setPosition(0.0); if (stateTimer.seconds() > 0.5) {stateTimer.reset(); shootStep++; } break;
                        case 8: flickServo.setPosition(0.3); if (stateTimer.seconds() > 0.5) {stateTimer.reset(); state = outtakeState.REST; } break;
                        }
                        break;

                    case BCA:
                        switch (shootStep){
                            case 0: transferServo.setPosition(bOut); if (stateTimer.seconds() > 0.5) {stateTimer.reset(); postrack = bOut; shootStep++; } break;
                            case 1: flickServo.setPosition(0.0); if (stateTimer.seconds() > 0.5) {stateTimer.reset(); shootStep++; } break;
                            case 2: flickServo.setPosition(0.3); if (stateTimer.seconds() > 0.5) {stateTimer.reset(); shootStep++; } break;

                            case 3: transferServo.setPosition(cOut); if (stateTimer.seconds() > 0.5) {stateTimer.reset(); postrack = cOut; shootStep++; } break;
                            case 4: flickServo.setPosition(0.0); if (stateTimer.seconds() > 0.5) {stateTimer.reset(); shootStep++; } break;
                            case 5: flickServo.setPosition(0.3); if (stateTimer.seconds() > 0.5) {stateTimer.reset(); shootStep++; } break;

                            case 6: transferServo.setPosition(aOut); if (stateTimer.seconds() > 0.5) {stateTimer.reset(); postrack = aOut; shootStep++; } break;
                            case 7: flickServo.setPosition(0.0); if (stateTimer.seconds() > 0.5) {stateTimer.reset(); shootStep++; } break;
                            case 8: flickServo.setPosition(0.3); if (stateTimer.seconds() > 0.5) {stateTimer.reset(); state = outtakeState.REST; } break;
                        }
                        break;

                    case CAB:
                        switch (shootStep){
                            case 0: transferServo.setPosition(cOut); if (stateTimer.seconds() > 0.5) {stateTimer.reset(); postrack = cOut; shootStep++; } break;
                            case 1: flickServo.setPosition(0.0); if (stateTimer.seconds() > 0.5) {stateTimer.reset(); shootStep++; } break;
                            case 2: flickServo.setPosition(0.3); if (stateTimer.seconds() > 0.5) {stateTimer.reset(); shootStep++; } break;

                            case 3: transferServo.setPosition(aOut); if (stateTimer.seconds() > 0.5) {stateTimer.reset(); postrack = aOut; shootStep++; } break;
                            case 4: flickServo.setPosition(0.0); if (stateTimer.seconds() > 0.5) {stateTimer.reset(); shootStep++; } break;
                            case 5: flickServo.setPosition(0.3); if (stateTimer.seconds() > 0.5) {stateTimer.reset(); shootStep++; } break;

                            case 6: transferServo.setPosition(bOut); if (stateTimer.seconds() > 0.5) {stateTimer.reset(); postrack = bOut; shootStep++; } break;
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

                if (C_INTAKE) {
                    //intakeServo.setPosition(0.0);
                    intakeServo.setPower(-1);
                    isEnter = true;
                } else if (C_SPIT){
                    intakeServo.setPower(1);
                }else {
                    //intakeServo.setPosition(0.5);
                    intakeServo.setPower(0);
                }

            if (!shooting) {

                if (C_MOVE_LEFT && !gamepad2.left_bumper && !gamepad2.right_bumper) {
                    transferServo.setPosition(aIn);
                    postrack = aIn;
                } else if (C_MOVE_RIGHT && !gamepad2.left_bumper && !gamepad2.right_bumper) {
                    transferServo.setPosition(aOut);
                    postrack = aOut;
                } else if (C_MOVE_LEFT && gamepad2.left_bumper && !gamepad2.right_bumper) {
                    transferServo.setPosition(bIn);
                    postrack = bIn;
                } else if (C_MOVE_RIGHT && gamepad2.left_bumper && !gamepad2.right_bumper) {
                    transferServo.setPosition(bOut);
                    postrack = bOut;
                } else if (C_MOVE_LEFT && !gamepad2.left_bumper && gamepad2.right_bumper) {
                    transferServo.setPosition(cIn);
                    postrack = cIn;
                } else if (C_MOVE_RIGHT && !gamepad2.left_bumper && gamepad2.right_bumper) {
                    transferServo.setPosition(cOut);
                    postrack = cOut;
                }

                if(!shooting){
                if (C_FLICK) {
                    flickServo.setPosition(0.0);
                    isEnter = true;
                } else {
                    flickServo.setPosition(0.3);
                }
            }
            }


            // Send calculated power to wheels

            if(!turnCodeOn) {
                leftFrontDrive.setPower(leftFrontPower * speed * invDir);
                rightFrontDrive.setPower(rightFrontPower * speed * invDir);
                leftBackDrive.setPower(leftBackPower * speed * invDir);
                rightBackDrive.setPower(rightBackPower * speed * invDir);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Distance from Center of Red Goal (cm): ", camera.centerDistanceCM());
            telemetry.addData("target RPM", targRPM);
            telemetry.addData("flywheel measured velocity", flywheel.getVelocity());
            telemetry.addData("pattern", pattern);

            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower * speed * invDir, rightFrontPower * speed * invDir);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower * speed * invDir, rightBackPower * speed * invDir);
            telemetry.addData("Speed", "%4.2f", speed);
            telemetry.addData("Invert Direction", "%1b", invertDir);

            telemetry.update();

            sleep(CYCLE_MS);

        }
    }
}