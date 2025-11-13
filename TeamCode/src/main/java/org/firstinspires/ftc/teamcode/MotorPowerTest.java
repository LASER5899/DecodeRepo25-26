package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

// use the blue goal only
// blue goal pos is defined as (-62, 62) inches, with the extra 2 inches for safety
// for now, assume fixed heading
@TeleOp(name="Motor Power Test", group="Linear OpMode")
public class MotorPowerTest extends LinearOpMode {


    private DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;
    private DcMotor outtake_motor;

    private SparkFunOTOS otos;
    private VoltageSensor myControlHubVoltageSensor;
    double presentVoltage;

    final static double TICKS_PER_REV = 537.7;
    final static double WHEEL_RADIUS  = 1.88976; // inches
    final static double GEAR_RATIO    = 1.0;

    public static double ticksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    //TODO: DO NOT PUSH! I DELETED SHOTPOSITIONER

    @Override
    public void runOpMode() {

        final int CYCLE_MS = 50;

        final double speed = 0.4;   // used to manage half speed, defaults to full speed

        double posMag, distMag, distNorm;

        final double distMax = 62 * Math.sqrt(2);


        final double distMin = (62 * Math.sqrt(2)) - 29 ; //TODO: Find this empirically

        final double goalX = -62;

        final double goalY = 62;

        final double minPow = 0.35;

        final double maxPow = 0.85;

        final double a = 0;
        final double b = 0;
        final double c = 0;

        double outtakeCalcPower;

        double C_LATERAL, C_AXIAL, C_YAW;

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "right_back_drive");

        otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

        outtake_motor   = hardwareMap.get(DcMotor.class, "outtake_drive");

        myControlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        presentVoltage = myControlHubVoltageSensor.getVoltage();

        double outtakeMotorPower = -0.05;

        /* MotorPowerTest_OTOS_Config odoDrive =
                new MotorPowerTest_OTOS_Config(this, leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive, otos);
        odoDrive.configureOtos(); */

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();



        while (opModeIsActive()) {

            presentVoltage = myControlHubVoltageSensor.getVoltage();

            double leftTicks = (leftFrontDrive.getCurrentPosition() + leftBackDrive.getCurrentPosition()) / 2.0;
            double rightTicks = (rightFrontDrive.getCurrentPosition() + rightBackDrive.getCurrentPosition()) / 2.0;

            double distFromStart = ticksToInches((int) ((leftTicks + rightTicks) / 2.0));
            distMag = distMax - distFromStart;

            distNorm = ( (distMag - distMin) / (distMax - distMin) );
            distNorm = Range.clip(distNorm, 0, 1); // extra safety layer

            double quadCurve = a * distNorm * distNorm + b * distNorm + c;

            outtakeCalcPower = minPow + (maxPow - minPow) * quadCurve;
            outtakeCalcPower = Range.clip(outtakeCalcPower, 0, 1);


            //begin otos power calculation

            /*SparkFunOTOS.Pose2D vecPos = odoDrive.getPose();


            posMag = Math.sqrt( (Math.pow(vecPos.x,2) + Math.pow(vecPos.y,2)));

            distMag = Math.hypot( (goalX - vecPos.x), (goalY - vecPos.y) );

            distNorm = ( (distMag - distMin) / (distMax - distMin) );
            distNorm = Range.clip(distNorm, 0, 1); // extra safety layer

            outtakeCalcPower = minPow + (maxPow - minPow) * Math.sqrt(distNorm);
            outtakeCalcPower = Range.clip(outtakeCalcPower, 0, 1); // extra safety layer*/


            //end power calculation

            //begin teleop drive config

            C_AXIAL       = gamepad1.left_stick_y; //keep
            C_LATERAL     = gamepad1.left_stick_x;
            C_YAW         = gamepad1.right_stick_x;
            double max;
            double axial   = -C_AXIAL;
            double lateral =  C_LATERAL;
            double yaw     =  C_YAW;
            double leftFrontPower  = axial; //+ lateral + yaw;
            double rightFrontPower = axial; //- lateral - yaw;
            double rightBackPower  = axial; //+ lateral - yaw;
            double leftBackPower   = axial; //- lateral + yaw;
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }
            leftFrontDrive.setPower(leftFrontPower * speed);
            rightFrontDrive.setPower(rightFrontPower * speed);
            leftBackDrive.setPower(leftBackPower * speed);
            rightBackDrive.setPower(rightBackPower * speed);

            //end teleop drive config


            //begin test mode

            if (gamepad1.a && (outtakeMotorPower > -0.95)) {
                outtakeMotorPower -= 0.01;
            } else if (gamepad1.b && (outtakeMotorPower <= -0.01) ) {
                outtakeMotorPower += 0.01;
            }

            outtake_motor.setPower(outtakeMotorPower);
            //end test mode



            /*
            TODO: final goal: based on distance and voltage

            current: based on distance from close
            - get otos live reading position
             have two initial positions for the corner of the far and close goal
            - for now, robot angle is fixed
            - square root curve
            1. normalize distance: distNorm = ( (magDist - distMin) / (distMax - distMin) )
            2. apply sqrt: rawPow = Math.sqrt(distNorm)
            3. go from normalized to the range: minPow + (maxPow - minPow) * Math.sqrt(distNorm);

             */

            /*hb
            max dist 0.80
            min dist power 0.81, 33 inches from back*/


            telemetry.addData("MotorPower", "%4.2f", outtakeMotorPower);
            telemetry.addData("PresentVoltage", "%4.2f", presentVoltage);

            /*telemetry.addData("x", vecPos.x);
            telemetry.addData("y", vecPos.y);
            telemetry.addData("theta", vecPos.h);*/
            telemetry.update();

            sleep(CYCLE_MS);

        }
    }
}
