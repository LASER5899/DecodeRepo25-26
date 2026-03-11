package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
// import org.firstinspires.ftc.teamcode.MotorPowerTest_OTOS_Config;
public class MotorPowerTest_OTOS_Config {

    //TODO: get values

    public double SPEED_GAIN  = 0.03;
    public double STRAFE_GAIN = 0.15;
    public double TURN_GAIN   = 0.03;

    public double MAX_AUTO_SPEED  = 0.4;
    public double MAX_AUTO_STRAFE = 0.4;
    public double MAX_AUTO_TURN   = 0.4;

    private final LinearOpMode opMode;
    private final DcMotor lf, lb, rf, rb;
    private final SparkFunOTOS otos;
    private final ElapsedTime runtime = new ElapsedTime();

    public MotorPowerTest_OTOS_Config(LinearOpMode opMode,
                         DcMotor leftFront, DcMotor leftBack,
                         DcMotor rightFront, DcMotor rightBack,
                         SparkFunOTOS otos) {
        this.opMode = opMode;
        this.lf = leftFront; this.lb = leftBack;
        this.rf = rightFront; this.rb = rightBack;
        this.otos = otos;
    }

    public void configureOtos() {
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);

        // TODO: get values
        otos.setOffset(new SparkFunOTOS.Pose2D(7, 0, 270));
        otos.setLinearScalar(1.008);
        otos.setAngularScalar(0.992);

        otos.calibrateImu();
        otos.resetTracking();
        otos.setPosition(new SparkFunOTOS.Pose2D(0, 0, -135)); //TODO: double check heading

        SparkFunOTOS.Version hw = new SparkFunOTOS.Version(); //hardware
        SparkFunOTOS.Version fw = new SparkFunOTOS.Version(); //firmware
        otos.getVersionInfo(hw, fw);
    }

    public SparkFunOTOS.Pose2D getPose() {
        SparkFunOTOS.Pose2D p = otos.getPosition();
        return new SparkFunOTOS.Pose2D(p.x, p.y, -p.h);
    }


    public void moveRobot(double x, double y, double yaw) {
        double leftFrontPower    =  x;// +y +yaw;
        double rightFrontPower   =  x;// -y -yaw;
        double leftBackPower     =  x;// -y +yaw;
        double rightBackPower    =  x;// +y -yaw;

        double max = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)), Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)));
        if (max > 1.0) {
            leftFrontPower /= max; rightFrontPower /= max; leftBackPower /= max; rightBackPower /= max;
        }

        lf.setPower(leftFrontPower * 0.4);
        rf.setPower(rightFrontPower * 0.4);
        lb.setPower(leftBackPower * 0.4);
        rb.setPower(rightBackPower * 0.4);


    }
}
