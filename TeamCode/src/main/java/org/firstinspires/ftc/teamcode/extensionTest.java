package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="lift", group="auto")
public class extensionTest extends LinearOpMode{
    public DcMotor liftLeft;
    public DcMotor liftRight;

    @Override
    public void runOpMode()  {
        liftLeft  = hardwareMap.get(DcMotor.class, "liftLeft");
        liftRight  = hardwareMap.get(DcMotor.class, "liftRight");
        liftRight.setDirection(DcMotorSimple.Direction.REVERSE);

       /* liftLeft.setTargetPosition(500);
        liftRight.setTargetPosition(500);

        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/

        for(int i = 0; i <300; i ++){
            sleep(5);
            liftLeft.setPower(0.5);
            //sleep(5);
            liftRight.setPower(0.5);
        }
    }

}
