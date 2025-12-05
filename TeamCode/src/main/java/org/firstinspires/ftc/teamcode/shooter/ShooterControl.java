package org.firstinspires.ftc.teamcode.shooter;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class ShooterControl {

    private DcMotorEx flywheel;
    final double TICKS_PER_REV = 537.7;

    ElapsedTime timer = new ElapsedTime();
    double dt;

    double targetRPM; // don't set an initial value here, this should be passed from teleop
    double rampingRPM = 0;
    double maxRPM = 5000; //TODO: find a value
    double currentRPM, smoothedRPM; // currentRPM from getVelocity will always have noise, so smoothedRPM will make it a little more steady
    double correctPow;
    double maxAccel = 2000; //TODO: find a value

    double Kp = 0;
    double Kf = 1.0 / maxRPM;
    double Ki, Kd, integralSum, derivative; // not using ID, so not using these
    double error, lastError;

    boolean firstLoop = true;
    public ShooterControl(HardwareMap hwMap) {
        // map motors and servos
        flywheel = hwMap.get(DcMotorEx.class, "outtake_drive");
        flywheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        timer.reset();
    }


    public double rpmToPower(double targetRPM) {

        // obtain the encoder position
        currentRPM = flywheel.getVelocity() * 60 / TICKS_PER_REV;

        if (firstLoop) { // need a working initial value for smoothedRPM
            smoothedRPM = currentRPM;
            firstLoop = false;
        }

        smoothedRPM = 0.9 * smoothedRPM + 0.1 * currentRPM;

        error = targetRPM - smoothedRPM;

        correctPow = Kp * error + Kf * targetRPM;
        correctPow = Range.clip(correctPow,-1,1);

        return correctPow;

    }

    public void flywheelUpdate() {

        dt = timer.seconds();

        if (rampingRPM < targetRPM) {
            rampingRPM += maxAccel * dt;
            rampingRPM = Range.clip(rampingRPM,0,targetRPM);
        }

        else if (rampingRPM > targetRPM) {
            rampingRPM -= maxAccel * dt;
            rampingRPM = Range.clip(rampingRPM,targetRPM,maxRPM);
        }

        correctPow = rpmToPower(rampingRPM);
        flywheel.setPower(correctPow);

        timer.reset();

    }






}

// the following things would be for PID, but we're doing PF

/*
if (Math.abs(error) < 200) {integralSum +=  (error * dt);}
derivative = (error - lastError) / dt;
correctPow = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
lastError = error;
*/

// ctrlaltftc has some good resources