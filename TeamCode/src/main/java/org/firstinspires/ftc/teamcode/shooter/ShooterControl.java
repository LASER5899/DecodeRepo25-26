package org.firstinspires.ftc.teamcode.shooter;

import static java.lang.Math.ceil;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class ShooterControl {



    public void setTargetRPM(double rpm) { this.targetRPM = rpm; }

    public void setMaxAccel(double maxAccel) { this.maxAccel = maxAccel; }

    public void setHoldSeconds(double holdSeconds) { this.holdSeconds = holdSeconds; }
    public double getTargetRPM() { return targetRPM; }
    public double getRampingRPM() { return rampingRPM; }
    public double getActualRPM() { return smoothedRPM; }
    public double getRawRPM() { return currentRPM; }
    public double getPower() { return correctPow; }


    public void setKp(double kp) { this.Kp = kp; }
    public void setKf(double kf) { this.Kf = kf; }
    public double getKp() { return Kp; }
    public double getKf() { return Kf; }

    enum FlywheelState {RAMP, HOLD, DOWN};

    FlywheelState state = FlywheelState.RAMP;



    private DcMotorEx flywheel;
    final double TICKS_PER_REV = 537.7;

    ElapsedTime timer = new ElapsedTime();

    ElapsedTime timerTop = new ElapsedTime();
    //double dt;

    //double holdTop;

    double holdSeconds;

    double targetRPM; // don't set an initial value here, this should be passed from teleop
    double rampingRPM = 0;
    public boolean rToggle = false;
    double maxRPM = 5000; //TODO: find a value
    double currentRPM, smoothedRPM; // currentRPM from getVelocity will always have noise, so smoothedRPM will make it a little more steady
    double correctPow;
    double maxAccel; //= 1000; //TODO: find a value

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

        smoothedRPM = ceil(0.9 * smoothedRPM + 0.1 * currentRPM);

        error = targetRPM - smoothedRPM;

        correctPow = Kp * error + Kf * targetRPM;
        correctPow = Range.clip(correctPow,-1,1);

        return correctPow;

    }

    public void flywheelUpdate() {

       double dt = timer.seconds();

       double holdTop = timerTop.seconds();

        switch(state) {

            case RAMP:
                if (rampingRPM < targetRPM) {
                    rampingRPM += maxAccel * dt;
                    rampingRPM = Range.clip(rampingRPM,0,targetRPM);
                    //if ((rampingRPM == targetRPM) && smoothedRPM > (targetRPM-10)){rToggle = true;}
                    if (rampingRPM == targetRPM){
                        timerTop.reset();
                        state = FlywheelState.HOLD;
                    }

                }
                break;

            case HOLD:
                rampingRPM = targetRPM;
                if (timerTop.seconds() >= holdSeconds) {
                    state = FlywheelState.DOWN;
                }
                break;


            case DOWN:
                rampingRPM -= maxAccel * dt;
                if (rampingRPM > 500){
                    rampingRPM -= maxAccel * dt;
                    rampingRPM = Range.clip(rampingRPM,0,targetRPM);

                }
                if (rampingRPM < 500) {
                    rampingRPM = 500;
                    state = FlywheelState.RAMP;
                }

                //rampingRPM = Range.clip(rampingRPM,targetRPM,maxRPM);
                rampingRPM = Range.clip(rampingRPM,0,targetRPM);
                //if (rampingRPM <= 400){rToggle = false;}
                //if (rampingRPM < 150 ){rToggle = false;}
                break;


        }



        /*else if (rampingRPM > 0 && rToggle) { //targetRPM
            rampingRPM -= maxAccel * dt;
            //rampingRPM = Range.clip(rampingRPM,targetRPM,maxRPM);
            rampingRPM = Range.clip(rampingRPM,0,targetRPM);
            if (rampingRPM <= 400){rToggle = false;}
            //if (rampingRPM < 150 ){rToggle = false;}
        }*/

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