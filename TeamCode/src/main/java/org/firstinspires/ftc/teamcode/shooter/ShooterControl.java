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

    public void setBatteryVoltage(double v) { this.batteryVoltage = Math.max(9,v); }
    public double getTargetRPM() { return targetRPM; }
    public double getRampingRPM() { return rampingRPM; }
    public double getActualRPM() { return smoothedRPM; }
    public double getRawRPM() { return currentRPM; }
    public double getPower() { return correctPow; }
    public double getBatteryVoltage() { return batteryVoltage; }


    public void setKp(double kp) { this.Kp = kp; }
    public void setKf(double kf) { this.Kf = kf; }
    public void setKi(double ki) { this.Ki = ki; }
    public void setKd(double kd) { this.Kd = kd; }
    public double getKp() { return Kp; }
    public double getKf() { return Kf; }
    public double getKi() { return Ki; }
    public double getKd() { return Kd; }

    enum FlywheelState {RAMP, HOLD, DOWN};

    FlywheelState state = FlywheelState.RAMP;



    private DcMotorEx flywheel;
    final double TICKS_PER_REV = 112;

    ElapsedTime timer = new ElapsedTime();

    ElapsedTime timerTop = new ElapsedTime();

    double holdSeconds;

    double batteryVoltage = 12;

    double targetRPM; // don't set an initial value here, this should be passed from teleop
    double rampingRPM = 0;
    public boolean rToggle = false;
    double maxRPM = 5000; //TODO: find a value
    double currentRPM, smoothedRPM; // currentRPM from getVelocity will always have noise, so smoothedRPM will make it a little more steady
    double correctPow;
    double maxAccel; //= 1000; //TODO: find a value

    double Kp = 0.015;
    double Kf = 0.00988;
    double Kd = 0.00; //TODO: find;
    double Ki, integralSum, derivative; // prob won't use ki kd but have in case
    double error, lastError;

    boolean firstLoop = true;
   public ShooterControl(HardwareMap hwMap) {

        flywheel = hwMap.get(DcMotorEx.class, "outtake_drive");
        flywheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setDirection(DcMotorEx.Direction.REVERSE); // pick the correct one

        timer.reset();
    }
    public double rpmToPower(double targetRPM, double dt)  {

       //dt = Range.clip(timer.seconds(), 0, 0.05);

        // get encoder pos
        currentRPM = flywheel.getVelocity() * 60 / TICKS_PER_REV;

        if (firstLoop) { // need a working initial value for smoothedRPM
            smoothedRPM = currentRPM;
            firstLoop = false;
        }

        smoothedRPM = (0.9 * smoothedRPM + 0.1 * currentRPM);

        error = targetRPM - smoothedRPM;

        if (Math.abs(error) < 200) {integralSum +=  (error * dt);} // prevent spiking/windup

        if (dt >= 1e-4) {derivative = (error - lastError) / dt;} // skip if dt is too small because then derivative will be huge

        correctPow = (Kp * error) + (Ki * integralSum) + (Kd * derivative) + (Kf * targetRPM / batteryVoltage); //13.6 ish at beginning
        correctPow = Range.clip(correctPow,-1,1);

        lastError = error;

        return correctPow;

    }

    public void flywheelUpdate() {

        double dt = timer.seconds();
        dt = Range.clip(dt, 0.005, 0.05);
       //double dt = Math.max(dt, 0.005);

       double holdTop = timerTop.seconds();

        /*switch(state) {

            case RAMP:
                if (rampingRPM < targetRPM) {
                    rampingRPM += maxAccel * dt;
                    rampingRPM = Range.clip(rampingRPM,0,targetRPM);
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
                //rampingRPM -= maxAccel * dt;
                if (rampingRPM > 500){
                    rampingRPM -= maxAccel * dt;
                    rampingRPM = Range.clip(rampingRPM,0,targetRPM);

                }
                if (rampingRPM < 500) {
                    rampingRPM = 500;
                    state = FlywheelState.RAMP;
                }

                rampingRPM = Range.clip(rampingRPM,0,targetRPM);

                break;

        }*/

        correctPow = rpmToPower(rampingRPM, dt);
        flywheel.setPower(correctPow);

        timer.reset();

    }

    public void flywheelHold(){

        //double dt = Range.clip(timer.seconds(), 0, 0.05);
        double dt = timer.seconds();
        dt = Range.clip(dt, 0.005, 0.05);
        double holdTop = timerTop.seconds();

        correctPow = rpmToPower(targetRPM, dt);
        flywheel.setPower(correctPow);

        timer.reset();

    }

}

