package org.firstinspires.ftc.teamcode.tuning.shooter;
import org.firstinspires.ftc.teamcode.shooter.ShooterControl;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.config.Config;
//@Config
@TeleOp(name="Shooter Tuning", group="Linear OpMode")
//@Disabled
public class ShooterTuning extends LinearOpMode {

   // public static double TARGET_RPM =  2500;
    //public static double kP = 0; //tune kF frist
    //public static double kF = 0.00016;
    //public static PIDCoefficients TURNING_PID = new PIDCoefficients();

    private ElapsedTime runtime = new ElapsedTime();

    private ShooterControl shooter;



    @Override


    public void runOpMode() {

        final int CYCLE_MS = 10;


        shooter = new ShooterControl(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            shooter.setTargetRPM(RobotConstants.TARGET_RPM);
            shooter.setKf(RobotConstants.kF);
            shooter.setKp(RobotConstants.kP);
            shooter.setMaxAccel(RobotConstants.maxAccel);
            shooter.flywheelUpdate();







            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time:" + runtime.toString());
            telemetry.addData("actualRPM", shooter.getActualRPM());  // smoothedRPM
            telemetry.addData("rawRPM", shooter.getRawRPM());        // currentRPM
            telemetry.addData("targetRPM", shooter.getTargetRPM());  // what you command
            telemetry.addData("rampingRPM", shooter.getRampingRPM());
            telemetry.addData("power", shooter.getPower());
            //telemetry.addData("rampingRPM", ShooterControl.rToggle());
            telemetry.update();

            //sleep(CYCLE_MS);

            if (!opModeIsActive()) {

            }
        }
    }
}