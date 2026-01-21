package org.firstinspires.ftc.teamcode.tuning.shooter;
import org.firstinspires.ftc.teamcode.shooter.ShooterControl;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.config.Config;
//@Config
@TeleOp(name="Shooter Tuning", group="Linear OpMode")
//@Disabled
public class ShooterTuning extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private ShooterControl shooter;

    private DcMotorEx flywheel;
    private Servo flickServo;

    @Override


    public void runOpMode() {

        final int CYCLE_MS = 10;

        shooter = new ShooterControl(hardwareMap);
        flickServo = hardwareMap.get(Servo.class, "flick_servo");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //VoltageSensor myControlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();

        //TODO: in the tested code, hardwareMap.get voltagesensor was being used, but it's changed. test again



        double presentVoltage;

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            presentVoltage = battery.getVoltage();
            shooter.setBatteryVoltage(presentVoltage);

            shooter.setTargetRPM(RobotConstants.TARGET_RPM);

            shooter.setKf(RobotConstants.kF);
            shooter.setKp(RobotConstants.kP);
            shooter.setKi(RobotConstants.kI);
            shooter.setKd(RobotConstants.kD);

            shooter.setMaxAccel(RobotConstants.maxAccel);

            //shooter.flywheelUpdate();
            shooter.flywheelHold();

            if (gamepad2.y) {
                flickServo.setPosition(0.0);

            } else {
                flickServo.setPosition(0.3);
            }


            telemetry.addData("Status", "Run Time:" + runtime.toString());
            telemetry.addData("actualRPM", shooter.getActualRPM());  // smoothedRPM
            telemetry.addData("rawRPM", shooter.getRawRPM());        // currentRPM
            telemetry.addData("targetRPM", shooter.getTargetRPM());  // what you command
            telemetry.addData("rampingRPM", shooter.getRampingRPM());
            telemetry.addData("power", shooter.getPower());

            telemetry.update();

            sleep(CYCLE_MS);

            if (!opModeIsActive()) {

            }
        }
    }
}