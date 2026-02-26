package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;

@TeleOp(name="TRANSFER RESET", group="Linear OpMode")
public class Transfer_Reset extends LinearOpMode {

    private Servo transferServo;


    @Override
    public void runOpMode() {





        final int CYCLE_MS = 50;

        transferServo = hardwareMap.get(Servo.class, "transfer_servo");

        double positrack = 0;


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();



        transferServo.setPosition(0);
        waitForStart();

        while (opModeIsActive()) {

            if (gamepad2.a){
                positrack += 0.01;
            }
            if (gamepad2.b){
                positrack -= 0.01;
            }

            transferServo.setPosition(positrack);





            // KEYBINDS
            /*
             * 1) Axial:    Driving axial and backward               Left-joystick axial/Backward
             * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
            */
            telemetry.addData("POSITION", transferServo.getPosition());

            telemetry.update();

            sleep(CYCLE_MS);

        }
    }
}