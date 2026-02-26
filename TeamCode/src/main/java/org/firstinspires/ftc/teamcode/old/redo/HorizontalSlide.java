package org.firstinspires.ftc.teamcode.old.redo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.old.redo.RobotHardware;


// this is all copied from verticalslide none of this works yet :(
public class HorizontalSlide {

    public enum HorizontalSlideState {
        HOME, LOW, MID, HIGH, RESETTING
    }
    private final DcMotor slide;
    private HorizontalSlideState state = HorizontalSlideState.HOME;

    public HorizontalSlide(HardwareMap hardwareMap, String deviceName) {
        slide = hardwareMap.get(DcMotor.class, deviceName);
        slide.setTargetPosition(10);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // passing dcmotor commands to our class
    public void setTargetPosition(int pos) { slide.setTargetPosition(pos); }
    public void setMode(DcMotor.RunMode mode) { slide.setMode(mode);}
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) { slide.setZeroPowerBehavior(behavior);}
    public void setDirection(DcMotor.Direction direction) { slide.setDirection(direction);}
    public boolean isBusy() { return slide.isBusy();}
    public void setPower(double p) { slide.setPower(p); }
    public int getCurrentPosition() { return slide.getCurrentPosition(); }


    public void update(boolean upPressed, boolean downPressed, boolean resetPressed) {
        switch(state) {
            case HOME:
                slide.setTargetPosition(0);
                slide.setPower(1.0);
                if (upPressed) state = HorizontalSlideState.LOW;
                else if (resetPressed) state = HorizontalSlideState.RESETTING; // juuuust in case something weird happened with the encoders
                break;
            case LOW:
                slide.setTargetPosition(78);
                slide.setPower(1.0);
                if (upPressed) state = HorizontalSlideState.MID;
                else if (resetPressed) state = HorizontalSlideState.RESETTING;
                break;
            case MID:
                slide.setTargetPosition(2669);
                slide.setPower(1.0);
                if (upPressed) state = HorizontalSlideState.HIGH;
                else if (downPressed) state = HorizontalSlideState.HOME;
                else if (resetPressed) state = HorizontalSlideState.RESETTING;
                break;
            case HIGH:
                slide.setTargetPosition(5024);
                slide.setPower(0.7);
                if (downPressed) state = HorizontalSlideState.HOME;
                else if (resetPressed) state = HorizontalSlideState.RESETTING;
                break;
            case RESETTING:
                slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // if encoder skips or drifts or smth
                slide.setPower(-0.5);
                if (!resetPressed) state = HorizontalSlideState.HOME; // assumes a hold to reset
                break;


        }
    }
}
