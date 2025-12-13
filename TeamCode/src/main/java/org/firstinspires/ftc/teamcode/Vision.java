package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;


import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
//package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;




import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;





/*
 *      This class is made to give the position of the goal relative to the robot.
 *
 *
 *       aprilTagSetUp(WebcamName camera) sets up the april tag runner with the camera needed. Please use this function to initialize before running the opMode
 *       scanForPattern() returns the game pattern as one of the options below. If the Pattern is currently none, it will scan for the tag and change it to
 *          Possible Patterns: Vision.Pattern.none, Vision.Pattern.PPG, Vision.Pattern.PGP, Vision.Pattern.GPP
 *       scanForTarget() Updates all values on the goal target.
 *
 *
 *
 *
 *      These three are particularly useful in aiming:::
 *       centerDistanceCM() returns how far away the goal is.
 *       horizAngle() returns the angle left and right the goal is relative the the camera.
 *       vertAngle() returns the angle to the top of the goal.
 *
 */
public class Vision {

    // public Vision(HardwareMap map) {
    //     this.hardwareMap = map;
    //}

    //CHANGE STUFF HERE
    static final double TAG_TO_CENTER = 22.5; //put the horizontal distance from tag to center of goal here
    static final double TAG_TO_TOP = 23.5; //put the vertical distance from center of tag to top of goal here

    static boolean devModeOn = false; //change default mode of devMode here. Can be changed in code using Vision.toggleDevMode();

    //String webCamName="Camera1";//put the webCamName here.
    //HardwareMap hardwareMap;
    //do not change

    WebcamName theWebCam;
    //Assign the web cam used here.
    AprilTagProcessor tagProcessor;
    AprilTagProcessor.Builder myAprilTagProcessorBuilder;
    AprilTagGameDatabase aprilTagGameDatabase;
    private int AprilTagId;
    private int TargetId = 0;

    private double xDistance, yDistance, zDistance, range;

    private double pitch, roll, yaw;
    private double elevation, bearing;


    AprilTagDetection tag;


    //Telemetry telemetry = ;


    //public void setCamera(String wcn){
    //    webCamName =wcn;
    //}
    public enum Target {
        red,
        blue

    }

    public enum Pattern {
        GPP,
        PGP,
        PPG,
        none
    }

    public void aprilTagSetUp(WebcamName camera) {
        //telemetry.addLine("aprilTagSetUp");

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .build();
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(camera)  // uncomment once it exists
                .setCameraResolution(new Size(640, 480))
                .build();

    }

    public void scanForTarget() {
        //telemetry.addLine("scanny scan scan");
        if (!tagProcessor.getDetections().isEmpty()) {
            //telemetry.addLine("Not empty yo :)")

            tag = tagProcessor.getDetections().get(0);
            if (TargetId == (int) tag.id) {
                xDistance = tag.ftcPose.x * 2.54;
                yDistance = tag.ftcPose.y * 2.54;
                zDistance = tag.ftcPose.z * 2.54;
                range = tag.ftcPose.range * 2.54;

                pitch = tag.ftcPose.pitch;
                roll = tag.ftcPose.roll;
                yaw = tag.ftcPose.yaw;

                elevation = tag.ftcPose.elevation;
                bearing = tag.ftcPose.bearing;
            } else {

                xDistance = -1;
                yDistance = -1;
                zDistance = -1;
                range = -1;

                pitch = 0;
                roll = 0;
                yaw = 0;

                elevation = 0;
                bearing = 0;

            }


            /*if (devModeOn){
                telemetry.addData("xDistance",xDistance);
                telemetry.addData("yDistance",yDistance);
                telemetry.addData("zDistance",zDistance);
                telemetry.addData("range",range);

                telemetry.addData("pitch",pitch);
                telemetry.addData("xDistance",xDistance);
                telemetry.addData("xDistance",xDistance);
                telemetry.addData("xDistance",xDistance);
                telemetry.addData("xDistance",xDistance);
                telemetry.addData("horizontal Angle",horizAngle());
                telemetry.addData("center Distance",centerDistanceCM());
                telemetry.addData("Vertical angle ",vertAngle());
                telemetry.update();

            }*/

        } else {
            xDistance = -1;
            yDistance = -1;
            zDistance = -1;
            range = -1;

            pitch = 0;
            roll = 0;
            yaw = 0;

            elevation = 0;
            bearing = 0;
            AprilTagId = -1;

        }
    }

    public double centerDistanceCM() {

        scanForTarget();
        double centerDistance;
        if (range == -1.0) {
            centerDistance = -1;
        } else {
            centerDistance = Math.sqrt(TAG_TO_CENTER * TAG_TO_CENTER + range * range - 2 * TAG_TO_CENTER * range * Math.cos(Math.toRadians(180 - yaw)));
        }

        return centerDistance;
    }

    public double horizAngle() {
        scanForTarget();
        double horizAngle;
        if (range == -1) {
            horizAngle = -1;
        } else if (yaw >= 0) {

            horizAngle = Math.toDegrees(Math.asin(TAG_TO_CENTER * Math.sin(Math.toRadians(180 - yaw)) / centerDistanceCM()));
            //distance = the distance of target from base of robot
        } else {
            horizAngle = Math.toDegrees(Math.asin(TAG_TO_CENTER * Math.sin(Math.toRadians(180 + yaw)) / centerDistanceCM()));
        }
        return horizAngle;
    }

    public double vertAngle() {
        double angle;
        scanForTarget();
        if (range == -1) {
            angle = -1;
        } else {
            angle = Math.toDegrees(Math.atan((zDistance + TAG_TO_TOP) / range)); //distance = the distance of target from base of robot
        }
        return angle;
    }


    public void setTarget(Target target) {
        if (target == Target.red) {
            TargetId = 24;
        }
        if (target == Target.blue) {
            TargetId = 20;

        }


    }

    /*static public void DevModeOn(){
        devModeOn= true;
    }*/
    //               ---------RETURN FUNCTIONS---------
    public double getyDistance() {
        scanForTarget();
        return yDistance;
    }

    public double getzDistance() {
        scanForTarget();
        return zDistance;
    }

    public double getxDistance() {
        scanForTarget();
        return xDistance;
    }

    public double getRange() {
        scanForTarget();
        return range;
    }

    public double getPitch() {
        scanForTarget();
        return pitch;
    }

    public double getRoll() {
        scanForTarget();
        return roll;
    }

    public double getYaw() {
        scanForTarget();
        return yaw;
    }

    public double getElevation() {
        scanForTarget();
        return elevation;
    }

    public double getBearing() {
        scanForTarget();
        return bearing;
    }

    public int getId() {
        return tag.id;
    }

    /*        xDistance=-1;
    yDistance=-1;
    zDistance=-1;
    range=-1;

    pitch=0;
    roll=0;
    yaw=0;

    elevation=0;
    bearing=0;
    AprilTagId = -1;*/


    String gamePattern = "none";
    int scanForPatternRun = 0;

    public String scanForPattern() {


        if (gamePattern.equals("none") && !tagProcessor.getDetections().isEmpty()) {
            //telemetry.addLine("Not empty yo :)")

            tag = tagProcessor.getDetections().get(0);
            if (23 == (int) tag.id) {
                gamePattern = "PPG";
                scanForPatternRun++;
            }
            if (22 == (int) tag.id) {
                gamePattern = "PGP";
                scanForPatternRun++;
            }
            if (21 == (int) tag.id) {
                gamePattern = "GPP";
                scanForPatternRun++;
            }

        }
        return gamePattern;

    }

    public double alignmentValue() {
        scanForTarget();
        if (range != -1) {
            double degrees_to_center = Math.toDegrees(Math.asin(TAG_TO_CENTER * Math.sin(180 - yaw) / centerDistanceCM()));
            double bearing = getBearing();
            return degrees_to_center - bearing;
        } return -10000;
    }



}