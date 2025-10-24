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
 *       setCamera(cameraName) sets the camera that will be used.
 *       aprilTagSetup() sets up the april tag with the library needed
 *       centerDistanceReturn() returns how far away the goal is.
 *       horizAngleReturn() returns the angle left and right the goal is relative the the camera.
 *       vertAngleReturn() returns the angle to the top of the goal.
 *       Vision.DevMode() turns showing all distances and readings in telemetry
 *
 *
 *
 */
public class Vision{

   // public Vision(HardwareMap map) {
   //     this.hardwareMap = map;
    //}

    //CHANGE STUFF HERE
    static final double TAG_TO_CENTER = 8.75; //put the horizontal distance from tag to center of goal here
    static final double TAG_TO_TOP= 23.5; //put the vertical distance from center of tag to top of goal here

    static boolean devModeOn=false; //change default mode of devMode here. Can be changed in code using Vision.toggleDevMode();

    //String webCamName="Camera1";//put the webCamName here.
    //HardwareMap hardwareMap;
    //do not change

    WebcamName theWebCam;
    //Assign the web cam used here.
    AprilTagProcessor  tagProcessor;
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
    public enum Target{
        red,
        blue

    }
    public enum Pattern {
        GPP,
        PGP,
        PPG,
        none
    }

    public void aprilTagSetUp(WebcamName camera){
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
    public void scanForTarget(){
        //telemetry.addLine("scanny scan scan");
        if (!tagProcessor.getDetections().isEmpty()){
            //telemetry.addLine("Not empty yo :)")

                tag = tagProcessor.getDetections().get(0);
                if(TargetId==(int)tag.id) {
                    xDistance = tag.ftcPose.x;
                    yDistance = tag.ftcPose.y;
                    zDistance = tag.ftcPose.z;
                    range = tag.ftcPose.range;

                    pitch = tag.ftcPose.pitch;
                    roll = tag.ftcPose.roll;
                    yaw = tag.ftcPose.yaw;

                    elevation = tag.ftcPose.elevation;
                    bearing = tag.ftcPose.bearing;
                }
                else{

                    xDistance = -1;
                    yDistance = -1;
                    zDistance = -1;
                    range = -1;

                    pitch = 0;
                    roll = 0 ;
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
                telemetry.addData("horizontal Angle",horizAngleReturn());
                telemetry.addData("center Distance",centerDistanceReturn());
                telemetry.addData("Vertical angle ",vertAngleReturn());
                telemetry.update();

            }*/

        }
        else{
            xDistance=-1;
            yDistance=-1;
            zDistance=-1;
            range=-1;

            pitch=0;
            roll=0;
            yaw=0;

            elevation=0;
            bearing=0;
            AprilTagId = -1;

        }
    }
    public double centerDistanceReturn(){

        scanForTarget();
        double centerDistance;
        if (range==-1.0){
            centerDistance=-1;
        }
        else {
            centerDistance = Math.sqrt(TAG_TO_CENTER * TAG_TO_CENTER + range * range - 2 * TAG_TO_CENTER * TAG_TO_CENTER * Math.cos(Math.toRadians(180 - yaw)));
        }

        return centerDistance;
    }

    public double horizAngleReturn(){
        scanForTarget();
        double horizAngle;
        if(range==-1) {
            horizAngle=-100.0;
        }
        else{
            horizAngle = Math.asin(TAG_TO_CENTER*Math.sin(Math.toRadians(180-yaw))/centerDistanceReturn());
            //distance = the distance of target from base of robot

        }
        return horizAngle;
    }
    public double vertAngleReturn(){
        double angle;
        angle = Math.tan((zDistance+TAG_TO_TOP)/range); //distance = the distance of target from base of robot
        return angle;
    }


    public void setTarget(Target target){
        if(target == Target.red){
            TargetId=24;
        }
        if(target == Target.blue){
            TargetId=20;

        }












    }
    static public void DevModeOn(){
        devModeOn= true;
    }
    //               ---------RETURN FUNCTIONS---------
    public double getyDistance(){
        return yDistance;
    }
    public double getzDistance(){
        return zDistance;
    }
    public double getxDistance(){
        return xDistance;
    }
    public double getRange(){
        return range;
    }
    public double getPitch(){
        return pitch;
    }
    public double getRoll(){
        return roll;
    }
    public double getYaw(){
        return yaw;
    }
    public double getElevation(){
        return yDistance;
    }
    public double getBearing(){
        return bearing;
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



    Pattern gamePattern = Pattern.none;
    int scanForPatternRun = 0;
    public Vision.Pattern scanForPattern() {


        if (!tagProcessor.getDetections().isEmpty()) {
            //telemetry.addLine("Not empty yo :)")

            tag = tagProcessor.getDetections().get(0);
            if (23 == (int) tag.id) {
                gamePattern= Pattern.GPP;
                scanForPatternRun++;
            }
            if (22 == (int) tag.id) {
                gamePattern = Pattern.PGP;
                scanForPatternRun++;
            }
            if (21 == (int) tag.id) {
                gamePattern = Pattern.PPG;
                scanForPatternRun++;
            }

        }
        return gamePattern;
    }
}