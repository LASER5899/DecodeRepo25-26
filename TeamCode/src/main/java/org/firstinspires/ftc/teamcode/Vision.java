package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.math.*;


/*
 *      This class is made to give the position of the goal relative to the robot.
 *
 *       setCamera() sets the camera that will be used.
 *       aprilTagSetup() sets up the april tag with the library needed
 *       centerDistanceReturn() returns how far away the goal is.
 *       horizAngleReturn() returns the angle left and right the goal is relative the the camera.
 *       vertAngleReturn() returns the angle to the top of the goal.
 *       Vision.toggleDevMode() turns on and off showing all distances and readings in telemetry
 *
 *
 *
 */
public class Vision{


    //CHANGE STUFF HERE
    static final double TAG_TO_CENTER = 8.75; //put the horizontal distance from tag to center of goal here
    static final double TAG_TO_TOP= 23.5; //put the vertical distance from center of tag to top of goal here

    static boolean devModeOn=true; //change default mode of devMode here. Can be changed in code using Vision.toggleDevMode();

    String webCamName="webCam";//put the webCamName here.

    //do not change

    WebcamName theWebCam;
    //Assign the web cam used here.
    AprilTagProcessor  tagProcessor;
    AprilTagProcessor.Builder myAprilTagProcessorBuilder;
    AprilTagGameDatabase aprilTagGameDatabase;
    int AprilTagId;
    int TargetId = 0;

    double xDistance, yDistance, zDistance, range;

    double pitch, roll, yaw;
    double elevation, bearing;
    AprilTagDetection tag;





    public void setCamera(String wcn){
        webCamName =wcn;
    }
    public enum Target{
        red,
        blue

    }

    public void aprilTagSetUp(){


        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class,webCamName))  // uncomment once it exists
                .setCameraResolution(new Size(640, 480))
                .build();

    }
    public void scanForTarget(){
        if (tagProcessor.getDetections().size()>0){
            if (TargetId == 0 ){
                telemetry.addLine("You need to set target  target.red or target.blue using the Vision.setTarget() function before attempting to read the target distances");
            }
            else if(TargetId==(int)tag.id) {
                tag = tagProcessor.getDetections().get(0);
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
            if (devModeOn){
                telemetry.addData("xDistance",xDistance);
                telemetry.addData("yDistance",yDistance);
                telemetry.addData("zDistance",zDistance);
                telemetry.addData("range",range);

                telemetry.addData("pitch",pitch);
                telemetry.addData("xDistance",xDistance);
                telemetry.addData("xDistance",xDistance);
                telemetry.addData("xDistance",xDistance);
                telemetry.addData("xDistance",xDistance);
            }

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
        angle = (zDistance+TAG_TO_TOP)/range; //distance = the distance of target from base of robot
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
    static public void toggleDevModeOn(){
        devModeOn= !devModeOn;
    }
}
