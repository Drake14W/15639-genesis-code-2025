package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
public class AprilTagTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //All of the code below should probably be imported into the MainTeleOp and code built around it if used in mainphase

        //Initializes the AprilTag software
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                //shows scanned tag's ID
                .setDrawTagID(true)

                .setDrawAxes(true)

                .setDrawCubeProjection(true)


                //This line ends the Apriltag stuff; methods go between this and the first line
                .build();

        //initializes the VisionPortal software
        VisionPortal visionPortal = new VisionPortal.Builder()
                //Associates AprilTagProcessor with VisionPortal
                .addProcessor(tagProcessor)

                //declares the camera
                .setCamera(hardwareMap.get(CameraName.class, "Webcam 1"))

                //stream format
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)

                //start live stream
                .enableLiveView(true)

                //sets camera resolution; turn down if performance issues
                .setCameraResolution(new Size(1280, 720))

                //as above
                .build();

        waitForStart();


        while (!isStopRequested() && opModeIsActive()) {

            if (tagProcessor.getDetections().size() > 0) {

                AprilTagDetection tag = tagProcessor.getDetections().get(0);

                telemetry.addData("x", tag.ftcPose.x);
                telemetry.addData("y", tag.ftcPose.y);
                telemetry.addData("z", tag.ftcPose.z);
                telemetry.addData("yaw", tag.ftcPose.yaw);
                telemetry.addData("bearing", tag.ftcPose.bearing);
                telemetry.addData("roll", tag.ftcPose.roll);
                telemetry.addData("pitch", tag.ftcPose.pitch);
                telemetry.addData("range", tag.ftcPose.range);
                telemetry.addData("elevation", tag.ftcPose.elevation);

                telemetry.update();
            }



        }

    }

}
