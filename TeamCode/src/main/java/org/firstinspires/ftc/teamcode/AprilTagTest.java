package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.CameraControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

@TeleOp
public class AprilTagTest extends LinearOpMode {

    final double CAMERA_HEIGHT = 32;
    final double CAMERA_X_OFFSET = 7; //To the left of the launcher is positive
    final double CAMERA_Z_OFFSET = 25; //Forward from the launcher is positive
    final double CAMERA_YAW = 0; //Left to right angle (positive is left)
    final double CAMERA_PITCH = 25; //Up-down angle (positive is up)
    final double CAMERA_ROLL = 0; //Up-down rotation angle (like this for positive: ↓---↑)
    final double EXIT_HEIGHT = 21;

    @Override
    public void runOpMode() throws InterruptedException {
        //All of the code below should probably be imported into the MainTeleOp and code built around it if used in mainphase

        Position cam_position = new Position(DistanceUnit.CM, CAMERA_X_OFFSET, CAMERA_HEIGHT-EXIT_HEIGHT, CAMERA_Z_OFFSET, 0);
        YawPitchRollAngles yaw_pitch_roll_angles = new YawPitchRollAngles(AngleUnit.DEGREES, CAMERA_YAW, CAMERA_PITCH-90, CAMERA_ROLL, 0);
        //Initializes the AprilTag software
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                //shows scanned tag's ID
                .setDrawTagID(true)

                .setDrawAxes(true)

                .setDrawCubeProjection(true)
                .setCameraPose(cam_position, yaw_pitch_roll_angles)

                .setLensIntrinsics(622.001f, 622.001f, 319.803f, 241.251)

                //This line ends the Apriltag stuff; methods go between this and the first line
                .build();

        //initializes the VisionPortal software
        VisionPortal visionPortal = new VisionPortal.Builder()
                //Associates AprilTagProcessor with VisionPortal
                .addProcessor(tagProcessor)

                //declares the camera
                .setCamera(hardwareMap.get(CameraName.class, "c920"))

                //stream format
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)

                //start live stream
                .enableLiveView(true)

                //sets camera resolution; turn down if performance issues
                .setCameraResolution(new Size(640, 480))

                //as above
                .build();

        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            sleep(10);
        }

        telemetry.addLine("Camera streaming");
        telemetry.update();

        ExposureControl exposure_control = visionPortal.getCameraControl(ExposureControl.class);
        exposure_control.setExposure(exposure_control.getMinExposure(TimeUnit.MILLISECONDS) + 1, TimeUnit.MILLISECONDS);
        GainControl gain_control = visionPortal.getCameraControl(GainControl.class);
        gain_control.setGain(gain_control.getMaxGain());

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
