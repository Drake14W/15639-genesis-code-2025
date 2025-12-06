package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.HashMap;
import java.util.concurrent.TimeUnit;

@Autonomous(name="AutoRed1", group="Linear OpMode")
public class AutoRed1 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //Map of all motors
    private HashMap<String, DcMotor> motors = new HashMap<>();
    //Map of wheel motor powers
    private HashMap<String, Double> motor_powers = new HashMap<>();

    //Map of all servos
    private HashMap<String, Servo> servos = new HashMap<>();
    //Map of all servos positions
    private HashMap<String, Double> servo_positions = new HashMap<>();

    //Map of all CRServos
    private HashMap<String, CRServo> crservos = new HashMap<>();
    //Map of all CRServo powers
    private HashMap<String, Double> crservo_powers = new HashMap<>();

    //Used to describe movement
    private double axial, lateral, yaw;

    final double ROTATION_SPEED = 0.75;
    final double MAX_MOVEMENT_SPEED = 1.0;
    double movement_speed = MAX_MOVEMENT_SPEED;

    //Other assorted globals
    final double INTAKE_SPEED = 1.0;
    final double INTAKE_SERVO_SPEED = 1.0;
    double FLYWHEEL_SPEED = 1.0;
    double flicker_down_pos;
    double flicker_mid_pos;
    double flicker_up_pos;
    boolean telemetry_output = true;

    //RPM Calculations
    final double FLYWHEEL_CPR = 28;
    double last_flywheel_revs = 0;
    double flywheel_rpm;
    double last_rpm_time;

    //Camera and AprilTag stuff
    AprilTagProcessor tagProcessor;
    VisionPortal visionPortal;
    AprilTagDetection tag;
    double tag_range;
    double tag_elevation;
    double tag_bearing;
    final double BEARING_RANGE = 5*(Math.PI/180); //We accept +/- 3 degrees when aiming
    final double APRIL_TAG_ROTATION_SPEED = 0.1;
    final double APRIL_TAG_PERMITTED_DELAY = 0.2; //We allow 0.2 seconds of delay between detecting the april tag and acting upon it, otherwise we ignore it

    //Movement values for aimbot (april tag) macro
    double aimbot_macro_yaw;
    double aimbot_flywheel_power;
    double aimbot_intake_crservo_power;
    double aimbot_intake_power;
    boolean aimbot_shooting;

    //Calculation values for aimbot
    double exit_velocity;
    final double MAX_FLYWHEEL_RPM = 18000;
    final double GRAVITY = 9.81;
    final double BUCKET_HEIGHT = 98.45;
    final double CAMERA_HEIGHT = 32;
    final double CAMERA_X_OFFSET = 7; //To the left of the launcher is positive
    final double CAMERA_Z_OFFSET = 25; //Forward from the launcher is positive
    final double CAMERA_YAW = 0; //Left to right angle (positive is left)
    final double CAMERA_PITCH = 25; //Up-down angle (positive is up)
    final double CAMERA_ROLL = 0; //Up-down rotation angle (like this for positive: ↓---↑)
    final double EXIT_HEIGHT = 21;
    final double EXIT_ANGLE = Math.toRadians(50);
    final double BUCKET_WIDTH = 37; //In cm
    final double MAGIC_FLYWHEEL_NUMBER = 5.5;
    final double AIMBOT_CORRECTION = 3*(Math.PI/180);
    double flywheel_rpm_error;
    double aimbot_needed_flywheel_rpm;
    final double AIMBOT_PROPORTIONAL_COEFFIEICENT = 0.00005;
    final double AIMBOT_INTEGRAL_COEFFICIENT = 0.00001;
    final double AIMBOT_DERIVATIVE_COEFFICIENT = 0.0005;
    double rpm_error_integral_val;
    double rpm_error_deriv_val;
    double pid_time_delta;
    double pid_last_time;
    double rpm_previous_error;
    final double shooting_length = 3000; //Milliseconds
    final double aimbot_length = shooting_length*5; //Milliseconds
    final double FLICKING_LENGTH = 400; //Milliseconds
    double aimbot_starting_time;
    final double FLICKER_DOWN_POS = 0.25;
    final double FLICKER_MID_POS = 0.12;
    final double FLICKER_UP_POS = 0.0;
    int balls_shot;
    final double FLYWHEEL_RPM_ERROR_RANGE = 100;

    private void aimbot() {
        pid_last_time = runtime.milliseconds();
        rpm_error_integral_val = 0;
        rpm_previous_error = 0;
        aimbot_shooting = false;
        aimbot_starting_time = 0;
        balls_shot = 0;

        //Turn on image processor
        visionPortal.setProcessorEnabled(tagProcessor, true);

        servo_positions.put("flicker", FLICKER_DOWN_POS);

        while ((runtime.milliseconds() - aimbot_starting_time) < aimbot_length) {
            if (tagProcessor.getDetections().size() > 0) {
                try {
                    tag = tagProcessor.getDetections().get(0);
                }
                catch (Exception e){
                    continue;
                }
                //1000000000 converts from nanoseconds to seconds
                if ((((double)(System.nanoTime() - tag.frameAcquisitionNanoTime)) < APRIL_TAG_PERMITTED_DELAY*1000000000) && (tag.id == 24)) {
                    //Scan the tag
                    telemetry.addData("ID", tag.metadata.id);
                    tag_bearing = tag.ftcPose.bearing;
                    tag_elevation = tag.ftcPose.elevation;
                    tag_range = tag.ftcPose.range;

                    //Get the flywheel running and PID this motherfucker (XD alright)
                    //Calculate exit velocity we need
                    exit_velocity = (tag_range + BUCKET_WIDTH) * Math.sqrt((GRAVITY * 100) / (2 * Math.pow(Math.cos(EXIT_ANGLE), 2) * ((tag_range + BUCKET_WIDTH) * Math.tan(EXIT_ANGLE) - (BUCKET_HEIGHT - EXIT_HEIGHT))));

                    //Calculate flywheel motor power
                    //I'm not good enough at physics for this shit (Damn is this Jamie?)
                    aimbot_needed_flywheel_rpm = MAGIC_FLYWHEEL_NUMBER * exit_velocity;

                    flywheel_rpm_error = aimbot_needed_flywheel_rpm - flywheel_rpm;

                    pid_time_delta = runtime.milliseconds() - pid_last_time;
                    rpm_error_integral_val = flywheel_rpm_error*pid_time_delta;
                    rpm_error_deriv_val = (flywheel_rpm_error - rpm_previous_error) / pid_time_delta;
                    pid_last_time = runtime.milliseconds();
                    rpm_previous_error = flywheel_rpm_error;

                    aimbot_flywheel_power += AIMBOT_PROPORTIONAL_COEFFIEICENT * flywheel_rpm_error + AIMBOT_INTEGRAL_COEFFICIENT * rpm_error_integral_val + AIMBOT_DERIVATIVE_COEFFICIENT * rpm_error_deriv_val;

                    if (aimbot_flywheel_power < 0) {
                        aimbot_flywheel_power = 0;
                    }
                    if (aimbot_flywheel_power > 1) {
                        aimbot_flywheel_power = 1;
                    }

                    aimbot_intake_crservo_power = 0;
                    telemetry.addData("Wanted RPM", aimbot_needed_flywheel_rpm);
                    telemetry.addData("Exit Velocity", exit_velocity/100);
                    telemetry.addData("RPM Error", flywheel_rpm_error);
                    telemetry.addData("Flywheel Power", aimbot_flywheel_power);
                    //We're rotated too far left
                    if (tag_bearing < -(BEARING_RANGE+AIMBOT_CORRECTION)) {
                        telemetry.addLine("Too far left");
                        aimbot_macro_yaw = APRIL_TAG_ROTATION_SPEED;
                    }
                    //We're rotated too far right
                    else if (tag_bearing > (BEARING_RANGE-AIMBOT_CORRECTION)) {
                        telemetry.addLine("Too far right");
                        aimbot_macro_yaw = -APRIL_TAG_ROTATION_SPEED;
                    }
                    //Shoot
                    else if (flywheel_rpm_error < FLYWHEEL_RPM_ERROR_RANGE) {
                        telemetry.addLine("Shooting");
                        aimbot_macro_yaw = 0;
                        aimbot_shooting = true;
                        telemetry.addData("Exit Velocity", exit_velocity);
                        telemetry.addData("Flywheel Power", aimbot_flywheel_power);
                    }

                    if (aimbot_shooting) {
                        if (balls_shot < 3) {
                            aimbot_intake_crservo_power = INTAKE_SERVO_SPEED;
                            aimbot_intake_power = INTAKE_SPEED;
                            if ((runtime.milliseconds() - aimbot_starting_time) > (shooting_length*(balls_shot+1))) {
                                //Flick it
                                servo_positions.put("flicker", FLICKER_UP_POS);
                                balls_shot++;

                            }
                            else if ((runtime.milliseconds() - aimbot_starting_time) > (shooting_length*(balls_shot))+FLICKING_LENGTH) {
                                //Unflick it
                                servo_positions.put("flicker", FLICKER_DOWN_POS);
                            }
                        }
                    }
                    else {
                        aimbot_starting_time = runtime.milliseconds();
                        aimbot_intake_crservo_power = 0;
                        aimbot_intake_power = 0;
                    }


                }
                //GGs, we've fucked up
                else {
                    //Turn off image processor
                    visionPortal.setProcessorEnabled(tagProcessor, false);
                    
                    return;
                }
            }

            telemetry.addData("Balls Shot:", balls_shot);
            telemetry.addData("Shooting:", aimbot_shooting);
            telemetry.addData("Flywheel RPM Error:", flywheel_rpm_error);
            telemetry.update();

            //Execute aimbot macro
            motor_powers.put("front_left", aimbot_macro_yaw);
            motor_powers.put("front_right", -aimbot_macro_yaw);
            motor_powers.put("back_left", aimbot_macro_yaw);
            motor_powers.put("back_right", -aimbot_macro_yaw);

            motor_powers.put("flywheel", aimbot_flywheel_power);

            motor_powers.put("intake", aimbot_intake_power);
            crservo_powers.put("intake_servo1", aimbot_intake_crservo_power);
            crservo_powers.put("intake_servo2", aimbot_intake_crservo_power);

            //Execute powers
            for (String key : motor_powers.keySet()) {
                motors.get(key).setPower(motor_powers.get(key));
                //Reset motor power
                motor_powers.put(key, 0.0);
            }
            for (String key : crservos.keySet()) {
                crservos.get(key).setPower(crservo_powers.get(key));
                //Reset motor power
                crservo_powers.put(key, 0.0);
            }
            for (String key : servos.keySet()) {
                servos.get(key).setPosition(servo_positions.get(key));
            }

            flywheel_rpm = (motors.get("flywheel").getCurrentPosition()/FLYWHEEL_CPR - last_flywheel_revs) / (runtime.milliseconds() - last_rpm_time) * 60000;
            last_flywheel_revs = motors.get("flywheel").getCurrentPosition()/FLYWHEEL_CPR;
            last_rpm_time = runtime.milliseconds();
        }
        //Turn off image processor
        visionPortal.setProcessorEnabled(tagProcessor, false);
        flywheel_rpm = 0;
        servo_positions.put("flicker", FLICKER_UP_POS);

        motor_powers.put("intake", 0.0);
        crservo_powers.put("intake_servo1", 0.0);
        crservo_powers.put("intake_servo2", 0.0);

        for (String key : motor_powers.keySet()) {
            motors.get(key).setPower(motor_powers.get(key));
            //Reset motor power
            motor_powers.put(key, 0.0);
        }
    }

    @Override
    public void runOpMode() {
        //Create and assign map entries for all motors
        motors.put("front_left", hardwareMap.get(DcMotor.class, "front_left_motor"));
        motors.put("back_left", hardwareMap.get(DcMotor.class, "back_left_motor"));
        motors.put("front_right", hardwareMap.get(DcMotor.class, "front_right_motor"));
        motors.put("back_right", hardwareMap.get(DcMotor.class, "back_right_motor"));

        motors.put("intake", hardwareMap.get(DcMotor.class, "intake_motor"));

        motors.put("flywheel", hardwareMap.get(DcMotor.class, "flywheel_motor"));

        motors.put("lift_motor1", hardwareMap.get(DcMotor.class, "lift_motor1"));
        motors.put("lift_motor2", hardwareMap.get(DcMotor.class, "lift_motor2"));

        //Create and assign map entries for all servos
        servos.put("flicker", hardwareMap.get(Servo.class, "flicker"));

        //Create and assign map entries for all CRServos
        crservos.put("intake_servo1", hardwareMap.get(CRServo.class, "intake_servo1"));
        crservos.put("intake_servo2", hardwareMap.get(CRServo.class, "intake_servo2"));

        //Reset encoders
        for (String key : motors.keySet()) {
            motors.get(key).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            if (key != "flywheel") {
                motors.get(key).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            else {
                motors.get(key).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }

        //Set direction of motors
        motors.get("front_left").setDirection(DcMotor.Direction.REVERSE);
        motors.get("back_left").setDirection(DcMotor.Direction.REVERSE);
        motors.get("front_right").setDirection(DcMotor.Direction.FORWARD);
        motors.get("back_right").setDirection(DcMotor.Direction.FORWARD);

        motors.get("intake").setDirection(DcMotorSimple.Direction.REVERSE);

        motors.get("flywheel").setDirection(DcMotorSimple.Direction.REVERSE);

        motors.get("lift_motor1").setDirection(DcMotorSimple.Direction.REVERSE);
        motors.get("lift_motor2").setDirection(DcMotorSimple.Direction.REVERSE);

        //Set direction of servos
        crservos.get("intake_servo1").setDirection(DcMotorSimple.Direction.FORWARD);
        crservos.get("intake_servo2").setDirection(DcMotorSimple.Direction.REVERSE);

        //Turn on the brakes for 0 power
        for (String key : motors.keySet()) {
            if (key == "flywheel") {
                motors.get(key).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            else {
                motors.get(key).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
        }

        //Motor powers
        for (String key : motors.keySet()) {
            motor_powers.put(key, 0.0);
        }

        //Settings for servos
        flicker_down_pos = 0.25;
        flicker_mid_pos = 0.12;
        flicker_up_pos = 0.0;


        //CRServos Powers
        for (String key : crservos.keySet()) {
            crservo_powers.put(key, 0.0);
        }

        //Initialize camera and april tag processing stuff
        Position cam_position = new Position(DistanceUnit.CM, CAMERA_X_OFFSET, CAMERA_HEIGHT-EXIT_HEIGHT, CAMERA_Z_OFFSET, 0);
        YawPitchRollAngles yaw_pitch_roll_angles = new YawPitchRollAngles(AngleUnit.DEGREES, CAMERA_YAW, CAMERA_PITCH-90, CAMERA_ROLL, 0);
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setCameraPose(cam_position, yaw_pitch_roll_angles)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.RADIANS)
                //For some fucking reason the camera isn't getting autodetected so we have to do it manually
                .setLensIntrinsics(622.001f, 622.001f, 319.803f, 241.251)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                //declares the camera
                .setCamera(hardwareMap.get(CameraName.class, "c920"))
                //stream format
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                //sets camera resolution; turn down if performance issues
                .setCameraResolution(new Size(640, 480))
                //as above
                .build();

        //Disable processor to start (conserve bandwidth)
        visionPortal.setProcessorEnabled(tagProcessor, false);

        //Wait for camera to start up
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            sleep(10);
        }

        //Adjust camera settings
        ExposureControl exposure_control = visionPortal.getCameraControl(ExposureControl.class);
        if (exposure_control.getMode() != ExposureControl.Mode.Manual) {
            exposure_control.setMode(ExposureControl.Mode.Manual);
        }
        exposure_control.setExposure(exposure_control.getMinExposure(TimeUnit.MILLISECONDS) + 1, TimeUnit.MILLISECONDS);
        GainControl gain_control = visionPortal.getCameraControl(GainControl.class);
        gain_control.setGain(gain_control.getMaxGain());

        //Create the PID controller
        PID pid = new PID(motors.get("front_left"), motors.get("back_left"), motors.get("front_right"), motors.get("back_right"), telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Wait until the the start button is pressed on the driver hub
        waitForStart();

        last_rpm_time = runtime.milliseconds();

        //Reset runtime var
        runtime.reset();

        //Move off wall
        pid.move(10);

        //Rotate to face goal
        //pid.rotate(-30);
        motors.get("front_left").setPower(0.25);
        motors.get("back_left").setPower(0.25);
        motors.get("front_right").setPower(-0.25);
        motors.get("back_right").setPower(-0.25);
        sleep(600);
        motors.get("front_left").setPower(0);
        motors.get("back_left").setPower(0);
        motors.get("front_right").setPower(0);
        motors.get("back_right").setPower(0);
        aimbot_flywheel_power = 0;
        aimbot();

        //Move off line
        pid.move(15);
    }
}
