/*
MIT License

Copyright (c) 2025 ChessMan14, angeldescended

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
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
import java.lang.Math;
import java.util.concurrent.TimeUnit;

//This decorator puts this opmode into selected the name and group on the driver hub menu
@TeleOp(name="AprilTagTeleOp", group="Linear OpMode")
//Since java is weird, this is essentially the equivalent of a main method in C, but instead it's a class. Also, we "extend" this class from the library class LinearOpMode which makes this into a proper teleop opmode
public class AprilTagTeleOp extends LinearOpMode {
    //Create the variables for the motors and servos and initializes a variable that keeps track of how long the opmode has been running
    private ElapsedTime runtime = new ElapsedTime();

    //Map of all motors
    private HashMap<String, DcMotor> motors = new HashMap<>();
    //Map of wheel motor powers]
    private HashMap<String, Double> motor_powers = new HashMap<>();

    //Map of all servos
    private HashMap<String, Servo> servos = new HashMap<>();
    //Map of all servos positions
    private HashMap<String, Double> servo_positions = new HashMap<>();

    //Map of all CRServos
    private HashMap<String, CRServo> crservos = new HashMap<>();
    //Map of all CRServo powers
    private HashMap<String, Double> crservo_powers = new HashMap<>();

    //Action map
    //The structure for the action map works like this:
    //Each entry represents an action like moving the robot manually or a macro
    //Each bit in byte portion of the map entry is used to represent a part of the robot or some other "level" that the action needs to itself
    //E.g    0 0 0 1 0 0 0
    //             ^
    //             |
    //This bit might represent the arm and this byte is saying this macro needs the intake wheel.
    //Since it is set to 1, this action needs the intake wheel all to itself.
    //The last bit represents if the action is active.

    //The levels:
    //Level 0 (lsb):    General wheel control
    //Level 1:          Rotation control
    //Level 2:          Intake control
    //Level 3:          Flywheel control
    //Level 4:          Intake servo control
    //Level 5:          Lorem ipsum
    //Level 6:          Lorem ipsum
    //Level 7:          On/Off
    private HashMap<String, Byte> action_map = new HashMap<>();

    //Custom gamepads (see CustomGamepad.java)
    private CustomGamepad custom_gamepad_1, custom_gamepad_2;

    //Used to describe movement
    private double axial, lateral, trigger_yaw, stick_yaw, yaw;

    //Globals for movement
    double LONG_LAT_RADIAL_DEADZONE = 0.0;  //Decimal from 0-1
    double LONG_LAT_ANGULAR_DEADZONE = 2;   //In degrees
    double ROTATION_RADIAL_DEADZONE = 0.0;  //Decimal from 0-1
    double ROTATION_ANGULAR_DEADZONE = 0.0; //In degrees
    double ROTATION_SPEED = 0.75;
    double MOVEMENT_SPEED = 1.0;

    //Other assorted globals
    double INTAKE_SPEED = 1.0;
    double INTAKE_ANGULAR_DEADZONE = 0.0;
    double INTAKE_RADIAL_DEADZONE = 0.05;
    double INTAKE_SERVO_SPEED = 1.0;
    double FLYWHEEL_SPEED = 0.25;
    double FLYWHEEL_ANGULAR_DEADZONE = 0.0;
    double FLYWHEEL_RADIAL_DEADZONE = 0.20;

    //Used to set bytes to "on"
    byte ON_BITMASK = (byte) 0b10000000;

    private boolean check_mask(String key) {
        byte bit_mask_result;
        for (String iter_key : action_map.keySet()) {
            if (iter_key != key) {
                if (action_map.get(iter_key) < 0) {
                    bit_mask_result = (byte) (action_map.get(iter_key) & action_map.get(iter_key));
                    if ((bit_mask_result == 0) || (bit_mask_result == -127)) {
                        return false;
                    }
                }
            }
        }

        return true;
    }

    //Camera and AprilTag stuff
    AprilTagProcessor tagProcessor;
    VisionPortal visionPortal;
    AprilTagDetection tag;
    double tag_range;
    double tag_elevation;
    double tag_bearing;
    final double BEARING_RANGE = 3*(Math.PI/180); //We accept +/- 3 degrees when aiming
    final double APRIL_TAG_ROTATION_SPEED = 0.1;

    //Movement values for aimbot (april tag) macro
    double aimbot_macro_yaw;
    double aimbot_flywheel_power;
    double aimbot_flywheel_time_finished;
    final double AIMBOT_FLYWHEEL_RUN_TIME = 1000; //In ms

    //Calculation values for aimbot
    double exit_velocity;
    final double MAX_FLYWHEEL_RPM = 18000;
    final double GRAVITY = 9.81;
    final double BUCKET_HEIGHT = 98.45;
    final double CAMERA_HEIGHT = 25;
    final double CAMERA_X_OFFSET = 15; //To the left of the launcher is positive
    final double CAMERA_Z_OFFSET = 32; //Forward from the launcher is positive
    final double CAMERA_YAW = 0; //Left to right angle (positive is left)
    final double CAMERA_PITCH = 22; //Up-down angle (positive is up)
    final double CAMERA_ROLL = 0; //Up-down rotation angle (like this for positive: ↓---↑)
    final double EXIT_HEIGHT = 25;
    final double EXIT_ANGLE = Math.toRadians(50);
    final double BALL_RADIUS = 6.25;
    final double BUCKET_WIDTH = 37; //In cm
    final double MAGIC_FLYWHEEL_NUMBER = 90;

    //We have to override this function since it has already been defined in the parent class LinearOpMode
    @Override
    public void runOpMode() {
        //Add actions to hashmap
        action_map.put("manual_movement", (byte) 0b00000001);
        action_map.put("trigger_rotation", (byte) 0b00000010);
        action_map.put("stick_rotation", (byte) 0b00000010);
        action_map.put("manual_intake", (byte) 0b00000100);
        action_map.put("manual_flywheel", (byte) 0b00001000);
        action_map.put("manual_intake_servo", (byte) 0b00010000);

        //Create and assign map entries for all motors
        motors.put("front_left", hardwareMap.get(DcMotor.class, "front_left_motor"));
        motors.put("back_left", hardwareMap.get(DcMotor.class, "back_left_motor"));
        motors.put("front_right", hardwareMap.get(DcMotor.class, "front_right_motor"));
        motors.put("back_right", hardwareMap.get(DcMotor.class, "back_right_motor"));

        motors.put("intake", hardwareMap.get(DcMotor.class, "intake_motor"));

        motors.put("flywheel", hardwareMap.get(DcMotor.class, "flywheel_motor1"));

        //Create and assign map entries for all servos

        //Create and assign map entries for all CRServos
        crservos.put("intake_servo", hardwareMap.get(CRServo.class, "intake_servo"));

        //Reset encoders
        for (String key : motors.keySet()) {
            motors.get(key).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors.get(key).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        //Set direction of motors
        motors.get("front_left").setDirection(DcMotor.Direction.REVERSE);
        motors.get("back_left").setDirection(DcMotor.Direction.REVERSE);
        motors.get("front_right").setDirection(DcMotor.Direction.FORWARD);
        motors.get("back_right").setDirection(DcMotor.Direction.FORWARD);

        motors.get("intake").setDirection(DcMotorSimple.Direction.REVERSE);

        motors.get("flywheel").setDirection(DcMotorSimple.Direction.REVERSE);

        //Set direction of servos
        crservos.get("intake_servo").setDirection(DcMotorSimple.Direction.FORWARD);

        //Initialize custom gamepads
        custom_gamepad_1 = new CustomGamepad(gamepad1);
        custom_gamepad_2 = new CustomGamepad(gamepad2);

        //Turn on the brakes for 0 power
        for (String key : motors.keySet()) {
            motors.get(key).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        //Motor powers
        motor_powers.put("front_left", 0.0);
        motor_powers.put("front_right", 0.0);
        motor_powers.put("back_left", 0.0);
        motor_powers.put("back_right", 0.0);

        motor_powers.put("intake", 0.0);
        motor_powers.put("flywheel", 0.0);

        //Settings for servos

        //CRServos Powers
        crservo_powers.put("intake_servo", 0.0);

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
        exposure_control.setExposure(exposure_control.getMinExposure(TimeUnit.MILLISECONDS) + 1, TimeUnit.MILLISECONDS);
        GainControl gain_control = visionPortal.getCameraControl(GainControl.class);
        gain_control.setGain(gain_control.getMaxGain());

        //Macro action map entries
        action_map.put("aimbot", (byte) 0b00001001);

        //Funny Comment
        //This data is displayed on the driver hub console
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Wait until the the start button is pressed on the driver hub
        waitForStart();

        //Reset runtime var
        runtime.reset();

        //Main loop. This runs until stop is pressed on the driver hub
        while (opModeIsActive()) {
            //Update gamepad input
            custom_gamepad_1.update();
            custom_gamepad_2.update();

            //Manual movement controls
            //We use gamepad1 for actually moving the robot and gamepad2 for everything else

            //Controls latitudinal and longitudinal movement with left stick
            //Vertical movement (up is negative on the joystick)
            axial = custom_gamepad_1.get_left_stick_y(LONG_LAT_ANGULAR_DEADZONE, LONG_LAT_RADIAL_DEADZONE);
            //Horizontal movement
            lateral = custom_gamepad_1.get_left_stick_x(LONG_LAT_ANGULAR_DEADZONE, LONG_LAT_RADIAL_DEADZONE);
            //Rotation calculation
            trigger_yaw = ROTATION_SPEED * (-custom_gamepad_1.get_left_trigger() + custom_gamepad_1.get_right_trigger());
            stick_yaw = ROTATION_SPEED * (custom_gamepad_1.get_right_stick_x(ROTATION_ANGULAR_DEADZONE, ROTATION_RADIAL_DEADZONE));

            //Update manual movement state
            if ((Math.abs(axial) + Math.abs(lateral) + Math.abs(trigger_yaw) + Math.abs(stick_yaw)) > 0) {
                if (check_mask("manual_movement")) {
                    //Set the manual movement byte to on
                    action_map.put("manual_movement", (byte) (action_map.get("manual_movement") | ON_BITMASK));
                    if (Math.abs(trigger_yaw) > 0) {
                        //Set the trigger movement byte to on
                        action_map.put("trigger_rotation", (byte) (action_map.get("trigger_rotation") | ON_BITMASK));
                        action_map.put("stick_rotation", (byte) (action_map.get("stick_rotation") & (~ON_BITMASK)));
                    } else if (Math.abs(stick_yaw) > 0) {
                        //Set the trigger movement byte to on
                        action_map.put("stick_rotation", (byte) (action_map.get("stick_rotation") | ON_BITMASK));
                        action_map.put("trigger_rotation", (byte) (action_map.get("trigger_rotation") & (~ON_BITMASK)));
                    }
                }
            }
            else {
                action_map.put("manual_movement", (byte) (action_map.get("manual_movement") & (~ON_BITMASK)));
            }

            //Manual intake control
            if (check_mask("manual_intake") && (Math.abs(custom_gamepad_2.get_right_stick_y(INTAKE_ANGULAR_DEADZONE, INTAKE_RADIAL_DEADZONE)) > 0)) {
                action_map.put("manual_intake", (byte) (action_map.get("manual_intake") | ON_BITMASK));
                motor_powers.put("intake", INTAKE_SPEED * custom_gamepad_2.get_right_stick_y(INTAKE_ANGULAR_DEADZONE, INTAKE_RADIAL_DEADZONE));
            }
            else {
                action_map.put("manual_intake", (byte) (action_map.get("manual_intake") & (~ON_BITMASK)));
            }

            //Manual flywheel control
            if (check_mask("manual_flywheel") && (Math.abs(custom_gamepad_2.get_left_stick_y(INTAKE_ANGULAR_DEADZONE, INTAKE_RADIAL_DEADZONE)) > 0)) {
                action_map.put("manual_flywheel", (byte) (action_map.get("manual_flywheel") | ON_BITMASK));
                motor_powers.put("flywheel", FLYWHEEL_SPEED * custom_gamepad_2.get_left_stick_y(FLYWHEEL_ANGULAR_DEADZONE, FLYWHEEL_RADIAL_DEADZONE));
            }
            else {
                action_map.put("manual_flywheel", (byte) (action_map.get("manual_flywheel") & (~ON_BITMASK)));
            }

            //Manual intake servo control
            if (check_mask("manual_intake_servo") && (custom_gamepad_2.get_right_trigger() > 0)) {
                action_map.put("manual_intake_servo", (byte) (action_map.get("manual_intake_servo") | ON_BITMASK));
                crservo_powers.put("intake_servo", INTAKE_SERVO_SPEED * custom_gamepad_2.get_right_trigger());
            }
            else {
                action_map.put("manual_intake_servo", (byte) (action_map.get("manual_intake_servo") & (~ON_BITMASK)));
            }

            //Check for aimbot macro
            if ((custom_gamepad_2.get_dpad_up() || (action_map.get("aimbot") < 0)) && check_mask("aimbot")) {
                //If this is fresh
                if (action_map.get("aimbot") > 0) {
                    action_map.put("aimbot", (byte) (action_map.get("aimbot") | ON_BITMASK));
                    aimbot_flywheel_time_finished = runtime.milliseconds() + AIMBOT_FLYWHEEL_RUN_TIME;

                    //Turn on image processor
                    visionPortal.setProcessorEnabled(tagProcessor, true);
                }

                //Check if we're done
                if (runtime.milliseconds() > aimbot_flywheel_time_finished) {
                    action_map.put("aimbot", (byte) (action_map.get("aimbot") & (~ON_BITMASK)));

                    //Turn off image processor
                    visionPortal.setProcessorEnabled(tagProcessor, false);
                }
                else {
                    if (tagProcessor.getDetections().size() > 0) {
                        //Scan the tag
                        AprilTagDetection tag = tagProcessor.getDetections().get(0);
                        telemetry.addData("ID", tag.metadata.id);
                        tag_bearing = tag.ftcPose.bearing;
                        tag_elevation = tag.ftcPose.elevation;
                        tag_range = tag.ftcPose.range;

                        //We're rotated too far left
                        if (tag_bearing < -BEARING_RANGE) {
                            telemetry.addLine("Too far left");
                            aimbot_macro_yaw = APRIL_TAG_ROTATION_SPEED;
                            aimbot_flywheel_time_finished = runtime.milliseconds() + AIMBOT_FLYWHEEL_RUN_TIME;
                            aimbot_flywheel_power = 0;
                        }
                        //We're rotated too far right
                        else if (tag_bearing > BEARING_RANGE) {
                            telemetry.addLine("Too far right");
                            aimbot_macro_yaw = -APRIL_TAG_ROTATION_SPEED;
                            aimbot_flywheel_time_finished = runtime.milliseconds() + AIMBOT_FLYWHEEL_RUN_TIME;
                            aimbot_flywheel_power = 0;
                        }
                        //Shoot
                        else {
                            telemetry.addLine("Shooting");
                            aimbot_macro_yaw = 0;

                            //Calculate exit velocity we need
                            exit_velocity = (tag_range + BUCKET_WIDTH)* Math.sqrt((GRAVITY*100)/(2 * Math.pow(Math.cos(EXIT_ANGLE), 2) * ((tag_range + BUCKET_WIDTH) * Math.tan(EXIT_ANGLE) - (BUCKET_HEIGHT-EXIT_HEIGHT))));

                            //Calculate flywheel motor power
                            //I'm not good enough at physics for this shit
                            aimbot_flywheel_power = MAGIC_FLYWHEEL_NUMBER*exit_velocity;

                            telemetry.addData("Exit Velocity", exit_velocity);
                            telemetry.addData("Flywheel Power", aimbot_flywheel_power);
                            telemetry.addData("Flywheel Time Remaining", aimbot_flywheel_time_finished - runtime.milliseconds());
                        }
                    }
                }
            }
            if (custom_gamepad_2.get_dpad_down()) {
                //Turn off macro
                action_map.put("aimbot", (byte) (action_map.get("aimbot") & (~ON_BITMASK)));

                //Turn off image processor
                visionPortal.setProcessorEnabled(tagProcessor, false);
            }

            //Execute state actions
            //This checks if msb is set
            if (action_map.get("manual_movement") < 0) {
                //Check which yaw we're using (if any)
                if (action_map.get("trigger_rotation") < 0) {
                    yaw = trigger_yaw;
                }
                else if (action_map.get("stick_rotation") < 0) {
                    yaw = stick_yaw;
                }
                else {
                    yaw = 0;
                }

                //Set engine powers
                motor_powers.put("front_left", (axial + lateral + yaw)*MOVEMENT_SPEED);
                motor_powers.put("front_right", (axial - lateral - yaw)*MOVEMENT_SPEED);
                motor_powers.put("back_left", (axial - lateral + yaw)*MOVEMENT_SPEED);
                motor_powers.put("back_right", (axial + lateral - yaw)*MOVEMENT_SPEED);
            }

            //Execute aimbot macro
            if (action_map.get("aimbot") < 0) {
                motor_powers.put("front_left", aimbot_macro_yaw);
                motor_powers.put("front_right", -aimbot_macro_yaw);
                motor_powers.put("back_left", aimbot_macro_yaw);
                motor_powers.put("back_right", -aimbot_macro_yaw);

                motor_powers.put("flywheel1", aimbot_flywheel_power);
                motor_powers.put("flywheel2", aimbot_flywheel_power);
            }

            //Execute powers
            for (String key : motor_powers.keySet()) {
                motors.get(key).setPower(motor_powers.get(key));
                telemetry.addData(key, motor_powers.get(key));
                //Reset motor power
                motor_powers.put(key, 0.0);
            }
            for (String key : crservos.keySet()) {
                crservos.get(key).setPower(crservo_powers.get(key));
                telemetry.addData(key, crservo_powers.get(key));
                //Reset motor power
                crservo_powers.put(key, 0.0);
            }
            telemetry.update();
        }
    }
}