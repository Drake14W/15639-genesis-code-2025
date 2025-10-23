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

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.HashMap;
import java.lang.Math;

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
    //Level 4:          Lorem ipsum
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
    double MOVEMENT_SPEED = 0.5;

    //Other assorted globals
    double INTAKE_SPEED = 1.0;
    double INTAKE_ANGULAR_DEADZONE = 0.0;
    double INTAKE_RADIAL_DEADZONE = 0.05;
    double FLYWHEEL_SPEED = 1.0;
    double FLYWHEEL_ANGULAR_DEADZONE = 0.0;
    double FLYWHEEL_RADIAL_DEADZONE = 0.05;

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
    final double BEARING_RANGE = 3; //We accept +/- 3 degrees when aiming
    final double APRIL_TAG_ROTATION_SPEED = 0.35;

    //Movement values for aimbot (april tag) macro
    double aimbot_macro_yaw;
    double aimbot_flywheel_power;
    double aimbot_flywheel_time_finished;
    final double AIMBOT_FLYWHEEL_RUN_TIME = 1000; //In ms

    //Calculation values for aimbot
    double exit_velocity;
    final double MAX_FLYWHEEL_RPM = 6000;
    final double FLYWHEEL_RADIUS = 4; //In cm
    final double GRAVITY = 9.81;
    final double RAMP_HEIGHT = 25; //In cm
    final double BUCKET_HEIGHT = 98.45;
    final double CAMERA_HEIGHT = 45;
    final double EXIT_HEIGHT = 45;
    final double BUCKET_WIDTH = 50; //In cm
    final double INCHES_TO_CM = 2.54;

    //We have to override this function since it has already been defined in the parent class LinearOpMode
    @Override
    public void runOpMode() {
        //Add actions to hashmap
        action_map.put("manual_movement", (byte) 0b00000001);
        action_map.put("trigger_rotation", (byte) 0b00000010);
        action_map.put("stick_rotation", (byte) 0b00000010);
        action_map.put("manual_intake", (byte) 0b00000100);
        action_map.put("manual_flywheel", (byte) 0b00001000);

        //Create and assign map entries for all motors
        motors.put("front_left", hardwareMap.get(DcMotor.class, "front_left_motor"));
        motors.put("back_left", hardwareMap.get(DcMotor.class, "back_left_motor"));
        motors.put("front_right", hardwareMap.get(DcMotor.class, "front_right_motor"));
        motors.put("back_right", hardwareMap.get(DcMotor.class, "back_right_motor"));

        motors.put("intake", hardwareMap.get(DcMotor.class, "intake_motor"));

        motors.put("flywheel1", hardwareMap.get(DcMotor.class, "flywheel_motor1"));
        motors.put("flywheel2", hardwareMap.get(DcMotor.class, "flywheel_motor2"));

        //Create and assign map entries for all servos

        //Create and assign map entries for all CRServos

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

        motors.get("flywheel1").setDirection(DcMotorSimple.Direction.REVERSE);
        motors.get("flywheel2").setDirection(DcMotorSimple.Direction.FORWARD);

        //Set direction of servos

        //Initialize custom gamepads
        custom_gamepad_1 = new CustomGamepad(gamepad1);
        custom_gamepad_2 = new CustomGamepad(gamepad2);

        //Turn on the brakes for 0 power
        for (String key : motors.keySet()) {
            motors.get(key).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            ;
        }
        //Funny Comment
        //This data is displayed on the driver hub console
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Wait until the the start button is pressed on the driver hub
        waitForStart();

        //Reset runtime var
        runtime.reset();

        //Motor powers
        motor_powers.put("front_left", 0.0);
        motor_powers.put("front_right", 0.0);
        motor_powers.put("back_left", 0.0);
        motor_powers.put("back_right", 0.0);

        motor_powers.put("intake", 0.0);
        motor_powers.put("flywheel1", 0.0);
        motor_powers.put("flywheel2", 0.0);

        //Settings for servos

        //CRServos Powers

        //Initialize camera and april tag processing stuff
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                //declares the camera
                .setCamera(hardwareMap.get(CameraName.class, "Webcam 1"))
                //stream format
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                //start live stream
                //.enableLiveView(true)
                //sets camera resolution; turn down if performance issues
                .setCameraResolution(new Size(640, 480))
                //as above
                .build();

        //Disable processor to start (conserve bandwidth)
        visionPortal.setProcessorEnabled(tagProcessor, false);

        //Macro action map entries
        action_map.put("aimbot", (byte) 0b00001001);

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

            //Manual intake control
            if (check_mask("manual_intake")) {
                action_map.put("manual_intake", (byte) (action_map.get("manual_intake") | ON_BITMASK));
            }

            //Manual flywheel control
            if (check_mask("manual_flywheel")) {
                action_map.put("manual_flywheel", (byte) (action_map.get("flywheel") | ON_BITMASK));
            }

            //Check for aimbot macro
            if (custom_gamepad_2.get_dpad_up() || (action_map.get("aimbot") < 0)) {
                //If this is fresh
                if (action_map.get("aimbot") > 0) {
                    action_map.put("aimbot", (byte) (action_map.get("aimbot") | ON_BITMASK));
                    aimbot_flywheel_time_finished = runtime.milliseconds() + AIMBOT_FLYWHEEL_RUN_TIME;
                }

                //Check if we're done
                if (runtime.milliseconds() > aimbot_flywheel_time_finished) {
                    action_map.put("aimbot", (byte) (action_map.get("aimbot") & (~ON_BITMASK)));
                }
                else {
                    //Turn on image processor
                    visionPortal.setProcessorEnabled(tagProcessor, true);

                    //Scan the tag
                    AprilTagDetection tag = tagProcessor.getFreshDetections().get(0);
                    tag_range = tag.ftcPose.range * INCHES_TO_CM;
                    tag_bearing = Math.toRadians(tag.ftcPose.bearing);
                    tag_elevation = Math.atan((Math.sin(Math.toRadians(tag.ftcPose.elevation)) + (RAMP_HEIGHT-CAMERA_HEIGHT))/Math.cos(Math.toRadians(tag.ftcPose.elevation)));

                    //We're rotated too far left
                    if (tag_bearing > BEARING_RANGE) {
                        aimbot_macro_yaw = -APRIL_TAG_ROTATION_SPEED;
                        aimbot_flywheel_time_finished = runtime.milliseconds() + AIMBOT_FLYWHEEL_RUN_TIME;
                        aimbot_flywheel_power = 0;
                    }
                    //We're rotated too far right
                    else if (tag_bearing < -BEARING_RANGE) {
                        aimbot_macro_yaw = APRIL_TAG_ROTATION_SPEED;
                        aimbot_flywheel_time_finished = runtime.milliseconds() + AIMBOT_FLYWHEEL_RUN_TIME;
                        aimbot_flywheel_power = 0;
                    }
                    //Shoot
                    else {
                        aimbot_macro_yaw = 0;

                        //Calculate exit velocity we need
                        exit_velocity = (tag_range * Math.cos(tag_elevation) + BUCKET_WIDTH) * Math.sqrt(GRAVITY/(2 * Math.pow(Math.cos(tag_elevation), 2) * ((tag_range * Math.cos(tag_elevation) + BUCKET_WIDTH) * Math.tan(tag_elevation) - (BUCKET_HEIGHT-RAMP_HEIGHT))));

                        //Calculate flywheel motor power
                        aimbot_flywheel_power = Math.sqrt((Math.PI*Math.pow(exit_velocity, 2) + GRAVITY*RAMP_HEIGHT)/90*FLYWHEEL_RADIUS)/MAX_FLYWHEEL_RPM;
                    }
                }
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
                motor_powers.put("front_left", axial + lateral + yaw);
                motor_powers.put("front_right", axial - lateral - yaw);
                motor_powers.put("back_left", axial - lateral + yaw);
                motor_powers.put("back_right", axial + lateral - yaw);
            }

            //Execute intake
            if (action_map.get("manual_intake") < 0) {
                motor_powers.put("intake", INTAKE_SPEED * custom_gamepad_2.get_right_stick_y(INTAKE_ANGULAR_DEADZONE, INTAKE_RADIAL_DEADZONE));
            }

            //execute Flywheel
            if (action_map.get("manual_flywheel") < 0) {
                motor_powers.put("flywheel1", FLYWHEEL_SPEED * custom_gamepad_2.get_left_stick_y(FLYWHEEL_ANGULAR_DEADZONE, FLYWHEEL_RADIAL_DEADZONE));
                motor_powers.put("flywheel2", FLYWHEEL_SPEED * custom_gamepad_2.get_left_stick_y(FLYWHEEL_ANGULAR_DEADZONE, FLYWHEEL_RADIAL_DEADZONE));
            }

            //Execute aimbot macro
            if (action_map.get("aimbot") < 0) {
                motor_powers.put("front_left", yaw);
                motor_powers.put("front_right", -yaw);
                motor_powers.put("back_left", yaw);
                motor_powers.put("back_right", -yaw);

                motor_powers.put("flywheel1", aimbot_flywheel_power);
                motor_powers.put("flywheel2", aimbot_flywheel_power);
            }

            //Execute powers
            for (String key : motor_powers.keySet()) {
                motors.get(key).setPower(motor_powers.get(key));
            }
        }
    }
}