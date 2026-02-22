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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
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

//This decorator puts this opmode into selected the name and group on the driver hub menu
@Autonomous(name="AutoBlue2", group="Linear OpMode")
//Since java is weird, this is essentially the equivalent of a main method in C, but instead it's a class. Also, we "extend" this class from the library class LinearOpMode which makes this into a proper teleop opmode
public class AutoBlue2 extends LinearOpMode {
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
    final double LONG_LAT_RADIAL_DEADZONE = 0.0;  //Decimal from 0-1
    final double LONG_LAT_ANGULAR_DEADZONE = 2;   //In degrees
    final double ROTATION_RADIAL_DEADZONE = 0.0;  //Decimal from 0-1
    final double ROTATION_ANGULAR_DEADZONE = 0.0; //In degrees
    final double ROTATION_SPEED = 0.75;
    final double MAX_MOVEMENT_SPEED = 1.0;
    double movement_speed = MAX_MOVEMENT_SPEED;

    //Other assorted globals
    final double INTAKE_SPEED = 1.0;
    final double INTAKE_ANGULAR_DEADZONE = 0.0;
    final double INTAKE_RADIAL_DEADZONE = 0.05;
    final double INTAKE_SERVO_SPEED = 1.0;
    final double REVOLVER_FAST_SPEED = 0.4;
    final double REVOLVER_SLOW_SPEED = 0.05;
    final double REVOLVER_CPR = 537.7;
    final double REVOLVER_ROTATION_DISTANCE = (REVOLVER_CPR*(1.0/6.0));
    int revolver_distance_multiple;
    RevolverController revolver_controller;
    double revolver_target = 0;
    final double REVOLVER_RANGE = 5.0;
    final double MAX_TURRET_SPEED = 0.5;
    final double MIN_TURRET_SPEED = 0.05;
    final double MAX_TURRET_ROTATION = 2.2*REVOLVER_CPR;
    final double ROTATION_TURRET_CONVERSION = 0.5;
    double turret_starting_pos;
    int sweeping = 0;
    double FLYWHEEL_SPEED = 1.0;
    final double FLYWHEEL_ANGULAR_DEADZONE = 0.0;
    final double FLYWHEEL_RADIAL_DEADZONE = 0.20;
    double flicker_down_pos;
    double flicker_mid_pos;
    double flicker_up_pos;
    boolean telemetry_output = true;

    //RPM Calculations
    final double FLYWHEEL_CPR = 39.2;
    double last_flywheel_revs = 0;
    double flywheel_rpm;
    double last_rpm_time;

    //Used to set bytes to "on"
    final byte ON_BITMASK = (byte) 0b10000000;

    private boolean check_mask(String key) {
        byte bit_mask_result;
        for (String iter_key : action_map.keySet()) {
            if (iter_key != key) {
                if (action_map.get(iter_key) < 0) {
                    bit_mask_result = (byte) (action_map.get(key) & action_map.get(iter_key) & (~ON_BITMASK));
                    if (bit_mask_result != 0) {
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
    final double APRIL_TAG_PERMITTED_DELAY = 0.2; //We allow 0.2 seconds of delay between detecting the april tag and acting upon it, otherwise we ignore it

    //Movement values for aimbot (april tag) macro
    double aimbot_macro_yaw;
    double aimbot_flywheel_power;
    double aimbot_intake_power;
    boolean aimbot_shooting;

    boolean alliance = true;

    //Calculation values for aimbot
    double exit_velocity;
    final double MAX_FLYWHEEL_RPM = 18000;
    final double GRAVITY = 9.81;
    final double BUCKET_HEIGHT = 98.45;
    final double CAMERA_HEIGHT = 45;
    final double CAMERA_X_OFFSET = 0; //To the left of the launcher is positive
    final double CAMERA_Z_OFFSET = 0; //Forward from the launcher is positive
    final double CAMERA_YAW = 0; //Left to right angle (positive is left)
    final double CAMERA_PITCH = 15; //Up-down angle (positive is up)
    final double CAMERA_ROLL = 0; //Up-down rotation angle (like this for positive: ↓---↑)
    final double EXIT_HEIGHT = 40;
    final double EXIT_ANGLE = Math.toRadians(45);
    final double BUCKET_WIDTH = 37; //In cm
    final double MAGIC_FLYWHEEL_NUMBER = 2.30;
    double flywheel_rpm_error;
    double aimbot_needed_flywheel_rpm;
    final double DERIVATIVE_COEFFICIENT = 0.0;
    double rpm_error_integral_val;
    double rpm_error_deriv_val;
    double pid_time_delta;
    double pid_last_time;
    double rpm_previous_error;
    final double FLYWHEEL_RPM_ERROR_RANGE = 100; //We still shoot even if we're this far away from our desired rpm
    final double LAMBDA_VAL = 3.0;
    final double PROPORTIONAL_COEFFIEICENT = 1.0 / LAMBDA_VAL;
    final double INTEGRAL_COEFFICIENT = 1.0 / Math.pow(LAMBDA_VAL, 2);
    //final double DERIVATIVE_COEFFICIENT = 0.0005;

    final double mom_inertia = 0.000440456;
    final double friction_torque = 0.0011;
    final double drag_coefficient = 1e-7;

    final double MAX_FLYWHEEL_MOTOR_RPM = 6000;
    final double MOTOR_STALL_TORQUE = 0.1444087346;
    double u;
    private double inverse_model(double rpm, double u) {
        telemetry.addData("u:", u);
        double omega = Math.min(rpm*(((2*Math.PI)/60)), 0.95 * (MAX_FLYWHEEL_MOTOR_RPM*(((2*Math.PI)/60))));
        telemetry.addData("Omega:", omega);
        double denom = MOTOR_STALL_TORQUE * (1.0 - omega / (MAX_FLYWHEEL_MOTOR_RPM*(((2*Math.PI)/60))));
        telemetry.addData("Denom:", denom);

        if (denom <= 1e-6) {
            return 0.0;
        }

        double p = (mom_inertia * u + friction_torque + drag_coefficient * Math.pow(omega, 2)) / denom;
        telemetry.addData("Moment of Inertia:", mom_inertia*1000000);
        telemetry.addData("Friction Torque:", friction_torque);
        telemetry.addData("Drag Coeff:", drag_coefficient*10000);
        telemetry.addData("Suggested Power:", p);
        return Math.max(0.0, Math.min(1.0, p));
    }

    //To log data for PID tuning
    StringBuffer m_csvLogString = new StringBuffer();
    private static final String DATE_FORMAT_NOW = "yyyy-MM-dd_HH-mm-ss";

    double last_time = -1;

    //We have to override this function since it has already been defined in the parent class LinearOpMode
    @Override
    public void runOpMode() {
        //Create log buffer
        m_csvLogString.setLength(0);

        //Add actions to hashmap
        action_map.put("manual_movement", (byte) 0b00000001);
        action_map.put("trigger_rotation", (byte) 0b00000010);
        action_map.put("stick_rotation", (byte) 0b00000010);
        action_map.put("manual_intake", (byte) 0b00000100);
        action_map.put("manual_flywheel", (byte) 0b00001000);
        action_map.put("manual_intake_servo", (byte) 0b00010000);
        action_map.put("revolver", (byte) 0b00100000);
        action_map.put("flicker", (byte) 0b01000000);
        action_map.put("turret", (byte) 0b10000000);

        //Create and assign map entries for all motors
        motors.put("front_left", hardwareMap.get(DcMotor.class, "front_left_motor"));
        motors.put("back_left", hardwareMap.get(DcMotor.class, "back_left_motor"));
        motors.put("front_right", hardwareMap.get(DcMotor.class, "front_right_motor"));
        motors.put("back_right", hardwareMap.get(DcMotor.class, "back_right_motor"));

        motors.put("intake", hardwareMap.get(DcMotor.class, "intake_motor"));

        motors.put("flywheel", hardwareMap.get(DcMotor.class, "flywheel_motor"));

        motors.put("turret", hardwareMap.get(DcMotor.class, "turret"));
        motors.put("revolver", hardwareMap.get(DcMotor.class, "revolver"));

        //Create and assign map entries for all servos
        servos.put("flicker", hardwareMap.get(Servo.class, "flicker"));

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
        motors.get("turret").setDirection(DcMotorSimple.Direction.FORWARD);
        motors.get("revolver").setDirection(DcMotorSimple.Direction.REVERSE);

        turret_starting_pos = motors.get("turret").getCurrentPosition();
        servos.get("flicker").setDirection(Servo.Direction.FORWARD);

        //Initialize custom gamepads
        custom_gamepad_1 = new CustomGamepad(gamepad1);
        custom_gamepad_2 = new CustomGamepad(gamepad2);

        servo_positions.put("flicker", flicker_down_pos);

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
        flicker_down_pos = 0.2;
        flicker_mid_pos = 0.75;
        flicker_up_pos = 1.0;


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
                .setLensIntrinsics(1385.92f, 1385.92f, 951.982f, 534.084f)
                .build();

        revolver_controller = new RevolverController();
        NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        colorSensor.setGain(125);

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                //declares the camera
                .setCamera(hardwareMap.get(CameraName.class, "c920"))
                //stream format
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                //sets camera resolution; turn down if performance issues
                .setCameraResolution(new Size(1920, 1080))
                //as above
                .build();

        revolver_controller.balls[0] = 1;
        revolver_controller.balls[2] = 2;
        revolver_controller.balls[4] = 2;

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

        //Macro action map entries
        action_map.put("aimbot", (byte) 0b00011001);

        //Funny Comment
        //This data is displayed on the driver hub console
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        int actions[] = {6, -1};
        int action_index = 0;

        //Wait until the the start button is pressed on the driver hub
        waitForStart();

        last_rpm_time = runtime.milliseconds();

        //Reset runtime var
        runtime.reset();

        //Main loop. This runs until stop is pressed on the driver hub
        while (opModeIsActive()) {
            telemetry.addData("Action Index:", action_index);
            //Update motor rpm
            //x60000 converts from milliseconds to minutes
            flywheel_rpm = (motors.get("flywheel").getCurrentPosition()/FLYWHEEL_CPR - last_flywheel_revs) / (runtime.milliseconds() - last_rpm_time) * 60000;
            last_flywheel_revs = motors.get("flywheel").getCurrentPosition()/FLYWHEEL_CPR;
            last_rpm_time = runtime.milliseconds();
            telemetry.addData("Flywheel RPM", flywheel_rpm);

            if (action_index == -1) {
                break;
            }

            if (actions[action_index] == 0) {
                if (last_time < 0) {
                    last_time = runtime.seconds();
                }
                else if ((runtime.seconds()-last_time) > 0.5) {
                    action_index+=1;
                    last_time = -1;
                }
            }

            if (actions[action_index] == 6) {
                if (last_time < 0) {
                    last_time = runtime.seconds();
                }
                else {
                    if ((runtime.seconds()-last_time) > 0.5) {
                        action_index+=1;
                        last_time = -1;
                    }
                    else {
                        motor_powers.put("front_left", movement_speed);
                        motor_powers.put("front_right", movement_speed);
                        motor_powers.put("back_left", movement_speed);
                        motor_powers.put("back_right", movement_speed);
                    }
                }
            }

            //Colour sensor
            if (colorSensor.getNormalizedColors().green > 0.2) {
                if (colorSensor.getNormalizedColors().blue > colorSensor.getNormalizedColors().green) {
                    revolver_controller.set_ball(2);
                    telemetry.addLine("Purple");
                }
                else {
                    revolver_controller.set_ball(1);
                    telemetry.addLine("Green");
                }
            }
            else {
                revolver_controller.set_ball(0);
                telemetry.addLine("Gray");
            }

            //Revolver control
            if (check_mask("revolver")) {
                if ((Math.abs(motors.get("revolver").getCurrentPosition() - revolver_target) < REVOLVER_RANGE) || (revolver_target == 0)) {
                    if (custom_gamepad_2.get_dpad_right_just_pressed()) {
                        action_map.put("revolver", (byte) (action_map.get("revolver") | ON_BITMASK));
                        revolver_target = motors.get("revolver").getCurrentPosition() + REVOLVER_ROTATION_DISTANCE/4;

                        if (revolver_target < motors.get("revolver").getCurrentPosition()) {
                            motor_powers.put("revolver", -REVOLVER_FAST_SPEED);
                        }
                        else {
                            motor_powers.put("revolver", REVOLVER_FAST_SPEED);
                        }
                    }
                    else if (custom_gamepad_2.get_dpad_left_just_pressed()) {
                        action_map.put("revolver", (byte) (action_map.get("revolver") | ON_BITMASK));
                        revolver_target = motors.get("revolver").getCurrentPosition() - REVOLVER_ROTATION_DISTANCE/4;

                        if (revolver_target < motors.get("revolver").getCurrentPosition()) {
                            motor_powers.put("revolver", -REVOLVER_FAST_SPEED);
                        }
                        else {
                            motor_powers.put("revolver", REVOLVER_FAST_SPEED);
                        }
                    }
                    else if ((actions[action_index] == 3) && (aimbot_shooting)) {
                        action_map.put("revolver", (byte) (action_map.get("revolver") | ON_BITMASK));

                        revolver_distance_multiple = revolver_controller.shoot();
                        if (revolver_distance_multiple != 0) {
                            revolver_target = motors.get("revolver").getCurrentPosition() + revolver_distance_multiple*REVOLVER_ROTATION_DISTANCE;

                            if (revolver_target < motors.get("revolver").getCurrentPosition()) {
                                motor_powers.put("revolver", -REVOLVER_FAST_SPEED);
                            } else {
                                motor_powers.put("revolver", REVOLVER_FAST_SPEED);
                            }
                        }
                        action_index+=1;
                    }
                    else if (actions[action_index] == 7) {
                        action_map.put("revolver", (byte) (action_map.get("revolver") | ON_BITMASK));

                        revolver_distance_multiple = revolver_controller.intake();
                        if (revolver_distance_multiple != 0) {
                            revolver_target = motors.get("revolver").getCurrentPosition() + revolver_distance_multiple*REVOLVER_ROTATION_DISTANCE;

                            if (revolver_target < motors.get("revolver").getCurrentPosition()) {
                                motor_powers.put("revolver", -REVOLVER_FAST_SPEED);
                            } else {
                                motor_powers.put("revolver", REVOLVER_FAST_SPEED);
                            }
                        }
                        action_index+=1;
                    }
                    else {
                        action_map.put("revolver", (byte) (action_map.get("revolver") & (~ON_BITMASK)));
                        motor_powers.put("revolver", 0.0);
                    }

                    telemetry.addLine("In Position");
                }
                else {
                    if (revolver_target < motors.get("revolver").getCurrentPosition()) {
                        motor_powers.put("revolver", -((Math.abs(revolver_target - motors.get("revolver").getCurrentPosition())/REVOLVER_ROTATION_DISTANCE)*(REVOLVER_FAST_SPEED-REVOLVER_SLOW_SPEED)+REVOLVER_SLOW_SPEED));
                    }
                    else {
                        motor_powers.put("revolver", ((Math.abs(revolver_target - motors.get("revolver").getCurrentPosition())/REVOLVER_ROTATION_DISTANCE)*(REVOLVER_FAST_SPEED-REVOLVER_SLOW_SPEED)+REVOLVER_SLOW_SPEED));
                    }
                    telemetry.addLine("Out of Position");
                }
            }
            telemetry.addData("Revolver Position:", revolver_controller.position);
            telemetry.addData("Revolver 0:", revolver_controller.balls[0]);
            telemetry.addData("Revolver 1:", revolver_controller.balls[1]);
            telemetry.addData("Revolver 2:", revolver_controller.balls[2]);
            telemetry.addData("Revolver 3:", revolver_controller.balls[3]);
            telemetry.addData("Revolver 4:", revolver_controller.balls[4]);
            telemetry.addData("Revolver 5:", revolver_controller.balls[5]);

            if (actions[action_index] == 1) {
                action_map.put("turret", (byte) (action_map.get("turret") | ON_BITMASK));
                if (tagProcessor.getDetections().size() > 0) {
                    try {
                        tag = tagProcessor.getDetections().get(0);
                    } catch (Exception e) {
                        continue;
                    }
                    if ((tag.id >= 21) && (tag.id <= 23)) {
                        revolver_controller.set_pattern(tag.id);
                        action_index+=1;
                        action_map.put("turret", (byte) (action_map.get("turret") & (~ON_BITMASK)));
                    }
                }
                else if ((motors.get("turret").getCurrentPosition() - turret_starting_pos) > (MAX_TURRET_ROTATION*0.05)) {
                    motor_powers.put("turret", -(MAX_TURRET_SPEED+MIN_TURRET_SPEED)/4);
                }
                else {
                    motor_powers.put("turret", 0.0);
                }
            }
            if (actions[action_index] == 8) {
                action_map.put("turret", (byte) (action_map.get("turret") | ON_BITMASK));
                if ((motors.get("turret").getCurrentPosition() - turret_starting_pos) > (MAX_TURRET_ROTATION*0.05)) {
                    motor_powers.put("turret", (MAX_TURRET_SPEED+MIN_TURRET_SPEED)/4);
                }
                else {
                    motor_powers.put("turret", 0.0);
                    action_index+=1;
                }
            }

            //Check for aimbot macro (EVAN CHANGED the button FROM gamepad2 to 1)
            if (((actions[action_index] == 2) || (action_map.get("aimbot") < 0)) && check_mask("aimbot")) {
                //If this is fresh
                if (action_map.get("aimbot") > 0) {
                    action_map.put("aimbot", (byte) (action_map.get("aimbot") | ON_BITMASK));
                    pid_last_time = runtime.seconds();
                    rpm_error_integral_val = 0;
                    rpm_previous_error = 0;
                    aimbot_shooting = false;
                    action_index += 1;

                    aimbot_flywheel_power = 0;
                }
                else {
                    if ((tagProcessor.getDetections().size() > 0) || (aimbot_shooting)) {
                        try {
                            tag = tagProcessor.getDetections().get(0);
                        }
                        catch (Exception e){
                            continue;
                        }
                        //1000000000 converts from nanoseconds to seconds
                        if (((((double)(System.nanoTime() - tag.frameAcquisitionNanoTime)) < APRIL_TAG_PERMITTED_DELAY*1000000000) && (tag.id == (alliance ? 24 : 20))) || (aimbot_shooting)) {
                            //Scan the tag
                            telemetry.addData("ID", tag.metadata.id);
                            tag_bearing = tag.ftcPose.bearing;
                            tag_elevation = tag.ftcPose.elevation;
                            if (tag.ftcPose.range > 0) {
                                tag_range = tag.ftcPose.range;
                            }
                            telemetry.addData("Bearing:", tag_bearing*(180/Math.PI));

                            //Get the flywheel running and PID this motherfucker (XD alright)
                            //Calculate exit velocity we need
                            exit_velocity = (tag_range + BUCKET_WIDTH) * Math.sqrt((GRAVITY * 100) / (2 * Math.pow(Math.cos(EXIT_ANGLE), 2) * ((tag_range + BUCKET_WIDTH) * Math.tan(EXIT_ANGLE) - (BUCKET_HEIGHT - EXIT_HEIGHT))));

                            //Calculate flywheel motor power
                            //I'm not good enough at physics for this shit (Damn is this Jamie?)
                            aimbot_needed_flywheel_rpm = MAGIC_FLYWHEEL_NUMBER * exit_velocity;

                            flywheel_rpm_error = aimbot_needed_flywheel_rpm - flywheel_rpm;

                            pid_time_delta = runtime.seconds() - pid_last_time;
                            rpm_error_integral_val += flywheel_rpm_error * pid_time_delta;
                            pid_last_time = runtime.seconds();
                            rpm_previous_error = flywheel_rpm_error;

                            u = PROPORTIONAL_COEFFIEICENT * flywheel_rpm_error + INTEGRAL_COEFFICIENT * rpm_error_integral_val;
                            aimbot_flywheel_power = inverse_model(flywheel_rpm, u);

                            if ((aimbot_flywheel_power < 0.0) || (aimbot_flywheel_power > 1.0)) {
                                rpm_error_integral_val -= flywheel_rpm_error * pid_time_delta;
                            }

                            if (aimbot_flywheel_power < 0) {
                                aimbot_flywheel_power = 0;
                            }
                            if (aimbot_flywheel_power > 1) {
                                aimbot_flywheel_power = 1;
                            }

                            telemetry.addData("Wanted RPM", aimbot_needed_flywheel_rpm);
                            telemetry.addData("Exit Velocity", exit_velocity/100);
                            telemetry.addData("RPM Error", flywheel_rpm_error);
                            telemetry.addData("Flywheel Power", aimbot_flywheel_power);
                            //We're rotated too far left
                            if (((tag_bearing > -BEARING_RANGE) && (tag_bearing > -BEARING_RANGE) && (flywheel_rpm_error < FLYWHEEL_RPM_ERROR_RANGE)) || (aimbot_shooting)) {
                                telemetry.addLine("Shooting");
                                aimbot_macro_yaw = 0;
                                aimbot_shooting = true;
                                telemetry.addData("Exit Velocity", exit_velocity);
                                telemetry.addData("Flywheel Power", aimbot_flywheel_power);
                            }
                        }
                        //Log data for PID tuning purposes
                        m_csvLogString.append(runtime.milliseconds()).append(", ").append(flywheel_rpm).append(", ").append(aimbot_needed_flywheel_rpm).append(", ").append(aimbot_flywheel_power).append("\n");
                    }
                }
            }
            if (actions[action_index] == 5) {
                //Turn off macro
                //Turn off macro
                action_map.put("aimbot", (byte) (action_map.get("aimbot") & (~ON_BITMASK)));
                aimbot_shooting = false;
                aimbot_flywheel_power = 0;
                action_index += 1;
            }

            //Manual flicker control gamepad1
            if (check_mask("flicker")) {
                if (actions[action_index] == 4) {
                    action_map.put("flicker", (byte) (action_map.get("flicker") | ON_BITMASK));
                    servo_positions.put("flicker", flicker_up_pos);
                    revolver_controller.remove_ball();
                    action_index+=1;
                }
                else if (actions[action_index] == 8){
                    action_map.put("flicker", (byte) (action_map.get("flicker") & (~ON_BITMASK)));
                    servo_positions.put("flicker", flicker_down_pos);
                    action_index+=1;
                }
            }

            //Execute aimbot macro
            if (action_map.get("aimbot") < 0) {
                motor_powers.put("front_left", aimbot_macro_yaw);
                motor_powers.put("front_right", -aimbot_macro_yaw);
                motor_powers.put("back_left", aimbot_macro_yaw);
                motor_powers.put("back_right", -aimbot_macro_yaw);

                motor_powers.put("flywheel", aimbot_flywheel_power);

                motor_powers.put("intake", aimbot_intake_power);
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
            for (String key : servos.keySet()) {
                servos.get(key).setPosition(servo_positions.get(key));
                telemetry.addData(key, servo_positions.get(key));
            }
            for (String key : action_map.keySet()) {
                telemetry.addData(key, String.format("%8s", Integer.toBinaryString(action_map.get(key) & 0xFF)).replace(' ', '0'));
            }

            if (telemetry_output) {
                telemetry.update();
            }
        }
    }
}