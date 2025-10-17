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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;
import java.lang.Math;

//This decorator puts this opmode into selected the name and group on the driver hub menu
@TeleOp(name="MainTeleOp", group="Linear OpMode")
//Since java is weird, this is essentially the equivalent of a main method in C, but instead it's a class. Also, we "extend" this class from the library class LinearOpMode which makes this into a proper teleop opmode
public class MainTeleOp extends LinearOpMode {
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
    //Level 3:          Lorem ipsum
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

    //We have to override this function since it has already been defined in the parent class LinearOpMode
    @Override
    public void runOpMode() {
        //Add actions to hashmap
        action_map.put("manual_movement", (byte) 0b00000001);
        action_map.put("trigger_rotation", (byte) 0b00000010);
        action_map.put("stick_rotation", (byte) 0b00000010);
        action_map.put("manual_intake", (byte) 0b00000100);
        action_map.put("flywheel", (byte) 0b00001000);

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
            if (check_mask("flywheel")) {
                action_map.put("flywheel", (byte) (action_map.get("flywheel") | ON_BITMASK));
            }

            telemetry.addData("Axial", axial);
            telemetry.addData("Lateral", lateral);
            telemetry.addData("Yaw", stick_yaw);
            telemetry.addData("Manual Movement", action_map.get("manual_movement"));
            telemetry.addData("Flywheel", action_map.get("flywheel"));
            telemetry.update();

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

            //execute flywheel
            if (action_map.get("flywheel") < 0) {
                motor_powers.put("flywheel1", FLYWHEEL_SPEED * custom_gamepad_2.get_left_stick_y(FLYWHEEL_ANGULAR_DEADZONE, FLYWHEEL_RADIAL_DEADZONE));
                motor_powers.put("flywheel2", FLYWHEEL_SPEED * custom_gamepad_2.get_left_stick_y(FLYWHEEL_ANGULAR_DEADZONE, FLYWHEEL_RADIAL_DEADZONE));
            }

            //Execute powers
            for (String key : motor_powers.keySet()) {
                motors.get(key).setPower(motor_powers.get(key));
            }
        }
    }
}