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

import java.lang.Math;

import java.util.HashMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//NOTE: ALL DISTANCES ARE IN CENTIMETERS, ALL TIMES ARE IN SECONDS
public class PID {
    //Create the variables for the motors and initializes a variable that keeps track of how long the opmode has been running
    private ElapsedTime runtime = new ElapsedTime();

    //Map of all motors
    private HashMap<String, DcMotor> motors = new HashMap<>();
    private String[] motor_indexes = {"front_left", "back_left", "front_right", "back_right"};

    //Global speed percentage for movement
    private final double wheel_coefficient = 0.1;
    //Global speed percentage for rotation
    private final double rotation_coefficient = 0.2;

    private final double PROPORTIONAL_COEFFIEICENT = 4.0;
    private final double INTEGRAL_COEFFICIENT = 0.0;
    private  final double DERIVATIVE_COEFFICIENT = 0.0;

    private final double ticks_per_rotation = (1/537.7)*2*Math.PI*9.6*wheel_coefficient;
    private final double permitted_movement_error = 5;
    private final double permitted_angle_error = 3;
    private final double ticks_to_degree = (1/537.7)*2*Math.PI*9.6*rotation_coefficient;
    private final double max_motor_power = 0.5;
    private final double max_rotation_power = 0.75;

    private Telemetry telemetry;

    //FB Movement
    private double fb_distance_travelled = 0;
    private double fb_distance_away;
    private double fb_last_distance_away;
    private double fb_distance_deriv;
    private double fb_distance_integral;

    private double[] fb_last_wheel_positions = new double[4];
    private double[] fb_wheel_speeds = new double[4];
    private double[] fb_last_wheel_speeds = new double[4];
    private double[] fb_wheel_speed_errors = new double[4];
    private double[] fb_wheel_deriv = new double[4];
    private double[] fb_wheel_integral = new double[4];

    //RL Movement

    //Wheel powers
    //Order goes:
    //0: Front left
    //1: Front right
    //2: Back left
    //3: Back right
    public double[] wheel_powers = new double[4];

    public PID(DcMotor front_left_motor, DcMotor back_left_motor, DcMotor front_right_motor, DcMotor back_right_motor, Telemetry telemetry) {
        motors.put("front_left", front_left_motor);
        motors.put("back_left", back_left_motor);
        motors.put("front_right", front_right_motor);
        motors.put("back_right", back_right_motor);

        //Set motor directions
        motors.get("front_left").setDirection(DcMotor.Direction.REVERSE);
        motors.get("back_left").setDirection(DcMotor.Direction.REVERSE);
        motors.get("front_right").setDirection(DcMotor.Direction.FORWARD);
        motors.get("back_right").setDirection(DcMotor.Direction.FORWARD);

        //Use encoder
        motors.get("front_left").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors.get("back_left").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors.get("front_right").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors.get("back_right").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.telemetry = telemetry;
    }

    //0 = Still running
    //1 = Done
    public int moveFB(double distance) {
        //Find target speed for distance
        if (fb_distance_travelled == 0) {
            //We haven't moved
            fb_distance_away = 0;
            fb_last_distance_away = 0;
            fb_distance_integral = 0;

            for (int i = 0; i < 4; i++) {
                fb_last_wheel_positions[i] = 0;
                fb_wheel_speeds[i] = 0;
                fb_last_wheel_positions[i] = 0;
                fb_wheel_integral[i] = 0;
            }
        }
        if ((fb_distance_travelled > (distance-permitted_movement_error)) && (fb_distance_travelled < (distance+permitted_movement_error))) {
            //We're done
            fb_distance_travelled = 0;
        }

        //Find target wheel power for wheel speed
        return 0;
    }
    public void moveRL(double distance) {

    }
}