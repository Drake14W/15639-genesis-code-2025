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
public class NewPID {
    //Create the variables for the motors and initializes a variable that keeps track of how long the opmode has been running
    private ElapsedTime runtime = new ElapsedTime();

    //Map of all motors
    private HashMap<String, DcMotor> motors = new HashMap<>();
    private String[] motor_indexes = {"front_left", "back_left", "front_right", "back_right"};

    //Global speed percentage for movement
    private final double wheel_coefficient = 0.1;
    //Global speed percentage for rotation
    private final double rotation_coefficient = 0.2;

    private final double SPEED_PROPORTIONAL_COEFFIEICENT = 0.1;
    private final double SPEED_INTEGRAL_COEFFICIENT = 0.0;
    private final double SPEED_DERIVATIVE_COEFFICIENT = 0.0;
    private final double WHEEL_PROPORTIONAL_COEFFIEICENT = 0.369822;
    private final double WHEEL_INTEGRAL_COEFFICIENT = 0.0;
    private final double WHEEL_DERIVATIVE_COEFFICIENT = 0.0;
    private final double MOTOR_CPR = 537.7;

    private final double permitted_movement_error = 5;
    private final double permitted_angle_error = 3;
    private final double max_motor_power = 0.5;
    private final double max_stop_power = 0.05;
    private final double max_rotation_power = 0.75;

    private Telemetry telemetry;

    //FB Movement
    private double move_distance_travelled = 0;
    private double move_distance_away;
    private double move_last_distance_away;
    private double move_distance_deriv;
    private double move_distance_integral;
    private double move_last_time;
    private double move_time_delta;

    private double move_target_wheel_speed;
    private double[] move_last_wheel_positions = new double[4];
    private double[] move_wheel_speeds = new double[4];
    private double[] move_last_wheel_speeds = new double[4];
    private double[] move_wheel_speed_errors = new double[4];
    private double[] move_wheel_deriv = new double[4];
    private double[] move_wheel_integral = new double[4];
    final double[] move_last_wheel_speed_errors = new double[4];

    //RL Movement

    //Wheel powers
    //Order goes:
    //0: Front left
    //1: Front right
    //2: Back left
    //3: Back right
    public double[] wheel_powers = new double[4];

    public NewPID(DcMotor front_left_motor, DcMotor back_left_motor, DcMotor front_right_motor, DcMotor back_right_motor, Telemetry telemetry) {
        motors.put("front_left", front_left_motor);
        motors.put("back_left", back_left_motor);
        motors.put("front_right", front_right_motor);
        motors.put("back_right", back_right_motor);

        this.telemetry = telemetry;
    }

    //0 = Still running
    //1 = Done
    public int move(double distance, boolean lr) {
        //Find target speed for distance
        if (move_distance_travelled == 0) {
            //We haven't moved
            move_distance_away = distance;
            move_last_distance_away = 0;
            move_distance_integral = 0;
            move_last_time = runtime.seconds();

            for (int i = 0; i < 4; i++) {
                move_last_wheel_positions[i] = 0;
                move_wheel_speeds[i] = 0;
                move_last_wheel_positions[i] = 0;
                move_wheel_integral[i] = 0;
                move_last_wheel_speed_errors[i] = 0;
            }
        }
        if ((move_distance_travelled > (distance-permitted_movement_error)) && (move_distance_travelled < (distance+permitted_movement_error)) && (Math.abs(wheel_powers[0]) < max_stop_power) && (Math.abs(wheel_powers[1]) < max_stop_power) && (Math.abs(wheel_powers[2]) < max_stop_power) && (Math.abs(wheel_powers[3]) < max_stop_power)) {
            //We're done
            move_distance_travelled = 0;
            return 1;
        }

        move_distance_away = distance - move_distance_travelled;
        move_time_delta = runtime.seconds() - move_last_time;
        move_distance_integral += move_distance_away*move_time_delta;
        move_distance_deriv = (move_distance_away - move_last_distance_away) / move_time_delta;
        move_last_time = runtime.seconds();
        move_last_distance_away = move_distance_away;

        move_target_wheel_speed = (move_distance_away * SPEED_PROPORTIONAL_COEFFIEICENT + move_distance_deriv * SPEED_DERIVATIVE_COEFFICIENT + move_distance_integral * SPEED_INTEGRAL_COEFFICIENT) / 5.2;

        for (int i = 0; i < 4; i++) {
            move_wheel_speed_errors[i] = move_target_wheel_speed - move_wheel_speeds[i];

            move_wheel_integral[i] += move_wheel_speed_errors[i]*move_time_delta;
            move_wheel_deriv[i] = (move_wheel_speed_errors[i] - move_last_wheel_speed_errors[i]) / move_time_delta;
            move_last_wheel_speed_errors[i] = move_wheel_speed_errors[i];


            wheel_powers[i] = WHEEL_PROPORTIONAL_COEFFIEICENT * move_wheel_speed_errors[i] + WHEEL_DERIVATIVE_COEFFICIENT * move_wheel_deriv[i] + WHEEL_INTEGRAL_COEFFICIENT * move_wheel_integral[i];
            if (lr && ((i == 1) || (i == 2))) {
                wheel_powers[i] *= -1;
            }
            if (wheel_powers[i] > 1.0) {
                wheel_powers[i] = 1.0;
            }
            else if (wheel_powers[i] < -1.0) {
                wheel_powers[i] = -1.0;
            }
        }

        move_wheel_speeds[0] = ((motors.get("front_left").getCurrentPosition()/MOTOR_CPR) - move_last_wheel_positions[0]) * (2*Math.PI) / move_time_delta;
        move_last_wheel_positions[0] = motors.get("front_left").getCurrentPosition()/MOTOR_CPR;
        move_wheel_speeds[1] = ((motors.get("back_left").getCurrentPosition()/MOTOR_CPR) - move_last_wheel_positions[1]) * (2*Math.PI) / move_time_delta;
        move_last_wheel_positions[1] = motors.get("back_left").getCurrentPosition()/MOTOR_CPR;
        move_wheel_speeds[2] = ((motors.get("front_right").getCurrentPosition()/MOTOR_CPR) - move_last_wheel_positions[2]) * (2*Math.PI) / move_time_delta;
        move_last_wheel_positions[2] = motors.get("front_right").getCurrentPosition()/MOTOR_CPR;
        move_wheel_speeds[3] = ((motors.get("back_right").getCurrentPosition()/MOTOR_CPR) - move_last_wheel_positions[3]) * (2*Math.PI) / move_time_delta;
        move_last_wheel_positions[3] = motors.get("back_right").getCurrentPosition()/MOTOR_CPR;

        move_distance_travelled = (move_last_wheel_positions[0] + move_last_wheel_positions[1] + move_last_wheel_positions[2] + move_last_wheel_positions[3]) * 2 * Math.PI * 5.2 / 4;

        motors.get("front_left").setPower(wheel_powers[0]);
        motors.get("back_left").setPower(wheel_powers[1]);
        motors.get("front_right").setPower(wheel_powers[2]);
        motors.get("back_right").setPower(wheel_powers[3]);

        telemetry.addData("Front Left Power:", wheel_powers[0]);
        telemetry.addData("Back Left Power:", wheel_powers[1]);
        telemetry.addData("Front Right Power:", wheel_powers[2]);
        telemetry.addData("Back Right Power:", wheel_powers[3]);
        telemetry.addData("Front Left Speed:", move_wheel_speeds[0]);
        telemetry.addData("Back Left Speed:", move_wheel_speeds[1]);
        telemetry.addData("Front Right Speed:", move_wheel_speeds[2]);
        telemetry.addData("Back Right Speed:", move_wheel_speeds[3]);
        telemetry.addData("Front Left Speed Error:", move_wheel_speed_errors[0]);
        telemetry.addData("Back Left Speed Error:", move_wheel_speed_errors[1]);
        telemetry.addData("Front Right Speed Error:", move_wheel_speed_errors[2]);
        telemetry.addData("Back Right Speed Error:", move_wheel_speed_errors[3]);
        telemetry.addData("Distance Away:", move_distance_away);
        telemetry.addData("Distance Travelled:", move_distance_travelled);
        telemetry.addData("Target Wheel Velocity:", move_target_wheel_speed);
        telemetry.addData("Front Left Rotations:", motors.get("front_right").getCurrentPosition()/MOTOR_CPR);
        telemetry.addData("Time Delta", move_time_delta);
        telemetry.update();

        return 0;
    }
}