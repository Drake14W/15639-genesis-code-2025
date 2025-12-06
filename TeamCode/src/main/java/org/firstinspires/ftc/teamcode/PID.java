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

    private final double ticks_to_cm = (1/537.7)*2*Math.PI*9.6*wheel_coefficient;
    private final double permitted_movement_error = 5;
    private final double permitted_angle_error = 3;
    private final double ticks_to_degree = (1/537.7)*2*Math.PI*9.6*rotation_coefficient;
    private final double max_motor_power = 0.5;
    private final double max_rotation_power = 0.75;

    private Telemetry telemetry;

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

        this.telemetry = telemetry;
    }

    public void move(double distance) {
        //For all, order is front_left, back_left, front_right, back_right
        double[] motor_start_pos = {0, 0, 0, 0};
        double[] errors = {0, 0, 0, 0};
        double[] error_integral_val = {0, 0, 0, 0};
        double[] error_deriv_val = {0, 0, 0, 0};
        double pid_time_delta;
        double pid_last_time = runtime.milliseconds();
        double[] previous_error = {distance, distance, distance, distance};
        double max_error = distance;
        double motor_power;

        for (int i = 0; i < 4; i++) {
            motor_start_pos[i] = motors.get(motor_indexes[i]).getCurrentPosition();
        }

        while ((max_error > permitted_movement_error) || (max_error < -permitted_movement_error)) {
            pid_time_delta = runtime.milliseconds() - pid_last_time;
            for (int i = 0; i < 4; i++) {
                errors[i] = distance - (motors.get(motor_indexes[i]).getCurrentPosition() - motor_start_pos[i])*ticks_to_cm;
                error_integral_val[i] = errors[i]*pid_time_delta;
                error_deriv_val[i] = (errors[i] - previous_error[i]) / pid_time_delta;
                previous_error[i] = errors[i];

                motor_power = PROPORTIONAL_COEFFIEICENT * errors[i] + INTEGRAL_COEFFICIENT * error_integral_val[i] + DERIVATIVE_COEFFICIENT * error_deriv_val[i];

                if (motor_power > max_motor_power) {
                    motor_power = max_motor_power;
                }
                else if (motor_power < -max_motor_power) {
                    motor_power = -max_motor_power;
                }

                telemetry.addData("Motor Power:", motor_power);
                telemetry.addData("Error:", errors[i]);

                motors.get(motor_indexes[i]).setPower(motor_power);
            }
            pid_last_time = runtime.milliseconds();

            max_error = 0;
            for (int i = 0; i < 4; i++) {
                if (Math.abs(errors[i]) > Math.abs(max_error)) {
                    max_error = errors[i];
                }
            }

            telemetry.update();
        }
    }

    //0 degrees is directly forward. To the left is negative
    public void rotate(double degrees) {
        //For all, order is front_left, back_left, front_right, back_right
        double[] motor_start_pos = {0, 0, 0, 0};
        double[] errors = {0, 0, 0, 0};
        double[] error_integral_val = {0, 0, 0, 0};
        double[] error_deriv_val = {0, 0, 0, 0};
        double pid_time_delta;
        double pid_last_time = runtime.milliseconds();
        double[] previous_error = {degrees, degrees, degrees, degrees};
        double max_error = degrees;
        double motor_power;

        for (int i = 0; i < 4; i++) {
            motor_start_pos[i] = motors.get(motor_indexes[i]).getCurrentPosition();
        }

        while ((max_error > permitted_angle_error) || (max_error < -permitted_angle_error)) {
            pid_time_delta = runtime.milliseconds() - pid_last_time;
            for (int i = 0; i < 4; i++) {
                if (i <= 2) {
                    errors[i] = degrees - (motors.get(motor_indexes[i]).getCurrentPosition() - motor_start_pos[i]) * ticks_to_degree;
                }
                else {
                    errors[i] = degrees - (motor_start_pos[i] - motors.get(motor_indexes[i]).getCurrentPosition()) * ticks_to_degree;
                }
                error_integral_val[i] = errors[i]*pid_time_delta;
                error_deriv_val[i] = (errors[i] - previous_error[i]) / pid_time_delta;
                previous_error[i] = errors[i];

                motor_power = PROPORTIONAL_COEFFIEICENT * errors[i] + INTEGRAL_COEFFICIENT * error_integral_val[i] + DERIVATIVE_COEFFICIENT * error_deriv_val[i];

                if (motor_power > max_rotation_power) {
                    motor_power = max_rotation_power;
                }
                else if (motor_power < -max_rotation_power) {
                    motor_power = -max_rotation_power;
                }

                if (i == 2) {
                    motor_power*=1.25;
                }

                telemetry.addData("Motor Power:", motor_power);
                telemetry.addData("Error:", errors[i]);

                if (i <= 2) {
                    motors.get(motor_indexes[i]).setPower(motor_power);
                }
                else {
                    motors.get(motor_indexes[i]).setPower(-motor_power);
                }
            }
            pid_last_time = runtime.milliseconds();

            max_error = 0;
            for (int i = 0; i < 4; i++) {
                if (Math.abs(errors[i]) > Math.abs(max_error)) {
                    max_error = errors[i];
                }
            }

            telemetry.addLine("Rotating");
            telemetry.update();
        }
    }
}