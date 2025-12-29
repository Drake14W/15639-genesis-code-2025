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

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

//This decorator puts this opmode into selected the name and group on the driver hub menu
@TeleOp(name="FlywheelOptimizer", group="Linear OpMode")
//Since java is weird, this is essentially the equivalent of a main method in C, but instead it's a class. Also, we "extend" this class from the library class LinearOpMode which makes this into a proper teleop opmode
public class FlywheelOptimizer extends LinearOpMode {
    //Create the variables for the motors and servos and initializes a variable that keeps track of how long the opmode has been running
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor flywheel_motor;
    double flywheel_power;

    double FLYWHEEL_SPEED = 1.0;

    //RPM Calculations
    final double FLYWHEEL_CPR = 28;
    double last_flywheel_revs = 0;
    double flywheel_rpm;
    double last_rpm_time;

    final double flywheel_mass = 0.112;
    final double flywheel_radius = 0.072;
    final double flywheel_k = 3 / 4;
    final double rod_mass = 0.1;
    final double rod_radius = 0.01;
    final double rod_k = 1 / 2;

    final double mom_inertia = (
    flywheel_k * flywheel_mass * Math.pow(flywheel_radius, 2)
            + rod_k * rod_mass * Math.pow(rod_radius, 2)
            );
    final double friction_torque = 0.0011;
    final double drag_coefficient = 1e-7;

    final double MAX_FLYWHEEL_MOTOR_RPM = 6000;
    final double MOTOR_STALL_TORQUE = 0.1444087346;
    final double iters = 1;
    final double test_length = 10;
    double flywheel_rpm_error;
    double aimbot_flywheel_power;
    final double TARGET_RPM = 3000;
    double kp = 0.00005;
    double ki = 0.00001;
    double u;
    double rpm_error_integral_val;
    double pid_time_delta;
    double pid_last_time;
    double rpm_previous_error;

    //To log data for PID tuning
    StringBuffer m_csvLogString = new StringBuffer();
    double sum_diff;
    ArrayList<Double> sum_diffs;
    ArrayList<Double> lambda_vals;
    private static final String DATE_FORMAT_NOW = "yyyy-MM-dd_HH-mm-ss";

    private double inverse_model(double rpm, double u) {
        double omega = Math.min(rpm*(((2*Math.PI)/60)), 0.95 * (MAX_FLYWHEEL_MOTOR_RPM*(((2*Math.PI)/60))));
        double denom = MOTOR_STALL_TORQUE * (1.0 - omega / (MAX_FLYWHEEL_MOTOR_RPM*(((2*Math.PI)/60))));

        if (denom <= 1e-6) {
            return 0.0;
        }

        double p = (mom_inertia * u + friction_torque + drag_coefficient * Math.pow(omega, 2)) / denom;
        return Math.max(0.0, Math.min(1.0, p));
    }

    //We have to override this function since it has already been defined in the parent class LinearOpMode
    @Override
    public void runOpMode() {
        //Create log buffer
        m_csvLogString.setLength(0);

        flywheel_motor = hardwareMap.get(DcMotor.class, "flywheel_motor");
        flywheel_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheel_motor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Turn on the brakes for 0 power
        flywheel_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flywheel_power = 0.0;

        //Funny Comment
        //This data is displayed on the driver hub console
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Wait until the the start button is pressed on the driver hub
        waitForStart();

        for (double lambda_val = 5.0; lambda_val > 0.0; lambda_val+=0.2) {
            kp = 1.0 / lambda_val;
            ki = 1.0 / Math.pow(lambda_val, 2);
            for (int i = 0; i < iters; i++) {
                runtime.reset();
                last_rpm_time = runtime.milliseconds();
                flywheel_power = 0.0;
                pid_last_time = runtime.seconds();
                rpm_error_integral_val = 0;
                rpm_previous_error = 0;

                sum_diff = 0;
                while (runtime.seconds() < test_length) {
                    //Update motor rpm
                    //x60000 converts from milliseconds to minutes
                    flywheel_rpm = (flywheel_motor.getCurrentPosition() / FLYWHEEL_CPR - last_flywheel_revs) / (runtime.milliseconds() - last_rpm_time) * 60000;
                    last_flywheel_revs = flywheel_motor.getCurrentPosition() / FLYWHEEL_CPR;
                    last_rpm_time = runtime.milliseconds();
                    telemetry.addData("Flywheel RPM", flywheel_rpm);

                    flywheel_rpm_error = TARGET_RPM - flywheel_rpm;

                    pid_time_delta = runtime.seconds() - pid_last_time;
                    rpm_error_integral_val += flywheel_rpm_error * pid_time_delta;
                    pid_last_time = runtime.seconds();
                    rpm_previous_error = flywheel_rpm_error;

                    u = kp * flywheel_power + ki * rpm_error_integral_val;
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

                    telemetry.addData("Wanted RPM", TARGET_RPM);
                    telemetry.addData("RPM Error", flywheel_rpm_error);
                    telemetry.addData("Flywheel Power", aimbot_flywheel_power);

                    //Log data for PID tuning purposes
                    m_csvLogString.append(runtime.seconds()).append(", ").append(flywheel_rpm).append(", ").append(TARGET_RPM).append(", ").append(aimbot_flywheel_power).append("\n");

                    sum_diff += flywheel_rpm_error * pid_time_delta;
                }

                if (m_csvLogString.length() > 0) {
                    String csvPath = String.format("%s/FIRST/data/raw_data_lambda_%.2f_run_%d.csv", Environment.getExternalStorageDirectory().getAbsolutePath(), lambda_val, i);
                    try (FileWriter csvWriter = new FileWriter(csvPath, false)) {
                        csvWriter.write(m_csvLogString.toString());
                    } catch (IOException e) {
                        telemetry.addLine(e.getMessage());
                        telemetry.update();
                    }
                    m_csvLogString.setLength(0);
                }
                sleep(15000);
            }
            sum_diff /= iters;
            sum_diffs.add(sum_diff);
            lambda_vals.add(lambda_val);
        }

        for (int i = 0; i < lambda_vals.size(); i++) {
            m_csvLogString.append(lambda_vals.get(i)).append(",").append(sum_diffs.get(i)).append("\n");
        }
    }
}