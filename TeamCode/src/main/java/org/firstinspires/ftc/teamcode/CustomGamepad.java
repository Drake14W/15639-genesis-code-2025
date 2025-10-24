package org.firstinspires.ftc.teamcode;

import java.lang.Math;

import com.qualcomm.robotcore.hardware.Gamepad;

public class CustomGamepad {
    Gamepad gamepad;
    private boolean a, b, x, y;
    private boolean a_just_pressed, b_just_pressed, x_just_pressed, y_just_pressed;

    private double left_stick_x, left_stick_y, right_stick_x, right_stick_y;

    private double left_trigger, right_trigger;

    private boolean left_bumper, right_bumper;
    private boolean left_bumper_just_pressed, right_bumper_just_pressed;

    private boolean dpad_up, dpad_down, dpad_left, dpad_right;
    private boolean dpad_up_just_pressed, dpad_down_just_pressed, dpad_left_just_pressed, dpad_right_just_pressed;

    public CustomGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;

        //Initialize all our values to 0

        a = b = x = y = false;
        a_just_pressed = b_just_pressed = x_just_pressed = y_just_pressed = false;

        left_stick_x = left_stick_y = right_stick_x = right_stick_y = 0.0;

        left_trigger = right_trigger = 0.0;

        left_bumper = right_bumper = false;
        left_bumper_just_pressed = right_bumper_just_pressed = false;

        dpad_up = dpad_down = dpad_left = dpad_right = false;
        dpad_up_just_pressed = dpad_down_just_pressed = dpad_left_just_pressed = dpad_right_just_pressed = false;
    }

    //Update controller inputs
    public void update() {
        //Lettered updates
        a_just_pressed = false;
        if (gamepad.a) {
            if (!a) {
                a_just_pressed = true;
            }
            a = true;
        }
        else {
            a = false;
        }
        b_just_pressed = false;
        if (gamepad.b) {
            if (!b) {
                b_just_pressed = true;
            }
            b = true;
        }
        else {
            b = false;
        }
        y_just_pressed = false;
        if (gamepad.y) {
            if (!y) {
                y_just_pressed = true;
            }
            y = true;
        }
        else {
            y = false;
        }
        x_just_pressed = false;
        if (gamepad.x) {
            if (!x) {
                x_just_pressed = true;
            }
            x = true;
        }
        else {
            x = false;
        }

        //Stick updates
        left_stick_x = gamepad.left_stick_x;
        left_stick_y = -gamepad.left_stick_y;       //Note the negatives here. I'm flipping it because ftc is stupid and thinks up on the joystick is negative
        right_stick_x = gamepad.right_stick_x;
        right_stick_y = -gamepad.right_stick_y;     //Note the negatives here. I'm flipping it because ftc is stupid and thinks up on the joystick is negative

        //Trigger updates
        left_trigger = gamepad.left_trigger;
        right_trigger = gamepad.right_trigger;

        //Bumper updates
        left_bumper_just_pressed = false;
        if (gamepad.left_bumper) {
            if (!left_bumper) {
                left_bumper_just_pressed = true;
            }
            left_bumper = true;
        }
        else {
            left_bumper = false;
        }
        right_bumper_just_pressed = false;
        if (gamepad.right_bumper) {
            if (!right_bumper) {
                right_bumper_just_pressed = true;
            }
            right_bumper = true;
        }
        else {
            right_bumper = false;
        }

        //D-pad updates
        dpad_up_just_pressed = false;
        if (gamepad.dpad_up) {
            if (!dpad_up) {
                dpad_up_just_pressed = true;
            }
            dpad_up = true;
        }
        else {
            dpad_up = false;
        }
        dpad_down_just_pressed = false;
        if (gamepad.dpad_down) {
            if (!dpad_down) {
                dpad_down_just_pressed = true;
            }
            dpad_down = true;
        }
        else {
            dpad_down = false;
        }
        dpad_left_just_pressed = false;
        if (gamepad.dpad_left) {
            if (!dpad_left) {
                dpad_left_just_pressed = true;
            }
            dpad_left = true;
        }
        else {
            dpad_left = false;
        }
        dpad_right_just_pressed = false;
        if (gamepad.dpad_right) {
            if (!dpad_right) {
                dpad_right_just_pressed = true;
            }
            dpad_right = true;
        }
        else {
            dpad_right = false;
        }
    }

    //Lettered buttons
    public boolean get_a() {
        return a;
    }
    public boolean get_b() {
        return b;
    }
    public boolean get_x() {
        return x;
    }
    public boolean get_y() {
        return y;
    }

    //Lettered button raising edge inputs
    public boolean get_a_just_pressed() {
        return a_just_pressed;
    }
    public boolean get_b_just_pressed() {
        return b_just_pressed;
    }
    public boolean get_x_just_pressed() {
        return x_just_pressed;
    }
    public boolean get_y_just_pressed() {
        return y_just_pressed;
    }

    //Stick inputs
    //Angular deadzone in degrees, represents total angle in which it gets locked (not half)
    public double get_left_stick_x(double angular_deadzone, double radial_deadzone) {
        //Check radial deadzone
        double temp_left_stick_x = left_stick_x;
        double temp_left_stick_y = left_stick_y;
        if (Math.sqrt(Math.pow(temp_left_stick_x, 2) + Math.pow(temp_left_stick_y, 2)) < radial_deadzone) {
            return 0;
        }

        //Check angular deadzone
        double angular_deadzone_radians = Math.toRadians(angular_deadzone);
        double angle = Math.atan2(temp_left_stick_y, temp_left_stick_x);
        if ((angle < ((Math.PI/2) + angular_deadzone_radians/2)) && (angle > ((Math.PI/2) - angular_deadzone_radians/2))) {
            //In top angular deadzone
            return 0;
        }
        if ((angle < ((0 - Math.PI/2) + angular_deadzone_radians/2)) && (angle > ((0 - Math.PI/2) - angular_deadzone_radians/2))) {
            //In bottom angular deadzone
            return 0;
        }

        return temp_left_stick_x;
    }
    public double get_left_stick_y(double angular_deadzone, double radial_deadzone) {
        //Check radial deadzone
        double temp_left_stick_x = left_stick_x;
        double temp_left_stick_y = left_stick_y;
        if (Math.sqrt(Math.pow(temp_left_stick_x, 2) + Math.pow(temp_left_stick_y, 2)) < radial_deadzone) {
            return 0;
        }

        //Check angular deadzone
        double angular_deadzone_radians = Math.toRadians(angular_deadzone);
        double angle = Math.atan2(temp_left_stick_y, temp_left_stick_x);
        if ((angle < (0 + angular_deadzone_radians/2)) && (angle > (0 - angular_deadzone_radians/2))) {
            //In right angular deadzone
            return 0;
        }
        if ((angle > ((Math.PI) - angular_deadzone_radians/2)) || (angle < ((0 - Math.PI) + angular_deadzone_radians/2))) {
            //In left angular deadzone
            return 0;
        }

        return temp_left_stick_y;
    }
    public double get_right_stick_x(double angular_deadzone, double radial_deadzone) {
        //Check radial deadzone
        double temp_right_stick_x = right_stick_x;
        double temp_right_stick_y = right_stick_y;
        if (Math.sqrt(Math.pow(temp_right_stick_x, 2) + Math.pow(temp_right_stick_y, 2)) < radial_deadzone) {
            return 0;
        }

        //Check angular deadzone
        double angular_deadzone_radians = Math.toRadians(angular_deadzone);
        double angle = Math.atan2(temp_right_stick_y, temp_right_stick_x);
        if ((angle < ((Math.PI/2) + angular_deadzone_radians/2)) && (angle > ((Math.PI/2) - angular_deadzone_radians/2))) {
            //In top angular deadzone
            temp_right_stick_x = 0;
        }
        else if ((angle < ((0 - Math.PI/2) + angular_deadzone_radians/2)) && (angle > ((0 - Math.PI/2) - angular_deadzone_radians/2))) {
            //In bottom angular deadzone
            temp_right_stick_x = 0;
        }

        return temp_right_stick_x;
    }
    public double get_right_stick_y(double angular_deadzone, double radial_deadzone) {
        //Check radial deadzone
        double temp_right_stick_x = right_stick_x;
        double temp_right_stick_y = right_stick_y;
        if (Math.sqrt(Math.pow(temp_right_stick_x, 2) + Math.pow(temp_right_stick_y, 2)) < radial_deadzone) {
            return 0;
        }

        //Check angular deadzone
        double angular_deadzone_radians = Math.toRadians(angular_deadzone);
        double angle = Math.atan2(temp_right_stick_y, temp_right_stick_x);
        if ((angle < (0 + angular_deadzone_radians/2)) && (angle > (0 - angular_deadzone_radians/2))) {
            //In right angular deadzone
            return 0;
        }
        if ((angle < ((Math.PI) - angular_deadzone_radians/2)) && (angle < ((0 - Math.PI) + angular_deadzone_radians/2))) {
            //In right angular deadzone
            return 0;
        }

        return temp_right_stick_y;
    }

    //Trigger inputs
    public double get_left_trigger() {
        return left_trigger;
    }
    public double get_right_trigger() {
        return right_trigger;
    }

    //Bumper inputs
    public boolean get_left_bumper(){
        return left_bumper;
    }
    public boolean get_right_bumper() {
        return right_bumper;
    }

    //Bumper raising edge inputs
    public boolean get_right_bumper_just_pressed() {
        return right_bumper_just_pressed;
    }
    public boolean get_left_bumper_just_pressed() {
        return left_bumper_just_pressed;
    }

    //D-Pad inputs
    public boolean get_dpad_up() {
        return dpad_up;
    }
    public boolean get_dpad_down() {
        return dpad_down;
    }
    public boolean get_dpad_left() {
        return dpad_left;
    }
    public boolean get_dpad_right() {
        return dpad_right;
    }

    //D-Pad raising edge inputs
    public boolean get_dpad_up_just_pressed() {
        return dpad_up_just_pressed;
    }
    public boolean get_dpad_down_just_pressed() {
        return dpad_down_just_pressed;
    }
    public boolean get_dpad_left_just_pressed() {
        return dpad_left_just_pressed;
    }
    public boolean get_dpad_right_just_pressed() {
        return dpad_right_just_pressed;
    }
}