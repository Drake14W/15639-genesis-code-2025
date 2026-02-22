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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.Array;
import java.util.HashMap;

//NOTE: ALL DISTANCES ARE IN CENTIMETERS, ALL TIMES ARE IN SECONDS
public class RevolverController {

    public int[] balls = {0, -1, 0, -1, 0, -1};

    public int[] pattern = {2, 2, 2};
    public int pattern_pos = 0;

    public int position = 1;

    public void set_pattern(int pattern) {
        switch (pattern) {
            case 21:
                this.pattern[0] = 1;
                this.pattern[1] = 2;
                this.pattern[2] = 2;
                break;
            case 22:
                this.pattern[0] = 1;
                this.pattern[1] = 2;
                this.pattern[2] = 1;
                break;
            case 23:
                this.pattern[0] = 1;
                this.pattern[1] = 1;
                this.pattern[2] = 2;
                break;
        }
    }

    //1 = green
    //2 = purple
    public void set_ball(int colour) {
        if ((position%2) == 0) {
            balls[position] = colour;
        }
    }
    public void remove_ball() {
        if ((position%2) != 0) {
            if (position >= balls.length/2) {
                balls[position-balls.length/2] = 0;
            }
            else {
                balls[position+balls.length/2] = 0;
            }
        }
    }

    public void manual_rotate(int change) {
        position += change;
        if (position <= -1) {
            position = balls.length + position;
        }
        else if (position >= balls.length) {
            position = position % 6;
        }
    }

    public void reset_pattern() {
        pattern_pos = 0;
    }

    public int intake() {
        int best_pos = -1;
        for (int i = 0; i < balls.length; i++) {
            if ((balls[i] == 0) && ((Math.min((i - position) % balls.length, (position - i) % balls.length) < Math.min((best_pos - position) % balls.length, (position - best_pos) % balls.length)) || (best_pos < 0))) {
                best_pos = i;
            }
        }

        int movements;
        if (best_pos < 0) {
            movements = 0;
        }
        else if (Math.abs(best_pos-position) <= balls.length/2) {
            movements = best_pos-position;
        }
        else if (position < balls.length/2) {
            movements = best_pos-position-balls.length;
        }
        else {
            movements = balls.length-best_pos-position;
        }

        manual_rotate(movements);
        return movements;
    }
    public int shoot() {
        int best_pos = -1;
        for (int i = 0; i < balls.length; i++) {
            if ((balls[((i+3)%6)] == pattern[pattern_pos]) && ((Math.min((i - position) % balls.length, (position - i) % balls.length) < Math.min((best_pos - position) % balls.length, (position - best_pos) % balls.length)) || (best_pos < 0))) {
                best_pos = i;
            }
        }

        if (best_pos < 0) {
            for (int i = 0; i < balls.length; i++) {
                if ((balls[((i+3)%6)] > 0) && ((Math.min((i - position) % balls.length, (position - i) % balls.length) < Math.min((best_pos - position) % balls.length, (position - best_pos) % balls.length)) || (best_pos < 0))) {
                    best_pos = i;
                }
            }
            if (best_pos > 0) {
                if (pattern[0] == balls[best_pos]) {
                    pattern_pos = 1;
                }
                else {
                    pattern_pos = 0;
                }
            }
        }
        else {
            pattern_pos += 1;
            if (pattern_pos >= pattern.length) {
                pattern_pos = 0;
            }
        }

        int movements = 0;
        if (best_pos > 0) {
            if (Math.abs(best_pos - position) <= balls.length / 2) {
                movements = best_pos - position;
            }
            else if (position < balls.length / 2) {
                movements = best_pos - position - balls.length;
            }
            else {
                movements = balls.length - best_pos - position;
            }
            manual_rotate(movements);
        }

        return movements;
    }
}