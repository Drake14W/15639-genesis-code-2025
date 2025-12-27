import math
import time
import tkinter as tk

class Servo:
    def __init__(self):
        self.target_position = 0.5
        self.position = 0.5
        self._position_deg = 0
    def set_position(self, position):
        if position > 1.0:
            self.target_position = 1.0
        elif position < 0:
            self.target_position = 0
        else:
            self.target_position = position
    def get_position(self):
        return self.position
    def update(self):
        if abs(self.position - self.target_position) > 0.005:
            if self.position < self.target_position:
                self.position += (50/60)*time_interval
            elif self.position > self.target_position:
                self.position -= (50/60)*time_interval

        self._position_deg = servo_range * (self.position - 1/2)

def rotated_square_corners(cx, cy, side, theta):
    """
    Returns the 4 corners of a square after rotation.

    cx, cy : center of square
    side   : side length
    theta  : rotation angle in radians (CCW)
    """

    h = side / 2  # half side length

    # Unrotated corners relative to center
    corners = [
        (-h*1.2, -h),
        ( h*1.2, -h),
        ( h*1.2,  h),
        (-h*1.2,  h)
    ] 

    cos_t = math.cos(theta)
    sin_t = math.sin(theta)

    rotated = []
    for x, y in corners:
        xr = x * cos_t - y * sin_t + cx
        yr = x * sin_t + y * cos_t + cy
        rotated.append((xr, yr))

    return rotated

def sign(num):
    return -1 if num < 0 else 1

class ServoGUI:
    def __init__(self, root, servo):
        self.servo = servo
        self.root = root

        self.canvas_size = 800
        self.arm_length = 50
        self.robot_size = 100
        self.disp_x = 250
        self.disp_y = 250

        self.assumed_servo_rpm = 30
        self.time_needed_s = 0
        self.start_time = time.monotonic()

        self._bearing = 0.0
        self._robot_bearing = 0.0
        self.camera_horizontal_fov = 70.42
        self._robot_rot = 2.5 

        self.center = self.canvas_size // 2
        self.pivot_offset = self.center

        root.title("Cam Servo Visualization")

        self.canvas = tk.Canvas(
            root, width=self.canvas_size, height=self.canvas_size, bg="white"
        )
        self.canvas.pack(pady=10)

        self.root.bind('<KeyPress>', self.rot_handle)

        self.info = tk.Label(root, text="")
        self.info.pack()

        # Draw pivot
        self.canvas.create_oval(
            self.pivot_offset - 5,
            self.canvas_size - self.pivot_offset - 5,
            self.pivot_offset + 5,
            self.canvas_size - self.pivot_offset + 5,
            fill="black",
        )

        self.arm = None
        self.robot = None
        self.target = None
        self.draw_servo()
        self.draw_robot()

    def rot_handle(self, event):
        if event.keysym == "z":
            self._robot_rot += 5
            if self._robot_rot >= 360.0:
                self._robot_rot -= 360.0
        elif event.keysym == "x":
            self._robot_rot -= 5
            if self._robot_rot <= -360.0:
                self._robot_rot += 360.0
        
        if event.keysym == "Up":
            self.disp_y -= 10
        elif event.keysym == "Down":
            self.disp_y += 10
        
        if event.keysym == "Left":
            self.disp_x += 10
        elif event.keysym == "Right":
            self.disp_x -= 10


    def update(self):
        robot_angle = (self._robot_rot + self.servo._position_deg)%360
        #while robot_angle >= 180:
        #    robot_angle -= 360
        #while robot_angle <= -180:
        #    robot_angle += 360
        temp_bearing = -(90 - math.degrees(math.atan2(self.disp_y-10, self.disp_x+10)) - robot_angle)
        while temp_bearing >= 180:
            temp_bearing -= 360
        while temp_bearing <= -180:
            temp_bearing += 360

        print("Temp Bearing:", temp_bearing)
        if abs(temp_bearing) < self.camera_horizontal_fov/2:
            self._bearing = temp_bearing

        servo_pos = self.servo.get_position()
        servo_pos_deg = servo_range * (servo_pos - 1/2)

        print("Bearing:", self._bearing)
        print("Previous Servo Angle:", servo_pos_deg)
        print("Previous Normalized Angle:", self.servo.get_position())

        if abs(self._bearing) <= ((360-servo_range)/2):
            servo_pos = (servo_pos_deg - self._bearing)/servo_range + 1/2
        else:
            servo_pos = -sign(servo_pos_deg)*servo_range/2

        if servo_pos < 0:
            servo_pos = 0
        elif servo_pos > 1.0:
            servo_pos = 1.0
           
        servo_pos_deg = servo_range * (servo_pos - 1/2)

        time_diff = time.monotonic() - self.start_time

        if time_diff > self.time_needed_s:
            self.time_needed_s = abs(self.servo.get_position() - servo_pos)*(60/self.assumed_servo_rpm)
            self.servo.set_position(servo_pos)
            self.start_time = time.monotonic()
        elif abs(self._bearing) < (360-servo_range)/2:
            self.time_needed_s = abs(self.servo.get_position() - servo_pos)*(60/self.assumed_servo_rpm)
            self.servo.set_position(servo_pos)
            self.start_time = time.monotonic()
                     
        try:
            self.servo.update()

            robot_angle = (self._robot_rot + self.servo._position_deg)%360
            #while robot_angle >= 180:
            #    robot_angle -= 360
            #while robot_angle <= -180:
            #    robot_angle += 360
            temp_bearing = -(90 - math.degrees(math.atan2(self.disp_y-10, self.disp_x+10)) - robot_angle)
            while temp_bearing >= 180:
                temp_bearing -= 360
            while temp_bearing <= -180:
                temp_bearing += 360

            if abs(temp_bearing) < self.camera_horizontal_fov/2:
                self._bearing = temp_bearing
               
            self._robot_bearing = servo_pos_deg - self._bearing

            print("New Servo Angle:", self.servo._position_deg)
            print("New Normalized Angle:", self.servo.get_position())
            print("New Temp Bearing:", temp_bearing)
            print("New Bearing:", self._bearing, "\n")

            self.draw_servo()
            self.draw_robot()
        except ValueError:
            pass

    def draw_servo(self):
        if self.arm:
            self.canvas.delete(self.arm)

        angle_deg = self.servo._position_deg + self._robot_rot
        angle_rad = math.radians(angle_deg - 90)  # zero points up

        x = self.pivot_offset + self.arm_length * math.cos(angle_rad)
        y = self.canvas_size - self.pivot_offset + self.arm_length * math.sin(angle_rad)

        self.arm = self.canvas.create_line(
            self.pivot_offset,
            self.canvas_size - self.pivot_offset,
            x,
            y,
            width=4,
            fill="blue",
        )

        self.info.config(
                text=f"Normalized Position: {self.servo.position:.2f} | Robot Angle: {self._robot_rot:.2f}° | Bearing: {self._bearing:.2f}"
        )
    
    def draw_robot(self):
        if self.robot:
            self.canvas.delete(self.robot)

        angle_deg = self._robot_rot
        angle_rad = math.radians(angle_deg - 90)  # zero points up


        x = self.pivot_offset
        y = self.canvas_size - self.pivot_offset

        points = rotated_square_corners(x, y, self.robot_size, angle_rad)

        self.robot = self.canvas.create_polygon(
            points,
            width=2,
            outline="black",
            fill=""
        )

        if self.target:
            for i in self.target:
                self.canvas.delete(i)

        self.target = [self.canvas.create_oval(
            self.pivot_offset + self.disp_x - 10,
            self.canvas_size - self.pivot_offset - self.disp_y - 10,
            self.pivot_offset + self.disp_x + 10,
            self.canvas_size - self.pivot_offset - self.disp_y + 10,
            fill="black",
        ),
            self.canvas.create_line(0, self.canvas_size - self.pivot_offset - self.disp_y, self.canvas_size, self.canvas_size - self.pivot_offset - self.disp_y, width=2, fill="black"),
            self.canvas.create_line(self.pivot_offset + self.disp_x, 0, self.pivot_offset + self.disp_x, self.canvas_size, width=2, fill="black")]


        self.info.config(
                text=f"Normalized Position: {self.servo.position:.2f} | Robot Angle: {self._robot_rot:.2f}° | Bearing: {self._bearing:.2f}°| Robot Bearing: {self._robot_bearing:.2f}°" 
        )

'''
if __name__ == "__main__":
    servo_range = 300.0 #deg
    cam_servo = Servo();
    root = tk.Tk()
    gui = ServoGUI(root, cam_servo)
    time_interval = 0.5
    root.mainloop()
'''
if __name__ == "__main__":
    servo_range = 300.0  # deg
    cam_servo = Servo()
    root = tk.Tk()
    gui = ServoGUI(root, cam_servo)

    time_interval = 0.01  # seconds per update
    running = True

    def custom_mainloop():
        if not running:
            return

        gui.update()              # <-- call every iteration
        root.update_idletasks()   # process pending redraws
        root.update()             # process events

        # schedule next iteration (0 = every loop tick)
        root.after(int(time_interval*1000), custom_mainloop)

    # start custom loop
    root.after(10, custom_mainloop)
    root.mainloop()
