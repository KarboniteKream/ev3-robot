import ev3dev.ev3 as ev3
import subprocess
import time


def fast_read(file):
    file.seek(0)
    return int(file.read().decode().strip())


def fast_write(file, value):
    file.truncate(0)
    file.write(str(int(value)))
    file.flush()


def init_motors():
    left = open("ev3devices/outA/duty_cycle_sp", "w")
    right = open("ev3devices/outD/duty_cycle_sp", "w")

    with open("ev3devices/outA/command", "w") as f:
        f.write("reset")
    with open("ev3devices/outD/command", "w") as f:
        f.write("reset")

    time.sleep(LOOP_TIME)

    with open("ev3devices/outA/polarity", "w") as f:
        f.write("inversed")
    with open("ev3devices/outD/polarity", "w") as f:
        f.write("inversed")

    with open("ev3devices/outA/command", "w") as f:
        f.write("run-direct")
    with open("ev3devices/outD/command", "w") as f:
        f.write("run-direct")

    return left, right


def init_gyro():
    g = ev3.GyroSensor()
    assert g.connected

    g.mode = "GYRO-RATE"
    g.mode = "GYRO-ANG"

    g_error = 0.0
    g_error_rate = 0.0

    for _ in range(GYRO_NUM_CALIBRATION_LOOPS):
        g_error_rate += g.value()
        time.sleep(LOOP_TIME)
    g_error_rate /= GYRO_NUM_CALIBRATION_LOOPS

    return g, g_error, g_error_rate


def init_sensors():
    cs = ev3.ColorSensor()
    assert cs.connected
    cs.mode = "COL-REFLECT"
    return cs


def init_reset():
    ts = ev3.TouchSensor()
    assert ts.connected
    return ts


def set_motors(left_speed, right_speed):
    set_motor(LEFT_MOTOR, left_speed)
    set_motor(RIGHT_MOTOR, right_speed)


def set_motor(motor, speed):
    speed = max(-MAX_SPEED, min(speed, MAX_SPEED))
    fast_write(motor, speed)


def stop_motors():
    set_motors(0.0, 0.0)
    time.sleep(BREAK_TIME)


def move():
    global GYRO_ERROR

    base_speed = START_SPEED
    time_acceleration = 0.0
    acceleration_step_speed = ACCELERATION_STEP_SPEED

    angle_goal = 0
    integral = 0.0
    previous_angle_error = 0.0

    current_line = 0
    line = False
    time_line = 0.0

    while True:
        time_start = time.time()

        if RESET.value() == 1:
            break

        if time_acceleration >= ACCELERATION_STEP_TIME and base_speed < SPEED:
            base_speed += acceleration_step_speed
            acceleration_step_speed *= ACCELERATION_STEP_SPEED_FACTOR
            time_acceleration = 0.0

        angle = abs(GYRO.value() % 360) - GYRO_ERROR
        angle_error = angle_goal - angle

        if abs(angle_error) > 180:
            angle_error = (angle_error + 360) if (angle_error < 0) else (angle_error - 360)

        proportion = angle_error
        integral = (I_MOVE_FACTOR * integral) + angle_error
        derivative = angle_error - previous_angle_error
        previous_angle_error = angle_error
        speed_correction = P_MOVE * proportion + I_MOVE * integral + D_MOVE * derivative

        set_motors(base_speed + speed_correction, base_speed - speed_correction)

        color = COLOR.value()

        if not line and color < LINE_COLOR:
            line = True
            time_line = 0.0
            current_line += 1

        if line and time_line > LINE_TIME and color > FLOOR_COLOR:
            if current_line == END_LINE:
                break

            line = False

            if current_line in ROTATION_LINES:
                line = False

                stop_motors()

                base_speed = START_SPEED
                time_acceleration = 0.0
                acceleration_step_speed = ACCELERATION_STEP_SPEED

                angle_goal = 180 if (angle_goal == 0) else 0
                integral = 0.0
                previous_angle_error = 0.0

                if angle_goal == 180:
                    rotate(150)
                elif angle_goal == 0:
                    rotate(30)

                rotate(angle_goal)

                if angle_goal == 180:
                    GYRO_ERROR = 0
                elif angle_goal == 0:
                    GYRO_ERROR = 1.5 * (current_line / 10)

                continue

        time_diff = time.time() - time_start
        if time_diff < LOOP_TIME:
            time.sleep(LOOP_TIME - time_diff)

        time_step = max(time_diff, LOOP_TIME)
        time_acceleration += time_step
        time_line += time_step

        GYRO_ERROR += GYRO_ERROR_RATE

    stop_motors()


def rotate(angle_goal):
    global GYRO_ERROR

    integral = 0.0
    previous_angle_error = 0.0

    while True:
        time_start = time.time()

        if RESET.value() == 1:
            break

        angle = abs(GYRO.value() % 360) - GYRO_ERROR
        angle_error = angle_goal - angle

        if abs(angle_error) > 180:
            angle_error = (angle_error + 360) if (angle_error < 0) else (angle_error - 360)

        if abs(angle_error) <= 1:
            break

        proportion = angle_error
        integral = (I_ROTATE_FACTOR * integral) + angle_error
        derivative = angle_error - previous_angle_error
        previous_angle_error = angle_error
        speed_correction = P_ROTATE * proportion + I_ROTATE * integral + D_ROTATE * derivative

        # FIXME: Rewrite.
        asd = 35.0
        if abs(speed_correction) < asd:
            speed_correction = asd if (speed_correction > 0) else -asd

        # FIXME: Rewrite.
        qwe = 45.0
        if abs(speed_correction) > qwe:
            speed_correction = qwe if (speed_correction > 0) else -qwe

        set_motors(speed_correction, -speed_correction)

        time_diff = time.time() - time_start
        if time_diff < LOOP_TIME:
            time.sleep(LOOP_TIME - time_diff)

        GYRO_ERROR += GYRO_ERROR_RATE

    stop_motors()


LOOP_TIME = 0.01
GYRO_NUM_CALIBRATION_LOOPS = 100

START_SPEED = 20.0
SPEED = 75.0
MAX_SPEED = 100.0

BREAK_TIME = 1.0

ACCELERATION_STEP_TIME = 0.25
ACCELERATION_STEP_SPEED = 4.0
ACCELERATION_STEP_SPEED_FACTOR = 1.0

LINE_COLOR = 15
LINE_TIME = 0.3
FLOOR_COLOR = 20

ROTATION_LINES = [2, 4, 7, 10, 14]
END_LINE = 18

P_MOVE = 1.65
I_MOVE = 0.066
I_MOVE_FACTOR = 1.0
D_MOVE = 10.3125

P_ROTATE = 0.4
I_ROTATE = 0.0
I_ROTATE_FACTOR = 1.0
D_ROTATE = 1.4

subprocess.call(["./makelinks.sh"])

LEFT_MOTOR, RIGHT_MOTOR = init_motors()
GYRO, GYRO_ERROR, GYRO_ERROR_RATE = init_gyro()
COLOR = init_sensors()
RESET = init_reset()

move()
