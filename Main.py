import ev3dev.ev3 as ev3
import subprocess
import sys
import time


def FastRead(infile):
    infile.seek(0)
    return int(infile.read().decode().strip())


def FastWrite(outfile, value):
    outfile.truncate(0)
    outfile.write(str(int(value)))
    outfile.flush()


def init_motors():
    left = ev3.LargeMotor("outA")
    right = ev3.LargeMotor("outD")

    left.reset()
    right.reset()

    left.polarity = "inversed"
    right.polarity = "inversed"

    return left, right


def init_gyro():
    g = ev3.GyroSensor()
    assert g.connected

    g.mode = "GYRO-RATE"
    g.mode = "GYRO-ANG"

    g_error_rate = 0.0

    for _ in range(GYRO_NUM_CALIBRATION_LOOPS):
        g_error_rate += g.value()
        time.sleep(LOOP_TIME)
    g_error_rate /= GYRO_NUM_CALIBRATION_LOOPS

    return g, 0.0, g_error_rate


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
    set_motor(LEFT, left_speed)
    set_motor(RIGHT, right_speed)


def set_motor(motor, speed):
    speed = max(-MAX_SPEED, min(speed, MAX_SPEED))
    FastWrite(motor, speed)
    # motor.duty_cycle_sp = speed
    # motor.run_direct()


def stop_motors():
    set_motors(0.0, 0.0)
    # LEFT_MOTOR.stop(stop_action="brake")
    # RIGHT_MOTOR.stop(stop_action="brake")
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

        left_speed = base_speed
        right_speed = base_speed

        # angle = abs(GYRO.value() % 360) - GYRO_ERROR
        angle = abs(GYRO.value() % 360) - GYRO_ERROR
        F.write(str("%.2f\n" % angle))
        angle_error = angle_goal - angle

        if abs(angle_error) > 180:
            angle_error = (angle_error + 360) if (angle_error < 0) else (angle_error - 360)

        proportion = angle_error
        integral = (I_MOVE_FACTOR * integral) + angle_error
        derivative = angle_error - previous_angle_error
        previous_angle_error = angle_error
        speed_correction = P_MOVE * proportion + I_MOVE * integral + D_MOVE * derivative

        left_speed += speed_correction
        right_speed -= speed_correction

        set_motors(left_speed, right_speed)

        # print("%.2f %.2f %.2f %.2f %.2f %.2f" %
        #       (angle_error, integral, derivative, speed_correction, left_speed, right_speed))

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
                    GYRO_ERROR = 2 * (current_line / 5)
                elif angle_goal == 0:
                    GYRO_ERROR = 3 * (current_line / 5)

                continue

        time_diff = time.time() - time_start
        if time_diff < LOOP_TIME:
            time.sleep(LOOP_TIME - time_diff)

        time_step = max(time_diff, LOOP_TIME)
        time_acceleration += time_step
        time_line += time_step

        GYRO_ERROR += GYRO_ERROR_RATE

    stop_motors()


LAST_DIRECTION = -1

def rotate(angle_goal):
    global GYRO_ERROR
    global LAST_DIRECTION

    integral = 0.0
    previous_angle_error = 0.0

    success = 0
    direction = 0

    # angle = abs(GYRO.value() % 360) - GYRO_ERROR
    # angle = abs(GYRO.value() % 360) - GYRO_ERROR
    # angle_error = angle_goal - angle

    # if abs(angle_error) > 180:
    #     angle_error = (angle_error + 360) if (angle_error < 0) else (angle_error - 360)
    #
    # if angle_error < 0:
    #     direction = 1

    while True:
        time_start = time.time()

        if RESET.value() == 1:
            break

        angle = abs(GYRO.value() % 360) - GYRO_ERROR
        angle_error = angle_goal - angle

        if abs(angle_error) > 180:
            angle_error = (angle_error + 360) if (angle_error < 0) else (angle_error - 360)

        proportion = angle_error
        integral = (I_ROTATE_FACTOR * integral) + angle_error
        derivative = angle_error - previous_angle_error
        previous_angle_error = angle_error
        speed_correction = P_ROTATE * proportion + I_ROTATE * integral + D_ROTATE * derivative

        asd = 35.0
        if abs(speed_correction) < asd:
            speed_correction = asd if (speed_correction > 0) else -asd

        qwe = 50.0
        if abs(speed_correction) > qwe:
            speed_correction = qwe if (speed_correction > 0) else -qwe

        F.write("%.2f %.2f %.2f %.2f %.2f\n" %
                (angle, angle_error, integral, derivative, speed_correction))

        # TODO: Move up.
        if abs(angle_error) < 2:
            break
            success += 1
        else:
            success = 0

        if success == 5:
            # F.write("%.2f\n" % GYRO.value())
            # stop_motors()
            # F.write("%.2f\n" % GYRO.value())
            # sys.exit()
            break

        left_speed = speed_correction
        right_speed = -speed_correction

        set_motors(left_speed, right_speed)

        time_diff = time.time() - time_start
        if time_diff < LOOP_TIME:
            time.sleep(LOOP_TIME - time_diff)

        # time_step = max(time_diff, LOOP_TIME)

        GYRO_ERROR += GYRO_ERROR_RATE

    if LAST_DIRECTION == -1:
        if direction == 1:
            GYRO_ERROR += GYRO_MANUAL_CORRECTION
        else:
            GYRO_ERROR -= GYRO_MANUAL_CORRECTION
    elif LAST_DIRECTION == direction:
        if direction == 1:
            GYRO_ERROR += GYRO_MANUAL_CORRECTION
        else:
            GYRO_ERROR -= GYRO_MANUAL_CORRECTION

    LAST_DIRECTION = direction

    stop_motors()


LOOP_TIME = 0.01
GYRO_NUM_CALIBRATION_LOOPS = 100

START_SPEED = 20.0
SPEED = 90.0
MAX_SPEED = 100.0

BREAK_TIME = 1.0

ACCELERATION_STEP_TIME = 0.25
ACCELERATION_STEP_SPEED = 5.0
ACCELERATION_STEP_SPEED_FACTOR = 1.0

LINE_COLOR = 10
LINE_TIME = 0.25
FLOOR_COLOR = 20

ROTATION_LINES = [2, 4, 7, 10, 14]
# ROTATION_LINES = [4, 8]
END_LINE = 18

# P_MOVE = 1.2
# I_MOVE = 0.1
# I_MOVE_FACTOR = 0.75
# D_MOVE = 0.5

P_MOVE = 1.65
I_MOVE = 0.066
I_MOVE_FACTOR = 1.0
D_MOVE = 10.3125

# P_MOVE = 3.0
# I_MOVE = 0.2
# I_MOVE_FACTOR = 0.8
# D_MOVE = 5.0

GYRO_MANUAL_CORRECTION = 0

# Kc = 5.0
# wobble = 0.25
#
# P_MOVE = 0.6 * Kc
# I_MOVE = 2 * P_MOVE * LOOP_TIME / wobble
# I_MOVE_FACTOR = 0.75
# D_MOVE = P_MOVE * wobble / (8 * LOOP_TIME)

print(P_MOVE)
print(I_MOVE)
print(D_MOVE)

P_ROTATE = 0.5
I_ROTATE = 0.0
I_ROTATE_FACTOR = 1.0
D_ROTATE = 1.0

LEFT_MOTOR, RIGHT_MOTOR = init_motors()
GYRO, GYRO_ERROR, GYRO_ERROR_RATE = init_gyro()
print(GYRO_ERROR_RATE)
# GYRO_ERROR_RATE = 0.003
COLOR = init_sensors()
RESET = init_reset()

subprocess.call(['./makelinks.sh'])

F = open("./Output/PID", "w")

LEFT = open("ev3devices/outA/duty_cycle_sp", "w")
RIGHT = open("ev3devices/outD/duty_cycle_sp", "w")

# Reset the motors
with open('ev3devices/outA/command', 'w') as f:
    f.write('reset')
with open('ev3devices/outD/command', 'w') as f:
    f.write('reset')
time.sleep(0.01)

# Set motors in run-direct mode
with open('ev3devices/outA/polarity', 'w') as f:
    f.write('inversed')
with open('ev3devices/outD/polarity', 'w') as f:
    f.write('inversed')
time.sleep(0.01)

# Set motors in run-direct mode
with open('ev3devices/outA/command', 'w') as f:
    f.write('run-direct')
with open('ev3devices/outD/command', 'w') as f:
    f.write('run-direct')

move()

# rotate(90)
# rotate(180)
# stop_motors()
# rotate(90)
# rotate(0)
# stop_motors()
# rotate(90)
# rotate(180)
# stop_motors()
# rotate(90)
# rotate(0)
# stop_motors()
# rotate(90)
# rotate(180)
# stop_motors()
# rotate(90)
# rotate(0)
# stop_motors()

F.close()
