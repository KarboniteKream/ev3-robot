import ev3dev.ev3 as ev3
import time


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

    time.sleep(GYRO_NUM_CALIBRATION_LOOPS * LOOP_TIME)
    g_error = g.value()
    g_error_rate = g_error / GYRO_NUM_CALIBRATION_LOOPS

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
    speed = max(MIN_SPEED, min(speed, MAX_SPEED))
    motor.duty_cycle_sp = speed
    motor.run_direct()


def stop_motors():
    set_motors(0.0, 0.0)
    LEFT_MOTOR.stop(stop_action="brake")
    RIGHT_MOTOR.stop(stop_action="brake")
    time.sleep(BREAK_TIME)


def move_forward():
    global ACCELERATION_STEP_SPEED
    global GYRO_ERROR

    base_speed = START_SPEED
    time_acceleration = 0.0
    angle_goal = 0
    integral = 0.0
    previous_angle_error = 0.0
    on_line = False

    while True:
        time_start = time.time()

        if RESET.value() == 1:
            break

        if time_acceleration >= ACCELERATION_STEP_TIME and base_speed < SPEED:
            base_speed += ACCELERATION_STEP_SPEED
            ACCELERATION_STEP_SPEED *= ACCELERATION_STEP_SPEED_FACTOR
            time_acceleration = 0.0

        left_speed = base_speed
        right_speed = base_speed

        angle = abs(GYRO.value() % 360) - GYRO_ERROR
        angle_error = angle_goal - angle

        if abs(angle_error) > 180:
            angle_error = (angle_error + 360) if (angle_error < 0) else (angle_error - 360)

        proportion = angle_error
        # TODO: Should 'integral' be limited?
        integral = (I_FORWARD_FACTOR * integral) + angle_error
        derivative = angle_error - previous_angle_error
        previous_angle_error = angle_error
        speed_correction = P_FORWARD * proportion + I_FORWARD * integral + D_FORWARD * derivative

        left_speed += speed_correction
        right_speed -= speed_correction

        set_motors(left_speed, right_speed)

        print("%.2f %.2f %.2f %.2f %.2f %.2f" %
              (angle_error, integral, derivative, speed_correction, left_speed, right_speed))

        color = COLOR.value()

        if not on_line and color < LINE_COLOR:
            on_line = True
        elif on_line and color > FLOOR_COLOR:
            on_line = False

        time_diff = time.time() - time_start
        if time_diff < LOOP_TIME:
            time.sleep(LOOP_TIME - time_diff)

        time_step = max(time_diff, LOOP_TIME)
        time_acceleration += time_step

        GYRO_ERROR += GYRO_ERROR_RATE


LOOP_TIME = 0.01
GYRO_NUM_CALIBRATION_LOOPS = 500

MIN_SPEED = 10.0
START_SPEED = 30.0
SPEED = 90.0
MAX_SPEED = 100.0

BREAK_TIME = 0.5

ACCELERATION_STEP_TIME = 1.0
ACCELERATION_STEP_SPEED = 5.0
ACCELERATION_STEP_SPEED_FACTOR = 1.0

LINE_COLOR = 10
FLOOR_COLOR = 20

P_FORWARD = 1.0
I_FORWARD = 0.1
I_FORWARD_FACTOR = 0.75
D_FORWARD = 0.5

LEFT_MOTOR, RIGHT_MOTOR = init_motors()
GYRO, GYRO_ERROR, GYRO_ERROR_RATE = init_gyro()
COLOR = init_sensors()
RESET = init_reset()

move_forward()
stop_motors()
