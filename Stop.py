import ev3dev.ev3 as ev3

LEFT_MOTOR = ev3.LargeMotor("outA")
RIGHT_MOTOR = ev3.LargeMotor("outD")

LEFT_MOTOR.stop(stop_action="brake")
RIGHT_MOTOR.stop(stop_action="brake")

LEFT_MOTOR.reset()
RIGHT_MOTOR.reset()
