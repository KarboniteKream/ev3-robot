import matplotlib.pyplot as plt
import numpy as np


def read_values(file_name, value_col, join, to_i):
    y = []
    with open(file_name, 'r') as f:
        content = f.readlines()
        i = -1
        for line in content:
            values = line.strip().split(' ')
            if len(values) == 1:
                i += 1
                if to_i == i:
                    break
                elif join:
                    continue
                else:
                    yg.clear()
            else:
                y.append(values[value_col])
    return y


i_forward = 0
join_forwards = True
i_rotation = 0
join_rotations = True

yp = read_values("./Output/PID", 4, join_forwards, i_forward)
yp2 = read_values("./Output/PID", 5, join_forwards, i_forward)
yp3 = read_values("./Output/PID", 6, join_forwards, i_forward)
xp = np.arange(0.0, len(yp) * 0.001, 0.001)

yg = read_values("./Output/GYRO", 2, join_rotations, i_rotation)
xg = np.arange(0.0, len(yg) * 0.001, 0.001)

plt.figure(1)
plt.ylabel('Power')
plt.plot(xp, yp, 'r--', xp, yp2, 'g--', xp, yp3, 'b--')

plt.figure(2)
plt.ylabel('Angle')
plt.plot(xg, yg, 'ko', xg, yg, 'r--')

plt.show()
