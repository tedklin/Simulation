import unittest
import pytest
import numpy as np
import matplotlib.pyplot as plt

import Model
from Controller import PIDController, LowPassFilter


position = 0.0
velocity = 0.0
current_time = 0.0

kEpsilon = 0.0001


def simulate_time(time, voltage):
    global current_time
    global position
    global velocity

    time += current_time
    kSimTime = 0.001
    dt = min(time, kSimTime)
    while (current_time < time):
        position += dt * velocity
        velocity += dt * Model.get_acceleration(voltage, velocity)
        current_time += kSimTime


class SimulationTest(unittest.TestCase):

    def test_PID(self):
        string = "Time, Position, Velocity, Voltage\n"

        global position
        global current_time
        global velocity

        kP = 80
        kD = 0.2
        controller = PIDController(kP, kD)
        controller.goal = 0.5

        last_position = 0.0
        position = 0.0

        for i in range (0, 254):
            voltage = controller.update(position, last_position)
            last_position = position
            simulate_time(Model.kDt, voltage)
            assert position <= Model.kMaxHeight
            assert position >= Model.kMinHeight
            assert voltage <= Model.kMaxVoltage
            assert voltage >= -Model.kMaxVoltage
            string += str(current_time) + "," + str(position) + "," + str(velocity) + "," + str(voltage) + "\n"

        assert position == pytest.approx(position, abs=controller.goal*kEpsilon)

        f = open("output/pd.csv", 'w')
        f.write(string)
        f.close()


    def test_LowPassFilter(self):
        string = "Time, Position, Velocity, Voltage\n"

        global position
        global current_time
        global velocity

        kP = 80
        kD = 0
        kA = 0.6
        controller = LowPassFilter(kP, kD, kA)
        controller.goal = 0.5

        last_position = 0.0
        position = 0.0

        for i in range (0, 254):
            voltage = controller.update(position, last_position)
            last_position = position
            simulate_time(Model.kDt, voltage)
            assert position <= Model.kMaxHeight
            assert position >= Model.kMinHeight
            assert voltage <= Model.kMaxVoltage
            assert voltage >= -Model.kMaxVoltage
            string += str(current_time) + "," + str(position) + "," + str(velocity) + "," + str(voltage) + "\n"

        assert position == pytest.approx(position, abs=controller.goal*kEpsilon)

        f = open("output/low_pass_filter.csv", 'w')
        f.write(string)
        f.close()

