import math

"""Modeling the elevator subsystem from Austin's workshop."""

kStallTorque = 2.402  # Stall Torque in N m
kStallCurrent = 126.145  # Stall Current in Amps
kFreeSpeed = 5015.562  # Free Speed in RPM
kFreeCurrent = 1.170  # Free Current in Amps
kMass = 20.0  # Mass of the Elevator
kNumMotors = 2.0  # Number of motors
kResistance = 12.0 / kStallCurrent  # Resistance of the motor

Kv = ((kFreeSpeed / 60.0 * 2.0 * math.pi) / (12.0 - kResistance * kFreeCurrent))  # Motor velocity constant
Kt = (kNumMotors * kStallTorque) / kStallCurrent  # Torque constant
kG = 5  # Gear ratio
kr = 17 * 0.25 * 0.0254 / math.pi / 2.0  # Radius of pulley

kDt = 0.01  # Control loop time step

kMinHeight = 0.0
kMaxHeight = 10.0

kMaxVoltage = 12.0
kSafeVoltage = 4.0


def get_acceleration(voltage, velocity):
    return -Kt * kG * kG / (Kv * kResistance * kr * kr * kMass) * velocity + \
           kG * Kt / (kResistance * kr * kMass) * voltage
