import Model

"""Contains different types of controllers to move to setpoint."""

class PIDController:
    """PD Control"""
    _goal = 0.0

    def __init__(self, kP, kD):
        """Create a new controller."""
        self.kP = kP
        self.kD = kD

    @property
    def goal(self):
        return self._goal

    @goal.setter
    def goal(self, value):
        self._goal = value
        if value < Model.kMinHeight:
            self._goal = Model.kMinHeight
        elif value > Model.kMaxHeight:
            self._goal = Model.kMaxHeight

    def update(self, current_encoder_pos, last_encoder_pos):
        current_error = self.goal - current_encoder_pos
        last_error = self.goal - last_encoder_pos
        voltage = self.kP * current_error + self.kD * (current_error - last_error) / Model.kDt
        if voltage < -Model.kMaxVoltage:
            voltage = -Model.kMaxVoltage
        elif voltage > Model.kMaxVoltage:
            voltage = Model.kMaxVoltage
        return voltage


class LowPassFilter:
    """Low Pass Filter PD Control"""
    _goal = 0.0
    _intermediate_goal = 0.0

    def __init__(self, kP, kD, kA):
        """Create a new controller."""
        self.kP = kP
        self.kD = kD
        self.kA = kA

    @property
    def goal(self):
        return self._goal

    @goal.setter
    def goal(self, value):
        self._goal = value
        if value < Model.kMinHeight:
            self._goal = Model.kMinHeight
        elif value > Model.kMaxHeight:
            self._goal = Model.kMaxHeight

    @property
    def intermediate_goal(self):
        return self._intermediate_goal

    @intermediate_goal.setter
    def intermediate_goal(self, value):
        self._intermediate_goal = value
        if value < Model.kMinHeight:
            self._intermediate_goal = Model.kMinHeight
        elif value > Model.kMaxHeight:
            self._intermediate_goal = Model.kMaxHeight

    def update(self, current_encoder_pos, last_encoder_pos):
        self.intermediate_goal = self.goal * self.kA + last_encoder_pos * (1 - self.kA)
        current_error = self.intermediate_goal - current_encoder_pos
        last_error = self.intermediate_goal - last_encoder_pos
        voltage = self.kP * current_error + self.kD * (current_error - last_error) / Model.kDt
        if voltage < -Model.kMaxVoltage:
            voltage = -Model.kMaxVoltage
        elif voltage > Model.kMaxVoltage:
            voltage = Model.kMaxVoltage
        return voltage


class ProfiledController:
    """Motion Profiled Control"""
    _goal = 0.0

    def __init__(self, max_vel, max_acc, kP, kD):
        """Create a new controller."""
        self.max_vel = max_vel
        self.max_acc = max_acc
        self.kP = kP
        self.kD = kD

    @property
    def goal(self):
        return self._goal

    @goal.setter
    def goal(self, value):
        self._goal = value
        if value < Model.kMinHeight:
            self._goal = Model.kMinHeight
        elif value > Model.kMaxHeight:
            self._goal = Model.kMaxHeight

    def update(self, timestamp, current_encoder_pos, last_encoder_pos):
        current_error = self.goal - current_encoder_pos
        last_error = self.goal - last_encoder_pos
        voltage = self.kP * current_error + self.kD * (current_error - last_error) / Model.kDt
        if voltage < -Model.kMaxVoltage:
            voltage = -Model.kMaxVoltage
        elif voltage > Model.kMaxVoltage:
            voltage = Model.kMaxVoltage
        return voltage
