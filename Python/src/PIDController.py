import Model


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
