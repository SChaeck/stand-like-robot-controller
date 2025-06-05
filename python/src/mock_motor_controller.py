"""
Mock Motor Controller for simulation mode
"""

class MockMotorController:
    """Mock motor controller for simulation mode"""
    def __init__(self, motor_name, initial_position):
        self.motor_name = motor_name
        self.current_position = initial_position
        self.target_position = initial_position
        
    def read_current_position(self):
        """Return simulated current position"""
        return self.current_position
    
    def move(self, position):
        """Simulate motor movement"""
        self.target_position = position
        self.current_position = position  # Instant movement in simulation
    
    def set_goal_position(self, position):
        """Simulate setting goal position (alias for move)"""
        self.move(position)
        
    def torque_enable(self):
        """Mock torque enable"""
        print(f"MOCK: {self.motor_name} torque enabled")
        
    def set_moving_speed(self, speed_value):
        """Mock set moving speed"""
        print(f"MOCK: {self.motor_name} speed set to {speed_value}")
        
    def set_position_p_gain(self, speed_value):
        """Mock set position P gain"""
        print(f"MOCK: {self.motor_name} P gain set to {speed_value}") 