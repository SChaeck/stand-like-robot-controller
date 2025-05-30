import os
import yaml

from dynamixel_sdk.controllerXC import *
from dynamixel_sdk.controllerAX import *

class StandLikeRobot:
    def __init__(
        self, 
        port_address='/dev/tty.usbserial-FT8ISMU2', 
        metadata_path='stand_like_robot.yaml'
    ):
        self.port_handler = PortHandler(port_address)
        self.metadata = yaml.load(open(metadata_path, 'r'))
        self.init_joint_positions = [
            motor_info['initial_position']
            for motor_info in self.metadata['motors'].values()
        ]
        self.expected_joint_positions = self.init_joint_positions.copy()
        
        if os.name == 'nt':
            import msvcrt
            def getch():
                return msvcrt.getch().decode()
        else:
            import sys, tty, termios
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            def getch():
                try:
                    tty.setraw(sys.stdin.fileno())
                    ch = sys.stdin.read(1)
                finally:    
                    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                return ch
        
        # Open port
        if port_handler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

        # Set port baudrate
        if port_handler.setBaudRate(self.metadata['general_settings']['baudrate']):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()
            
        # Initialize motor controllers
        self.motor_controllers = [None] * len(self.metadata['motors'])
        for motor_name, motor_info in self.metadata['motors'].items():
            motor_id = motor_info['id']
            if motor_name.startswith('AX'):
                motor_controller = ControllerAX(self.port_handler, self.metadata['general_settings']['baudrate'], motor_id)
                motor_controller.torque_enable()
                motor_controller.set_moving_speed(speed_value=motor_info['speed'])
            elif motor_name.startswith('XC'):
                motor_controller = ControllerXC(self.port_handler, self.metadata['general_settings']['baudrate'], motor_id)
                motor_controller.torque_enable()
                motor_controller.set_position_p_gain(speed_value=motor_info['speed'])
            else:
                raise ValueError(f"Invalid motor name: {motor_name}")
            self.motor_controllers[motor_id] = motor_controller
        
    def get_current_joint_position(self):
        return [
            motor_controller.read_current_position()
            for motor_controller in self.motor_controllers
        ]   

    def reset(self):
        # 현재는 바로 이동. Trjaectory planning 필요.
        for motor_id, motor_controller in enumerate(self.motor_controllers):
            motor_controller.move(position=self.init_joint_positions[motor_id])

    # move를 감싸서 tarjectory planning을 하는 함수
    def 
            
            
            
            
            