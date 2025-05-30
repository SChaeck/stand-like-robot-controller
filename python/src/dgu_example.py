import os
from dynamixel_sdk.controllerXC import *
from dynamixel_sdk.controllerAX import *
import time

"""
Configuration for keyboard interrupt
"""
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


# Initialize parameters
DEVICENAME                  = '/dev/tty.usbserial-FT8ISMU2'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
BAUDRATE                    = 115200
XC1_ID                      = 3
AX1_ID                      = 0
AX2_ID                      = 1
XC_MOVING_STATUS_THRESHOLD  = 10
AX_MOVING_STATUS_THRESHOLD  = 10

# Set port handler
portHandler = PortHandler(DEVICENAME)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

# Define XC motor controller
XC1_controller = ControllerXC(portHandler, BAUDRATE, XC1_ID)
XC1_controller.torque_enable()
XC1_controller.set_position_p_gain(speed_value=600)
xc1_pos = 2048

# Define AX motor controller
AX1_controller = ControllerAX(portHandler, BAUDRATE, AX1_ID)
AX1_controller.torque_enable()
AX1_controller.set_moving_speed(speed_value=100)
ax1_pos = 512

AX2_controller = ControllerAX(portHandler, BAUDRATE, AX2_ID)
AX2_controller.torque_enable()
AX2_controller.set_moving_speed(speed_value=100)
ax2_pos = 512

prev_time = time.time()
ax1_goal_pos = 600
ax2_goal_pos = 820
xc1_goal_pos = 3000

while True :
    now_time = time.time()

    ax1_present_position = AX1_controller.read_current_position()
    ax2_present_position = AX2_controller.read_current_position()
    xc1_present_position = XC1_controller.read_current_position()

    if (now_time - prev_time >= 0.05):
        if (abs(ax1_present_position - ax1_goal_pos) < AX_MOVING_STATUS_THRESHOLD
             and abs(ax2_present_position - ax2_goal_pos) < AX_MOVING_STATUS_THRESHOLD
             and abs(xc1_present_position - xc1_goal_pos) < XC_MOVING_STATUS_THRESHOLD):
            break
        AX1_controller.move(ax1_pos)
        AX2_controller.move(ax2_pos)
        XC1_controller.move(xc1_pos)

        ax1_pos += 5
        ax2_pos += 5
        xc1_pos += 5

        if (ax1_pos >= ax1_goal_pos) :
            ax1_pos = ax1_goal_pos

        if (ax2_pos >= ax2_goal_pos) :
            ax2_pos = ax2_goal_pos

        if (xc1_pos >= xc1_goal_pos) :
            xc1_pos = xc1_goal_pos

        prev_time = now_time


ax1_goal_pos = 512
ax2_goal_pos = 512
xc1_goal_pos = 2048

while True :
    now_time = time.time()

    ax1_present_position = AX1_controller.read_current_position()
    ax2_present_position = AX2_controller.read_current_position()
    xc1_present_position = XC1_controller.read_current_position()

    if (now_time - prev_time >= 0.05):
        if (abs(ax1_present_position - ax1_goal_pos) < AX_MOVING_STATUS_THRESHOLD
             and abs(ax2_present_position - ax2_goal_pos) < AX_MOVING_STATUS_THRESHOLD
             and abs(xc1_present_position - xc1_goal_pos) < XC_MOVING_STATUS_THRESHOLD):
            break
        AX1_controller.move(ax1_pos)
        AX2_controller.move(ax2_pos)
        XC1_controller.move(xc1_pos)

        ax1_pos -= 5
        ax2_pos -= 5
        xc1_pos -= 5

        if (ax1_pos <= ax1_goal_pos) :
            ax1_pos = ax1_goal_pos

        if (ax2_pos <= ax2_goal_pos) :
            ax2_pos = ax2_goal_pos

        if (xc1_pos <= xc1_goal_pos) :
            xc1_pos = xc1_goal_pos

        prev_time = now_time


XC1_controller.torque_disable()
AX1_controller.torque_disable()
AX2_controller.torque_disable()

portHandler.closePort()