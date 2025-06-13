from dynamixel_sdk import *

PROTOCOL_VERSION1 = 1.0
ADDR_AX_TORQUE_ENABLE = 24
ADDR_AX_GOAL_POSITION = 30
ADDR_AX_PRESENT_POSITION = 36
ADDR_AX_POSITION_P_GAIN  = 32
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

class ControllerAX:
    def __init__(self, port_handler, baudrate, motor_id):
        self.port_handler = port_handler
        self.baudrate = baudrate
        self.motor_id = motor_id

        self.packetHandler = PacketHandler(PROTOCOL_VERSION1)

    def torque_enable(self):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.port_handler, self.motor_id, ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel AX motor ID #%d has been successfully connected" % self.motor_id)

    def torque_disable(self):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.port_handler, self.motor_id, ADDR_AX_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel AX motor ID #%d has been successfully disconnected" % self.motor_id)

    def set_moving_speed(self, speed_value):
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.port_handler, self.motor_id, ADDR_AX_POSITION_P_GAIN, speed_value)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel AX motor ID #%d has been successfully set speed to %d" % (self.motor_id,speed_value))

    def move(self, position):
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.port_handler, self.motor_id, ADDR_AX_GOAL_POSITION, position)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def read_current_position(self):
        ax_present_position, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.port_handler, self.motor_id, ADDR_AX_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        return ax_present_position