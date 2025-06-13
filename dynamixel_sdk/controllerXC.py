from dynamixel_sdk import *

PROTOCOL_VERSION2 = 2.0
ADDR_XC_TORQUE_ENABLE = 64
ADDR_XC_GOAL_POSITION = 116
ADDR_XC_PRESENT_POSITION = 132
ADDR_XC_POSITION_P_GAIN  = 84
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

class ControllerXC:
    def __init__(self, port_handler, baudrate, motor_id):
        self.port_handler = port_handler
        self.baudrate = baudrate
        self.motor_id = motor_id

        self.packetHandler = PacketHandler(PROTOCOL_VERSION2)

    def torque_enable(self):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.port_handler, self.motor_id, ADDR_XC_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel XC motor ID #%d has been successfully connected" % self.motor_id)

    def torque_disable(self):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.port_handler, self.motor_id, ADDR_XC_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel XC motor ID #%d has been successfully disconnected" % self.motor_id)

    def set_position_p_gain(self, speed_value):
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.port_handler, self.motor_id, ADDR_XC_POSITION_P_GAIN, speed_value)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel XC motor ID #%d has been successfully set P gain to %d" % (self.motor_id,speed_value))

    def move(self, position):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.port_handler, self.motor_id, ADDR_XC_GOAL_POSITION, position)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def read_current_position(self):
        xc_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.port_handler, self.motor_id, ADDR_XC_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        return xc_present_position