from machine import Pin, SPI
import time
import struct

running = True

# MCP2515 Commands
CMD_RESET = 0xC0
CMD_WRITE = 0x02
CMD_BITMOD = 0x05
CMD_LOAD_TX_BUF = 0x40
CMD_RTS_TX0 = 0x81
CMD_READ = 0x03

CAN_ID_0 = 0x07  #node 0
CAN_ID_1 = 0x27  #node 1
CAN_ID_2 = 0x47  #node 2
CAN_ID_3 = 0x67  #node 3

                           # First Received Buffer RX0  (HIGH priority)

RXB0SIDH = 0x61      #  RECEIVE BUFFER 0 STANDARD IDENTIFIER REGISTER HIGH
RXB0SIDL = 0x62      #  RECEIVE BUFFER 0 STANDARD IDENTIFIER REGISTER LOW
RXB0CTRL = 0x60      #  RXB0CTRL: RECEIVE BUFFER 0 CONTROL REGISTER
RXB0DLC  = 0x65      #  RECEIVE BUFFER 0 DATA LENGTH CODE REGISTER
RXB0Dm   = 0x66      #  RECEIVE BUFFER 0 DATA BYTE m REGISTER

RXB0EID8 = 0x63      #  RECEIVE BUFFER 0 EXTENDED IDENTIFIER REGISTER HIGH


                            # Second Received Buffer RX1

RXB1SIDH = 0x71      #  RECEIVE BUFFER 1 STANDARD IDENTIFIER REGISTER HIGH
RXB1SIDL = 0x72      #  RECEIVE BUFFER 1 STANDARD IDENTIFIER REGISTER LOW
RXB1CTRL = 0x70      #  RXB1CTRL: RECEIVE BUFFER 1 CONTROL REGISTER
RXB1DLC  = 0x75      #  RECEIVE BUFFER 1 DATA LENGTH CODE REGISTER
RXB1Dm   = 0x76      #  RECEIVE BUFFER 1 DATA BYTE m REGISTER

RXB1EID8 = 0x73      #  RECEIVE BUFFER 0 EXTENDED IDENTIFIER REGISTER HIGH

BFPCTRL  = 0x0C      #  RXnBF PIN CONTROL AND STATUS REGISTER


                               # Third Buffer TX2

TXB2SIDH = 0x51      # TRANSMIT BUFFER n STANDARD IDENTIFIER REGISTER HIGH
TXB2SIDL = 0x52      # TRANSMIT BUFFER n STANDARD IDENTIFIER REGISTER LOW
TXB2DLC  = 0x55      # TRANSMIT BUFFER n DATA LENGTH CODE REGISTER
TXB2D0   = 0x56      # TRANSMIT BUFFER n DATA BYTE m REGISTER

                               # Second Buffer TX1

TXB1SIDH = 0x41      # TRANSMIT BUFFER n STANDARD IDENTIFIER REGISTER HIGH
TXB1SIDL = 0x42      # TRANSMIT BUFFER n STANDARD IDENTIFIER REGISTER LOW
TXB1DLC  = 0x45      # TRANSMIT BUFFER n DATA LENGTH CODE REGISTER
TXB1D0   = 0x46      # TRANSMIT BUFFER n DATA BYTE m REGISTER

                                # First Buffer TX0

TXB0SIDH = 0x31      # TRANSMIT BUFFER n STANDARD IDENTIFIER REGISTER HIGH
TXB0SIDL = 0x32      # TRANSMIT BUFFER n STANDARD IDENTIFIER REGISTER LOW
TXB0DLC  = 0x35      # TRANSMIT BUFFER n DATA LENGTH CODE REGISTER
TXB0D0   = 0x36      # TRANSMIT BUFFER n DATA BYTE m REGISTER

CMD_RTS  = 0x80       # Request to send on TXB0

#odrive registers host -> odrive and odrive <- host
CMD_TRAJ_VEL_LIMIT = 0x11
CMD_ACCEL_DECEL = 0x12
CMD_Traj_Inertia = 0x13
CMD_velocity_limit = 0x0f
CMD_Pos_Gain = 0x1a
CMD_Velocity_Gain = 0x1b
CMD_CLEAR_ERROR = 0x18
CMD_Set_Controller_Mode = 0x0b

# SPI Setup
cs = Pin(13, Pin.OUT)
spi = SPI(1, sck=Pin(10), mosi=Pin(11), miso=Pin(12), baudrate=500000)
cs.value(1)

nodes = {
    0: {"encoder": {"position": 0.0, "velocity": 0.0}},
    1: {"encoder": {"position": 0.0, "velocity": 0.0}},
    2: {"encoder": {"position": 0.0, "velocity": 0.0}},
    3: {"encoder": {"position": 0.0, "velocity": 0.0}},
}

# Initial flags
trajectory_done_flag = {
    0: False,
    1: False,
    2: False,
    3: False,
}

def update_trajectory_done_flag(node_id, flag_value):
    if flag_value not in (0, 1):
        print(f"Error: flag_value must be 0 or 1, got {flag_value}")
        return

    if node_id in trajectory_done_flag:
         trajectory_done_flag[node_id] = flag_value
    else:
        print(f"Error: Node ID {node_id} not found.")

def update_nodes(nodes, node_id, encoder):
    if not isinstance(encoder, dict):
        print(f"Error: encoder must be a dict, got {type(encoder)}")
        return
    if node_id in nodes:
        nodes[node_id]["encoder"]['position'] = encoder["position"]
        nodes[node_id]["encoder"]['velocity'] = encoder["velocity"]
    else:
        print(f"Node {node_id} not found.")

def velocity_gain(node_id, vel_gain, vel_integer_gain):
    Cmd_vel_gain = CMD_Velocity_Gain
    can_id = (node_id << 5) | Cmd_vel_gain

    # CAN ID into TXB0SIDH/SIDL (standard ID, 11-bit)
    sid_high = (can_id >> 3) & 0xFF
    sid_low = (can_id & 0x07) << 5

    mcp_write(0x31, bytearray([sid_high]))  # TXB0SIDH
    mcp_write(0x32, bytearray([sid_low]))  # TXB0SIDL

    # Encode the payload: 2 floats (little endian)
    payload = struct.pack("<ff", vel_gain, vel_integer_gain)
    mcp_write(0x36, payload)  # TXB0D0
    mcp_write(0x35, bytearray([8]))  # TXB0DLC (8 bytes)

    # Request to send TXB0
    cs.value(0)
    spi.write(bytearray([CMD_RTS_TX0]))
    cs.value(1)
    time.sleep_ms(10)

def position_gain(node_id, pos_gain):
    Cmd_pos_gain = CMD_Pos_Gain
    can_id = (node_id << 5) | Cmd_pos_gain

    # CAN ID into TXB0SIDH/SIDL (standard ID, 11-bit)
    sid_high = (can_id >> 3) & 0xFF
    sid_low = (can_id & 0x07) << 5

    mcp_write(0x31, bytearray([sid_high]))  # TXB0SIDH
    mcp_write(0x32, bytearray([sid_low]))  # TXB0SIDL

    # Encode the payload: 2 floats (little endian)
    payload = struct.pack("<f", pos_gain)
    mcp_write(0x36, payload)  # TXB0D0
    mcp_write(0x35, bytearray([8]))  # TXB0DLC (8 bytes)

    # Request to send TXB0
    cs.value(0)
    spi.write(bytearray([CMD_RTS_TX0]))
    cs.value(1)
    time.sleep_ms(10)

def vel_current_limit(node_id, velocity_limit, current_limit):
    Cmd_VEL_LIMIT = CMD_velocity_limit
    can_id = (node_id << 5) | Cmd_VEL_LIMIT

    # CAN ID into TXB0SIDH/SIDL (standard ID, 11-bit)
    sid_high = (can_id >> 3) & 0xFF
    sid_low = (can_id & 0x07) << 5

    mcp_write(0x31, bytearray([sid_high]))  # TXB0SIDH
    mcp_write(0x32, bytearray([sid_low]))  # TXB0SIDL

    # Encode the payload: 2 floats (little endian)
    payload = struct.pack("<ff", velocity_limit, current_limit)
    mcp_write(0x36, payload)  # TXB0D0
    mcp_write(0x35, bytearray([8]))  # TXB0DLC (8 bytes)

    # Request to send TXB0
    cs.value(0)
    spi.write(bytearray([CMD_RTS_TX0]))
    cs.value(1)
    time.sleep_ms(10)

def traj_parameter(node_id, cmd_parameter, value):
    Cmd_VEL_LIMIT = cmd_parameter
    can_id = (node_id << 5) | Cmd_VEL_LIMIT

    # CAN ID into TXB0SIDH/SIDL (standard ID, 11-bit)
    sid_high = (can_id >> 3) & 0xFF
    sid_low = (can_id & 0x07) << 5

    mcp_write(0x31, bytearray([sid_high]))  # TXB0SIDH
    mcp_write(0x32, bytearray([sid_low]))   # TXB0SIDL

    # Encode the payload: 2 floats (little endian)
    payload = struct.pack("<f", value)
    mcp_write(0x36, payload)  # TXB0D0
    mcp_write(0x35, bytearray([8]))  # TXB0DLC (8 bytes)

    # Request to send TXB0
    cs.value(0)
    spi.write(bytearray([CMD_RTS_TX0]))
    cs.value(1)
    time.sleep_ms(10)

def traj_acc_dec(node_id, accel, deccel):
    Cmd_LIMIT = CMD_ACCEL_DECEL
    can_id = (node_id << 5) | Cmd_LIMIT

    # CAN ID into TXB0SIDH/SIDL (standard ID, 11-bit)
    sid_high = (can_id >> 3) & 0xFF
    sid_low = (can_id & 0x07) << 5

    mcp_write(0x31, bytearray([sid_high]))  # TXB0SIDH
    mcp_write(0x32, bytearray([sid_low]))   # TXB0SIDL

    # Encode the payload: 2 floats (little endian)
    payload = struct.pack("<ff", accel, deccel)
    mcp_write(0x36, payload)  # TXB0D0
    mcp_write(0x35, bytearray([8]))  # TXB0DLC (8 bytes)

    # Request to send TXB0
    cs.value(0)
    spi.write(bytearray([CMD_RTS_TX0]))
    cs.value(1)
    time.sleep_ms(10)

def mcp_read_register(addr):
    cs.value(0)
    spi.write(bytearray([CMD_READ, addr]))
    val = spi.read(1)[0]
    cs.value(1)
    return val

def mcp_write(addr, data):
    cs.value(0)
    spi.write(bytearray([CMD_WRITE, addr]) + data)
    cs.value(1)

def mcp_reset():
    cs.value(0)
    spi.write(bytearray([CMD_RESET]))
    cs.value(1)
    time.sleep(0.1)

def mcp_bit_modify(addr, mask, data):
    cs.value(0)
    spi.write(bytearray([CMD_BITMOD, addr, mask, data]))
    cs.value(1)

def mcp_init():
    mcp_reset()
    # CNF1, CNF2, CNF3: Set bitrate to 500kbps with 16MHz clock

    # mcp_write(0x2A, bytearray([0x01]))  # CNF1
    # mcp_write(0x29, bytearray([0x90]))  # CNF2
    # mcp_write(0x28, bytearray([0x02]))  # CNF3

    # suitable for longer  can and multi-node systems 500kbps

    mcp_write(0x2A, bytearray([0x01]))  # CNF1: BRP=0, SJW=1 (0b00000000)
    mcp_write(0x29, bytearray([0x80]))  # CNF2: BTLMODE=1, PHSEG1=5, PROPSEG=4
    mcp_write(0x28, bytearray([0x00]))  # CNF3: PHSEG2=5

    # Turn on normal mode (0x0 = Normal mode)

    mcp_bit_modify(0x0F, 0xE0, 0x00)
    time.sleep_us(100)

def send_set_input_position(node_id, pos, vel_ff, cur_ff):
    cmd_id = 0x0C
    can_id = (node_id << 5) | cmd_id

    # CAN ID into TXB0SIDH/SIDL (standard ID, 11-bit)
    sid_high = (can_id >> 3) & 0xFF
    sid_low = (can_id & 0x07) << 5

    mcp_write(0x31, bytearray([sid_high]))  # TXB0SIDH
    mcp_write(0x32, bytearray([sid_low]))   # TXB0SIDL

    # Encode the payload: 3 floats (little endian)
    payload = struct.pack("<fff", pos, vel_ff, cur_ff)
    mcp_write(0x36, payload)  # TXB0D0
    mcp_write(0x35, bytearray([12]))  # TXB0DLC (12 bytes)

    # Request to send TXB0
    cs.value(0)
    spi.write(bytearray([CMD_RTS_TX0]))
    cs.value(1)
    time.sleep_ms(30)

def send_set_input_vel(node_id, vel, torque):
    Cmd_id = 0x0D
    can_id = (node_id << 5) | Cmd_id

    # CAN ID into TXB0SIDH/SIDL (standard ID, 11-bit)
    sid_high = (can_id >> 3) & 0xFF
    sid_low = (can_id & 0x07) << 5

    mcp_write(0x31, bytearray([sid_high]))  # TXB0SIDH
    mcp_write(0x32, bytearray([sid_low]))   # TXB0SIDL

    # Encode the payload: 2 floats (little endian)
    payload = struct.pack("<ff", vel, torque)
    mcp_write(0x36, payload)  # TXB0D0
    mcp_write(0x35, bytearray([8]))  # TXB0DLC (8 bytes)

    # Request to send TXB0
    cs.value(0)
    spi.write(bytearray([CMD_RTS_TX0]))
    cs.value(1)
    time.sleep_ms(10)

def send_absolute_position(node_id, position):
    Cmd_id = 0x19
    can_id = (node_id << 5) | Cmd_id

    # CAN ID into TXB0SIDH/SIDL (standard ID, 11-bit)
    sid_high = (can_id >> 3) & 0xFF
    sid_low = (can_id & 0x07) << 5

    mcp_write(0x31, bytearray([sid_high]))  # TXB0SIDH
    mcp_write(0x32, bytearray([sid_low]))   # TXB0SIDL

    # Encode the payload: 2 floats (little endian)
    payload = struct.pack("<f", position)
    mcp_write(0x36, payload)  # TXB0D0
    mcp_write(0x35, bytearray([8]))  # TXB0DLC (8 bytes)

    # Request to send TXB0
    cs.value(0)
    spi.write(bytearray([CMD_RTS_TX0]))
    cs.value(1)
    time.sleep_ms(10)

def set_control_mode(node_id, input_mode, control_mode):
    Cmd_id = CMD_Set_Controller_Mode
    can_id = (node_id << 5) | Cmd_id

    sid_high = (can_id >> 3) & 0xFF
    sid_low = (can_id & 0x07) << 5

    mcp_write(0x31, bytearray([sid_high]))  # TXB0SIDH
    mcp_write(0x32, bytearray([sid_low]))   # TXB0SIDL

    payload = struct.pack('<II', control_mode, input_mode)
    mcp_write(0x36, payload)
    mcp_write(0x35, bytearray([8]))
    cs.value(0)
    spi.write(bytearray([0x81]))
    cs.value(1)
    time.sleep_ms(10)

                # InputMode                         ControlMode
                # 0x0 INACTIVE                      0x0 VOLTAGE_CONTROL
                # 0x1 PASSTHROUGH                   0x1 TORQUE_CONTROL
                # 0x2 VEL_RAMP                      0x2 VELOCITY_CONTROL
                # 0x3 POS_FILTER                    0x3 POSITION_CONTROL
                # 0x4 MIX_CHANNELS
                # 0x5 TRAP_TRAJ
                # 0x6 TORQUE_RAMP
                # 0x7 MIRROR
                # 0x8 TUNING

def send_calibration_cmd(can_id):

    data = [0x03, 0x00, 0x00, 0x00] + [0x00] * 4


    # --- 1. Write CAN ID to TX buffer
    sid_high = (can_id >> 3) & 0xFF
    sid_low  = (can_id << 5) & 0xE0
    mcp_write(TXB0SIDH, bytearray([sid_high]))
    mcp_write(TXB0SIDL, bytearray([sid_low]))

    # --- 2. Write data length (DLC = 4)
    mcp_write(TXB0DLC, bytearray([len(data)]))

    # --- 3. Write data bytes
    mcp_write(TXB0D0, bytearray(data))

    # --- 4. Send the message
    cs.value(0)
    spi.write(bytearray([CMD_RTS | 0x01]))  # RTS TXB0
    cs.value(1)
    time.sleep(10)

def send_closed_loop_cmd(can_id):

    data = [0x08, 0x00, 0x00, 0x00] + [0x00] * 4

    sid_high = (can_id >> 3) & 0xFF
    sid_low  = (can_id << 5) & 0xE0
    mcp_write(TXB0SIDH, bytearray([sid_high]))
    mcp_write(TXB0SIDL, bytearray([sid_low]))
    mcp_write(TXB0DLC, bytearray([len(data)]))
    mcp_write(TXB0D0, bytearray(data))
    cs.value(0)
    spi.write(bytearray([CMD_RTS | 0x01]))
    cs.value(1)
    time.sleep_ms(20)

def can_id(IDH, IDL, data_list):
    data = bytes(data_list)     # convert to bytes
    slave_id = (IDH >> 2)           #RXBnSIDH first 5 bits
    # print('slave_id', slave_id)
    IDH_2 = (IDH & 0b00000011) << 3   #RXBnSIDH last 2 bits
    IDL_3 = (IDL >> 5)               # RXBnSIDL first 3 bytes
    CMD_ID = IDH_2 + IDL_3
    # print('CMD_ID', hex(CMD_ID))


    if CMD_ID == 0x17:
        bus_voltage, bus_current = struct.unpack('<ff', data)
        print(f"Bus Voltage: {bus_voltage:.2f} V")
        print(f"Bus Current: {bus_current:.2f} A")

    if CMD_ID == 0x15:
        fet_temp, motor_temp  = struct.unpack('<ff', data)
        print(f"FET_Temperature: {fet_temp:.2f} °C")
        print(f"Bus Current: {motor_temp:.2f} °C")

    if CMD_ID == 0x14:
        Iq_Setpoint, Iq_Measured  = struct.unpack('<ff', data)
        print(f"Iq_Setpoint: {Iq_Setpoint:.2f} ")
        print(f"Iq_Measured: {Iq_Measured:.2f} ")

    if CMD_ID == 0x03:
        active_errors, disarm_reason = struct.unpack('<II', data)
        print(f"Active Errors: {bin(active_errors)}")
        print(f"Disarm Reason: {bin(disarm_reason)}")

    if CMD_ID == 0x09:
        position = struct.unpack('<f', data[:4])[0]  # Decode first 4 bytes as float (position)
        velocity = struct.unpack('<f', data[4:])[0]  # Decode next 4 bytes as float (velocity)
        # print("Position:", position)
        # print("Velocity:", velocity)
        encoder = {
            "position": position,
            "velocity": velocity,
        }
        return slave_id, encoder

    if CMD_ID == 0x1c:
        torque_target, torque_estimate = struct.unpack('<ff', data)

        print(f"Torque Target: {torque_target}")
        print(f"Torque Estimate: {torque_estimate}")

def heartbeat(IDH, IDL, data_list):
    data = bytes(data_list)  # convert to bytes
    slave_id = (IDH >> 2)  # RXBnSIDH first 5 bits
    # print('slave_id', slave_id)
    IDH_2 = (IDH & 0b00000011) << 3  # RXBnSIDH last 2 bits
    IDL_3 = (IDL >> 5)  # RXBnSIDL first 3 bytes
    CMD_ID = IDH_2 + IDL_3
    # print('CMD_ID', hex(CMD_ID))
    if CMD_ID == 0x01:   # Heartbeat
        # axis_error, axis_state, proc_result, traj_done_flag = struct.unpack('<HBBI', data)
        axis_error, axis_state, proc_result, traj_done_flag = struct.unpack('<IBBH', data)
        # print("Axis Error:", axis_error)
        # print("Axis State:", axis_state)
        # print("Procedure Result:", proc_result)
        print("Trajectory Done Flag:", traj_done_flag)
        print("slave_id", slave_id)

        return slave_id, traj_done_flag

def canintf():

    canintf_reg = mcp_read_register(0x2C)

    # print('canintf', hex(canintf))
    if canintf_reg & 0x01:  # RX0IF bit set (message in RXB0)
        sidh = mcp_read_register(RXB0SIDH)
        sidl = mcp_read_register(RXB0SIDL)
        dlc = mcp_read_register(RXB0DLC)
        data = [mcp_read_register(RXB0Dm + i) for i in range(dlc)]
        # slave_id, position = can_id(sidh, sidl, data)
        result = can_id(sidh, sidl, data)

        if result is not None:
            slave_id, encoder = result


            return slave_id, encoder

        mcp_bit_modify(0x2C, 0x21, 0x00)          # Clear TX0IF and RX0IF (bits 5 and 0)

        # result_2 = heartbeat(sidh, sidl, data)
        # if result_2 is not None:
        #     slave_id, traj_flag = result_2
        #
        #     return slave_id, traj_flag
        #
        # mcp_bit_modify(0x2C, 0x21, 0x00)          # Clear TX0IF and RX0IF (bits 5 and 0)

    # if canintf_reg & 0x02:  # RX1IF bit set (message in RXB1)
    #     sidh = mcp_read_register(RXB1SIDH)
    #     sidl = mcp_read_register(RXB1SIDL)
    #     dlc = mcp_read_register(RXB1DLC)
    #     data = [mcp_read_register(RXB1Dm + i) for i in range(dlc)]
    #     # slave_id, position = can_id(sidh, sidl, data)
    #     result2 = can_id(sidh, sidl, data)
    #     print('result2', result2)
    #     mcp_bit_modify(0x2C, 0x02, 0x00)
    #
    time.sleep_ms(20)

def canintf1():

    canintf_reg = mcp_read_register(0x2C)

    # print('canintf', hex(canintf))
    if canintf_reg & 0x01:  # RX0IF bit set (message in RXB0)
        sidh = mcp_read_register(RXB0SIDH)
        sidl = mcp_read_register(RXB0SIDL)
        dlc = mcp_read_register(RXB0DLC)
        data = [mcp_read_register(RXB0Dm + i) for i in range(dlc)]

        result_2 = heartbeat(sidh, sidl, data)
        if result_2 is not None:
            slave_id, traj_flag = result_2

            return slave_id, traj_flag

        mcp_bit_modify(0x2C, 0x21, 0x00)          # Clear TX0IF and RX0IF (bits 5 and 0)

    time.sleep_ms(20)

def clear_error(node_id):
    Cmd_clear_error = CMD_CLEAR_ERROR
    can_id = (node_id << 5) | Cmd_clear_error

    # CAN ID into TXB0SIDH/SIDL (standard ID, 11-bit)
    sid_high = (can_id >> 3) & 0xFF
    sid_low = (can_id & 0x07) << 5

    mcp_write(0x31, bytearray([sid_high]))  # TXB0SIDH
    mcp_write(0x32, bytearray([sid_low]))  # TXB0SIDL

    clear_err = [0x00, 0x00, 0x00, 0x00] + [0x00] * 4

    mcp_write(0x36, clear_err)  # TXB0D0
    mcp_write(0x35, bytearray([8]))  # TXB0DLC (8 bytes)

    # Request to send TXB0
    cs.value(0)
    spi.write(bytearray([CMD_RTS_TX0]))
    cs.value(1)
    time.sleep_ms(10)

mcp_reset()
mcp_init()


# send_calibration_cmd(CAN_ID_0)
# send_calibration_cmd(CAN_ID_1)
# send_calibration_cmd(CAN_ID_2)
# send_calibration_cmd(CAN_ID_3)


send_closed_loop_cmd(CAN_ID_0)
send_closed_loop_cmd(CAN_ID_1)
send_closed_loop_cmd(CAN_ID_2)
send_closed_loop_cmd(CAN_ID_3)

set_control_mode(0, 0x5, 0x3)
set_control_mode(1, 0x5, 0x3)
set_control_mode(2, 0x5, 0x3)
set_control_mode(3, 0x5, 0x3)

traj_parameter(3, CMD_TRAJ_VEL_LIMIT, 3)
traj_parameter(3, CMD_Traj_Inertia, 0.001)
traj_acc_dec(3, 3, 3)

traj_parameter(2, CMD_TRAJ_VEL_LIMIT, 3)

traj_parameter(2, CMD_Traj_Inertia, 0.001)
traj_acc_dec(2, 3, 3)

traj_parameter(1, CMD_TRAJ_VEL_LIMIT, 3)
traj_parameter(1, CMD_Traj_Inertia, 0.001)
traj_acc_dec(1, 3, 3)

traj_parameter(0, CMD_TRAJ_VEL_LIMIT, 3)
traj_parameter(0, CMD_Traj_Inertia, 0.001)
traj_acc_dec(0, 3, 3)

# clear_error(1)
# clear_error(2)

while True:
    #
    # heartbeat1 = canintf1()
    # if heartbeat1 is not None:
    #     node_id, traj_flag = heartbeat1
    #     update_trajectory_done_flag(node_id, traj_flag)
    #
    #     print(f"trajectory_done_flag_0 = {trajectory_done_flag[0]}")
    #     print(f"trajectory_done_flag_1 = {trajectory_done_flag[1]}")
    #     print(f"trajectory_done_flag_2 = {trajectory_done_flag[2]}")
    #     print(f"trajectory_done_flag_3 = {trajectory_done_flag[3]}")
    #
    #
    #     for node_id, flag in trajectory_done_flag.items():  # for printing
    #         print(f"Node {node_id}: Trajectory Done = {flag}")
    #
    #     mcp_bit_modify(0x2C, 0x21, 0x00)

    Packet = canintf()
    if Packet is not None:
        Node_id, encoder = Packet
        update_nodes(nodes, Node_id, encoder)

        # for node_id, info in nodes.items():
        #     enc = info["encoder"]
        #     print(f"Node {node_id}: Position = {enc['position']}, Velocity = {enc['velocity']}")

        position_0 = round(nodes[0]["encoder"]["position"], 0)
        position_1 = round(nodes[1]["encoder"]["position"], 0)
        position_2 = round(nodes[2]["encoder"]["position"], 0)
        position_3 = round(nodes[3]["encoder"]["position"], 0)

        print('position_0', position_0)
        print('position_1', position_1)
        print('position_2', position_2)
        print('position_3', position_3)
        mcp_bit_modify(0x2C, 0x21, 0x00)

        send_set_input_position(0, 20, 0.0, 0.0)  # left front motor
        send_set_input_position(1, -20, 0.0, 0.0)  # right front motor
        send_set_input_position(2, 20, 0.0, 0.0)  # left back motor
        send_set_input_position(3, -20, 0.0, 0.0)  # right back motor
        print('go to loop')

        if (position_0 == 20 and position_1 == -20 and position_2 == 20 and position_3 == -20):
            done = False
            while (not done) and position_0 !=0: #and position_1 !=0 and position_2 !=0 and position_3 != 0
                send_set_input_position(0, 0, 0.0, 0.0)
                send_set_input_position(1, 0, 0.0, 0.0)
                send_set_input_position(2, 0, 0.0, 0.0)
                send_set_input_position(3, 0, 0.0, 0.0)

                # heartbeat1 = canintf1()
                # if heartbeat1 is not None:
                #     node_id, traj_flag = heartbeat1
                #     update_trajectory_done_flag(node_id, traj_flag)

                    # for node_id, flag in trajectory_done_flag.items():  # for printing
                    #     print(f"Node {node_id}: Trajectory Done = {flag}")

                Packet = canintf()
                if Packet is not None:
                    Node_id, encoder = Packet
                    update_nodes(nodes, Node_id, encoder)

                    # for node_id, info in nodes.items():
                    #     enc = info["encoder"]
                    #     print(f"Node {node_id}: Position = {enc['position']}, Velocity = {enc['velocity']}")

                    position_0 = round(nodes[0]["encoder"]["position"], 0)
                    position_1 = round(nodes[1]["encoder"]["position"], 0)
                    position_2 = round(nodes[2]["encoder"]["position"], 0)
                    position_3 = round(nodes[3]["encoder"]["position"], 0)


                    if (
                            position_0 == 0 and
                            position_1 == 0 and
                            position_2 == 0 and
                            position_3 == 0
                    ):
                        done = True
                mcp_bit_modify(0x2C, 0x21, 0x00)

                # if all(trajectory_done_flag[i] == 1 for i in range(4)):
                #     done = True
