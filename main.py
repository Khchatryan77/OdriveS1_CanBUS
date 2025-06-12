from machine import Pin, SPI
import time
import struct

# MCP2515 Commands
CMD_RESET = 0xC0
CMD_WRITE = 0x02
CMD_BITMOD = 0x05
CMD_LOAD_TX_BUF = 0x40
CMD_RTS_TX0 = 0x81
CMD_READ = 0x03

# SPI Setup
cs = Pin(13, Pin.OUT)
spi = SPI(1, sck=Pin(10), mosi=Pin(11), miso=Pin(12), baudrate=500000)
cs.value(1)

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
    # See MCP2515 datasheet or calculator
    mcp_write(0x2A, bytearray([0x00]))  # CNF1
    mcp_write(0x29, bytearray([0x90]))  # CNF2
    mcp_write(0x28, bytearray([0x02]))  # CNF3

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

CAN_ID_0 = 0x07  #node 0
CAN_ID_1 = 0x27  #node 1
CAN_ID_2 = 0x47  #node 2
CAN_ID_3 = 0x67  #node 3


TXB0SIDH = 0x31
TXB0SIDL = 0x32
TXB0DLC  = 0x35
TXB0D0   = 0x36
CMD_RTS  = 0x80  # Request to send on TXB0


def send_calibration_cmd(can_id):

    data = [0x03, 0x00, 0x00, 0x00] + [0x00] * 4

    # data = [0x03, 0x00, 0x00, 0x00]  # AXIS_STATE_FULL_CALIBRATION_SEQUENCE

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
    time.sleep_ms(100)

def send_closed_loop_cmd(can_id):

    data = [0x08, 0x00, 0x00, 0x00] + [0x00] * 4 # AXIS_STATE_CLOSED_LOOP_CONTROL

    sid_high = (can_id >> 3) & 0xFF
    sid_low  = (can_id << 5) & 0xE0
    mcp_write(TXB0SIDH, bytearray([sid_high]))
    mcp_write(TXB0SIDL, bytearray([sid_low]))
    mcp_write(TXB0DLC, bytearray([len(data)]))
    mcp_write(TXB0D0, bytearray(data))
    cs.value(0)
    spi.write(bytearray([CMD_RTS | 0x01]))
    cs.value(1)
    time.sleep_ms(100)

mcp_init()

send_calibration_cmd(CAN_ID_0)
send_calibration_cmd(CAN_ID_1)

time.sleep(10)  # important for calibration

send_closed_loop_cmd(CAN_ID_0)
send_closed_loop_cmd(CAN_ID_1)

while True:
    send_set_input_position(0, 1, 0.0, 0.0)
    time.sleep_ms(10)
    send_set_input_position(1, 1, 0.0, 0.0)
    print("Sending position = 0.25")
    mcp_bit_modify(0x2C, 0x01, 0x00)
    time.sleep_ms(1000)

    send_set_input_position(0, -1, 0.0, 0.0)
    time.sleep_ms(10)
    send_set_input_position(1, -1, 0.0, 0.0)
    print("Sending position = -0.25")
    mcp_bit_modify(0x2C, 0x01, 0x00)

    time.sleep_ms(1000)

    canintf = mcp_read_register(0x2C)  # CANINTF
    print('canintf', hex(canintf))
    if canintf & 0x01:  # RX0IF bit set (message in RXB0)
        sidh = mcp_read_register(0x61)
        sidl = mcp_read_register(0x62)
        dlc = mcp_read_register(0x65)
        data = [mcp_read_register(0x66 + i) for i in range(dlc)]
        print(f"Received: IDH={hex(sidh)}, IDL={hex(sidl)}, DLC={dlc}, Data={data}")
        # Clear TX0IF and RX0IF (bits 5 and 0)
        mcp_bit_modify(0x2C, 0x21, 0x00)

