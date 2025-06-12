from machine import Pin, SPI
import time

# SPI setup
spi = SPI(1, baudrate=500000, polarity=0, phase=0,
          sck=Pin(10), mosi=Pin(11), miso=Pin(12))
cs = Pin(13, Pin.OUT)
cs.value(1)

# MCP2515 Commands
CMD_RESET = 0xC0
CMD_WRITE = 0x02
CMD_READ = 0x03
CMD_BIT_MODIFY = 0x05
CMD_RTS_TXB0 = 0x81

def mcp_write_register(addr, val):
    cs.value(0)
    spi.write(bytearray([CMD_WRITE, addr, val]))
    cs.value(1)

def mcp_read_register(addr):
    cs.value(0)
    spi.write(bytearray([CMD_READ, addr]))
    val = spi.read(1)[0]
    cs.value(1)
    return val

def mcp_bit_modify(addr, mask, data):
    cs.value(0)
    spi.write(bytearray([CMD_BIT_MODIFY, addr, mask, data]))
    cs.value(1)

def mcp_reset():
    cs.value(0)
    spi.write(bytearray([CMD_RESET]))
    cs.value(1)
    time.sleep_ms(10)


# Initialize
mcp_reset()

# Set CONFIG mode
mcp_write_register(0x0F, 0x80)  # CANCTRL
time.sleep_ms(10)

# Set bitrate (500 kbps example)
mcp_write_register(0x2A, 0x00)  # CNF1
mcp_write_register(0x29, 0x91)  # CNF2
mcp_write_register(0x28, 0x01)  # CNF3

# Set NORMAL mode
mcp_write_register(0x0F, 0x00)

while True:
    node_id = 1
    cmd_id = 0x0C
    can_id = (node_id << 5) | cmd_id

    # CAN ID into TXB0SIDH/SIDL (standard ID, 11-bit)
    sid_high = (can_id >> 3) & 0xFF
    sid_low = (can_id & 0x07) << 5

    mcp_write_register(0x31, sid_high)  # TXB0SIDH = 0x12 << 3 = 0x90 → 0x24 is example ID
    mcp_write_register(0x32, sid_low)  # TXB0SIDL


    mcp_write_register(0x35, 0x08)  # TXB0DLC = 8 bytes
    mcp_write_register(0x36,  0x15)
    for i in range(8):
        mcp_write_register(0x36 + i, i + 1)

    # Request to send
    cs.value(0)
    spi.write(bytearray([CMD_RTS_TXB0]))
    cs.value(1)

    print("Message sent")
    time.sleep(1)

