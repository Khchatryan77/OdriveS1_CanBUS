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
    canintf = mcp_read_register(0x2C)  # CANINTF
    if canintf & 0x01:  # RX0IF bit set (message in RXB0)
        sidh = mcp_read_register(0x61)
        sidl = mcp_read_register(0x62)
        dlc = mcp_read_register(0x65)
        data = [mcp_read_register(0x66 + i) for i in range(dlc)]
        print(f"Received: IDH={hex(sidh)}, IDL={hex(sidl)}, DLC={dlc}, Data={data}")

        # Clear RX0IF
        mcp_bit_modify(0x2C, 0x01, 0x00)

    time.sleep(0.1)