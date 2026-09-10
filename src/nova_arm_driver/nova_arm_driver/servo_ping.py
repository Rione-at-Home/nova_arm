from dynamixel_sdk import PortHandler, PacketHandler

port = PortHandler("/dev/ttyACM0")
ph = PacketHandler(1.0)
port.openPort()
port.setBaudRate(1000000)

for dxl_id in [1, 2, 3, 4, 5, 6, 11, 12, 13, 15, 16]:
    delay, comm, err = ph.read1ByteTxRx(port, dxl_id, 5)  # ADDR_RETURN_DELAY_TIME
    print(f"ID {dxl_id}: comm={comm} err={err} return_delay={delay}")