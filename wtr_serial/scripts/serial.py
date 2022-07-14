import serial
def send_str(port, baudrate, parity, value):

    port_set = serial.Serial(port=port, baudrate=baudrate, parity=parity)

    port_set.write(value)

    # delay 100ms if receive is blank, just waiting 5s.
    n = 0
    while port_set.inWaiting() == 0:
        time.sleep(0.1)
        n = n + 1
        if n > 50:
            # send frame again
            port_set.write(value)
            break
    # every 100ms check the data receive is ready
    byte_number_1 = 0
    byte_number_2 = 1
    while byte_number_1 != byte_number_2:
        byte_number_1 = port_set.inWaiting()
        time.sleep(0.1)
        byte_number_2 = port_set.inWaiting()

    receive_frame = port_set.read_all()

    return receive_frame

    