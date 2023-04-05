import serial
import math
from time import time, sleep
from communication.protocol import Protocol


PAYLOAD_SIZE = 27

def avg(l):
    return sum(l)/len(l)

def is_calibrated(rc):
    rc.request_states()
    sleep(0.05)
    states = rc.receive_states()
    if states is None:
        return False
    states = states[:-1]
    print(states)
    
    for state in states:
        if math.isnan(state):
            return False
    return True
    
def wait_calibrated(rc):
    while not is_calibrated(rc):
        sleep(0.5)

    sleep(0.2)
    rc.request_states()
    sleep(0.15)
    s = rc.receive_states()
    print(s)
    return s[0], s[1]

def locate():
    with Connection('COM6') as car:
        return wait_calibrated(car)



class Connection:
    def __init__(self, port: str = "COM3"):
        self.serial_port = serial.Serial(port=port, baudrate=115200, bytesize=8, timeout=1, stopbits=serial.STOPBITS_ONE)
    def __enter__(self):
        print("Connected")
        return self
    def __exit__(self,*arg):
        print("Disconnected")
        pass

    def get_states(self):
        self.request_states()
        return self.receive_states()

    def receive(self):
        while self.serial_port.in_waiting < PAYLOAD_SIZE:
            continue
        try:
            read = self.serial_port.readline().decode('ASCII')
            return read
        except:
            print("ERROR")
    def receive_states(self):
        serial_string = self.receive()
        if serial_string is None:
            print("ERROR: receiving")
            return

        self.last_state = serial_string
        if serial_string.startswith(Protocol.ANS_GET_STATES.value):
            # remove first character
            serial_string = serial_string[1:]
            # remove last character
            serial_string = serial_string[:-1]
            serial_list = serial_string.split(Protocol.DELIM.value)
            serial_list = [float(val) for val in serial_list]
            return serial_list
        print("Protocol violation: receive_states()")


    def request_states(self):
        self.serial_port.write(bytes(Protocol.COM_GET_STATES.value + Protocol.COM_END.value,'ASCII'))
    def writeouput(self,angle,speed):
        self.serial_port.write(bytes(Protocol.COM_WRITEOUTPUT.value +str(round(angle,7))+ Protocol.DELIM.value + str(round(speed,7)) +Protocol.COM_END.value,'ASCII'))

    def flush(self):
        self.serial_port.flushInput()
