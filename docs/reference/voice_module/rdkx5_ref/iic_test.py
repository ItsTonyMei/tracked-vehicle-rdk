import smbus
import time

time_delay = 0.03
bus_number = 5
bus = smbus.SMBus(bus_number)

address = 0x2b  
register = 0x03 
register1 = 0x64 

#播报词 Active broadcast content
This_red=0x60    
This_green=0x61
This_yellow=0x62
Recognize_yellow=0x63
Recognize_green=0x64
Recognize_blue=0x65
Recognize_red=0x66
init=0x67

def set_voice(data):
    bus.write_byte_data(address, register, data)

set_voice(init)
set_voice(init)
time.sleep(0.5)


while 1:
    data = bus.read_byte_data(address, register1)
    time.sleep(time_delay)
    print(f"Read data:{data}")
    
   
bus.close()