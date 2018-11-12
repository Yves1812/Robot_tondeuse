import sqlite3
from smbus2 import SMBus
import logging
import time
import datetime

#### Orthonormal reference ###############
# Y axis pointing towards true north     #
# X axis pointing towards (true) west    #
# unit = 1 meter                         #
# 0,0 at robot charging station          #
##########################################

roverdb='C:\\Users\\Yves1812\\Documents\\GitHub\\Robot_tondeuse\\Boards\\MU - web server\\app.db'
pi_I2C_bus=1
TP_board_I2C_address=0x60
position_register=1
speed_register=9
power_register=14
TICKS_PER_METER=350 #number of ticks per meter @ full speed + 10% needs to be confirmed and potentially adjusted


class Device(object):
    """Class for communicating with an I2C device using the adafruit-pureio pure
    python smbus library, or other smbus compatible I2C interface. Allows reading
    and writing 8-bit, 16-bit, and byte array values to registers
    on the device."""
    def __init__(self, address, busnum):
        """Create an instance of the I2C device at the specified address on the
        specified I2C bus number."""
        self._address = address
        self._bus = SMBus(busnum)
        self._logger = logging.getLogger('Adafruit_I2C.Device.Bus.{0}.Address.{1:#0X}'.format(busnum, address))

    def write8(self, value, register=0):
        """Write an 8-bit value to the specified register. use register 0 if not needed"""
        value = value & 0xFF
        self._bus.write_byte_data(self._address, register, value)
        self._logger.debug("Wrote 0x%02X to register 0x%02X",
                     value, register)

    def readU8(self, register=0):
        """Read an unsigned byte from the specified register. use 0 if egister is not needed"""
        result = self._bus.read_byte_data(self._address, register) & 0xFF
        self._logger.debug("Read 0x%02X from register 0x%02X",result, register)
        return result

    def writeList(self, data, register=0):
        """Write bytes to the specified register. Use 0 if register is not needed"""
        self._bus.write_i2c_block_data(self._address, register, data)
        self._logger.debug("Wrote to register 0x%02X: %s", register, data)

    def readList(self, length, register=0):
        """Read a length number of bytes from the specified register, use 0 if register is not needed.  Results
        will be returned as a bytearray."""
        results = self._bus.read_i2c_block_data(self._address, register, length)
        self._logger.debug("Read the following from register 0x%02X: %s", register, results)
        return results


class Rover(object):
    def __init__(self,x=0.0,y=0.0,speedx=0.0,speedy=0.0,teta_point=0,bearing8=0,ice_status=False, battery_level=None,traction_power=None, messages=[]):
        self.x=x
        self.y=y

        self.vx=speedx
        self.vy=speedy
        self.bearing8=bearing8
        self.teta_point=teta_point
        
        self.ice_status=ice_status
        self.battery_level=battery_level
        self.traction_power=traction_power
        self.messages=messages
        self.conn = sqlite3.connect(roverdb)
        self.conn.row_factory = sqlite3.Row
        self.cursor=self.conn.cursor()
        self.TP_board=Device(TP_board_I2C_address,pi_I2C_bus) # need to put here the right bus number

    def query_db_status(self):
        self.cursor.execute("select * from traction_status order by timestamp desc")
        latest=self.cursor.fetchone()
        if latest :
            self.x=latest['x']
            self.y=latest['y']
            self.vx=latest['vx']
            self.vy=latest['vy']
            self.ice_status=latest['ice_status']
            self.battery_level=latest['battery_level']
            self.traction_power=latest['power']
##            self.messages=latest['messages']
        
    def save_rover_status(self):
        values=(self.x, self.y, self.vx, self.vy,self.bearing8,self.teta_point,self.ice_status,self.battery_level,self.traction_power)##self.messages
        self.cursor.execute("""insert into traction_status (x, y,vx,vy,ice_status, battery_level, power) values (?, ?, ?, ?, ?, ?, ?)""", values)

    def get_rover_position(self):
        length=9
        decoder_data=TP_board.readList(position_register, length)
        self.x=(((decoder_data[0]*256+decoder_data[1])*256+decoder_data[2])*256+decoder_data[3]
        self.y=(((decoder_data[4]*256+decoder_data[5])*256+decoder_data[6])*256+decoder_data[7]
        self.bearing8=decoder_data[8]
##        self.x=int.from_bytes(decoder_data[0:3], byteorder='big', signed=True)
##        self.y=int.from_bytes(decoder_data[4:7], byteorder='big', signed=True)


    def get_rover_speed(self):
        length=6
        decoder_data=TP_board.readList(speed_register, length)
        self.vx=decoder_data[0]*256+decoder_data[1]
        self.vy=decoder_data[2]*256+decoder_data[3]
        self.teta_point=decoder_data[4]*256+decoder_data[5]

    def get_rover_power(self):
        length=3
        decoder_data=TP_board.readList(power_register, length)
        self.power=decoder_data[0]*256+decoder_data[1]
        self.ice_status=decoder_data[2]

    def set_routing(self):
        pass
    def emergency_stop(self):
        pass
    def set_max_speed(self):
        pass
    def manual(self):
        pass

class segment(object):
    def __init__(x0,y0,x1,y1,speed):
        self.x0=x0
        self.y0=y0
        self.x1=x1
        self.y1=y1
        self.speed=(int)(speed*255)
        self.target_ticks=((x1-x0)^2+(y1-y0)^2)^(1/2)*TICKS_PER_METER
        
print("Welcome to my rover")

myrover=Rover()
##myrover.query_db_status()
##print('x=',myrover.x)
##print('y=',myrover.y)


print(datetime.datetime.now())
for i in range(10000):
    myrover.get_rover_status(0)
print(datetime.datetime.now())

##    print("Bearing 8: ",myrover.bearing8*360/255)
##    print("Bearing 16: ", myrover.bearing16/10)
##    print("pitch: ",myrover.pitch)
##    print("Roll: ",myrover.roll)
##    print("")
#    time.sleep(10)
