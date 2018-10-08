import sqlite3
from smbus2 import SMBus
import logging
import time

roverdb='C:\\Users\\Yves1812\\Documents\\GitHub\\Robot_tondeuse\\Boards\\MU - web server\\app.db'
pi_I2C_bus=0
TP_board_I2C_address=0x60
encoder_register=1

#define CMPS12_ADDRESS 0x60  // Address of CMPS12 shifted right one bit for arduino wire library
#define ANGLE_8  1           // Register to read 8bit angle from


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

    def writeRaw8(self, value):
        """Write an 8-bit value on the bus (without register)."""
        value = value & 0xFF
        self._bus.write_byte(self._address, value)
        self._logger.debug("Wrote 0x%02X",
                     value)

    def write8(self, register, value):
        """Write an 8-bit value to the specified register."""
        value = value & 0xFF
        self._bus.write_byte_data(self._address, register, value)
        self._logger.debug("Wrote 0x%02X to register 0x%02X",
                     value, register)

    def write16(self, register, value):
        """Write a 16-bit value to the specified register."""
        value = value & 0xFFFF
        self._bus.write_word_data(self._address, register, value)
        self._logger.debug("Wrote 0x%04X to register pair 0x%02X, 0x%02X",
                     value, register, register+1)

    def writeList(self, register, data):
        """Write bytes to the specified register."""
        self._bus.write_i2c_block_data(self._address, register, data)
        self._logger.debug("Wrote to register 0x%02X: %s",
                     register, data)

    def readList(self, register, length):
        """Read a length number of bytes from the specified register.  Results
        will be returned as a bytearray."""
        results = self._bus.read_i2c_block_data(self._address, register, length)
        self._logger.debug("Read the following from register 0x%02X: %s",
                     register, results)
        return results

    def readRaw8(self):
        """Read an 8-bit value on the bus (without register)."""
        result = self._bus.read_byte(self._address) & 0xFF
        self._logger.debug("Read 0x%02X",
                    result)
        return result

    def readU8(self, register):
        """Read an unsigned byte from the specified register."""
        result = self._bus.read_byte_data(self._address, register) & 0xFF
        self._logger.debug("Read 0x%02X from register 0x%02X",
                     result, register)
        return result

    def readS8(self, register):
        """Read a signed byte from the specified register."""
        result = self.readU8(register)
        if result > 127:
            result -= 256
        return result

    def readU16(self, register, little_endian=True):
        """Read an unsigned 16-bit value from the specified register, with the
        specified endianness (default little endian, or least significant byte
        first)."""
        result = self._bus.read_word_data(self._address,register) & 0xFFFF
        self._logger.debug("Read 0x%04X from register pair 0x%02X, 0x%02X",
                           result, register, register+1)
        # Swap bytes if using big endian because read_word_data assumes little
        # endian on ARM (little endian) systems.
        if not little_endian:
            result = ((result << 8) & 0xFF00) + (result >> 8)
        return result

    def readS16(self, register, little_endian=True):
        """Read a signed 16-bit value from the specified register, with the
        specified endianness (default little endian, or least significant byte
        first)."""
        result = self.readU16(register, little_endian)
        if result > 32767:
            result -= 65536
        return result

    def readU16LE(self, register):
        """Read an unsigned 16-bit value from the specified register, in little
        endian byte order."""
        return self.readU16(register, little_endian=True)

    def readU16BE(self, register):
        """Read an unsigned 16-bit value from the specified register, in big
        endian byte order."""
        return self.readU16(register, little_endian=False)

    def readS16LE(self, register):
        """Read a signed 16-bit value from the specified register, in little
        endian byte order."""
        return self.readS16(register, little_endian=True)

    def readS16BE(self, register):
        """Read a signed 16-bit value from the specified register, in big
        endian byte order."""
        return self.readS16(register, little_endian=False)

class Rover(object):
    def __init__(self,x=0.0,y=0.0,speedx=0.0,speedy=0.0,teta_point=0,ice_status=False, battery_level=None,traction_power=None, messages=[]):
        self.x=x
        self.y=y
        self.vx=speedx
        self.vy=speedy
        self.teta_point=teta_point
        self.ice_status=ice_status
        self.battery_level=battery_level
        self.traction_power=traction_power
        self.messages=messages
        self.conn = sqlite3.connect(roverdb)
        self.conn.row_factory = sqlite3.Row
        self.cursor=self.conn.cursor()
        self.TP_board=Device(TP_board_I2C_address,pi_I2C_bus) # need to put here the right bus number

        self.bearing8=0
        self.bearing16=0
        self.pitch=0
        self.roll=0            



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
        values=(self.x, self.y,self.vx, self.vy, self.ice_status,self.battery_level,self.traction_power)##self.messages
        self.cursor.execute("""insert into traction_status (x, y,vx,vy,ice_status, battery_level, power) values (?, ?, ?, ?, ?, ?, ?)""", values)

    def get_rover_status(self, data_type):
        if (data_type == 0) : # read all
            print("Data type 0 not implemented yet")
            
        elif (data_type == 1) : # read decoder based estimates (x,y,vx,vy,teta_point)
            length=14
            decoder_data=TP_board.readList(encoder_register, length)
            self.x=int.from_bytes(decoder_data[0:3], byteorder='big', signed=True)
            self.y=int.from_bytes(decoder_data[4:7], byteorder='big', signed=True)
            self.vx=int.from_bytes(decoder_data[8:9], byteorder='big', signed=True)
            self.vy=int.from_bytes(decoder_data[10:11], byteorder='big', signed=True)
            self.teta_point=int.from_bytes(decoder_data[12:13], byteorder='big', signed=True)
            
        elif (data_type == 2) : # read GPS based estimates (x,y,vx,vy)
            print("Data type 2 not implemented yet")

        elif (data_type == 3) : # read other data (power, ice_status)
            print("Data type 3 not implemented yet")

        elif (data_type == 4) : # read messages
            print("Data type 4 not implemented yet")

        elif (data_type == 5) : # test with compass sensor
            length=5
            compass_data=TP_board.readList(encoder_register, length)
            self.bearing8=int.from_bytes(decoder_data[0], byteorder='big', signed=True)
            self.bearing16=int.from_bytes(decoder_data[1:2], byteorder='big', signed=True)
            self.pitch=int.from_bytes(decoder_data[3], byteorder='big', signed=True)
            self.roll=int.from_bytes(decoder_data[4], byteorder='big', signed=True)
        else :
            print("Unknown datatype to be red from TP board")

    def set_routing(self):
        pass
    def emergency_stop(self):
        pass
    def set_max_speed(self):
        pass
    def manual(self):
        pass

print("Welcome to my rover")

myrover=rover()
##myrover.query_db_status()
##print('x=',myrover.x)
##print('y=',myrover.y)

for i in range(1000):
    myrover.get_rover_status(5)
    print(myrover.bearing8)
    print(myrover.bearing16)
    print(myrover.pitch)
    print(myrover.roll)
    time.sleep(1)
