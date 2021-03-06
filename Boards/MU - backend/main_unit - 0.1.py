import sqlite3
from smbus2 import SMBus
import logging
from math import sqrt
import time
import datetime
from math import atan2, cos, sin
import json
from pprint import pprint

#### Orthonormal reference ###############
# Y axis pointing towards true north     #
# X axis pointing towards (true) east    #
# unit = 1 meter                         #
# 0,0 at robot charging station          #
##########################################

## Path and file names

##### Windows #####
##roverdb='C:\\Users\\Yves1812\\Documents\\GitHub\\Robot_tondeuse\\Boards\\MU - web server\\app.db'
##routing_path='C:\\Users\\Yves1812\\Documents\\Github\\Robot_tondeuse\\Data\\'

##### Raspberry Pi #####
roverdb='/home/pi/Documents/GitHub/Robot_tondeuse/Boards/MU - web server/app.db/'
routing_path='/home/pi/Documents/Github/Robot_tondeuse/Data/'

routing_file='test_routing'

##### I2C adresses #####
pi_I2C_bus=1
TP_board_I2C_address=0x16
active_segment_register=10
motors_power_register=20
routing_register=30


##### Rover physical data #####
TICKS_PER_METER=350 #number of ticks per meter @ full speed + 10% needs to be confirmed and potentially adjusted
TICKS_PER_SECOND=175
MOWING_WIDTH=0.4 #meter


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

    def writeByte(self, value):
        """Write an 8-bit value"""
        value = value & 0xFF
        self._bus.write_byte(self._address, register, value)
        self._logger.debug("Wrote 0x%02X to register 0x%02X",
                     value, register)

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
    def __init__(self,x=0.0,y=0.0,speedx=0.0,speedy=0.0,teta_point=0,bearing8=0,ice_status=False, battery_level=None,traction_power=[0,0,0,0], messages=[]):
        self.routing=routing()
        #position from completed segments
        self.x=x 
        self.y=y
        #delta position from current segment
        self.delta_x=0
        self.delta_y=0

        self.vx=speedx
        self.vy=speedy
        self.bearing8=bearing8
        self.teta_point=teta_point
        
        self.ice_status=ice_status
        self.battery_level=battery_level
        self.traction_power=traction_power
        self.messages=messages
        self.TP_board=Device(TP_board_I2C_address,pi_I2C_bus) # need to put here the right bus number
        
    def query_rover_status(self):
    # query rover status from the database
        #Connect to database
        self.conn = sqlite3.connect(roverdb)
        self.conn.row_factory = sqlite3.Row
        self.cursor=self.conn.cursor()

        # query database
        self.cursor.execute("select * from traction_status order by timestamp desc")
        latest=self.cursor.fetchone()
        if latest :
            self.x=latest['x']
            self.y=latest['y']
            self.vx=latest['vx']
            self.vy=latest['vy']
            self.ice_status=latest['ice_status']
            self.battery_level=latest['battery_level']
            ## Need to manage 4 power values            self.traction_power=latest['power']
            ## self.messages=latest['messages']
        self.conn.close()
        
    def save_rover_status(self):
        #Connect to database
        self.conn = sqlite3.connect(roverdb)
        self.conn.row_factory = sqlite3.Row
        self.cursor=self.conn.cursor()

        values=(self.x, self.y, self.vx, self.vy,self.bearing8,self.teta_point,self.ice_status,self.battery_level,self.traction_power[0],self.traction_power[1],self.traction_power[2],self.traction_power[3])##self.messages
        self.cursor.execute("""insert into traction_status (x, y,vx,vy,ice_status, battery_level, power) values (?, ?, ?, ?, ?, ?, ?)""", values)

        self.conn.commit()
        self.conn.close()

    def get_active_segment_status(self):
    # Obtain progress on current segment execution from TP board (via I2C)
        if self.routing.active_segment != None :
            length=14
#            decoder_data=[0,0,0,0,0,0,0,0,0,0,0,0,0,0]
#            print("Blank data segment: ",decoder_data)
            decoder_data=self.TP_board.readList(length, active_segment_register)
            print('Segment status from TP')
            print("Set of bytes received fro TP: ",decoder_data)
            print("Decoding...")
            self.routing.active_segment_status.segment_id=decoder_data[0]*256+decoder_data[1]
            self.routing.active_segment_status.ticks_cum=(((decoder_data[2]*256+decoder_data[3])*256+decoder_data[4])*256+decoder_data[5])
            self.routing.active_segment_status.millis_cum=(((decoder_data[6]*256+decoder_data[7])*256+decoder_data[8])*256+decoder_data[9])
            self.routing.active_segment_status.average_bearing=decoder_data[10]
            self.routing.active_segment_status.current_bearing=decoder_data[11]
            self.routing.active_segment_status.speed_step=decoder_data[12]
            #self.routing.active_segment_status.teta_point=decoder_data[13]
            self.routing.next_segment_needed=decoder_data[13]
            print('id:', self.routing.active_segment_status.segment_id)
            print('ticks_cum: ',self.routing.active_segment_status.ticks_cum)
            print('millis_cum: ',self.routing.active_segment_status.millis_cum)
            print('avergae bearing: ',self.routing.active_segment_status.average_bearing)
            print('current bearing: ',self.routing.active_segment_status.current_bearing)
            print('speed_step: ',self.routing.active_segment_status.speed_step)
            print('segment_needed: ',self.routing.next_segment_needed)
            print()

##            self.routing.active_segment_status.segment_id=1
##            self.routing.active_segment_status.ticks_cum=250
##            self.routing.active_segment_status.millis_cum=600
##            self.routing.active_segment_status.average_bearing=30
##            self.routing.active_segment_status.current_bearing=35
##            self.routing.active_segment_status.speed_step=200
##            self.routing.next_segment_needed=1
            if self.routing.next_segment_needed :
                self.update_rover_position()
                if (self.routing.active_segment+1 < len(self.routing.segments)):
                    self.push_segment(self.routing.active_segment+1)
                else:
                    pass
                    # last segment has been reached => stop and/or ask for a new routing

        else :
            print("no active segment to read")

    def update_rover_position(self):
    # update rover position data based on latest segment status and returns current x,y coordinate as a waypoint
        self.delta_x=self.routing.active_segment_status.ticks_cum / TICKS_PER_METER * cos(self.routing.active_segment_status.current_bearing*6.28/255)
        self.delta_y=self.routing.active_segment_status.ticks_cum / TICKS_PER_METER * sin(self.routing.active_segment_status.current_bearing*6.28/255)
        if self.routing.next_segment_needed :
            self.x+=self.delta_x
            self.y+=self.delta_y
            return waypoint(self.x, self.y)
        else:
            return waypoint(self.x+self.delta_x, self.y+self.delta_y)
        
    def get_rover_power(self):
        length=4
        decoder_data=self.TP_board.readList(length, motors_power_register)
        print("Power decoder data: ",decoder_data)
        self.traction_power[0]=decoder_data[0]
        self.traction_power[1]=decoder_data[1]
        self.traction_power[2]=decoder_data[2]
        self.traction_power[3]=decoder_data[3]
# not implemente  self.ice_status=decoder_data[2]

    def push_segment(self, segment):
        print("Pushing segment# :", segment)
        self.routing.segments[segment].print()

        data=[]
        data.append(self.routing.segments[segment].segment_type)
        data.append((self.routing.segments[segment].segment_id & 0xFF00) >> 8) #MSB first
        data.append((self.routing.segments[segment].segment_id & 0xFF)) #LSB second
        data.append((self.routing.segments[segment].target_ticks & 0xFF000000) >> 24 ) #MSB first
        data.append((self.routing.segments[segment].target_ticks & 0x00FF0000) >> 16) 
        data.append((self.routing.segments[segment].target_ticks & 0x0000FF00) >> 8) 
        data.append((self.routing.segments[segment].target_ticks & 0x000000FF)) 
        data.append(self.routing.segments[segment].target_bearing)
        data.append(self.routing.segments[segment].target_speed)
        i=0
        for item in data:
            print("Pushed byte:",i," ",item)
            i+=1
        # need to add error management
        self.TP_board.writeList(data,routing_register)
        self.routing.active_segment=segment
        self.routing.next_segment_needed=False

    def set_routing(self):
        self.routing.loadRouting()

    def move_rover(self):
        if (self.routing.active_segment == None):
            self.routing.active_segment=0
        self.push_segment(self.routing.active_segment)
        
    def emergency_stop(self):
        pass

    def manual(self):
        pass
        
class routing(object):
    def __init__(self, name="vide", routing_type=1):
        self.name=name
        self.routing_type=routing_type #1 for liaison, 2 for mow
        self.perimeter=[] #[waypoints] in case of move perimeter = 2 points (origin and destination)
        self.segments=[]
        self.speed = 255 #one overall, speed of rotation segments hardcoded in motor driver
        self.active_segment=None
        self.active_segment_status=segment_status()
        self.next_segment_needed=False
    
    def buildSegments(self):
        if (self.routing_type==1) :
            # Calculate straight line liaison
            waypoint0=self.perimeter[0]
            i=0
            for waypoint in self.perimeter[1:]:
                #need to confirm  atan angle in practice to be sure
                if (waypoint.x-waypoint0.x)==0 :
                    if ((waypoint.y-waypoint0.y)>0):
                        heading=0
                    else:
                        heading=127
                else:
                    heading = (int) (255/6.2832*(3.1416/2-atan2((waypoint.y-waypoint0.y),(waypoint.x-waypoint0.x))))
                self.segments.append(segment(0,i,0,heading,250))
                self.segments.append(segment(1,i+1,(int)((sqrt((waypoint.x-waypoint0.x)**2+(waypoint.y-waypoint0.y)**2))*TICKS_PER_METER), heading, self.speed))
                i=i+2
                waypoint0=waypoint
        if (self.routing_type==2) :
            # Calculate mowing trajectory

            # Calculation logic is to follow perimeter and move inner by mowing width at each lap
            # Option 1 use HOMOTETIE of center = center of gravity of waypoints will work for convex shapes only => need to cut surface in covex polygons
            # Option 2 TRANSLATE segments inner by mowing_width, check it does not cross any n-1 lap segment and calculate arrival point as intersection between translated segment and next one at lap n-1 minus mowing width
            # If it does cross a former segment, go to (intersection - width/2) and resume calculation from this waypoint
            # Go for option 2 as it should work on all kinds of shapes - becarefull when opposite segments are not // eg. triangle
            # When reaching last waypoint of lap
            # - move towards inner side of area by Width orthogonaly to segment 1 or less if this would cross a segment of lap n-1, in this case set uncomplete inner move flag
            # - draw segment // to segment 1 from lap before until crossing segment 2 minus 1/2 width
            # ...
            # - if incomplete inner move flag, check if completing inner move is now possible, do what is possible and adjust flag accordingly
            # - draw segment // to segment 1 from lap before until crossing segment 2 minus 1/2 width
            # ...


            # Check if perimeter is closed, else close it with a straight linebetween last waypoint and first
            if (self.perimeter[0].x != self.perimeter[len(self.perimeter)].x or self.perimeter[0].y != self.perimeter[len(self.perimeter)].y): 
                self.perimeter.append(self.perimeter[0])
            # Calculate lap and create next lap until 
##            while distance to other border > MOWING_WIDTH
            waypoint0=self.perimeter[0]
            i=0
            for waypoint in self.perimeter[1:]:
                #need to confirm  atan angle in practice to be sure
                if (waypoint.x-waypoint0.x)==0 :
                    if ((waypoint.y-waypoint0.y)>0):
                        heading=0
                    else:
                        heading=127
                else:
                    heading = (int) (255/6.2832*(3.1416/2-atan2((waypoint.y-waypoint0.y),(waypoint.x-waypoint0.x))))
                self.segments.append(segment(0,i,0,heading,250))
                self.segments.append(segment(1,i+1,(int)((sqrt((waypoint.x-waypoint0.x)**2+(waypoint.y-waypoint0.y)**2))*TICKS_PER_METER), heading, self.speed))
                i=i+2
                waypoint0=waypoint
                
    def distance_to_opposite_border(self) :
        # Returns distance to other border @90 degree vs current segment
        pass
        
    def loadRouting(self):
        with open(routing_path+self.name+".json", 'r') as routing_file:
          routing_data=json.load(routing_file)
        self.name=routing_data["name"]
        self.routing_type=routing_data["routing_type"]
        for item in routing_data["perimeter"]:
           self.perimeter.append(waypoint(item[0],item[1]))

class waypoint(object):
    def __init__(self,x,y):
        self.x=x
        self.y=y
        # GPS coordinates to be implemented     

class map(object):
    def __init__(name, northW, northE, southW, southE):
        # need to manage non square areas in blocks generation
        self.name=name
        self.corners=[]
        self.corners.append(northW)
        self.corners.append(northE)
        self.corners.append(southW)
        self.corners.append(southE)
        self.blocks=[x[:] for x in [[True] * 10] * 10]
        #   position of the block (x/y, GPS, needed or just using indexes in map * size)

    def laod(self):
        pass

    def save(self):
        pass


class segment(object):
    def __init__(self,segment_type=1,segment_id=0,target_ticks=None, target_bearing=None, target_speed=250):
        self.segment_type=0 #0 in case rotation, 1 in case straight
        self.segment_id=segment_id
        self.target_ticks=target_ticks
        self.target_bearing=target_bearing
        self.target_speed=target_speed
    def print(self):
        print("Segment Type: ",self.segment_type) #0 in case rotation, 1 in case straight
        print("Segment id: ",self.segment_id)
        print("Segment target ticks: ",self.target_ticks)
        print("Segment target bearing: ", self.target_bearing)
        print("Segment target speed: ", self.target_speed)
        print("")
        

class segment_status(object):
    #segment status stores progress in delivering current segment from TP Board
    def __init__(self,segment_id=None):
        self.segment_type=0 #0 in case rotation, 1 in case straight
        self.segment_id=segment_id
        self.ticks_cum=0
        self.millis_cum=0
        self.average_bearing=None
        self.current_bearing=None
        #self.teta_point=None not implemented
        self.speed_step=None #speed_step is the speed mesured between the last two position measures made by TP board (@10Hz)

if __name__ == "__main__":

    print("Welcome to my rover, Yves !")
    myrover=Rover()
    myrover.routing.name=routing_file
    myrover.routing.loadRouting()
    myrover.routing.buildSegments()
    for item in myrover.routing.segments:
        item.print()
    myrover.routing.active_segment=0
    print("Active_segment #:", myrover.routing.active_segment)
#    myrover.move_rover()
## delay until Mega is up
    time.sleep(5)
    for i in range(3):
        time.sleep(1)
        myrover.get_active_segment_status() # test get segment status and push segment if requested
        myrover.get_rover_power()
                                     
    ##myrover.query_db_status()
    ##print('x=',myrover.x)
    ##print('y=',myrover.y)
    ##
    ##print(datetime.datetime.now())
    ##for i in range(10000):
    ##    myrover.get_rover_status(0)
    ##print(datetime.datetime.now())
    ##    print("Bearing 8: ",myrover.bearing8*360/255)
    ##    print("Bearing 16: ", myrover.bearing16/10)
    ##    print("pitch: ",myrover.pitch)
    ##    print("Roll: ",myrover.roll)
    ##    print("")
    #    time.sleep(10)


