import sqlite3
import logging
from math import sqrt
import time
import datetime
from math import atan2, cos, sin, acos, radians, tan
import json
from pprint import pprint
import os.path
from collections import deque

## I2C dirvers Paths and file names depending on environment ##
try:
    from smbus2 import SMBus # means on Pi
    ### Use Raspberry Pi path #####
    roverdb='/home/pi/Documents/GitHub/Robot_tondeuse/Boards/MU - web server/app.db/'
    routing_path='/home/pi/Documents/Github/Robot_tondeuse/Data/'
    command_path='/home/pi/Documents/Github/Robot_tondeuse/Data/Commands/'
except ImportError:
    print("Warning - could not import smbus, proceeding with simulated data") #Means in windows dev environment
##    ##### Use Windows path YB #####
##    roverdb='C:\\Users\\Yves1812\\Documents\\GitHub\\Robot_tondeuse\\Boards\\MU - web server\\app.db'
##    routing_path='C:\\Users\\Yves1812\\Documents\\Github\\Robot_tondeuse\\Data\\'
##    command_path='C:\\Users\\Yves1812\\Documents\\Github\\Robot_tondeuse\\Data\\Commands\\'
    ##### Use Windows path #####
    roverdb='C:\\user\\U417266\\GitHub\\Robot_tondeuse\\Boards\\MU - web server\\app.db'
    routing_path='C:\\user\\U417266\\GitHub\\Robot_tondeuse\\Data\\'
    command_path='C:\\user\\U417266\\Github\\Robot_tondeuse\\Data\\Commands\\'




#### Orthonormal reference #########################################################
# Y axis pointing towards true north                                               #
# X axis pointing towards (true) east                                              #
# unit = 1 meter                                                                   #
# 0,0 at robot charging station                                                    #
####################################################################################

#### Segment data          #########################################################
# speed in ticks / seconds in [-700;700]                                           #
# ticks in ticks                                                                   #
# bearing8 as a byte [0;255]                                                       #
#                                                                                  #
####################################################################################


active_routing='active_routing'

##### I2C adresses #####
pi_I2C_bus=1
TP_board_I2C_address=0x16
active_segment_register=10
motors_power_register=20
routing_register=30


##### Rover physical data #####
TICKS_PER_METER=350 #number of ticks per meter @ full speed + 10% needs to be confirmed and potentially adjusted
TICKS_PER_SECOND=175
MOWING_WIDTH=0.5 #meter
HALF_MOWING_WIDTH=MOWING_WIDTH/2


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
        self._logger = logging.getLogger('Rover_log')

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
        # Make logger available
        self._logger = logging.getLogger('Rover_log')

        #Initialization
        self.checks_succeeded=None
        self.initialization_completed=False
        
        self.routing=routing()
        #position from completed segments
        self.x=x 
        self.y=y
        #delta position from current segment
        self.delta_x=0
        self.delta_y=0

        #rover last known dynamics
        self.vx=speedx
        self.vy=speedy
        self.bearing8=bearing8
        self.teta_point=teta_point

        #rover last known state data
        self.ice_status=ice_status
        self.battery_level=battery_level
        self.traction_power=traction_power
        self.messages=messages

        #Rover movement status
        self.moving=0 # 0=paused, 1=running
        
        #Rover Commands queue
        self.commands=deque() #append, popleft
        self.detected_routing=None
                
        try :
            self.TP_board=Device(TP_board_I2C_address,pi_I2C_bus)
        except :
            self._logger.warning("Error initializing I2C Bus, proceeding with simulated data")
            self.TP_board=None
        
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
            decoder_data_none_needed=[0,1,0,0,0,2,0,0,0,4,8,16,32,0] # no seg needed
            decoder_data_next_needed=[0,1,0,0,0,2,0,0,0,4,8,16,32,1] # no seg needed
            try:
                decoder_data=self.TP_board.readList(length, active_segment_register)
                self._logger.debug("Set of bytes received from TP: "+str(decoder_data))
            except:
                self._logger.warning("Error reading decoder, using dummy decoder data for segment status")
                decoder_data=decoder_data_next_needed
            self.routing.active_segment_status.segment_id=decoder_data[0]*256+decoder_data[1]
            self.routing.active_segment_status.ticks_cum=(((decoder_data[2]*256+decoder_data[3])*256+decoder_data[4])*256+decoder_data[5])
            self.routing.active_segment_status.millis_cum=(((decoder_data[6]*256+decoder_data[7])*256+decoder_data[8])*256+decoder_data[9])
            self.routing.active_segment_status.average_bearing8=decoder_data[10]
            self.routing.active_segment_status.current_bearing8=decoder_data[11]
            self.routing.active_segment_status.speed_step=(decoder_data[12]-127)*350/255
            #self.routing.active_segment_status.teta_point=decoder_data[13]
            self.routing.next_segment_needed=decoder_data[13]
            self._logger.debug("Receied segment status "+'id: '+str(self.routing.active_segment_status.segment_id)+
                          ' ticks_cum: '+str(self.routing.active_segment_status.ticks_cum)+
                          ' millis_cum: '+str(self.routing.active_segment_status.millis_cum)+
                          ' avergae bearing: '+str(self.routing.active_segment_status.average_bearing8)+
                          ' current bearing: '+str(self.routing.active_segment_status.current_bearing8)+
                          ' speed_step: '+str(self.routing.active_segment_status.speed_step)+
                          ' segment_needed: '+str(self.routing.next_segment_needed))

        else :
            self._logger.warning("Failed to read segment from TP - no active segment to read")

    def update_rover_position(self):
    # update rover position data based on latest segment status and returns current x,y coordinate as a waypoint
        try:
            self.delta_x=self.routing.active_segment_status.ticks_cum / TICKS_PER_METER * cos(self.routing.active_segment_status.average_bearing8*6.28/255)
            self.delta_y=self.routing.active_segment_status.ticks_cum / TICKS_PER_METER * sin(self.routing.active_segment_status.average_bearing8*6.28/255)
        except :
            if (self.routing.active_segment_status.ticks_cum == 0):
                self.delta_x=0
                self.delta_y=0
            else:
                self._logger.warning("Failed to read segment from TP - no active segment to read")
                return None                
        if self.routing.next_segment_needed :
            self.x+=self.delta_x
            self.y+=self.delta_y
            return waypoint(self.x, self.y)
        else:
            return waypoint(self.x+self.delta_x, self.y+self.delta_y)
        
    def get_rover_power(self):
        length=4
        try:
            decoder_data=self.TP_board.readList(length, motors_power_register)
        except:
            print("Using dummy decoder data for power data")
            decoder_data=[10,20,30,40]
        self._logger.debug("Power decoder data: "+str(decoder_data))
        self.traction_power[0]=decoder_data[0]
        self.traction_power[1]=decoder_data[1]
        self.traction_power[2]=decoder_data[2]
        self.traction_power[3]=decoder_data[3]
# not implemented  self.ice_status=decoder_data[2]

    def push_segment(self, segment):
    # Pushes active segment to TP, including play/pause with self.moving parameter
    # if TP identifies this is an update (same segment id), it will complete updated segment - completed number of ticks at new speed
    # This design choice allows for a non fully uptodate segment status in MU and faster emergency stop

#        self.routing.segments[segment].print()
        data=[]
        data.append(self.routing.segments[segment].segment_type)
        data.append((self.routing.segments[segment].segment_id & 0xFF00) >> 8) #MSB first
        data.append((self.routing.segments[segment].segment_id & 0xFF)) #LSB second
        data.append((self.routing.segments[segment].target_ticks & 0xFF000000) >> 24 ) #MSB first
        data.append((self.routing.segments[segment].target_ticks & 0x00FF0000) >> 16) 
        data.append((self.routing.segments[segment].target_ticks & 0x0000FF00) >> 8) 
        data.append((self.routing.segments[segment].target_ticks & 0x000000FF)) 
        data.append(self.routing.segments[segment].target_bearing8)
        data.append((self.routing.segments[segment].target_speed*self.moving+350)/2/350*255) # shift from [-350;350] to [0;255]
        new_seg_str=""
        for item in data:
            new_seg_str+=str(item)+":"
        # need to add error management
        self._logger.debug("Pushing segment bytes to TP. "+new_seg_str)
        try:
            self.TP_board.writeList(data,routing_register)
        except :
            self._logger.warning("Failed to push new segment to TP board - Continuing in test mode")
        self.routing.active_segment=segment

    def detectRouting(self):
        try:
            with os.scandir(command_path) as it:
                for entry in it:
                    if entry.name.startswith('routing_') and entry.is_file():
                        self.detected_routing=entry.name[8:-6] #name format should be routing_###.route
        except OSError as e:
            self._logger.warning("Routing path not found")
            return -1

    def set_routing(self, new_routing):
        # Set routing set active routing, need to move_rover to get robot moving
        if (self.routing.loadRouting(new_routing)==0):
            self._logger.debug("New routing loaded")
            self.routing.buildSegments()
            self.routing.active_segment=0
            self.routing.routing_name=new_routing
            self._logger.debug("Routing "+self.routing.routing_name+" made active")
            return 0
        else:
            self._logger.warning("Failed to set routing "+new_routing)
            return -1

    def move_rover(self):
        # Pushes active segment to TP for execution, if no segment specified, segment 0 is pushed
        if (self.routing.active_segment == None):
            self.routing.active_segment=0
        self.push_segment(self.routing.active_segment)
        
    def emergency_stop(self):
        pass

    def manual(self):
        pass

    def getCommands(self):
        with os.scandir(command_path) as it:
            for entry in it:
                if entry.name.startswith('cmd_') and entry.is_file():
                    self.commands.append(entry.name[4:-4]) #command format == cmd_play.cmd
                    os.remove(command_path+entry.name)

    def processCommands(self):
        current=self.moving
        while self.commands :
            command=self.commands.popleft()
            print(command)
            if command == "play":
                self.moving=1
            elif command == "pause" :
                self.moving=0
            elif command == "changeroute" : # will load new route but not play it, play will still be needed
                try :
                    self.detectRouting()
                    self._logger.debug("New routing found: "+str(self.detected_routing))
                except:
                    self._logger.warning("Failed to change route - No new routing found")
                    # Try to load  routing
                if (self.detected_routing!=None) :
                    loading=self.set_routing(self.detected_routing)
                    if (loading==0):
                        self.moving=0
                        self._logger.debug("New routing loaded and activated.")
                    else : 
                        self._logger.warning("Could not load new routing.")
                self.detected_routing == None
        if (current!=self.moving):
            self.move_rover() # will repush current segment to TP with ajusted speed (0 or target), TP will manage resume right after last completed tick

    def initialize(self):
        # Do some checks and set check_succeeded accordingly
        self.checks_succeeded=True
        # Search initial routing
        if self.detected_routing == None :
            try :
                self.detectRouting()
                self._logger.debug("Initial routing found: "+str(self.detected_routing))
            except:
                self._logger.warning("No initial routing found.")
                # Try to load initial routing
        if ((self.detected_routing!=None) and (self.set_routing(self.detected_routing)!=0)):
            self.detected_routing == None
            self._logger.warning("Error loading initial routing.")
        if (self.checks_succeeded and  self.routing.routing_name!=None):
            self.initialization_completed=True
            self._logger.debug("Initialization completed and initial routing loaded")

    def loop(self):
        #Search for pending commands and process them
        self.getCommands()
        if (self.commands) :
            self.processCommands()
        else :
            self.get_active_segment_status()
            if self.routing.next_segment_needed :
                self.update_rover_position()
                self.routing.next_segment_needed=False
                if (self.routing.active_segment+1 < len(self.routing.segments)):
                    self._logger.info("Pushing next segment to TP. "+str(self.routing.active_segment+1))
                    self.push_segment(self.routing.active_segment+1)
                else:
                    pass
                    # last segment has been reached => stop and/or ask for a new routing
        
class routing(object):
    def __init__(self, name=None, routing_type=1):
        self.routing_name=name
        self.routing_type=routing_type #1 for liaison, 2 for mow
        self.perimeter=[] #[waypoints] in case of move perimeter = 2 points (origin and destination)
        self.segments=[]
        self.speed = 255 #one overall, speed of rotation segments hardcoded in motor driver
        self.active_segment=None
        self.active_segment_status=segment_status()
        self.next_segment_needed=False
    
    def buildSegments(self):
        if (self.routing_type==1) :
            # Calculate straight line liaison, each line is composed of a rotation segment to point in the right direction and a straight segment to move
            waypoint0=myrover.update_rover_position()
            i=1 # start segment ids @ 1, TP interprets 0 has no initial seg loaded
            for waypoint in self.perimeter:
                #need to confirm  atan angle in practice to be sure
                if (waypoint.x-waypoint0.x)==0 :
                    if ((waypoint.y-waypoint0.y)>0):
                        heading=0
                    else:
                        heading=127
                else:
                    heading = (int) (255/6.2832*(3.1416/2-atan2((waypoint.y-waypoint0.y),(waypoint.x-waypoint0.x))))
                self.segments.append(segment(0,i,0,heading,125)) # rotation speed to be determined
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
                
# where to place this + optimize for 1 million blocks to scan per heading on a 100m x 100m map
    def mowing_potential(self,heading) :
        # Returns # of unmown blocks in heading from myrover current position
        #y=ax+b => move to ax+by+c=0 to accomodate heading North and South
        mowing_pot=0

        i_rover=int(myrover.x/mymap.block_size)
        j_rover=int(myrover.y/mymap.block_size)
        
        if heading==0 :
            a=1 #heading = angle to north
            b=0
            c=-myrover.x
            delta=HALF_MOWING_WIDTH
        elif heading==180:
            a=-1 #heading = angle to south
            b=0
            c=myrover.x
            delta=HALF_MOWING_WIDTH
        else:
            a=tan(radians(heading+90))
            b=-1
            c=(myrover.y-a*myrover.x)
            delta=abs(HALF_MOWING_WIDTH/cos(radians(heading+90)))

#### a>0
####    C-x,C+y < y+z  &  C+x,C-y>y-z
#### a<0
####    C+x,C+y < y+z & C-x,C-y>y-z
            
        half_block=mymap.block_size/2

        if a > 0 and (heading < 180) :
            for i in range(i_rover+1):
#                print(heading,i)
                ji=int((mymap.blocks[i][0].center.x*a+c)/mymap.block_size)
                for j in range(max(0,ji-3),min(ji+3,len(mymap.blocks[i]))):
                    block_center=mymap.blocks[i][j].center
                    if ((block_center.x-half_block)*a+b*(block_center.y+half_block)+c+delta>=0) and ((block_center.x+half_block)*a+b*(block_center.y-half_block)+c-delta<=0):
                        if not mymap.blocks[i][j].mowned_status:
                            mowing_pot+=1
##                          else :
##                              print(block_center.x,block_center.y)
                        
        elif a > 0 and (heading > 180) :
            for i in range(i_rover,len(mymap.blocks)-1):
                ji=int((mymap.blocks[i][0].center.x*a+c)/mymap.block_size)
                for j in range(max(0,ji-3),min(ji+3,len(mymap.blocks[i]))):
                    block_center=mymap.blocks[i][j].center
                    if ((block_center.x-half_block)*a+b*(block_center.y+half_block)+c+delta>=0) and ((block_center.x+half_block)*a+b*(block_center.y-half_block)+c-delta<=0):
                        if not mymap.blocks[i][j].mowned_status :
                            mowing_pot+=1
##                          else :
##                              print(block_center.x,block_center.y)


        elif a < 0 and (heading < 180) :
            for i in range(i_rover+1):
#                print(heading,i)
                ji=int((mymap.blocks[i][0].center.x*a+c)/mymap.block_size)
                for j in range(max(0,ji-3),min(ji+3,len(mymap.blocks[i]))):
                    block_center=mymap.blocks[i][j].center
                    if ((block_center.x+half_block)*a+b*(block_center.y+half_block)+c+delta>=0) and ((block_center.x-half_block)*a+b*(block_center.y-half_block)+c-delta<=0):
                        if not mymap.blocks[i][j].mowned_status:
                            mowing_pot+=1
##                          else :
##                              print(block_center.x,block_center.y)
                        
        elif a < 0 and (heading > 180) :
            for i in range(i_rover,len(mymap.blocks)-1):
#                print(heading,i)
                ji=int((mymap.blocks[i][0].center.x*a+c)/mymap.block_size)
                for j in range(max(0,ji-3),min(ji+3,len(mymap.blocks[i]))):
                    block_center=mymap.blocks[i][j].center
                    if ((block_center.x+half_block)*a+b*(block_center.y+half_block)+c+delta>=0) and ((block_center.x-half_block)*a+b*(block_center.y-half_block)+c-delta<=0):
                        if not mymap.blocks[i][j].mowned_status :
                            mowing_pot+=1
##                        else :
##                            print(block_center.x,block_center.y)


        elif heading == 0 :
            for i in range(max(0,i_rover-3),min(i_rover+3,len(mymap.blocks)-1)):
                for j in range(j_rover,len(mymap.blocks[i])):
                    block_center=mymap.blocks[i][j].center
                    if ((block_center.x+half_block)+c+delta>=0) and ((block_center.x-half_block)+c-delta<=0):
                        if not mymap.blocks[i][j].mowned_status :
                            mowing_pot+=1
##                        else :
##                            print(block_center.x,block_center.y)

        elif heading ==180 :
            for i in range(max(0,i_rover-3),min(i_rover+3,len(mymap.blocks)-1)):
                for j in range(0,j_rover):
                    block_center=mymap.blocks[i][j].center
                    if (-block_center.x-half_block+c+delta>=0) and (-block_center.x+half_block+c-delta<=0):
                        if not mymap.blocks[i][j].mowned_status :
                            mowing_pot+=1
##                        else :
##                            print(block_center.x,block_center.y)

        return mowing_pot

                
    def loadRouting(self, new_routing):
        try:
            with open(routing_path+new_routing+".json", 'r') as routing_file:
              routing_data=json.load(routing_file)
            if (new_routing==routing_data["name"]):
                self.routing_name=routing_data["name"]
                self.routing_type=routing_data["routing_type"]
                for item in routing_data["perimeter"]:
                    # Need to add a check for file structure
                   self.perimeter.append(waypoint(item[0],item[1]))
                return 0
            else :
                print("Unconsistent routing name")
                return -1
        except OSError as e:
            print("File not found.")
            return -1

class waypoint(object):
    # point with coordinates in meters in local referential
    def __init__(self,x,y):
        self.x=x
        self.y=y

class GPSpoint(object):
    # point with GPS coordinates
    def __init__(self,lat,long):
        self.lat=lat
        self.long=long

    def moved(self,delta_lat,delta_long):
        return GPSpoint(self.lat+delta_lat, self.long+delta_long)

    def distance(self, destination):
        #distance to destination GPSpoint
        return acos(sin(radians(self.lat))*sin(radians(destination.lat))+cos(radians(self.lat))*cos(radians(destination.lat))*cos(radians(self.long-destination.long)))*6378*1000

class block(object):
    def __init__(self,center, mowned_status=False, obstacle=0):
        # obstacle 0=unknown, -1=clear, 1=permanent, 2=temporary
        self.center=center # coordinates in m in local referential
        self.mowned_status=mowned_status
        self.obstacle=obstacle
    
class Map(object):
    def __init__(self, name, origin, lat_height, long_width):
        # origin = GPSpoint lat and long = floats in decimal degrees
        self.block_size=0.1 #size of blocks in meter
        self.name=name
        self.origin=origin
        self.lat_height=lat_height
        self.long_width=long_width
        self.height=self.origin.distance(self.origin.moved(self.lat_height,0))
        self.width=self.origin.distance(self.origin.moved(0,self.long_width))
        self.blocks=[[]]
        print(self.height, self.width)
        for i in range(0,int(self.width/self.block_size)):
            self.blocks.append([])
            for j in range(0,int(self.height/self.block_size)):
                self.blocks[i].append(block(waypoint(self.block_size/2+i*self.block_size, self.block_size/2+j*self.block_size),False,0))

    def load(self):
        pass

    def save(self):
        pass


class segment(object):
    def __init__(self,segment_type=1,segment_id=0,target_ticks=None, target_bearing8=None, target_speed=250):
        self.segment_type=segment_type #0 in case rotation, 1 in case straight
        self.segment_id=segment_id
        self.target_ticks=target_ticks
        self.target_bearing8=target_bearing8
        self.target_speed=target_speed

    def print(self):
        print("Segment Type: ",self.segment_type) #0 in case rotation, 1 in case straight
        print("Segment id: ",self.segment_id)
        print("Segment target ticks: ",self.target_ticks)
        print("Segment target bearing: ", self.target_bearing8)
        print("Segment target speed: ", self.target_speed)
        print("")
        

class segment_status(object):
    #segment status stores progress in delivering current segment from TP Board
    def __init__(self,segment_id=None):
        self.segment_type=0 #0 in case rotation, 1 in case straight
        self.segment_id=segment_id
        self.ticks_cum=0
        self.millis_cum=0
        self.average_bearing8=None
        self.current_bearing8=None
        #self.teta_point=None not implemented
        self.speed_step=None #speed_step is the speed mesured between the last two position measures made by TP board (@10Hz)
## Opportunity to simplify by moving status variables to segment class


if __name__ == "__main__":
    print("Welcome to my rover, Yves !")
    # create logger 'Rover_log'
    logger = logging.getLogger('Rover_log')
    logger.setLevel(logging.DEBUG)    
    # create file handler which logs even debug messages
    fh = logging.FileHandler('rover.log')
    fh.setLevel(logging.WARNING)
    # create console handler with a higher log level
    ch = logging.StreamHandler()
    ch.setLevel(logging.DEBUG)
    # create formatter and add it to the handlers
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    fh.setFormatter(formatter)
    ch.setFormatter(formatter)
    # add the handlers to the logger
    logger.addHandler(fh)
    logger.addHandler(ch)


## Initialization ##
    try:
        myrover=Rover()
        logger.debug("My rover object initialized")

        origin=GPSpoint(47.495518, 2.065403)
        mymap=Map("Jardin", origin,0.000277,-0.000414)
        logger.debug("My map object initialized")

        try:
            #Try initialization every second until success
            while not myrover.initialization_completed :
                myrover.initialize()
                time.sleep(0.1)

            while True:
                myrover.loop()
                time.sleep(1)
                print("Position du rover:", myrover.x,":", myrover.y, " @ ", datetime.datetime.now())
                                                
        except KeyboardInterrupt:
            print('User interrupted!')
    except:
        print("Unknown error")



##            for i in range(0,361,10):
##                print(i,":",myrover.routing.fast_mowing_potential(i)) # 616/622
