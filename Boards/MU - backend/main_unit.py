import sqlite3

roverdb='C:\\Users\\Yves1812\\Documents\\GitHub\\Robot_tondeuse\\Boards\\MU - web server\\app.db'

class rover(object):
    def __init__(self,x=0.0,y=0.0,speedx=0.0,speedy=0.0,ice_status=False, battery_level=None,traction_power=None, messages=[]):
        self.x=x
        self.y=y
        self.vx=speedx
        self.vy=speedy
        self.ice_status=ice_status
        self.battery_level=battery_level
        self.traction_power=traction_power
        self.messages=messages
        self.conn = sqlite3.connect(roverdb)
        self.conn.row_factory = sqlite3.Row
        self.cursor=self.conn.cursor()

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

    def get_rover_status(self):
        pass

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
myrover.query_db_status()
print('x=',myrover.x)
print('y=',myrover.y)

