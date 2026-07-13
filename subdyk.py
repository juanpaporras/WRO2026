from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Direction
from pybricks.tools import StopWatch

class Robot:
    _instance = None  
    def __init__(self):
        self.hub = PrimeHub()
        self.left_motor = Motor(Port.F, Direction.COUNTERCLOCKWISE)
        self.right_motor = Motor(Port.B, Direction.CLOCKWISE)
        ##self.left_arm = Motor(Port.E, Direction.CLOCKWISE)
        ##self.right_arm = Motor(Port.B, Direction.CLOCKWISE)
        #self.sensor_left = ColorSensor(Port.A) 
        #self.sensor_right = ColorSensor(Port.D)
        self.stopwatch = StopWatch()
        self.hub.imu.settings(angular_velocity_threshold=2, acceleration_threshold=300)
        print("eyender is ready")  
    @classmethod
    def get_instance(cls):
        if cls._instance is None:
            cls._instance = cls()  
        return cls._instance  

    def get_stopwatch(self):
        return self.stopwatch   
    
def get_robot():
    return Robot.get_instance()