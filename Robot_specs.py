from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch

class Robot:
    _instance = None  
    
    def __init__(self):
        self.hub = PrimeHub()
        
        self.left_motor = Motor(Port.F, Direction.COUNTERCLOCKWISE)
        self.right_motor = Motor(Port.B, Direction.CLOCKWISE)
      
        DIAMETRO_RUEDA = 63
        DISTANCIA_EJES = 152
        
        self.drive_base = DriveBase(
            self.left_motor, 
            self.right_motor, 
            wheel_diameter=DIAMETRO_RUEDA, 
            axle_track=DISTANCIA_EJES
        )
        
        self.drive_base.settings(
            straight_speed=700,         
            straight_acceleration=1800, 
            turn_rate=700,            
            turn_acceleration=2000      
        )
        
        self.drive_base.use_gyro(False)
        
        self.stopwatch = StopWatch()
        self.hub.imu.settings(angular_velocity_threshold=3, acceleration_threshold=300)

    @classmethod
    def get_instance(cls):
        if cls._instance is None:
            cls._instance = cls()  
        return cls._instance  

    def get_stopwatch(self):
        return self.stopwatch   
    
def get_robot():
    return Robot.get_instance()
