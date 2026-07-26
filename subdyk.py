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
      
        DIAMETRO_RUEDA = 62.5
        DISTANCIA_EJES =152.2
        
    
        self.drive_base = DriveBase(
            self.left_motor, 
            self.right_motor, 
            wheel_diameter=DIAMETRO_RUEDA, 
            axle_track=DISTANCIA_EJES
        )
        
    

        self.drive_base.settings(
            straight_speed=450,         
            straight_acceleration=450, 
            turn_rate=350,            
            turn_acceleration=800      
        )
        
        self.drive_base.use_gyro(True)
        
        self.stopwatch = StopWatch()
        self.hub.imu.settings(angular_velocity_threshold=2, acceleration_threshold=300)

    @classmethod
    def get_instance(cls):
        if cls._instance is None:
            cls._instance = cls()  
        return cls._instance  

    def get_stopwatch(self):
        return self.stopwatch   
    
def get_robot():
    return Robot.get_instance()