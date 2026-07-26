from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch

class Robot:
    _instance = None  
    
    def __init__(self):
        self.hub = PrimeHub()
        
        # Inversión de giros según el ensamble del robot
        self.left_motor = Motor(Port.F, Direction.COUNTERCLOCKWISE)
        self.right_motor = Motor(Port.B, Direction.CLOCKWISE)
        
        # Parámetros geométricos de la base motriz (en milímetros)
        DIAMETRO_RUEDA = 62.5
        DISTANCIA_EJES =152.2  # L_teórico: Debe ajustarse con la prueba empírica
        
        # Inicialización del núcleo cinemático en C
        self.drive_base = DriveBase(
            self.left_motor, 
            self.right_motor, 
            wheel_diameter=DIAMETRO_RUEDA, 
            axle_track=DISTANCIA_EJES
        )
        
        # Perfiles trapezoidales: straight_speed, straight_acceleration, turn_rate, turn_acceleration
        # En subdyk.py, dentro de tu clase Robot:

        self.drive_base.settings(
            straight_speed=450,         # Límite físico nominal bajo carga (aprox 130-140 RPM)
            straight_acceleration=450, # Aceleración sumamente agresiva, alcanza vel max en ~0.37s
            turn_rate=350,              # Giros veloces sin inducir derrape centrífugo excesivo
            turn_acceleration=800       # Rampa rápida para acoplamiento de giros pivotados
        )
        
        # Activación de la compensación de rumbo (Fusión Sensorial con el IMU interno)
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