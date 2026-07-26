from subdyk import get_robot
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Button, Port, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
import math
from pybricks.iodevices import PUPDevice

class SensorLineaMBC_Diagnostico:
    def __init__(self, puerto: Port):
        self.dispositivo = PUPDevice(puerto)
        self.error_espacial = 0.0
        self.ancho_linea = 0.0
        print("INFO DEL SENSOR:", self.dispositivo.info())

    async def actualizar_lectura(self):
        try:
            data = await self.dispositivo.read(9)
            if len(data) >= 7:
                d5 = (data[4] << 8) | data[5] 
                self.error_espacial = float((data[4] & 0xFF) - 8)
                self.ancho_linea = float((data[4] >> 8) & 0xFF) 
            else:
                print("Trama corta:", data)
        except Exception as e:
            print("ERROR:", e)

sensor_mbc = SensorLineaMBC_Diagnostico(Port.D)

robot = get_robot()
hub = robot.hub
left_motor = robot.left_motor
right_motor = robot.right_motor

async def obtener_error_lpf2():
    await sensor_mbc.actualizar_lectura()
    return sensor_mbc.error_espacial

def obtener_target_continuo(target_relativo, heading_actual):
    diferencia = (target_relativo - heading_actual) % 360
    if diferencia > 180:
        diferencia -= 360
    return heading_actual + diferencia

def obtener_heading_normalizado():
    raw_heading = hub.imu.heading()
    return ((raw_heading + 180) % 360) - 180

def normalizar_error_angular(error):
    return ((error + 180) % 360) - 180

async def getBlackLineError():
    valor_reflexion = await line.reflection()
    return (valor_reflexion / 10) - 8

async def avance_gyro(target_heading, final_angle, max_speed=85, stall_stop=False):
    heading_actual = hub.imu.heading()
    diferencia = (target_heading - heading_actual) % 360
    if diferencia > 180:
        diferencia -= 360
        
    if abs(diferencia) >= 0.5:
        robot.drive_base.turn(diferencia, wait=False)
        while not robot.drive_base.done():
            await wait(10)
            
    # Escalado de max_speed (porcentaje 0-100) a velocidad lineal en mm/s (máx 450 mm/s)
    vel_mm_s = (max_speed / 100.0) * 450.0
    robot.drive_base.settings(straight_speed=vel_mm_s)
            
    distancia_mm = (final_angle / 360.0) * (math.pi * 62.5)
    robot.drive_base.straight(distancia_mm, wait=False)
    
    while not robot.drive_base.done():
        if stall_stop and robot.drive_base.stalled():
            robot.drive_base.stop()
            break
        await wait(10)

async def retrocede_gyro(target_heading, final_angle, max_speed=80, stall_stop=False):
    heading_actual = hub.imu.heading()
    diferencia = (target_heading - heading_actual) % 360
    if diferencia > 180:
        diferencia -= 360
        
    if abs(diferencia) >= 0.5:
        robot.drive_base.turn(diferencia, wait=False)
        while not robot.drive_base.done():
            await wait(10)
    vel_mm_s = (max_speed / 100.0) * 450.0
    robot.drive_base.settings(straight_speed=vel_mm_s)
            
    distancia_mm = (final_angle / 360.0) * (math.pi * 62.5)
    robot.drive_base.straight(-distancia_mm, wait=False)
    
    while not robot.drive_base.done():
        if stall_stop and robot.drive_base.stalled():
            robot.drive_base.stop()
            break
        await wait(10)

async def giro(target_heading, tipo_carga="vacio"):
    if tipo_carga == "pesado":
        robot.drive_base.settings(turn_rate=150, turn_acceleration=200)
    elif tipo_carga == "medio":
        robot.drive_base.settings(turn_rate=220, turn_acceleration=300)
    else:
        robot.drive_base.settings(turn_rate=250, turn_acceleration=400)

    heading_actual = hub.imu.heading()
    diferencia = (target_heading - heading_actual) % 360
    if diferencia > 180:
        diferencia -= 360
        
    if abs(diferencia) < 0.5:
        return
        
    robot.drive_base.turn(diferencia, wait=False)
    while not robot.drive_base.done():
        await wait(10)
        
    left_motor.hold()
    right_motor.hold()
    await hub.speaker.beep(1000, 50)

async def giro_pivot(target_heading, pivote_izquierdo=True, tipo_carga="vacio"):
    if tipo_carga == "pesado":
        max_speed, min_speed, decel_angle = 100, 40, 70.0
        kp, ki, kd, zona_integral, limite_integral = 3.0, 1.0, 0.06, 12.0, 15.0
    elif tipo_carga == "medio":
        max_speed, min_speed, decel_angle = 100, 35, 60.0
        kp, ki, kd, zona_integral, limite_integral = 3.1, 0.9, 0.05, 11.0, 15.0
    else:
        max_speed, min_speed, decel_angle = 100, 32, 50.0
        kp, ki, kd, zona_integral, limite_integral = 3.2, 0.8, 0.05, 10.0, 15.0

    heading_inicial = hub.imu.heading()
    diferencia = (target_heading - heading_inicial) % 360
    if diferencia > 180:
        diferencia -= 360
    target_continuo = heading_inicial + diferencia
    
    error = target_continuo - hub.imu.heading()
    prev_error = error 
    
    integral_error = 0.0
    derivada_filtrada = 0.0
    ALFA_FILTRO = 0.3  
    VOLTAJE_REFERENCIA = 8200.0  
    current_vel_limit = min_speed
    
    timer_lazo = StopWatch()
    timer_lazo.reset()
    watch_estancamiento = StopWatch()
    watch_estancamiento.reset()
    error_anterior_bloqueo = 999.0
    
    if pivote_izquierdo:
        left_motor.hold()
    else:
        right_motor.hold()
        
    while True:
        dt = timer_lazo.time() / 1000.0
        timer_lazo.reset()
        if dt <= 0.0: dt = 0.01
        
        error = target_continuo - hub.imu.heading()
        abs_error = abs(error)
        
        if abs_error > 360 or abs_error <= 0.5:
            break
            
        if abs(error - error_anterior_bloqueo) < 0.02:
            if watch_estancamiento.time() > 800: 
                break
        else:
            watch_estancamiento.reset()
            error_anterior_bloqueo = error
            
        if abs_error <= decel_angle:
            escala = abs_error / decel_angle
            current_vel_limit = min_speed + (max_speed - min_speed) * escala
        else:
            if current_vel_limit < max_speed:
                current_vel_limit += 1.5
            else:
                current_vel_limit = max_speed
                
        termino_p = kp * error
        if (error * prev_error) <= 0:
            integral_error = 0.0
            
        if abs_error < zona_integral:
            integral_error += error * dt
            integral_error = max(min(integral_error, limite_integral), -limite_integral)
        else:
            integral_error = 0.0
            
        termino_i = ki * integral_error
        derivada_cruda = (error - prev_error) / dt
        derivada_filtrada = (derivada_filtrada * (1.0 - ALFA_FILTRO)) + (derivada_cruda * ALFA_FILTRO)
        termino_d = kd * derivada_filtrada
        
        salida_pid = termino_p + termino_i + termino_d
        salida_pid = max(min(salida_pid, current_vel_limit), -current_vel_limit)
        
        if (salida_pid * error) > 0 and abs_error > 0.8:
            if abs(salida_pid) < min_speed:
                salida_pid = min_speed if salida_pid > 0 else -min_speed
                
        voltaje_actual = max(hub.battery.voltage(), 5000)
        factor_voltaje = VOLTAJE_REFERENCIA / voltaje_actual
        velocidad_motor = max(min(salida_pid * factor_voltaje, 100.0), -100.0)
        
        if pivote_izquierdo:
            right_motor.dc(-velocidad_motor)
        else:
            left_motor.dc(velocidad_motor)
            
        prev_error = error
        await wait(10)
        
    left_motor.hold()
    right_motor.hold()
    await hub.speaker.beep(1000, 50)

async def giro_arco(radio_mm, angulo_giro=90, velocidad_crucero=400):
    robot.drive_base.settings(straight_speed=velocidad_crucero, turn_rate=300)
    robot.drive_base.curve(radio_mm, angulo_giro, wait=False)
    
    while not robot.drive_base.done():
        await wait(10)
        
    left_motor.hold()
    right_motor.hold()
    await hub.speaker.beep(1000, 50)

async def avance_hasta_linea(target_heading, stall_stop=False):
    heading_actual = hub.imu.heading()
    diferencia = (target_heading - heading_actual) % 360
    if diferencia > 180:
        diferencia -= 360
        
    if abs(diferencia) >= 0.5:
        robot.drive_base.turn(diferencia, wait=False)
        while not robot.drive_base.done():
            await wait(10)
            
    robot.drive_base.straight(2000, wait=False)
    
    while not robot.drive_base.done():
        error_mbc = await obtener_error_lpf2()
        
        if abs(error_mbc) < 6:
            robot.drive_base.stop()
            break
            
        if stall_stop and robot.drive_base.stalled():
            robot.drive_base.stop()
            break
            
        await wait(1)

async def retrocede_hasta_linea_gyro(target_heading, stall_stop=True, umbral_linea=1.0):
    heading_actual = hub.imu.heading()
    diferencia = (target_heading - heading_actual) % 360
    if diferencia > 180:
        diferencia -= 360
        
    if abs(diferencia) >= 0.5:
        robot.drive_base.turn(diferencia, wait=False)
        while not robot.drive_base.done():
            await wait(10)
            
    robot.drive_base.straight(-2000, wait=False)
    lecturas_linea_consecutivas = 0
    
    while not robot.drive_base.done():
        error_linea = await getBlackLineError()
        
        if abs(error_linea) < umbral_linea:
            lecturas_linea_consecutivas += 1
            if lecturas_linea_consecutivas >= 3:
                robot.drive_base.stop()
                break  
        else:
            lecturas_linea_consecutivas = 0  
            
        if stall_stop and robot.drive_base.stalled():
            robot.drive_base.stop()
            break
            
        await wait(1)