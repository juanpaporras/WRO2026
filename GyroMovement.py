import math
from Robot_specs import get_robot
from pybricks.pupdevices import Motor
from pybricks.parameters import Port
from pybricks.tools import wait, StopWatch

# =============================================================================
# INICIALIZACIÓN DE HARDWARE Y ACTUADORES
# =============================================================================

robot = get_robot()
hub = robot.hub
left_motor = robot.left_motor
right_motor = robot.right_motor

# Configuración del IMU para mitigar vibraciones mecánicas
hub.imu.settings(angular_velocity_threshold=3.0, acceleration_threshold=250)

# =============================================================================
# FUNCIONES AUXILIARES Y MATEMÁTICAS
# =============================================================================

def obtener_heading_normalizado():
    raw_heading = hub.imu.heading()
    return ((raw_heading + 180) % 360) - 180

def normalizar_error_angular(error):
    return ((error + 180) % 360) - 180

# =============================================================================
# FUNCIONES DE MOVIMIENTO Y GIROS (CONTROL DC + PD)
# =============================================================================

async def avance(target_heading, distancia_mm, max_dc=100.0, min_dc=16.0, accel_mm=50.0, decel_mm=140.0, kp=3.5, kd=0.18):
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)

    DIAMETRO_RUEDA = 63.0
    FACTOR_CONVERSION = (DIAMETRO_RUEDA * 3.141) / 360.0

    recorrido = 0.0
    error_previo = 0.0
    derivada_filtrada = 0.0
    ALFA_FILTRO = 0.35

    cronometro_lazo = StopWatch()
    cronometro_lazo.reset()

    while abs(recorrido) < abs(distancia_mm):
        dt = cronometro_lazo.time() / 1000.0
        cronometro_lazo.reset()
        if dt <= 0: dt = 0.001

        ang_izq = left_motor.angle()
        ang_der = right_motor.angle()
        recorrido = ((ang_izq + ang_der) / 2.0) * FACTOR_CONVERSION
        
        dist_restante = max(0.0, abs(distancia_mm) - abs(recorrido))

        if abs(recorrido) < accel_mm:
            proporcion = abs(recorrido) / accel_mm
            dc_base = min_dc + (max_dc - min_dc) * proporcion
        elif dist_restante < decel_mm:
            proporcion = dist_restante / decel_mm
            dc_base = min_dc + (max_dc - min_dc) * proporcion
        else:
            dc_base = max_dc  
            
        signo_movimiento = 1 if distancia_mm > 0 else -1
        dc_base *= signo_movimiento

        heading_actual = hub.imu.heading()
        error_ang = target_heading - heading_actual
        error_ang = (error_ang + 180) % 360 - 180

        derivada_cruda = (error_ang - error_previo) / dt
        derivada_filtrada = (derivada_filtrada * (1.0 - ALFA_FILTRO)) + (derivada_cruda * ALFA_FILTRO)

        termino_p = kp * error_ang
        termino_d = kd * derivada_filtrada
        correccion = termino_p + termino_d

        dc_izq = max(min(dc_base + correccion, 100.0), -100.0)
        dc_der = max(min(dc_base - correccion, 100.0), -100.0)

        left_motor.dc(dc_izq)
        right_motor.dc(dc_der)

        error_previo = error_ang
        await wait(20)

    left_motor.hold()
    right_motor.hold()
    await wait(80) 


async def retrocede(target_heading, distancia_mm, max_dc=100.0, min_dc=16.0, accel_mm=50.0, decel_mm=140.0, kp=3.5, kd=0.18):
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)

    DIAMETRO_RUEDA = 63.0
    FACTOR_CONVERSION = (DIAMETRO_RUEDA * 3.141) / 360.0

    recorrido = 0.0
    error_previo = 0.0
    derivada_filtrada = 0.0
    ALFA_FILTRO = 0.35

    cronometro_lazo = StopWatch()
    cronometro_lazo.reset()

    while abs(recorrido) < abs(distancia_mm):
        dt = cronometro_lazo.time() / 1000.0
        cronometro_lazo.reset()
        if dt <= 0: dt = 0.001

        ang_izq = left_motor.angle()
        ang_der = right_motor.angle()
        recorrido = ((abs(ang_izq) + abs(ang_der)) / 2.0) * FACTOR_CONVERSION
        
        dist_restante = max(0.0, abs(distancia_mm) - abs(recorrido))

        if abs(recorrido) < accel_mm:
            proporcion = abs(recorrido) / accel_mm
            dc_base = min_dc + (max_dc - min_dc) * proporcion
        elif dist_restante < decel_mm:
            proporcion = dist_restante / decel_mm
            dc_base = min_dc + (max_dc - min_dc) * proporcion
        else:
            dc_base = max_dc
            
        dc_base *= -1

        heading_actual = hub.imu.heading()
        error_ang = target_heading - heading_actual
        error_ang = (error_ang + 180) % 360 - 180

        derivada_cruda = (error_ang - error_previo) / dt
        derivada_filtrada = (derivada_filtrada * (1.0 - ALFA_FILTRO)) + (derivada_cruda * ALFA_FILTRO)

        termino_p = kp * error_ang
        termino_d = kd * derivada_filtrada
        correccion = termino_p + termino_d

        dc_izq = max(min(dc_base - correccion, 100.0), -100.0)
        dc_der = max(min(dc_base + correccion, 100.0), -100.0)

        left_motor.dc(dc_izq)
        right_motor.dc(dc_der)

        error_previo = error_ang
        await wait(20)

    left_motor.hold()
    right_motor.hold()
    await wait(80)


async def giro(target_heading, max_dc=100.0, min_dc=22.0, kp=2.0, kd=0.05, tolerancia=1, tiempo_estabilizacion=80):
    error_previo = 0.0
    derivada_filtrada = 0.0
    ALFA_FILTRO = 0.35 

    cronometro_lazo = StopWatch()
    cronometro_estabilizacion = StopWatch()
    cronometro_lazo.reset()
    cronometro_estabilizacion.reset()
    
    estabilizando = False

    while True:
        dt = cronometro_lazo.time() / 1000.0
        cronometro_lazo.reset()
        if dt <= 0: dt = 0.001

        heading_actual = hub.imu.heading()
        error_ang = target_heading - heading_actual
        error_ang = (error_ang + 180) % 360 - 180 
        
        abs_error = abs(error_ang)

        derivada_cruda = (error_ang - error_previo) / dt
        derivada_filtrada = (derivada_filtrada * (1.0 - ALFA_FILTRO)) + (derivada_cruda * ALFA_FILTRO)
        
        termino_d = kd * derivada_filtrada

        if abs_error <= tolerancia:
            left_motor.hold()
            right_motor.hold()
            
            if not estabilizando:
                estabilizando = True
                cronometro_estabilizacion.reset()
            elif cronometro_estabilizacion.time() >= tiempo_estabilizacion:
                break
        else:
            estabilizando = False
            
            ZONA_FRENADO = 35.0
            if abs_error > ZONA_FRENADO:
                limite_dc = max_dc
                termino_p = max_dc if error_ang > 0 else -max_dc
            else:
                proporcion = abs_error / ZONA_FRENADO
                limite_dc = min_dc + (max_dc - min_dc) * proporcion
                termino_p = kp * error_ang  
            
            salida_pd = termino_p + termino_d
            salida_aplicada = max(min(salida_pd, limite_dc), -limite_dc)
            
            if abs(salida_aplicada) < min_dc:
                salida_aplicada = min_dc if error_ang > 0 else -min_dc

            left_motor.dc(salida_aplicada)
            right_motor.dc(-salida_aplicada)

        error_previo = error_ang
        await wait(10)

    left_motor.hold()
    right_motor.hold()
    await wait(60)


async def giro_pivot(target_heading, pivote_izquierdo=True, max_dc=80.0, min_dc=18.0, kp=2.2, kd=0.15, tolerancia=1, tiempo_estabilizacion=80):
    error_previo = 0.0
    derivada_filtrada = 0.0
    ALFA_FILTRO = 0.35 

    cronometro_lazo = StopWatch()
    cronometro_estabilizacion = StopWatch()
    cronometro_lazo.reset()
    cronometro_estabilizacion.reset()
    
    estabilizando = False

    if pivote_izquierdo:
        left_motor.hold()
    else:
        right_motor.hold()

    while True:
        dt = cronometro_lazo.time() / 1000.0
        cronometro_lazo.reset()
        if dt <= 0: dt = 0.001

        heading_actual = hub.imu.heading()
        error_ang = target_heading - heading_actual
        error_ang = (error_ang + 180) % 360 - 180 
        abs_error = abs(error_ang)

        derivada_cruda = (error_ang - error_previo) / dt
        derivada_filtrada = (derivada_filtrada * (1.0 - ALFA_FILTRO)) + (derivada_cruda * ALFA_FILTRO)
        
        termino_d = kd * derivada_filtrada

        if abs_error <= tolerancia:
            left_motor.hold()
            right_motor.hold()
            
            if not estabilizando:
                estabilizando = True
                cronometro_estabilizacion.reset()
            elif cronometro_estabilizacion.time() >= tiempo_estabilizacion:
                break
        else:
            estabilizando = False
            
            ZONA_FRENADO = 30.0
            if abs_error > ZONA_FRENADO:
                limite_dc = max_dc
                termino_p = max_dc if error_ang > 0 else -max_dc
            else:
                proporcion = abs_error / ZONA_FRENADO
                limite_dc = min_dc + (max_dc - min_dc) * proporcion
                termino_p = kp * error_ang
            
            salida_pd = termino_p + termino_d
            salida_aplicada = max(min(salida_pd, limite_dc), -limite_dc)
            
            if abs(salida_aplicada) < min_dc:
                salida_aplicada = min_dc if error_ang > 0 else -min_dc

            if pivote_izquierdo:
                right_motor.dc(-salida_aplicada)
            else:
                left_motor.dc(salida_aplicada)

        error_previo = error_ang
        await wait(10)

    left_motor.hold()
    right_motor.hold()
    await wait(60)