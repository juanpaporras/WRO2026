from pybricks.parameters import Port
from pybricks.tools import wait, StopWatch
from pybricks.iodevices import PUPDevice
from GyroMovement import hub, left_motor, right_motor, robot, giro

# =============================================================================
# CONFIGURACIÓN DEL SENSOR MBC
# =============================================================================

class SensorLineaMBC:
    def __init__(self, puerto: Port):
        self.dispositivo = PUPDevice(puerto)
        self.error_espacial = 0.0
        self.ancho_linea = 0.0
        
        # PRE-ASIGNACIÓN DE MEMORIA: 
        # Creamos la lista una sola vez en el arranque para evitar 
        # asignaciones dinámicas en el bucle principal.
        self.valores_individuales = [0] * 8  

    async def actualizar_lectura(self):
        try:
            data = await self.dispositivo.read(9)
            if len(data) >= 5:
                # Modificación directa "in-place" sobre la lista existente
                vals = self.valores_individuales
                
                # Desempaquetado optimizado de los 4 bytes de luz (8 valores de 8 bits)
                d0, d1, d2, d3 = data[0], data[1], data[2], data[3]
                
                vals[0] = d0 & 0xFF
                vals[1] = (d0 >> 8) & 0xFF
                vals[2] = d1 & 0xFF
                vals[3] = (d1 >> 8) & 0xFF
                vals[4] = d2 & 0xFF
                vals[5] = (d2 >> 8) & 0xFF
                vals[6] = d3 & 0xFF
                vals[7] = (d3 >> 8) & 0xFF
                
                # Extracción directa de error espacial y ancho de línea
                meta = data[4]
                self.error_espacial = float((meta & 0xFF) - 8)
                self.ancho_linea = float((meta >> 8) & 0xFF) 
        except Exception:
            pass

sensor_mbc = SensorLineaMBC(Port.D)

async def getBlackLineWidth():
    await sensor_mbc.actualizar_lectura()
    return sensor_mbc.ancho_linea

# =============================================================================
# FUNCIONES DE SEGUIMIENTO ÓPTICO Y LÍNEAS
# =============================================================================

async def seguir_linea_distancia(distancia_mm, vel_crucero_mm_s=700, accel_dist_mm=50, decel_dist_mm=100, vel_minima_mm_s=80, kp=3, kd=2.1, ki=0.0, freno=True):
    start_dist = robot.drive_base.distance()
    last_error = 0.0
    derivada_filtrada = 0.0
    alfa_filtro = 0.35 
    MAX_TURN_RATE = 60.0 
    dt_fijo = 0.010
    
    while True:
        dist_actual = robot.drive_base.distance() - start_dist
        if dist_actual >= distancia_mm:
            break 
            
        dist_faltante = distancia_mm - dist_actual
        if dist_actual < accel_dist_mm:
            escala = dist_actual / accel_dist_mm
            vel_lineal = vel_minima_mm_s + (vel_crucero_mm_s - vel_minima_mm_s) * escala
        elif dist_faltante < decel_dist_mm and not freno:
            escala = dist_faltante / decel_dist_mm
            vel_lineal = vel_minima_mm_s + (vel_crucero_mm_s - vel_minima_mm_s) * escala
        else:
            vel_lineal = vel_crucero_mm_s
            
        await sensor_mbc.actualizar_lectura()
        error = sensor_mbc.error_espacial
        
        P = error
        derivada_cruda = (error - last_error) / dt_fijo
        derivada_filtrada = (derivada_filtrada * (1.0 - alfa_filtro)) + (derivada_cruda * alfa_filtro)
        D = derivada_filtrada
        
        if abs(error) <= 0.2:
            error = 0.0
            
        turn_rate = -((kp * P) + (kd * D))
        turn_rate = max(min(turn_rate, MAX_TURN_RATE), -MAX_TURN_RATE)
        
        robot.drive_base.drive(vel_lineal, turn_rate)
        last_error = error
        await wait(10)

    if freno:
        robot.drive_base.stop()
        await wait(50) 


async def seguir_linea_hasta_interseccion(lado="centro", vel_crucero_mm_s=600, kp=5, kd=2.8, ki=0.0, umbral_interseccion=8, centrado_estricto=False, target_heading=None, montarse=True, distancia_montaje=20):
    last_error = 0.0
    derivada_filtrada = 0.0
    alfa_filtro = 0.35
    MAX_TURN_RATE = 60.0
    dt_fijo = 0.010 
    
    if lado == "izquierda":
        umbral_interseccion = 7
        setpoint = -6.0
        vel_crucero_mm_s = 300
    elif lado == "derecha": 
        umbral_interseccion = 7
        setpoint = 6.0 
        vel_crucero_mm_s = 300
    else: 
        setpoint = 0.0  
        
    while True:
        await sensor_mbc.actualizar_lectura()
        ancho_linea = int(sensor_mbc.ancho_linea)
        
        if ancho_linea >= umbral_interseccion:
            if not montarse:
                robot.drive_base.stop()
                break 
            else:
                robot.drive_base.drive(vel_crucero_mm_s, 0)
                distancia_referencia = robot.drive_base.distance()
                
                while abs(robot.drive_base.distance() - distancia_referencia) < distancia_montaje:
                    await wait(1)
                
                robot.drive_base.stop()
                
                if target_heading is not None:
                    await giro(target_heading)
                break 

        error = sensor_mbc.error_espacial - setpoint
        P = error
        derivada_cruda = (error - last_error) / dt_fijo
        derivada_filtrada = (derivada_filtrada * (1.0 - alfa_filtro)) + (derivada_cruda * alfa_filtro)
        D = derivada_filtrada
        
        turn_rate = -((kp * P) + (kd * D))
        turn_rate = max(min(turn_rate, MAX_TURN_RATE), -MAX_TURN_RATE)

        robot.drive_base.drive(vel_crucero_mm_s, turn_rate)
        
        last_error = error
        await wait(10)
        
    await wait(150) 
    
    if centrado_estricto:
        heading_lock = target_heading if target_heading is not None else hub.imu.heading()
        
        while True:
            ancho_actual = await getBlackLineWidth()
            if ancho_actual >= 7:
                break 
                
            error_yaw = heading_lock - hub.imu.heading()
            error_yaw = ((error_yaw + 180) % 360) - 180
            
            robot.drive_base.drive(-45, error_yaw * 2.5) 
            await wait(10)
            
        robot.drive_base.stop()
        await wait(100) 
        
        if target_heading is not None:
            await giro(target_heading)
            robot.drive_base.stop()


async def avance_hasta_linea(target_heading, distancia_ignorar=0, umbral_negro=10, umbral_blanco=70, montarse=False, distancia_montaje=50, min_sensores_linea=2, max_dc=80.0, min_dc=20.0, kp=3.5, kd=0.18):
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)

    DIAMETRO_RUEDA = 63.0
    FACTOR_CONVERSION = (DIAMETRO_RUEDA * 3.141) / 360.0

    error_previo = 0.0
    derivada_filtrada = 0.0
    ALFA_FILTRO = 0.35
    cronometro_lazo = StopWatch()
    cronometro_lazo.reset()

    while distancia_ignorar > 0:
        ang_izq = left_motor.angle()
        ang_der = right_motor.angle()
        recorrido = ((ang_izq + ang_der) / 2.0) * FACTOR_CONVERSION
        if abs(recorrido) >= distancia_ignorar:
            break

        dt = cronometro_lazo.time() / 1000.0
        cronometro_lazo.reset()
        if dt <= 0: dt = 0.001

        heading_actual = hub.imu.heading()
        error_ang = target_heading - heading_actual
        error_ang = (error_ang + 180) % 360 - 180

        termino_p = kp * error_ang
        correccion = max(min(termino_p, 15.0), -15.0)

        left_motor.dc(max_dc + correccion)
        right_motor.dc(max_dc - correccion)
        await wait(10)

    estado_busqueda = 0  
    lecturas_consecutivas = 0

    while True:
        dt = cronometro_lazo.time() / 1000.0
        cronometro_lazo.reset()
        if dt <= 0: dt = 0.001

        await sensor_mbc.actualizar_lectura()
        valores = sensor_mbc.valores_individuales

        if estado_busqueda == 0:
            sensores_blancos = sum(1 for v in valores if v >= umbral_blanco)
            if sensores_blancos >= min_sensores_linea:
                lecturas_consecutivas += 1
                if lecturas_consecutivas >= 2:
                    estado_busqueda = 1  
                    lecturas_consecutivas = 0
            else:
                lecturas_consecutivas = 0
                
        elif estado_busqueda == 1:
            sensores_negros = sum(1 for v in valores if v <= umbral_negro)
            if sensores_negros >= min_sensores_linea:
                lecturas_consecutivas += 1
                if lecturas_consecutivas >= 2:
                    if not montarse:
                        break
                    else:
                        left_motor.reset_angle(0)
                        while True:
                            ang_actual = (abs(left_motor.angle()) + abs(right_motor.angle())) / 2.0 * FACTOR_CONVERSION
                            if ang_actual >= distancia_montaje:
                                break
                            await wait(5)
                        break
            else:
                lecturas_consecutivas = 0

        heading_actual = hub.imu.heading()
        error_ang = target_heading - heading_actual
        error_ang = (error_ang + 180) % 360 - 180

        derivada_cruda = (error_ang - error_previo) / dt
        derivada_filtrada = (derivada_filtrada * (1.0 - ALFA_FILTRO)) + (derivada_cruda * ALFA_FILTRO)
        
        correccion = (kp * error_ang) + (kd * derivada_filtrada)
        
        dc_izq = max(min(max_dc + correccion, 100.0), -100.0)
        dc_der = max(min(max_dc - correccion, 100.0), -100.0)

        left_motor.dc(dc_izq)
        right_motor.dc(dc_der)

        error_previo = error_ang
        await wait(10)

    left_motor.hold()
    right_motor.hold()
    await wait(60)


async def retrocede_hasta_linea(target_heading, distancia_ignorar=0, umbral_negro=10, umbral_blanco=70, montarse=False, distancia_montaje=50, min_sensores_linea=2, max_dc=80.0, min_dc=20.0, kp=3.5, kd=0.18):
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)

    DIAMETRO_RUEDA = 63.0
    FACTOR_CONVERSION = (DIAMETRO_RUEDA * 3.141) / 360.0

    error_previo = 0.0
    derivada_filtrada = 0.0
    ALFA_FILTRO = 0.35
    cronometro_lazo = StopWatch()
    cronometro_lazo.reset()

    while distancia_ignorar > 0:
        ang_izq = left_motor.angle()
        ang_der = right_motor.angle()
        recorrido = ((abs(ang_izq) + abs(ang_der)) / 2.0) * FACTOR_CONVERSION
        if recorrido >= distancia_ignorar:
            break

        dt = cronometro_lazo.time() / 1000.0
        cronometro_lazo.reset()
        if dt <= 0: dt = 0.001

        heading_actual = hub.imu.heading()
        error_ang = target_heading - heading_actual
        error_ang = (error_ang + 180) % 360 - 180

        termino_p = kp * error_ang
        correccion = max(min(termino_p, 15.0), -15.0)

        left_motor.dc(-max_dc - correccion)
        right_motor.dc(-max_dc + correccion)
        await wait(10)

    estado_busqueda = 0  
    lecturas_consecutivas = 0

    while True:
        dt = cronometro_lazo.time() / 1000.0
        cronometro_lazo.reset()
        if dt <= 0: dt = 0.001

        await sensor_mbc.actualizar_lectura()
        valores = sensor_mbc.valores_individuales

        if estado_busqueda == 0:
            sensores_blancos = sum(1 for v in valores if v >= umbral_blanco)
            if sensores_blancos >= min_sensores_linea:
                lecturas_consecutivas += 1
                if lecturas_consecutivas >= 2:
                    estado_busqueda = 1  
                    lecturas_consecutivas = 0
            else:
                lecturas_consecutivas = 0
                
        elif estado_busqueda == 1:
            sensores_negros = sum(1 for v in valores if v <= umbral_negro)
            if sensores_negros >= min_sensores_linea:
                lecturas_consecutivas += 1
                if lecturas_consecutivas >= 2:
                    if not montarse:
                        break
                    else:
                        left_motor.reset_angle(0)
                        while True:
                            ang_actual = (abs(left_motor.angle()) + abs(right_motor.angle())) / 2.0 * FACTOR_CONVERSION
                            if ang_actual >= distancia_montaje:
                                break
                            await wait(5)
                        break
            else:
                lecturas_consecutivas = 0

        heading_actual = hub.imu.heading()
        error_ang = target_heading - heading_actual
        error_ang = (error_ang + 180) % 360 - 180

        derivada_cruda = (error_ang - error_previo) / dt
        derivada_filtrada = (derivada_filtrada * (1.0 - ALFA_FILTRO)) + (derivada_cruda * ALFA_FILTRO)
        
        correccion = (kp * error_ang) + (kd * derivada_filtrada)
        
        dc_base = -max_dc
        dc_izq = max(min(dc_base - correccion, 100.0), -100.0)
        dc_der = max(min(dc_base + correccion, 100.0), -100.0)

        left_motor.dc(dc_izq)
        right_motor.dc(dc_der)

        error_previo = error_ang
        await wait(10)

    left_motor.hold()
    right_motor.hold()
    await wait(60)