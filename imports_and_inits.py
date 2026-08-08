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
        self.valores_individuales = [0] * 8  

    async def actualizar_lectura(self):
        try:
            data = await self.dispositivo.read(9)
            
            if len(data) >= 5:
                luces = []
                for i in range(4):
                    luces.append(data[i] & 0xFF)         
                    luces.append((data[i] >> 8) & 0xFF)  
                
                self.valores_individuales = luces
                
                self.error_espacial = float((data[4] & 0xFF) - 8)
                self.ancho_linea = float((data[4] >> 8) & 0xFF) 
                
        except Exception:
            pass 

    def es_negro_puro(self, umbral_max=25, sensores_centro=[3, 4]):
        for i in sensores_centro:
            if self.valores_individuales[i] > umbral_max:
                return False
        return True


sensor_mbc = SensorLineaMBC_Diagnostico(Port.D)
robot = get_robot()
hub = robot.hub
left_motor = robot.left_motor
right_motor = robot.right_motor


async def diagnostico_espectral_mbc():
    """
    Función de muestreo continuo para calibrar umbrales de reflectancia.
    Imprime el arreglo de los 8 fotodetectores y aísla los valores centrales.
    """
    print("==================================================")
    print(" INICIO DE MUESTREO: DIAGNÓSTICO ESPECTRAL MBC ")
    print("==================================================")
    print("Instrucciones: Desplaza el robot manualmente sobre la pista.")
    print("Observa los valores en verde, azul y negro puro.")
    
    while True:
        # 1. Actualizar las lecturas desde el microcontrolador del sensor
        await sensor_mbc.actualizar_lectura()
        
        # 2. Extraer la matriz de reflectancia (valores de 0 a 100)
        arreglo_luces = sensor_mbc.valores_individuales
        
        # 3. Formatear y aislar los fotodetectores centrales (índices 3 y 4)
        if len(arreglo_luces) == 8:
            centro_izq = arreglo_luces[3]
            centro_der = arreglo_luces[4]
            
            # Se imprime la matriz completa y se destacan los canales de interés
            print(f"RAW [0-7]: {arreglo_luces} | Centro (3,4): [{centro_izq}, {centro_der}]")
        else:
            print("Esperando trama de datos válida...")
            
        # 4. Retardo para estabilizar la lectura en consola
        await wait(250)

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


async def avance_gyro(target_heading, final_angle, max_speed=85, stall_stop=False, freno=False, aceleracion=1100):
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
    
    # Se inyecta la variable de aceleración junto con la velocidad lineal
    robot.drive_base.settings(straight_speed=vel_mm_s, straight_acceleration=aceleracion)
            
    distancia_mm = (final_angle / 360.0) * (math.pi * 62.5)
    
    # Asignación de comportamiento al finalizar la distancia (frenar o mantener inercia)
    accion_final = Stop.HOLD if freno else Stop.NONE
    robot.drive_base.straight(distancia_mm, then=accion_final, wait=False)
    
    # --- ALGORITMOS DE DETECCIÓN DE ATASCO ---
    cronometro_stall = StopWatch()
    cronometro_total = StopWatch()
    angulo_anterior = left_motor.angle()
    
    while not robot.drive_base.done():
        
        # Aplicamos el período de gracia de 400ms a AMBAS validaciones para ignorar el "jerk" de arranque
        if stall_stop and cronometro_total.time() > 400:

            # 2. DETECCIÓN POR ENCODERS (Bloqueo mecánico de motores)
            if cronometro_stall.time() > 150:
                angulo_actual = left_motor.angle()
                
                # Si en 150ms avanzó menos de 3 grados, está atascado físicamente
                if abs(angulo_actual - angulo_anterior) < 3:
                    robot.drive_base.stop()
                    return False 
                    
                # Si se mueve bien, actualizamos la referencia y reiniciamos el reloj de atasco
                angulo_anterior = angulo_actual
                cronometro_stall.reset()
                
        # Si aún estamos en el período de gracia (arrancando), mantenemos el reloj de stall reseteado
        elif stall_stop:
            angulo_anterior = left_motor.angle()
            cronometro_stall.reset()
                
        await wait(10)
        
    return True

async def retrocede_gyro(target_heading, final_angle, max_speed=80, stall_stop=False, freno=False):
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
    
    # Asignación de comportamiento al finalizar la distancia (frenar o mantener inercia)
    accion_final = Stop.HOLD if freno else Stop.NONE
    robot.drive_base.straight(-distancia_mm, then=accion_final, wait=False)
    
    # --- ALGORITMOS DE DETECCIÓN DE ATASCO ---
    cronometro_stall = StopWatch()
    cronometro_total = StopWatch() 
    angulo_anterior = left_motor.angle()
    
    while not robot.drive_base.done():
        
        # Aplicamos el período de gracia de 400ms a AMBAS validaciones
        if stall_stop and cronometro_total.time() > 400:

            # 2. DETECCIÓN POR ENCODERS (Bloqueo mecánico de motores)
            if cronometro_stall.time() > 150:
                angulo_actual = left_motor.angle()
                
                # Si en 150ms retrocedió menos de 3 grados, chocó con pared
                if abs(angulo_actual - angulo_anterior) < 3:
                    robot.drive_base.stop()
                    return False
                    
                angulo_anterior = angulo_actual
                cronometro_stall.reset()
                
        # Reset constante durante el período de gracia
        elif stall_stop:
            angulo_anterior = left_motor.angle()
            cronometro_stall.reset()
                
        await wait(10)
        
    return True

async def giro(target_heading, tipo_carga="vacio"):
    if tipo_carga == "pesado":
        robot.drive_base.settings(turn_rate=150, turn_acceleration=200)
    elif tipo_carga == "medio":
        robot.drive_base.settings(turn_rate=220, turn_acceleration=700)
    else:
        robot.drive_base.settings(turn_rate=380, turn_acceleration=1200)

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

async def giro_pivot(target_heading, pivote_izquierdo=True):

    max_speed, min_speed, decel_angle = 100, 45, 20.0 
    # Aumenta drásticamente Kp. Aumenta Kd para evitar overshooting. Reduce Ki para control fino final.
    kp, ki, kd, zona_integral, limite_integral = 8.5, 0.4, 0.12, 5.0, 25.0
    # if tipo_carga == "pesado":
    #     max_speed, min_speed, decel_angle = 100, 45, 50.0
    #     kp, ki, kd, zona_integral, limite_integral = 3.0, 1.0, 0.06, 12.0, 15.0

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

async def giro_arco(target_heading, radio_mm, velocidad_crucero=300):
    # 1. Calculamos la diferencia angular usando la IMU
    heading_actual = hub.imu.heading()
    diferencia = (target_heading - heading_actual) % 360
    if diferencia > 180:
        diferencia -= 360
        
    # Si ya está en el ángulo deseado (con un margen de 0.5°), no hace nada
    if abs(diferencia) < 0.5:
        return
        
    # 2. Configuramos la velocidad del chasis
    robot.drive_base.settings(straight_speed=velocidad_crucero, turn_rate=300)
    
    # 3. Ejecutamos la curva
    # El radio dicta el tamaño del arco y 'diferencia' es exactamente 
    # la cantidad de grados que necesita sumar/restar para llegar al target.
    robot.drive_base.curve(radio_mm, diferencia, wait=False)
    
    # 4. Espera asíncrona
    while not robot.drive_base.done():
        await wait(10)
        
    # 5. Frenado activo
    left_motor.hold()
    right_motor.hold()

async def avance_hasta_linea(target_heading, stall_stop=False, umbral_reflectancia=22,freno=True,giro=True):
    heading_actual = hub.imu.heading()
    diferencia = (target_heading - heading_actual) % 360
    if diferencia > 180:
        diferencia -= 360
        
    if giro and abs(diferencia) >= 0.5:
        robot.drive_base.turn(diferencia, wait=False)
        while not robot.drive_base.done():
            await wait(10)
            
    robot.drive_base.straight(2000, wait=False)
    
    # Filtro anti-ruido (requiere 2 lecturas consecutivas para frenar)
    lecturas_negras_consecutivas = 0
    
    while not robot.drive_base.done():
        # Actualizamos la geometría y la reflectancia al mismo tiempo
        await sensor_mbc.actualizar_lectura()
        
        error_mbc = sensor_mbc.error_espacial
        
        # Filtro Riguroso: Está centrado Y el color es definitivamente negro
        if abs(error_mbc) < 6 and sensor_mbc.es_negro_puro(umbral_max=umbral_reflectancia):
            lecturas_negras_consecutivas += 1
            if lecturas_negras_consecutivas >= 2:
                robot.drive_base.stop()
                if freno:
                    break
                else:
                    await avance_gyro(target_heading, 50, max_speed=30)
        else:
            lecturas_negras_consecutivas = 0
            
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