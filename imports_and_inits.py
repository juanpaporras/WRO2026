from subdyk import get_robot
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Button, Port, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
import umath
from pybricks.iodevices import PUPDevice
class SensorLineaMBC_Diagnostico:
    def __init__(self, puerto: Port):
        self.dispositivo = PUPDevice(puerto)
        self.error_espacial = 0.0
        self.ancho_linea = 0.0
        print("INFO DEL SENSOR:", self.dispositivo.info())

    async def actualizar_lectura(self):
        try:
            # Leemos los datos tal como indica el tutorial
            # Asegúrate de recibir los 9 bytes
            data = await self.dispositivo.read(9)
            
            if len(data) >= 7:
                # Extraemos data5 (que es el índice 4 si la lista empieza en 0)
                # Basado en la imagen, el dato clave es data5
                d5 = (data[4] << 8) | data[5] # Combinamos bytes si es necesario, 
                                              # o usa la estructura que tu driver ya tiene.
                
                # SIGUIENDO EXACTAMENTE LA LÓGICA DE LA IMAGEN:
                # La imagen muestra data5 como el contenedor de error y ancho
                self.error_espacial = float((data[4] & 0xFF) - 8)
                self.ancho_linea = float((data[4] >> 8) & 0xFF) # <--- AQUÍ ESTÁ EL ANCHO CORRECTO
                
                # Si esto no te da el valor correcto, prueba usando data[5] en lugar de data[4]
                # dependiendo de cómo Pybricks entrega la lista de bytes.
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
    """
    Normaliza el error al rango [-180, 180]. 
    (Mantener para compatibilidad externa, NO usar dentro de bucles PID).
    """
    return ((error + 180) % 360) - 180

async def getBlackLineError():
    valor_reflexion = await line.reflection()
    return (valor_reflexion / 10) - 8

async def avance_gyro(target_heading, final_angle, decel_start_angle, start_speed=30, max_speed=85, end_speed=35, accel_rate=1, reset_gyro=False, stall_stop=False):

    if reset_gyro:
        hub.imu.reset_heading(0)
        
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    
    kp_base = 1.5   
    kd_base = 0.5   
    
    derivada_filtrada = 0.0
    ALFA_FILTRO = 0.15     
    
    MAX_PWM = 100 
    MAX_CORRECTION = 30   
    
    heading_inicial = hub.imu.heading()
    target_continuo = obtener_target_continuo(target_heading, heading_inicial)
    prev_error = heading_inicial - target_continuo
    
    current_speed = abs(start_speed)
    max_speed = abs(max_speed)
    end_speed = abs(end_speed)
    
    timer_lazo = StopWatch()
    timer_lazo.reset()
    stopwatch_stall = StopWatch() 
    last_movement_time = stopwatch_stall.time()  

    while True:
        grados_recorridos = (abs(left_motor.angle()) + abs(right_motor.angle())) / 2.0
        if grados_recorridos >= final_angle:
            break
            
        dt = timer_lazo.time() / 1000.0
        timer_lazo.reset()
        if dt <= 0.0: dt = 0.01

        remaining_distance = final_angle - grados_recorridos
        if grados_recorridos >= decel_start_angle:
            scale = remaining_distance / max(1, (final_angle - decel_start_angle))
            current_speed = end_speed + (max_speed - end_speed) * scale
        else:
            if current_speed < max_speed:
                current_speed += accel_rate 
                
        error = hub.imu.heading() - target_continuo
        
    
        if abs(error) < 0.5:
            correction = 0.0
        else:
            derivada_cruda = (error - prev_error) / dt
            derivada_filtrada = (derivada_filtrada * (1.0 - ALFA_FILTRO)) + (derivada_cruda * ALFA_FILTRO)
            
            correction = (kp_base * error) + (kd_base * derivada_filtrada)
            correction = max(min(correction, MAX_CORRECTION), -MAX_CORRECTION)

        if (current_speed + abs(correction)) > MAX_PWM:
            vel_base_aplicada = MAX_PWM - abs(correction)
        else:
            vel_base_aplicada = current_speed

        if stall_stop:
            if abs(abs(left_motor.angle()) - abs(right_motor.angle())) > 100: break
            current_time = stopwatch_stall.time()
            left_sp = abs(left_motor.speed())
            if (current_time - last_movement_time) > 200 and left_sp < 10: break
            if left_sp > 10: last_movement_time = current_time
                
        left_motor.dc(vel_base_aplicada - correction)
        right_motor.dc(vel_base_aplicada + correction)
        
        prev_error = error 
        await wait(10)
        
    left_motor.hold()
    right_motor.hold()

async def retrocede_gyro(target_heading, final_angle, decel_start_angle, start_speed=30, max_speed=75, end_speed=35, accel_rate=1, reset_gyro=False, stall_stop=False):

    if reset_gyro:
        hub.imu.reset_heading(0)
        
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    
    kp_base = 1.5   
    ki_base = 0.1  
    kd_base = 0.5    
    
    integral_error = 0.0
    LIMITE_INTEGRAL = 50.0
    derivada_filtrada = 0.0
    ALFA_FILTRO = 0.15      
    
    MAX_PWM = 100 
    MAX_CORRECTION = 25   
    
    heading_inicial = hub.imu.heading()
    target_continuo = obtener_target_continuo(target_heading, heading_inicial)
    prev_error = heading_inicial - target_continuo
    
    current_speed = abs(start_speed)
    max_speed = abs(max_speed) 
    end_speed = abs(end_speed)
    
    timer_lazo = StopWatch()
    timer_lazo.reset()
    stopwatch_stall = StopWatch() 
    last_movement_time = stopwatch_stall.time()  

    while True:
        grados_recorridos = (abs(left_motor.angle()) + abs(right_motor.angle())) / 2.0
        if grados_recorridos >= final_angle:
            break
            
        dt = timer_lazo.time() / 1000.0
        timer_lazo.reset()
        if dt <= 0.0: dt = 0.01

        remaining_distance = final_angle - grados_recorridos
        if grados_recorridos >= decel_start_angle:
            scale = remaining_distance / max(1, (final_angle - decel_start_angle))
            current_speed = end_speed + (max_speed - end_speed) * scale
        else:
            if current_speed < max_speed:
                current_speed += accel_rate 
                
        error = hub.imu.heading() - target_continuo
        
        integral_error += error * dt
        integral_error = max(min(integral_error, LIMITE_INTEGRAL), -LIMITE_INTEGRAL)
        
        derivada_cruda = (error - prev_error) / dt
        derivada_filtrada = (derivada_filtrada * (1.0 - ALFA_FILTRO)) + (derivada_cruda * ALFA_FILTRO)
        
        correction = (kp_base * error) + (ki_base * integral_error) + (kd_base * derivada_filtrada)
        correction = max(min(correction, MAX_CORRECTION), -MAX_CORRECTION)

        if (current_speed + abs(correction)) > MAX_PWM:
            vel_base_aplicada = MAX_PWM - abs(correction)
        else:
            vel_base_aplicada = current_speed

        if stall_stop:
            if abs(abs(left_motor.angle()) - abs(right_motor.angle())) > 100: break
            current_time = stopwatch_stall.time()
            left_sp = abs(left_motor.speed())
            if (current_time - last_movement_time) > 200 and left_sp < 10: break
            if left_sp > 10: last_movement_time = current_time
                
        left_motor.dc(-vel_base_aplicada - correction)
        right_motor.dc(-vel_base_aplicada + correction)
        
        prev_error = error 
        await wait(10)
        
    left_motor.hold()
    right_motor.hold()

async def giro(target_heading, tipo_carga="vacio", reset_gyro=False, max_speed=None, min_speed=None, accel_rate=1.5, decel_angle=None):

    if tipo_carga == "pesado":
        max_speed = 60 if max_speed is None else max_speed
        min_speed = 40 if min_speed is None else min_speed
        decel_angle = 70.0 if decel_angle is None else decel_angle
        kp = 3.0
        ki = 1.0
        kd = 0.06
        zona_integral = 12.0
        limite_integral = 15.0
    elif tipo_carga == "medio":
        max_speed = 70 if max_speed is None else max_speed
        min_speed = 35 if min_speed is None else min_speed
        decel_angle = 60.0 if decel_angle is None else decel_angle
        kp = 3.1
        ki = 0.9
        kd = 0.05
        zona_integral = 11.0
        limite_integral = 15.0
    else:  # "vacio"
        max_speed = 100 if max_speed is None else max_speed
        min_speed = 20 if min_speed is None else min_speed
        decel_angle = 50.0 if decel_angle is None else decel_angle
        kp = 3.2
        ki = 0.7
        kd = 0.05
        zona_integral = 3.5
        limite_integral = 12.0

    if reset_gyro:
        hub.imu.reset_heading(0)
        
    heading_inicial = hub.imu.heading()
    target_continuo = obtener_target_continuo(target_heading, heading_inicial)
    prev_error = target_continuo - heading_inicial
    
    integral_error = 0.0
    derivada_filtrada = 0.0
    ALFA_FILTRO = 0.3  
    
    VOLTAJE_REFERENCIA = 8200.0  
    FACTOR_DERIVA_IZQ = 1.0      
    FACTOR_DERIVA_DER = 1.0      
    
    current_vel_limit = min_speed
    tiempo_acumulado = 0.0  
    
    timer_lazo = StopWatch()
    timer_lazo.reset()
    
    watch_estancamiento = StopWatch()
    watch_estancamiento.reset()
    error_anterior_bloqueo = 999.0
    
    while True:
        dt = timer_lazo.time() / 1000.0
        timer_lazo.reset()
        if dt <= 0.0: dt = 0.01
        tiempo_acumulado += dt
        
        error = target_continuo - hub.imu.heading()
        abs_error = abs(error)
        
        if abs_error <= 1:
            break
            
        if abs(error - error_anterior_bloqueo) < 0.02:
            if watch_estancamiento.time() > 500: 
                break
        else:
            watch_estancamiento.reset()
            error_anterior_bloqueo = error
            
        if abs_error <= decel_angle:
            escala = abs_error / decel_angle
            target_vel_limit = min_speed + (max_speed - min_speed) * escala
            current_vel_limit = target_vel_limit 
        else:
            if current_vel_limit < max_speed:
                current_vel_limit += accel_rate
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
        
        if salida_pid > current_vel_limit:
            salida_pid = current_vel_limit
        elif salida_pid < -current_vel_limit:
            salida_pid = -current_vel_limit
            
        if (salida_pid * error) > 0 and abs_error > 0.8:
            if abs(salida_pid) < min_speed:
                salida_pid = min_speed if salida_pid > 0 else -min_speed
                
        control_speed_base = salida_pid
        
        voltaje_actual = hub.battery.voltage()
        if voltaje_actual < 5000: voltaje_actual = 5000 
        
        factor_voltaje = VOLTAJE_REFERENCIA / voltaje_actual
        control_speed_compensado = control_speed_base * factor_voltaje
        
        velocidad_izq_bruta = control_speed_compensado * FACTOR_DERIVA_IZQ
        velocidad_der_bruta = -control_speed_compensado * FACTOR_DERIVA_DER
        
        velocidad_izq = max(min(velocidad_izq_bruta, 100.0), -100.0)
        velocidad_der = max(min(velocidad_der_bruta, 100.0), -100.0)
        
        left_motor.dc(velocidad_izq)
        right_motor.dc(velocidad_der)
        
        prev_error = error
        await wait(10)
        
    left_motor.hold()
    right_motor.hold()
    await hub.speaker.beep(1000, 50)

async def giro_pivot(target_heading, pivote_izquierdo=True, tipo_carga="vacio", reset_gyro=False, max_speed=None, min_speed=None, accel_rate=1.5, decel_angle=None):
    
    if tipo_carga == "pesado":
        max_speed = 100 if max_speed is None else max_speed
        min_speed = 40 if min_speed is None else min_speed
        decel_angle = 70.0 if decel_angle is None else decel_angle
        kp = 3.0
        ki = 1.0
        kd = 0.06
        zona_integral = 12.0
        limite_integral = 15.0
    elif tipo_carga == "medio":
        max_speed = 100 if max_speed is None else max_speed
        min_speed = 35 if min_speed is None else min_speed
        decel_angle = 60.0 if decel_angle is None else decel_angle
        kp = 3.1
        ki = 0.9
        kd = 0.05
        zona_integral = 11.0
        limite_integral = 15.0
    else:  # "vacio"
        max_speed = 100 if max_speed is None else max_speed
        min_speed = 32 if min_speed is None else min_speed
        decel_angle = 50.0 if decel_angle is None else decel_angle
        kp = 3.2
        ki = 0.8
        kd = 0.05
        zona_integral = 10.0
        limite_integral = 15.0

    if reset_gyro:
        hub.imu.reset_heading(0)
        
    heading_inicial = hub.imu.heading()
    
    diferencia = (target_heading - heading_inicial) % 360
    if diferencia > 180:
        diferencia -= 360
    target_continuo = heading_inicial + diferencia
    
    error = target_continuo - heading_inicial
    prev_error = error 
    
    integral_error = 0.0
    derivada_filtrada = 0.0
    ALFA_FILTRO = 0.3  
    VOLTAJE_REFERENCIA = 8200.0  
    
    current_vel_limit = min_speed
    tiempo_acumulado = 0.0  
    
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
        tiempo_acumulado += dt
        
        error = target_continuo - hub.imu.heading()
        abs_error = abs(error)
        
        if abs_error > 360:
            break
            
        if abs_error <= 0.5:
            break
            
        if abs(error - error_anterior_bloqueo) < 0.02:
            if watch_estancamiento.time() > 800: 
                break
        else:
            watch_estancamiento.reset()
            error_anterior_bloqueo = error
            
        if abs_error <= decel_angle:
            escala = abs_error / decel_angle
            target_vel_limit = min_speed + (max_speed - min_speed) * escala
            current_vel_limit = target_vel_limit 
        else:
            if current_vel_limit < max_speed:
                current_vel_limit += accel_rate
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
        
        if salida_pid > current_vel_limit:
            salida_pid = current_vel_limit
        elif salida_pid < -current_vel_limit:
            salida_pid = -current_vel_limit
            
        if (salida_pid * error) > 0 and abs_error > 0.8:
            if abs(salida_pid) < min_speed:
                salida_pid = min_speed if salida_pid > 0 else -min_speed
                
        control_speed_base = salida_pid
        
        voltaje_actual = hub.battery.voltage()
        if voltaje_actual < 5000: voltaje_actual = 5000 
        
        factor_voltaje = VOLTAJE_REFERENCIA / voltaje_actual
        control_speed_compensado = control_speed_base * factor_voltaje
        
        velocidad_motor = max(min(control_speed_compensado, 100.0), -100.0)
        
        if pivote_izquierdo:
            right_motor.dc(-velocidad_motor)
        else:
            left_motor.dc(velocidad_motor)
            
        prev_error = error
        await wait(10)
        
    left_motor.hold()
    right_motor.hold()
    await hub.speaker.beep(1000, 50)

async def avance_hasta_linea(target_heading, start_speed=40, max_speed=50, accel_rate=1.2, stall_stop=False):
    kp_base = 5
    ki_base = 0.001
    adapt_rate = 0.0001 
    max_integral = 500
    
    stopwatch = StopWatch() 
    last_movement_time = stopwatch.time()  

    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    
    current_speed = abs(start_speed)
    max_speed = abs(max_speed)
    accel_rate = abs(accel_rate)
    
    integral_error = 0
    kp_min, kp_max = 0.5 * kp_base, 2 * kp_base
    ki_min, ki_max = 0.5 * ki_base, 2 * ki_base
    
    heading_inicial = hub.imu.heading()
    target_continuo = obtener_target_continuo(target_heading, heading_inicial)
    
    while abs(await obtener_error_lpf2) >= 6:
        left_angle = abs(left_motor.angle()) 
        
        if current_speed < max_speed:
            current_speed += accel_rate
            current_speed = min(current_speed, max_speed)
            
        error = hub.imu.heading() - target_continuo
        
        integral_error += error  
        integral_error = max(-max_integral, min(integral_error, max_integral))
        
        kp = kp_max - adapt_rate * abs(error)
        ki = ki_base + adapt_rate * abs(integral_error)
        kp = max(kp_min, min(kp, kp_max))
        ki = max(ki_min, min(ki, ki_max))
        
        correction = kp * error + ki * integral_error
        
        if stall_stop:
            if abs(left_angle - abs(right_motor.angle())) > 100:
                break
            current_time = stopwatch.time()
            left_sp = abs(left_motor.speed())
            if (current_time - last_movement_time) > 200 and left_sp < 10:
                break
            if left_sp > 10:
                last_movement_time = current_time
                
        left_motor.dc(current_speed - correction)
        right_motor.dc(current_speed + correction)
        
        await wait(1)
        
    left_motor.hold()
    right_motor.hold()

async def retrocede_hasta_linea_gyro(target_heading, start_speed=40, max_speed=50, accel_rate=1.2, reset_gyro=False, kp_value=7, ki_value=0.001, kd_value=0, stall_stop=True, lecturas_requeridas=3, umbral_linea=1.0):
    kp_base = kp_value  
    ki_base = ki_value  
    kd_base = kd_value  
    adapt_rate = 0.0001 
    max_integral = 500
    
    stopwatch = StopWatch()  
    last_movement_time = stopwatch.time() 
    
    if reset_gyro:
        hub.imu.reset_heading(0)
        
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    
    current_speed = -abs(start_speed)  
    max_speed = -abs(max_speed)
    accel_rate = abs(accel_rate)
    
    # --- PROYECCIÓN CONTINUA ---
    heading_inicial = hub.imu.heading()
    target_continuo = obtener_target_continuo(target_heading, heading_inicial)
    prev_error = heading_inicial - target_continuo
    
    integral_error = 0
    
    kp_min, kp_max = 0.5 * kp_base, 2 * kp_base
    ki_min, ki_max = 0.5 * ki_base, 2 * ki_base
    
    lecturas_linea_consecutivas = 0
    
    while True:
        error_linea = await getBlackLineError()
        
        if abs(error_linea) < umbral_linea:
            lecturas_linea_consecutivas += 1
            if lecturas_linea_consecutivas >= lecturas_requeridas:
                break  
        else:
            lecturas_linea_consecutivas = 0  
            
        left_angle = abs(left_motor.angle())  
        
        if current_speed > max_speed:
            current_speed -= accel_rate
            current_speed = max(current_speed, max_speed)
            
        # Error lineal continuo
        error = hub.imu.heading() - target_continuo
        
        integral_error += error  
        integral_error = max(-max_integral, min(integral_error, max_integral))
        
        kp = kp_base + adapt_rate * abs(error)
        ki = ki_base + adapt_rate * abs(integral_error)
        kp = max(kp_min, min(kp, kp_max))
        ki = max(ki_min, min(ki, ki_max))
        
        correction = (kp * error) + (ki * integral_error) + (kd_base * (error - prev_error))
        prev_error = error 
        
        if stall_stop:
            if abs(left_angle - abs(right_motor.angle())) > 100:
                break
                
            current_time = stopwatch.time()
            left_sp = abs(left_motor.speed())
            
            if (current_time - last_movement_time) > 200 and left_sp < 10:
                break
            if left_sp > 10:
                last_movement_time = current_time
                
        left_motor.dc(current_speed - correction)
        right_motor.dc(current_speed + correction)
        
        await wait(1)  

    left_motor.hold()
    right_motor.hold()