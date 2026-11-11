from pybricks.parameters import Port
from pybricks.tools import wait, StopWatch
from pybricks.iodevices import PUPDevice
from GyroMovement import hub, left_motor, right_motor, robot, giro
from Extras import boton

# Umbrales del sensor MBC (0-255) usados por todas las funciones de detección de línea
UMBRAL_NEGRO = 5    # Calibrar: lectura <= este valor se considera negro
UMBRAL_BLANCO = 60  # Calibrar: lectura >= este valor se considera blanco

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

async def getBlackLineWidth():
    await sensor_mbc.actualizar_lectura()
    return sensor_mbc.ancho_linea

# =============================================================================
# FUNCIONES DE SEGUIMIENTO ÓPTICO Y LÍNEAS
# =============================================================================

# Siguiendo un borde ("izquierda"/"derecha"), si el robot se mete sobre la línea el kp se
# multiplica por este factor para que salga rápido y se estabilice en el borde.
# Del lado de afuera usa el kp normal (ahí el error casi no crece y no hace falta más).
FACTOR_ADENTRO = 2.0   # Calibrar: más alto = sale más rápido de la línea; muy alto = zigzag en el borde

def _esta_adentro(lado, error):
    """True si, siguiendo un borde, el robot quedó encima de la línea (lado de adentro).
    error > 0 corrige a la izquierda, error < 0 corrige a la derecha."""
    if lado == "derecha":
        return error < 0   # la línea se movió hacia el centro/derecha del sensor
    if lado == "izquierda":
        return error > 0   # la línea se movió hacia el centro/izquierda del sensor
    return False

async def seguir_linea_distancia(distancia_mm, vel_crucero_mm_s=700, accel_dist_mm=50, decel_dist_mm=100, vel_minima_mm_s=80, kp=3, kd=2.1, ki=0.0, freno=True, lado="centro", vel_referencia=700, factor_adentro=FACTOR_ADENTRO):
    # lado: "centro" sigue el centro de la línea; "izquierda"/"derecha" siguen ese borde
    # (mismo offset que seguir_linea_hasta_interseccion).
    # kp es el valor calibrado a vel_referencia (mm/s). En cada ciclo se escala con la
    # velocidad actual (kp * vel / vel_referencia) para que corrija igual por milímetro a
    # cualquier velocidad. kd NO se escala: su efecto por milímetro ya no depende de la velocidad.
    start_dist = robot.drive_base.distance()
    last_error = 0.0
    derivada_filtrada = 0.0
    alfa_filtro = 0.35
    MAX_TURN_RATE = 60.0
    dt_fijo = 0.010

    if lado == "izquierda":
        setpoint = -7.0
    elif lado == "derecha":
        setpoint = 7.0
    else:
        setpoint = 0.0

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
        error = sensor_mbc.error_espacial - setpoint

        P = error
        derivada_cruda = (error - last_error) / dt_fijo
        derivada_filtrada = (derivada_filtrada * (1.0 - alfa_filtro)) + (derivada_cruda * alfa_filtro)
        D = derivada_filtrada
        
        if abs(error) <= 0.2:
            error = 0.0

        kp_vel = kp * vel_lineal / vel_referencia   # kp proporcional a la velocidad actual
        if _esta_adentro(lado, error):
            kp_vel *= factor_adentro                # sale más rápido si se metió sobre la línea
        turn_rate = -((kp_vel * P) + (kd * D))
        turn_rate = max(min(turn_rate, MAX_TURN_RATE), -MAX_TURN_RATE)

        robot.drive_base.drive(vel_lineal, turn_rate)
        last_error = error
        await wait(10)

    if freno:
        robot.drive_base.stop()
        await wait(50) 


async def seguir_linea_hasta_interseccion(lado="centro", vel_crucero_mm_s=600, kp=6, kd=2.8, ki=0.0, umbral_interseccion=8, centrado_estricto=False, target_heading=None, montarse=False, distancia_montaje=20, debug=False, vel_referencia=600, factor_adentro=FACTOR_ADENTRO):
    # debug=True: imprime ancho, error del sensor y giro cada vez que el giro es brusco (>= 40 °/s)
    # kp es el valor calibrado a vel_referencia (mm/s); se escala con vel_crucero_mm_s
    # (kp * vel / vel_referencia) para que corrija igual por milímetro a cualquier velocidad.
    # kd NO se escala: su efecto por milímetro ya no depende de la velocidad.
    kp_vel = kp * vel_crucero_mm_s / vel_referencia
    last_error = 0.0
    derivada_filtrada = 0.0
    alfa_filtro = 0.35
    MAX_TURN_RATE = 60.0
    dt_fijo = 0.010 
    
    if lado == "izquierda":
        umbral_interseccion = 6
        setpoint = -7.0
    elif lado == "derecha": 
        umbral_interseccion = 6
        setpoint = 7.0 
    else: 
        setpoint = 0.0  
        
    while True:
        await sensor_mbc.actualizar_lectura()
        ancho_linea = int(sensor_mbc.ancho_linea)
        
        if ancho_linea >= umbral_interseccion:
            if not montarse:
                # stop() deja las ruedas sueltas y el robot seguía rodando con la última
                # corrección del seguidor (se torcía). hold() frena las dos ruedas en seco.
                robot.drive_base.stop()
                left_motor.hold()
                right_motor.hold()
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

        # Si el sensor NO ve línea, error_espacial vale -8 (lectura vacía) y el PD haría girar
        # al robot a toda velocidad. En ese caso avanza recto con el giroscopio hasta encontrarla.
        if ancho_linea == 0:
            if target_heading is not None:
                error_yaw = ((target_heading - hub.imu.heading() + 180) % 360) - 180
                robot.drive_base.drive(vel_crucero_mm_s, error_yaw * 2.5)
            else:
                robot.drive_base.drive(vel_crucero_mm_s, 0)
            last_error = 0.0
            derivada_filtrada = 0.0
            await wait(10)
            continue

        error = sensor_mbc.error_espacial - setpoint
        P = error
        derivada_cruda = (error - last_error) / dt_fijo
        derivada_filtrada = (derivada_filtrada * (1.0 - alfa_filtro)) + (derivada_cruda * alfa_filtro)
        D = derivada_filtrada
        
        kp_uso = kp_vel * factor_adentro if _esta_adentro(lado, error) else kp_vel
        turn_rate = -((kp_uso * P) + (kd * D))
        turn_rate = max(min(turn_rate, MAX_TURN_RATE), -MAX_TURN_RATE)

        if debug and abs(turn_rate) >= 40:
            print("BRUSCO ancho", ancho_linea, "err_sensor", sensor_mbc.error_espacial,
                  "P", kp_uso * P, "D", kd * D, "giro", turn_rate)

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


async def _montaje_pid(target_heading, distancia_mm, dc, kp, kd, sentido=1):
    """Avanza (sentido=1) o retrocede (sentido=-1) distancia_mm corrigiendo el rumbo
    con el giroscopio (PD), igual que el resto del movimiento. Se usa para montarse
    sobre la línea después de detectarla."""
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    FACTOR_CONVERSION = (63.0 * 3.141) / 360.0
    ALFA_FILTRO = 0.35

    error_previo = ((target_heading - hub.imu.heading() + 180) % 360) - 180
    derivada_filtrada = 0.0
    cronometro_lazo = StopWatch()
    cronometro_lazo.reset()
    dc_base = abs(dc) * sentido

    while True:
        recorrido = ((abs(left_motor.angle()) + abs(right_motor.angle())) / 2.0) * FACTOR_CONVERSION
        if recorrido >= distancia_mm:
            break

        dt = cronometro_lazo.time() / 1000.0
        cronometro_lazo.reset()
        if dt <= 0.005: dt = 0.01

        error_ang = ((target_heading - hub.imu.heading() + 180) % 360) - 180
        derivada_cruda = (error_ang - error_previo) / dt
        derivada_filtrada = (derivada_filtrada * (1.0 - ALFA_FILTRO)) + (derivada_cruda * ALFA_FILTRO)
        correccion = (kp * error_ang) + (kd * derivada_filtrada)

        # Mismo signo de corrección al avanzar y al retroceder
        left_motor.dc(max(min(dc_base + correccion, 100.0), -100.0))
        right_motor.dc(max(min(dc_base - correccion, 100.0), -100.0))

        error_previo = error_ang
        await wait(10)


async def avance_hasta_linea(target_heading, distancia_ignorar=0, umbral_negro=UMBRAL_NEGRO, umbral_blanco=UMBRAL_BLANCO, montarse=False, distancia_montaje=50, min_sensores_linea=2, max_dc=80.0, min_dc=20.0, kp=3.5, kd=0.18, esperar_blanco=True):
    # esperar_blanco=False: busca el negro desde el inicio (útil si el robot ya está sobre blanco)
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

    estado_busqueda = 0 if esperar_blanco else 1
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
                        # Montaje con PID de giroscopio (antes solo seguía con la última potencia)
                        await _montaje_pid(target_heading, distancia_montaje, max_dc, kp, kd, sentido=1)
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


async def avance_contando_lineas(target_heading, num_lineas, distancia_ignorar=10, umbral_negro=UMBRAL_NEGRO, umbral_blanco=UMBRAL_BLANCO, min_sensores_linea=1, montarse=False, distancia_montaje=10, max_dc=80.0, min_dc=25.0, dc_aproximacion=40.0, accel_mm=50.0, kp=3.5, kd=0.18):
    """Avanza de forma continua (sin frenar) cruzando líneas negras y se
    detiene al llegar a la línea número `num_lineas`.

    - Cada línea se cuenta al pasar de blanco a negro; para contar la siguiente
      el sensor tiene que volver a ver blanco, así no cuenta la misma dos veces.
    - Acelera al principio y, cuando solo falta la última línea, baja a
      `dc_aproximacion` para frenar con precisión.
    - Si `montarse=True`, al ver la última línea avanza `distancia_montaje` mm más
      (sin dejar de corregir el rumbo) antes de frenar.
    """
    if num_lineas <= 0:
        return

    left_motor.reset_angle(0)
    right_motor.reset_angle(0)

    DIAMETRO_RUEDA = 63.0
    FACTOR_CONVERSION = (DIAMETRO_RUEDA * 3.141) / 360.0

    error_previo = 0.0
    derivada_filtrada = 0.0
    ALFA_FILTRO = 0.35
    cronometro_lazo = StopWatch()
    cronometro_lazo.reset()

    lineas_contadas = 0
    estado_busqueda = 0          # 0 = esperando blanco, 1 = esperando negro
    lecturas_consecutivas = 0
    montando = False
    distancia_inicio_montaje = 0.0

    while True:
        dt = cronometro_lazo.time() / 1000.0
        cronometro_lazo.reset()
        if dt <= 0: dt = 0.001

        recorrido = ((left_motor.angle() + right_motor.angle()) / 2.0) * FACTOR_CONVERSION

        # --- Detección de líneas (solo después de la distancia a ignorar) ---
        if montando:
            if recorrido - distancia_inicio_montaje >= distancia_montaje:
                break
        elif recorrido >= distancia_ignorar:
            await sensor_mbc.actualizar_lectura()
            valores = sensor_mbc.valores_individuales

            if estado_busqueda == 0:
                if sum(1 for v in valores if v >= umbral_blanco) >= min_sensores_linea:
                    lecturas_consecutivas += 1
                    if lecturas_consecutivas >= 2:
                        estado_busqueda = 1
                        lecturas_consecutivas = 0
                else:
                    lecturas_consecutivas = 0
            else:
                if sum(1 for v in valores if v <= umbral_negro) >= min_sensores_linea:
                    lecturas_consecutivas += 1
                    if lecturas_consecutivas >= 2:
                        lineas_contadas += 1
                        estado_busqueda = 0      # esperar blanco antes de la siguiente
                        lecturas_consecutivas = 0
                        if lineas_contadas >= num_lineas:
                            if not montarse:
                                break
                            montando = True
                            distancia_inicio_montaje = recorrido
                else:
                    lecturas_consecutivas = 0

        # --- Velocidad: acelera al inicio, baja solo antes de la última línea ---
        if lineas_contadas >= num_lineas - 1:
            dc_objetivo = dc_aproximacion
        else:
            dc_objetivo = max_dc

        if recorrido < accel_mm:
            dc_base = min_dc + (dc_objetivo - min_dc) * max(recorrido, 0.0) / accel_mm
        else:
            dc_base = dc_objetivo

        # --- Corrección de rumbo con el giroscopio ---
        error_ang = target_heading - hub.imu.heading()
        error_ang = (error_ang + 180) % 360 - 180

        derivada_cruda = (error_ang - error_previo) / dt
        derivada_filtrada = (derivada_filtrada * (1.0 - ALFA_FILTRO)) + (derivada_cruda * ALFA_FILTRO)
        correccion = (kp * error_ang) + (kd * derivada_filtrada)

        left_motor.dc(max(min(dc_base + correccion, 100.0), -100.0))
        right_motor.dc(max(min(dc_base - correccion, 100.0), -100.0))

        error_previo = error_ang
        await wait(10)

    left_motor.hold()
    right_motor.hold()
    await wait(60)


async def retrocede_hasta_linea(target_heading, distancia_ignorar=0, umbral_negro=UMBRAL_NEGRO, umbral_blanco=UMBRAL_BLANCO, montarse=False, distancia_montaje=50, min_sensores_linea=2, max_dc=80.0, min_dc=20.0, kp=3.5, kd=0.18):
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

        left_motor.dc(-max_dc + correccion)   # Mismo signo de corrección que al avanzar
        right_motor.dc(-max_dc - correccion)
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
                        # Montaje con PID de giroscopio (antes solo seguía con la última potencia)
                        await _montaje_pid(target_heading, distancia_montaje, max_dc, kp, kd, sentido=-1)
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
        dc_izq = max(min(dc_base + correccion, 100.0), -100.0)   # Mismo signo de corrección que al avanzar
        dc_der = max(min(dc_base - correccion, 100.0), -100.0)

        left_motor.dc(dc_izq)
        right_motor.dc(dc_der)

        error_previo = error_ang
        await wait(10)

    left_motor.hold()
    right_motor.hold()
    await wait(60)