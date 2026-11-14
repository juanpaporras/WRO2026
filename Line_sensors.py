from pybricks.parameters import Port
from pybricks.tools import wait, StopWatch
from pybricks.iodevices import PUPDevice
from GyroMovement import hub, left_motor, right_motor, robot, giro

# Umbrales del sensor MBC (0-255) usados por todas las funciones de detección de línea
UMBRAL_NEGRO = 5    # Calibrar: lectura <= este valor se considera negro
UMBRAL_BLANCO = 60  # Calibrar: lectura >= este valor se considera blanco

# Desaceleración en los montajes (avanzar después de detectar la línea)
MONTAJE_DC_FINAL = 25     # Calibrar: potencia con la que TERMINA el montaje de avance_hasta_linea (más baja = frena más suave; muy baja = no llega)
MONTAJE_VEL_FINAL = 80    # Calibrar: velocidad (mm/s) con la que TERMINA el montaje de seguir_linea_hasta_interseccion

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


async def seguir_linea_hasta_interseccion(lado="centro", vel_crucero_mm_s=600, kp=6, kd=2.8, ki=0.0, umbral_interseccion=8, target_heading=None, montarse=False, distancia_montaje=20, debug=False, vel_referencia=600, factor_adentro=FACTOR_ADENTRO):
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
                # Montaje con desaceleración: baja de vel_crucero a MONTAJE_VEL_FINAL a lo largo del montaje
                distancia_referencia = robot.drive_base.distance()
                vel_final = min(MONTAJE_VEL_FINAL, vel_crucero_mm_s)

                while True:
                    recorrido_montaje = abs(robot.drive_base.distance() - distancia_referencia)
                    if recorrido_montaje >= distancia_montaje:
                        break
                    proporcion = recorrido_montaje / distancia_montaje if distancia_montaje > 0 else 1.0
                    vel = vel_crucero_mm_s + (vel_final - vel_crucero_mm_s) * proporcion
                    robot.drive_base.drive(vel, 0)
                    await wait(5)

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


async def _montaje_pid(target_heading, distancia_mm, dc, kp, kd, sentido=1, dc_final=None):
    """Avanza (sentido=1) o retrocede (sentido=-1) distancia_mm corrigiendo el rumbo
    con el giroscopio (PD), igual que el resto del movimiento. Se usa para montarse
    sobre la línea después de detectarla.
    Desacelera de forma lineal: empieza en 'dc' y termina en 'dc_final' (MONTAJE_DC_FINAL)."""
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    FACTOR_CONVERSION = (63.0 * 3.141) / 360.0
    ALFA_FILTRO = 0.35

    error_previo = ((target_heading - hub.imu.heading() + 180) % 360) - 180
    derivada_filtrada = 0.0
    cronometro_lazo = StopWatch()
    cronometro_lazo.reset()
    dc_inicial = abs(dc)
    if dc_final is None:
        dc_final = MONTAJE_DC_FINAL
    dc_final = min(abs(dc_final), dc_inicial)   # nunca acelera durante el montaje

    while True:
        recorrido = ((abs(left_motor.angle()) + abs(right_motor.angle())) / 2.0) * FACTOR_CONVERSION
        if recorrido >= distancia_mm:
            break

        # Desaceleración: baja la potencia de dc_inicial a dc_final a lo largo del montaje
        proporcion = recorrido / distancia_mm if distancia_mm > 0 else 1.0
        dc_base = (dc_inicial + (dc_final - dc_inicial) * proporcion) * sentido

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


# =============================================================================
# REGISTRO DE DATOS DEL SEGUIDOR (para analizar y calibrar)
# Guarda las muestras en memoria durante el recorrido (imprimir en cada ciclo lo haría más lento
# y cambiaría cómo sigue la línea) y al final imprime un resumen corto con resumen().
# =============================================================================
class Registro:
    """Uso:
        reg = Registro()
        await seguir_linea_contando_lineas(..., registro=reg)
        await reg.resumen()
    Datos por muestra: t (ms), tramo, mm, ancho, err_sensor (error_espacial crudo), setpoint,
    giro (°/s), heading (x10), vel (mm/s), modo, lineas.
    modo: 0 = siguiendo con PD, 1 = cruzando línea (gyro), 2 = sin línea o lectura descartada (gyro)."""
    def __init__(self, max_muestras=700):
        self.max = max_muestras
        self.n = 0
        self.tramo = 0
        self.targets = {}          # rumbo objetivo de cada tramo
        self.reloj = StopWatch()
        # Listas preasignadas de enteros (poca memoria y sin crecer durante el recorrido)
        self.t = [0] * max_muestras
        self.tr = [0] * max_muestras
        self.mm = [0] * max_muestras
        self.ancho = [0] * max_muestras
        self.err = [0] * max_muestras
        self.sp = [0] * max_muestras
        self.giro = [0] * max_muestras
        self.head = [0] * max_muestras
        self.vel = [0] * max_muestras
        self.modo = [0] * max_muestras
        self.lineas = [0] * max_muestras

    def nuevo_tramo(self, target_heading=0):
        self.tramo += 1
        self.targets[self.tramo] = target_heading

    async def resumen(self):
        """Imprime un RESUMEN corto por tramo (cabe en la consola de VS Code, ~8 líneas por tramo).
        err = error_sensor - setpoint (0 = justo en el borde deseado)."""
        print("=== RESUMEN SEGUIDOR:", self.n, "muestras ===")
        for tr in range(1, self.tramo + 1):
            idx = [k for k in range(self.n) if self.tr[k] == tr]
            if not idx:
                continue
            target = self.targets.get(tr, 0)
            k0, k1 = idx[0], idx[-1]
            dur = self.t[k1] - self.t[k0]
            periodo = dur / (len(idx) - 1) if len(idx) > 1 else 0
            # --- Error y giro mientras sigue con PD (modo 0) ---
            pd = [k for k in idx if self.modo[k] == 0]
            hist = {}
            suma = 0
            suma_abs = 0
            cambios = 0
            prev = None
            for k in pd:
                e = self.err[k] - self.sp[k]
                hist[e] = hist.get(e, 0) + 1
                suma += e
                suma_abs += abs(e)
                if prev is not None and e != prev:
                    cambios += 1
                prev = e
            n_pd = len(pd) if pd else 1
            sat = sum(1 for k in idx if abs(self.giro[k]) >= 59)
            # --- Saturación justo después de cada cruce (modo 1 -> 0) ---
            sat_post = []
            for j in range(1, len(idx)):
                if self.modo[idx[j - 1]] == 1 and self.modo[idx[j]] != 1:
                    c = 0
                    for m in range(j, min(j + 10, len(idx))):
                        if abs(self.giro[idx[m]]) >= 59:
                            c += 1
                    sat_post.append(c)
            # --- Ancho ---
            anchos_pd = [self.ancho[k] for k in pd]
            anchos_cruce = [self.ancho[k] for k in idx if self.modo[k] == 1]
            # --- Líneas contadas (mm donde sube el contador) ---
            lineas_mm = []
            for j in range(1, len(idx)):
                if self.lineas[idx[j]] > self.lineas[idx[j - 1]]:
                    lineas_mm.append(self.mm[idx[j]])
            # --- Heading respecto al rumbo objetivo ---
            desv = [((self.head[k] / 10 - target + 180) % 360) - 180 for k in idx]
            sin_linea = sum(1 for k in idx if self.modo[k] == 2)

            print("--- TRAMO {} (rumbo {}) ---".format(tr, target))
            print("muestras {} | {} ms | ciclo {:.1f} ms | {} mm".format(len(idx), dur, periodo, self.mm[k1]))
            print("err prom {:.2f} | err abs prom {:.2f} | cambios de err {} | hist {}".format(
                suma / n_pd, suma_abs / n_pd, cambios, sorted(hist.items())))
            print("giro saturado {} ciclos | tras cada cruce {}".format(sat, sat_post))
            print("ancho siguiendo min {} max {} | ancho cruzando min {} max {}".format(
                min(anchos_pd) if anchos_pd else "-", max(anchos_pd) if anchos_pd else "-",
                min(anchos_cruce) if anchos_cruce else "-", max(anchos_cruce) if anchos_cruce else "-"))
            print("lineas en mm {} | sin linea o lectura descartada {} ciclos".format(lineas_mm, sin_linea))
            print("heading desv min {:.1f} max {:.1f} inicio {:.1f} fin {:.1f}".format(
                min(desv), max(desv), desv[0], desv[-1]))
            await wait(50)
        print("=== FIN RESUMEN ===")

    def agregar(self, mm, ancho, err_sensor, setpoint, giro, heading, vel, modo, lineas):
        if self.n >= self.max:
            return
        k = self.n
        self.t[k] = self.reloj.time()
        self.tr[k] = self.tramo
        self.mm[k] = int(mm)
        self.ancho[k] = int(ancho)
        self.err[k] = int(err_sensor)
        self.sp[k] = int(setpoint)
        self.giro[k] = int(giro)
        self.head[k] = int(heading * 10)
        self.vel[k] = int(vel)
        self.modo[k] = modo
        self.lineas[k] = lineas
        self.n = k + 1


UMBRAL_INTERSECCION_BORDE = 7   # Calibrar (prueba: siguiendo llega a 5, cruzando a 8): ancho de línea (sensores) a partir del cual, siguiendo un borde, se considera que cruza una línea

async def retroceder_hasta_linea(target_heading, vel_mm_s=100, umbral=None, distancia_max=150):
    """Retrocede recto (gyro) a velocidad baja y CONSTANTE hasta que el sensor detecta una línea
    de color (ancho >= umbral en 2 lecturas seguidas) y frena. Sirve como referencia física:
    siempre se detiene en el mismo punto respecto a la línea, venga de donde venga.
    Devuelve True si encontró la línea, False si recorrió distancia_max sin verla."""
    if umbral is None:
        umbral = UMBRAL_INTERSECCION_BORDE
    inicio = robot.drive_base.distance()
    lecturas = 0
    encontrada = False
    while abs(robot.drive_base.distance() - inicio) < distancia_max:
        await sensor_mbc.actualizar_lectura()
        if int(sensor_mbc.ancho_linea) >= umbral:
            lecturas += 1
            if lecturas >= 2:
                encontrada = True
                break
        else:
            lecturas = 0
        error_yaw = ((target_heading - hub.imu.heading() + 180) % 360) - 180
        robot.drive_base.drive(-vel_mm_s, error_yaw * 2.5)
        await wait(10)
    robot.drive_base.stop()
    left_motor.hold()
    right_motor.hold()
    await wait(50)
    if not encontrada:
        print("retroceder_hasta_linea: no encontro la linea en", distancia_max, "mm")
    return encontrada


DISTANCIA_ENTRE_LINEAS = 155  # Calibrar: mm entre líneas de color (prueba: ~158-161); usar un poco menos para frenar a tiempo
FRENAR_ANTES_MM = 60          # Calibrar: mm antes de la línea objetivo en que baja a vel_aproximacion (más = más seguro, más lento)

ERROR_MAXIMO_VALIDO = 6  # Calibrar: |error - setpoint| mayor a esto es una lectura imposible (borde del lado opuesto): ese ciclo va con gyro

async def seguir_linea_contando_lineas(lado, num_lineas, target_heading, offset=5.0, vel_crucero_mm_s=450, vel_aproximacion_mm_s=250, vel_minima_mm_s=80, accel_mm=50, distancia_ignorar=50, montarse=False, distancia_montaje=20, umbral_interseccion=UMBRAL_INTERSECCION_BORDE, kp=6, kd=2.8, vel_referencia=600, factor_adentro=FACTOR_ADENTRO, registro=None):
    """Sigue la línea vertical por un BORDE contando las líneas que la cruzan, SIN frenar en las
    intermedias. En la línea número num_lineas frena (o se monta distancia_montaje con desaceleración).
    - lado: "izquierda" (setpoint -offset) o "derecha" (setpoint +offset).
    - target_heading: rumbo del giroscopio; se usa al cruzar cada línea (el sensor ve una línea ancha
      y el error no sirve), si pierde la línea y durante el montaje.
    - distancia_ignorar: mm iniciales en los que no cuenta líneas (por ejemplo la línea de donde sale).
    - vel_aproximacion_mm_s: velocidad desde que cruzó la penúltima línea (llega más suave a la última).
    - registro: objeto Registro para guardar los datos del recorrido (None = no guarda nada)."""
    setpoint = -abs(offset) if lado == "izquierda" else abs(offset)
    inicio = robot.drive_base.distance()
    if registro is not None:
        registro.nuevo_tramo(target_heading)
    last_error = 0.0
    derivada_filtrada = 0.0
    alfa_filtro = 0.35
    MAX_TURN_RATE = 60.0
    dt_fijo = 0.010

    lineas_contadas = 0
    mm_ultima_linea = 0.0      # mm donde contó la última línea (para saber cuánto falta a la siguiente)
    sobre_linea = False        # True mientras está cruzando una línea (para no contarla 2 veces)
    lecturas_fuera = 0
    primer_ciclo_pd = True     # al arrancar / tras cruzar o perder la línea: sin derivada (evita el tirón)

    while True:
        recorrido = abs(robot.drive_base.distance() - inicio)

        # --- Velocidad: rampa al arrancar y más lento SOLO cerca de la última línea ---
        # Tras la penúltima línea sigue a crucero hasta FRENAR_ANTES_MM antes de donde debería
        # estar la última. Si va a 1 sola línea (no sabe dónde empezó) va a aproximación desde el inicio.
        if lineas_contadas >= num_lineas - 1 and (lineas_contadas == 0 or
                recorrido - mm_ultima_linea >= DISTANCIA_ENTRE_LINEAS - FRENAR_ANTES_MM):
            vel_objetivo = vel_aproximacion_mm_s
        else:
            vel_objetivo = vel_crucero_mm_s
        if recorrido < accel_mm:
            vel = vel_minima_mm_s + (vel_objetivo - vel_minima_mm_s) * recorrido / accel_mm
        else:
            vel = vel_objetivo

        await sensor_mbc.actualizar_lectura()
        ancho = int(sensor_mbc.ancho_linea)

        # --- Conteo de líneas ---
        if ancho >= umbral_interseccion:
            if not sobre_linea and recorrido >= distancia_ignorar:
                sobre_linea = True
                lineas_contadas += 1
                mm_ultima_linea = recorrido
                if lineas_contadas >= num_lineas:
                    break
            lecturas_fuera = 0
        elif sobre_linea:
            lecturas_fuera += 1
            if lecturas_fuera >= 3:   # ya salió de la línea: puede contar la siguiente
                sobre_linea = False

        error_yaw = ((target_heading - hub.imu.heading() + 180) % 360) - 180
        error = sensor_mbc.error_espacial - setpoint
        lectura_rara = abs(error) > ERROR_MAXIMO_VALIDO
        if ancho >= umbral_interseccion or ancho == 0 or lectura_rara:
            # Cruzando una línea, sin ver la línea o lectura imposible (borde del lado opuesto):
            # recto con el giroscopio ese ciclo
            turn_rate = error_yaw * 2.5
            modo = 1 if ancho >= umbral_interseccion else 2
            robot.drive_base.drive(vel, turn_rate)
            primer_ciclo_pd = True      # al volver al PD no calcula derivada contra un valor viejo
        else:
            if primer_ciclo_pd:
                # Primer ciclo: la derivada contra last_error = 0 daba un valor enorme y saturaba
                # el giro (±60) varios ciclos después de cada línea. Arranca la derivada en 0.
                last_error = error
                derivada_filtrada = 0.0
                primer_ciclo_pd = False
            derivada_cruda = (error - last_error) / dt_fijo
            derivada_filtrada = (derivada_filtrada * (1.0 - alfa_filtro)) + (derivada_cruda * alfa_filtro)
            kp_vel = kp * vel / vel_referencia
            if _esta_adentro(lado, error):
                kp_vel *= factor_adentro
            turn_rate = -((kp_vel * error) + (kd * derivada_filtrada))
            turn_rate = max(min(turn_rate, MAX_TURN_RATE), -MAX_TURN_RATE)
            modo = 0
            robot.drive_base.drive(vel, turn_rate)
            last_error = error

        if registro is not None:
            registro.agregar(recorrido, ancho, sensor_mbc.error_espacial, setpoint, turn_rate,
                             hub.imu.heading(), vel, modo, lineas_contadas)
        await wait(10)

    if montarse and distancia_montaje > 0:
        # Montaje recto con giroscopio, desacelerando hasta MONTAJE_VEL_FINAL
        referencia = robot.drive_base.distance()
        vel_inicial = vel
        vel_final = min(MONTAJE_VEL_FINAL, vel_inicial)
        while True:
            recorrido_montaje = abs(robot.drive_base.distance() - referencia)
            if recorrido_montaje >= distancia_montaje:
                break
            proporcion = recorrido_montaje / distancia_montaje
            vel_m = vel_inicial + (vel_final - vel_inicial) * proporcion
            error_yaw = ((target_heading - hub.imu.heading() + 180) % 360) - 180
            robot.drive_base.drive(vel_m, error_yaw * 2.5)
            await wait(5)

    robot.drive_base.stop()
    left_motor.hold()
    right_motor.hold()
    await wait(60)
