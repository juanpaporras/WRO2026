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

async def avance(target_heading, distancia_mm, max_dc=100.0, min_dc=30.0, accel_mm=50.0, decel_mm=140.0, kp=3.5, kd=0.18, freno=True, rampa_inicio=True):
    # freno=False: no desacelera al final ni frena; los motores siguen girando para
    #   encadenar con el siguiente movimiento (otro avance, seguir línea...) sin detenerse.
    # rampa_inicio=False: arranca directo en max_dc (útil si viene de un movimiento sin freno).
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)

    DIAMETRO_RUEDA = 63.0
    FACTOR_CONVERSION = (DIAMETRO_RUEDA * 3.141) / 360.0

    recorrido = 0.0
    # Igual que en retrocede: evita el golpe de la derivada en el primer ciclo
    error_previo = ((target_heading - hub.imu.heading() + 180) % 360) - 180
    derivada_filtrada = 0.0
    ALFA_FILTRO = 0.35

    cronometro_lazo = StopWatch()
    cronometro_lazo.reset()

    while abs(recorrido) < abs(distancia_mm):
        dt = cronometro_lazo.time() / 1000.0
        cronometro_lazo.reset()
        if dt <= 0.005: dt = 0.02   # primer ciclo / ciclo muy corto: usa el periodo normal

        ang_izq = left_motor.angle()
        ang_der = right_motor.angle()
        recorrido = ((ang_izq + ang_der) / 2.0) * FACTOR_CONVERSION
        
        dist_restante = max(0.0, abs(distancia_mm) - abs(recorrido))

        if rampa_inicio and abs(recorrido) < accel_mm:
            proporcion = abs(recorrido) / accel_mm
            dc_base = min_dc + (max_dc - min_dc) * proporcion
        elif freno and dist_restante < decel_mm:
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

    if freno:
        left_motor.hold()
        right_motor.hold()
        await wait(80)
    # Sin freno: los motores quedan con la última potencia hasta la siguiente orden


async def retrocede(target_heading, distancia_mm, max_dc=100.0, min_dc=30.0, accel_mm=50.0, decel_mm=140.0, kp=3.5, kd=0.18, freno=True, rampa_inicio=True):  # Calibrar min_dc: potencia mínima (arranque y final); muy baja = le cuesta arrancar
    # freno=False: no desacelera al final ni frena; los motores siguen girando para
    #   encadenar con el siguiente movimiento (otro retrocede, retrocede_hasta_linea...) sin detenerse.
    # rampa_inicio=False: arranca directo en max_dc (útil si viene de un movimiento sin freno).
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)

    DIAMETRO_RUEDA = 63.0
    FACTOR_CONVERSION = (DIAMETRO_RUEDA * 3.141) / 360.0

    recorrido = 0.0
    # Arranca la derivada con el error real: si empieza en 0, el primer ciclo
    # calcula (error - 0) / 0.001 y da un golpe enorme a una rueda (arrancón).
    error_previo = ((target_heading - hub.imu.heading() + 180) % 360) - 180
    derivada_filtrada = 0.0
    ALFA_FILTRO = 0.35

    cronometro_lazo = StopWatch()
    cronometro_lazo.reset()

    while abs(recorrido) < abs(distancia_mm):
        dt = cronometro_lazo.time() / 1000.0
        cronometro_lazo.reset()
        if dt <= 0.005: dt = 0.02   # primer ciclo / ciclo muy corto: usa el periodo normal

        ang_izq = left_motor.angle()
        ang_der = right_motor.angle()
        recorrido = ((abs(ang_izq) + abs(ang_der)) / 2.0) * FACTOR_CONVERSION
        
        dist_restante = max(0.0, abs(distancia_mm) - abs(recorrido))

        if rampa_inicio and abs(recorrido) < accel_mm:
            proporcion = abs(recorrido) / accel_mm
            dc_base = min_dc + (max_dc - min_dc) * proporcion
        elif freno and dist_restante < decel_mm:
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

        # Mismo signo que en avance(): para girar a la derecha la rueda izquierda
        # debe ir "más adelante" que la derecha, aunque el robot vaya en reversa.
        dc_izq = max(min(dc_base + correccion, 100.0), -100.0)
        dc_der = max(min(dc_base - correccion, 100.0), -100.0)

        left_motor.dc(dc_izq)
        right_motor.dc(dc_der)

        error_previo = error_ang
        await wait(20)

    if freno:
        left_motor.hold()
        right_motor.hold()
        await wait(80)
    # Sin freno: los motores quedan con la última potencia hasta la siguiente orden


async def retrocede_hasta_pared(target_heading, dc=60.0, distancia_max=1000.0, ignorar_mm=20.0, velocidad_stall=70.0, tiempo_stall=150, kp=2.0, nuevo_heading=None):
    """Retrocede hasta chocar con una pared y frena (stall stop).

    - dc: potencia del retroceso (Calibrar: suficiente para moverse, no tanta que rebote).
    - distancia_max: seguridad; si no choca en esta distancia (mm), frena igual.
    - ignorar_mm: no detecta choque en los primeros mm (al arrancar la velocidad es baja).
    - velocidad_stall: si las ruedas van más lento que esto (grados/s) se considera choque.
    - tiempo_stall: cuánto tiempo (ms) tienen que estar frenadas para confirmar el choque.
    - nuevo_heading: si se da, reinicia el giroscopio a ese valor al chocar
      (la pared deja el robot recto, sirve para corregir el error acumulado).
    Devuelve True si chocó con la pared, False si llegó a distancia_max sin chocar.
    """
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)

    DIAMETRO_RUEDA = 63.0
    FACTOR_CONVERSION = (DIAMETRO_RUEDA * 3.141) / 360.0

    cronometro_stall = StopWatch()
    frenado = False
    choco = False

    while True:
        recorrido = ((abs(left_motor.angle()) + abs(right_motor.angle())) / 2.0) * FACTOR_CONVERSION
        if recorrido >= distancia_max:
            break

        # --- Detección de choque: las ruedas casi no giran aunque se les da potencia ---
        velocidad = (abs(left_motor.speed()) + abs(right_motor.speed())) / 2.0
        if recorrido > ignorar_mm and velocidad < velocidad_stall:
            if not frenado:
                frenado = True
                cronometro_stall.reset()
            elif cronometro_stall.time() >= tiempo_stall:
                choco = True
                break
        else:
            frenado = False

        # --- Corrección de rumbo (mismo signo que avance/retrocede). kp bajo para que,
        # al tocar la pared, el robot pueda enderezarse contra ella sin pelear con el gyro ---
        error_ang = ((target_heading - hub.imu.heading() + 180) % 360) - 180
        correccion = kp * error_ang
        if frenado:
            correccion = 0.0   # tocando la pared: deja que la pared lo enderece

        left_motor.dc(max(min(-dc + correccion, 100.0), -100.0))
        right_motor.dc(max(min(-dc - correccion, 100.0), -100.0))
        await wait(10)

    left_motor.hold()
    right_motor.hold()
    await wait(80)

    if choco and nuevo_heading is not None:
        hub.imu.reset_heading(nuevo_heading)
    return choco


async def avance_hasta_pared(target_heading, dc=60.0, distancia_max=1000.0, ignorar_mm=20.0, velocidad_stall=70.0, tiempo_stall=150, kp=2.0, nuevo_heading=None):
    """Avanza hasta chocar con una pared y frena (stall stop). Igual que retrocede_hasta_pared
    pero hacia adelante.

    - dc: potencia del avance (Calibrar: suficiente para moverse, no tanta que rebote).
    - distancia_max: seguridad; si no choca en esta distancia (mm), frena igual.
    - ignorar_mm: no detecta choque en los primeros mm (al arrancar la velocidad es baja).
    - velocidad_stall: si las ruedas van más lento que esto (grados/s) se considera choque.
    - tiempo_stall: cuánto tiempo (ms) tienen que estar frenadas para confirmar el choque.
    - nuevo_heading: si se da, reinicia el giroscopio a ese valor al chocar
      (la pared deja el robot recto, sirve para corregir el error acumulado).
    Devuelve True si chocó con la pared, False si llegó a distancia_max sin chocar.
    """
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)

    DIAMETRO_RUEDA = 63.0
    FACTOR_CONVERSION = (DIAMETRO_RUEDA * 3.141) / 360.0

    cronometro_stall = StopWatch()
    frenado = False
    choco = False

    while True:
        recorrido = ((abs(left_motor.angle()) + abs(right_motor.angle())) / 2.0) * FACTOR_CONVERSION
        if recorrido >= distancia_max:
            break

        # --- Detección de choque: las ruedas casi no giran aunque se les da potencia ---
        velocidad = (abs(left_motor.speed()) + abs(right_motor.speed())) / 2.0
        if recorrido > ignorar_mm and velocidad < velocidad_stall:
            if not frenado:
                frenado = True
                cronometro_stall.reset()
            elif cronometro_stall.time() >= tiempo_stall:
                choco = True
                break
        else:
            frenado = False

        # --- Corrección de rumbo (mismo signo que avance). kp bajo para que,
        # al tocar la pared, el robot pueda enderezarse contra ella sin pelear con el gyro ---
        error_ang = ((target_heading - hub.imu.heading() + 180) % 360) - 180
        correccion = kp * error_ang
        if frenado:
            correccion = 0.0   # tocando la pared: deja que la pared lo enderece

        left_motor.dc(max(min(dc + correccion, 100.0), -100.0))
        right_motor.dc(max(min(dc - correccion, 100.0), -100.0))
        await wait(10)

    left_motor.hold()
    right_motor.hold()
    await wait(80)

    if choco and nuevo_heading is not None:
        hub.imu.reset_heading(nuevo_heading)
    return choco


async def giro(target_heading, max_dc=100.0, min_dc=22.0, kp=2.0, kd=0.05, tolerancia=1, tiempo_estabilizacion=80, tiempo_max=3000):
    # tiempo_max: seguridad (ms); si no llega al objetivo en ese tiempo, termina igual para no quedarse pegado.
    # Arranca la derivada con el error real para evitar el golpe del primer ciclo
    error_previo = ((target_heading - hub.imu.heading() + 180) % 360) - 180
    derivada_filtrada = 0.0
    ALFA_FILTRO = 0.35

    cronometro_lazo = StopWatch()
    cronometro_estabilizacion = StopWatch()
    cronometro_total = StopWatch()
    cronometro_lazo.reset()
    cronometro_estabilizacion.reset()

    estabilizando = False

    # Anti-estancamiento: si el robot no gira (le falta poco y min_dc no alcanza), suma potencia extra
    empuje_extra = 0.0
    heading_previo = hub.imu.heading()

    while True:
        if cronometro_total.time() >= tiempo_max:
            print("giro: tiempo máximo, objetivo", target_heading, "actual", hub.imu.heading())
            break

        dt = cronometro_lazo.time() / 1000.0
        cronometro_lazo.reset()
        if dt <= 0.005: dt = 0.01

        heading_actual = hub.imu.heading()

        if abs(heading_actual - heading_previo) < 0.2:
            empuje_extra = min(empuje_extra + 1.0, 25.0)
        else:
            empuje_extra = max(empuje_extra - 2.0, 0.0)
        heading_previo = heading_actual
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
            
            minimo = min_dc + empuje_extra   # mínimo + extra si está estancado
            if abs(salida_aplicada) < minimo:
                salida_aplicada = minimo if error_ang > 0 else -minimo

            left_motor.dc(salida_aplicada)
            right_motor.dc(-salida_aplicada)

        error_previo = error_ang
        await wait(10)

    left_motor.hold()
    right_motor.hold()
    await wait(60)


async def giro_arco(target_heading, radio_mm, velocidad_mm_s=250.0, adelante=True, vel_minima_mm_s=60.0, zona_frenado=20.0, tolerancia=1.0, tiempo_max=None, freno=True, lado=None):
    # freno=False: no baja la velocidad al final ni frena; al llegar deja las dos ruedas
    #   a la misma velocidad (recto) para encadenar con avance, retrocede o seguir línea sin detenerse.
    """Gira siguiendo un arco de radio `radio_mm` (medido al CENTRO del robot)
    hasta que el heading llegue a `target_heading`.

    - lado: "derecha" (horario, el heading SUBE) o "izquierda" (antihorario, el heading BAJA).
      Si es None, toma el camino más corto. Para semicírculos (180°) INDÍCALO siempre:
      con 180° exactos el camino más corto es ambiguo.
      Puede girar más de 180° (ej. de 90 a 0 por la derecha = 270°).
    - tiempo_max: seguridad (ms). None = se calcula solo según el largo del arco.
    - adelante=False hace el arco en reversa.
    - Usa control de velocidad de los motores (run), así cada rueda mantiene su
      velocidad aunque cambie la batería y el arco sale con el radio correcto.
    - Al acercarse al objetivo (zona_frenado grados) baja la velocidad para no pasarse.
    - radio_mm menor que la mitad del eje (76 mm) hace que la rueda interior vaya hacia atrás.
    """
    EJE_MM = 152.0                       # Distancia entre ruedas (Robot_specs: DISTANCIA_EJES)
    MM_S_A_GRADOS_S = 360.0 / (63.0 * 3.141)   # mm/s -> grados/s de la rueda (rueda de 63 mm)

    heading_inicio = hub.imu.heading()
    if lado == "derecha":
        giro_total = (target_heading - heading_inicio) % 360          # 0..360, positivo
    elif lado == "izquierda":
        giro_total = -((heading_inicio - target_heading) % 360)       # -360..0, negativo
    else:
        giro_total = ((target_heading - heading_inicio + 180) % 360) - 180   # camino más corto
    if abs(giro_total) <= tolerancia:
        return
    signo_giro = 1 if giro_total > 0 else -1   # 1 = horario (derecha), -1 = antihorario (izquierda)
    sentido = 1 if adelante else -1
    radio_mm = max(abs(radio_mm), 1.0)

    # El heading de Pybricks es continuo (no se reinicia en ±180), así que se usa
    # un objetivo continuo: sirve para semicírculos y giros de más de 180°.
    objetivo_continuo = heading_inicio + giro_total

    if tiempo_max is None:
        largo_arco = abs(giro_total) * 3.141 / 180.0 * radio_mm
        tiempo_max = largo_arco / max(velocidad_mm_s, 1.0) * 1000.0 * 2.0 + 1500

    cronometro_total = StopWatch()

    while True:
        if cronometro_total.time() >= tiempo_max:
            print("giro_arco: tiempo máximo, objetivo", target_heading, "actual", hub.imu.heading())
            break

        # Grados que faltan en el sentido del giro (negativo = ya se pasó)
        error_ang = (objetivo_continuo - hub.imu.heading()) * signo_giro
        if error_ang <= tolerancia:
            break

        # Velocidad: completa y baja en los últimos grados (sin freno: siempre completa)
        if freno:
            proporcion = min(abs(error_ang) / zona_frenado, 1.0)
            v = vel_minima_mm_s + (velocidad_mm_s - vel_minima_mm_s) * proporcion
        else:
            v = velocidad_mm_s

        # Rueda exterior e interior según el radio: v_rueda = v_centro ± w * EJE/2, con w = v/R
        v_centro = v * sentido
        v_giro = signo_giro * v * (EJE_MM / 2.0) / radio_mm
        v_izq = v_centro + v_giro
        v_der = v_centro - v_giro

        left_motor.run(v_izq * MM_S_A_GRADOS_S)
        right_motor.run(v_der * MM_S_A_GRADOS_S)
        await wait(10)

    if freno:
        left_motor.hold()
        right_motor.hold()
        await wait(60)
    else:
        # Sin freno: sigue recto a la misma velocidad hasta la siguiente orden
        v_recto = velocidad_mm_s * sentido * MM_S_A_GRADOS_S
        left_motor.run(v_recto)
        right_motor.run(v_recto)


async def giro_pivot(target_heading, pivote_izquierdo=True, vel_max=700.0, vel_min=60.0, aceleracion=1800.0, desaceleracion=2000.0, tolerancia=1, tiempo_estabilizacion=80, tiempo_max=3000, freno=True):
    """Giro sobre una rueda (la otra queda frenada) con CONTROL DE VELOCIDAD (motor.run).

    El motor ajusta solo la fuerza necesaria para mantener la velocidad pedida, así que
    no se estanca al final aunque arrastre todo el robot o la batería esté más baja.
    Perfil trapezoidal: acelera parejo, crucero y frena parejo (desaceleración constante),
    como un auto que frena suave hasta el punto exacto, sin arrastrarse al final.
    - vel_max: velocidad de crucero de la rueda que empuja (°/s). Calibrar.
    - vel_min: velocidad mínima solo para el último pedacito (°/s). Calibrar.
    - aceleracion: °/s² al arrancar (más baja = arranque más suave). Calibrar.
    - desaceleracion: °/s² al frenar (más baja = frena antes y más suave; más alta = frena
      más tarde y más brusco; si se pasa del objetivo, bájala). Calibrar.
    - freno=False: gira a vel_max, termina apenas llega o se pasa y no frena (para encadenar).
    """
    # En pivot la rueda que empuja recorre un círculo de radio = distancia entre ruedas:
    # grados de rueda por cada grado que gira el robot = EJE / radio de rueda
    EJE_MM = 152.0
    RUEDA_POR_GRADO = EJE_MM / (63.0 / 2.0)   # ≈ 4.8
    error_ang = ((target_heading - hub.imu.heading() + 180) % 360) - 180
    signo_error_previo = 1 if error_ang > 0 else -1

    cronometro_lazo = StopWatch()
    cronometro_estabilizacion = StopWatch()
    cronometro_total = StopWatch()
    estabilizando = False
    vel_anterior = 0.0

    if pivote_izquierdo:
        left_motor.hold()
        motor_empuja = right_motor
        sentido_motor = -1   # girar a la derecha (error > 0) = rueda derecha hacia atrás
    else:
        right_motor.hold()
        motor_empuja = left_motor
        sentido_motor = 1    # girar a la derecha (error > 0) = rueda izquierda hacia adelante

    while True:
        if cronometro_total.time() >= tiempo_max:
            print("giro_pivot: tiempo máximo, objetivo", target_heading, "actual", hub.imu.heading())
            break

        dt = cronometro_lazo.time() / 1000.0
        cronometro_lazo.reset()
        if dt <= 0.005: dt = 0.01

        error_ang = ((target_heading - hub.imu.heading() + 180) % 360) - 180
        abs_error = abs(error_ang)
        signo_error = 1 if error_ang > 0 else -1

        # Sin freno: termina en cuanto llega o se pasa, sin estabilizar ni frenar
        if not freno and (abs_error <= tolerancia or signo_error != signo_error_previo):
            break

        if signo_error != signo_error_previo:
            vel_anterior = 0.0   # se pasó: vuelve despacio en sentido contrario
        signo_error_previo = signo_error

        if abs_error <= tolerancia:
            left_motor.hold()
            right_motor.hold()
            vel_anterior = 0.0
            if not estabilizando:
                estabilizando = True
                cronometro_estabilizacion.reset()
            elif cronometro_estabilizacion.time() >= tiempo_estabilizacion:
                break
        else:
            estabilizando = False

            # Velocidad objetivo: frenado con desaceleración constante, v = √(2·a·d),
            # d = grados de rueda que faltan. Baja suave y parejo hasta el objetivo.
            if freno:
                grados_rueda_faltan = abs_error * RUEDA_POR_GRADO
                vel_frenado = (2.0 * desaceleracion * grados_rueda_faltan) ** 0.5
                vel_objetivo = max(min(vel_frenado, vel_max), vel_min)
            else:
                vel_objetivo = vel_max

            # Rampa de arranque (solo limita la subida; la bajada la da el perfil de frenado)
            vel = min(vel_objetivo, vel_anterior + aceleracion * dt)
            vel = max(vel, vel_min)
            vel_anterior = vel

            motor_empuja.run(sentido_motor * signo_error * vel)

        await wait(10)

    if freno:
        left_motor.hold()
        right_motor.hold()
        await wait(60)
    # Sin freno: la rueda sigue girando a la última velocidad hasta la siguiente orden

async def giro_pivot_dc(target_heading, pivote_izquierdo=True, max_dc=80.0, min_dc=28.0, kp=2.2, kd=0.15, tolerancia=1, tiempo_estabilizacion=80, tiempo_max=3000, freno=True):
    # VERSIÓN ANTERIOR (control por potencia dc). Se deja para comparar; giro_pivot usa control de velocidad.
    # freno=False: gira a max_dc todo el tiempo (sin desacelerar), termina apenas llega o se pasa
    #   del objetivo y no frena: los motores quedan con la última potencia para encadenar
    #   con el siguiente movimiento (avance, retrocede, seguir línea...) sin detenerse.
    # min_dc: Calibrar. En pivot UNA sola rueda arrastra todo el robot, necesita más potencia mínima que el giro normal.
    # tiempo_max: seguridad (ms); si no llega al objetivo en ese tiempo, termina igual para no quedarse pegado.
    # Arranca la derivada con el error real para evitar el golpe del primer ciclo
    error_previo = ((target_heading - hub.imu.heading() + 180) % 360) - 180
    derivada_filtrada = 0.0
    ALFA_FILTRO = 0.35

    cronometro_lazo = StopWatch()
    cronometro_estabilizacion = StopWatch()
    cronometro_total = StopWatch()
    cronometro_lazo.reset()
    cronometro_estabilizacion.reset()

    estabilizando = False

    # Anti-estancamiento: si la rueda que empuja casi no gira, se va sumando potencia extra.
    # Se mide con la velocidad del motor (más fiable que el giroscopio a baja velocidad).
    empuje_extra = 0.0
    VEL_ESTANCADO = 60.0     # Calibrar: °/s de la rueda por debajo de esto = estancado
    EMPUJE_MAX = 45.0        # Calibrar: potencia extra máxima que se puede sumar
    signo_error_previo = 1 if error_previo > 0 else -1

    if pivote_izquierdo:
        left_motor.hold()
        motor_empuja = right_motor
    else:
        right_motor.hold()
        motor_empuja = left_motor

    while True:
        if cronometro_total.time() >= tiempo_max:
            break

        dt = cronometro_lazo.time() / 1000.0
        cronometro_lazo.reset()
        if dt <= 0.005: dt = 0.01

        heading_actual = hub.imu.heading()
        error_ang = target_heading - heading_actual
        error_ang = (error_ang + 180) % 360 - 180
        abs_error = abs(error_ang)

        signo_error = 1 if error_ang > 0 else -1

        # Sin freno: termina en cuanto llega o se pasa, sin estabilizar ni frenar
        if not freno and (abs_error <= tolerancia or signo_error != signo_error_previo):
            break

        if signo_error != signo_error_previo:
            # Se pasó del objetivo: quita el empuje extra para no rebotar al otro lado
            empuje_extra = 0.0
        elif abs_error > tolerancia:
            vel_rueda = abs(motor_empuja.speed())
            if vel_rueda < VEL_ESTANCADO:
                # Estancado: sube el empuje (≈ +15% de potencia cada 100 ms)
                empuje_extra = min(empuje_extra + 1.5, EMPUJE_MAX)
            elif vel_rueda > 2 * VEL_ESTANCADO:
                # Ya se mueve bien: baja el empuje despacio (no de golpe, para no volver a frenarse)
                empuje_extra = max(empuje_extra - 0.5, 0.0)
        signo_error_previo = signo_error

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
            if not freno or abs_error > ZONA_FRENADO:
                limite_dc = max_dc
                termino_p = max_dc if error_ang > 0 else -max_dc
            else:
                proporcion = abs_error / ZONA_FRENADO
                limite_dc = min_dc + (max_dc - min_dc) * proporcion
                termino_p = kp * error_ang

            if freno:
                salida_pd = termino_p + termino_d
                salida_aplicada = max(min(salida_pd, limite_dc), -limite_dc)
            else:
                salida_aplicada = termino_p   # potencia fija, sin PD que la reduzca

            minimo = min_dc + empuje_extra   # mínimo + extra si está estancado
            if abs(salida_aplicada) < minimo:
                salida_aplicada = minimo if error_ang > 0 else -minimo

            if pivote_izquierdo:
                right_motor.dc(-salida_aplicada)
            else:
                left_motor.dc(salida_aplicada)

        error_previo = error_ang
        await wait(10)

    if freno:
        left_motor.hold()
        right_motor.hold()
        await wait(60)
    # Sin freno: los motores quedan con la última potencia hasta la siguiente orden