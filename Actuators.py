from pybricks.pupdevices import Motor
from pybricks.parameters import Port
from pybricks.tools import wait, multitask, StopWatch
from Funciones_ESP32 import servo
# =============================================================================
# INICIALIZACIÓN DE MOTORES DE ACCESORIOS
# =============================================================================

motorElevador = Motor(Port.E)
motorgarra = Motor(Port.A)

# =============================================================================
# FUNCIONES CINEMÁTICAS Y DE CONTROL DE ACCESORIOS
# =============================================================================

async def motor_dc_por_grados(motor, grados, potencia):
    # El SENTIDO lo da el signo de la potencia; los grados son siempre la cantidad a girar (se usa abs).
    grados = abs(grados)
    if potencia > 0:
        angulo_objetivo = motor.angle() + grados
        motor.dc(potencia)
        while motor.angle() < angulo_objetivo:
            await wait(10)
    elif potencia < 0:
        angulo_objetivo = motor.angle() - grados
        motor.dc(potencia)
        while motor.angle() > angulo_objetivo:
            await wait(10)
    motor.hold()

async def motor_dc_por_segundos(motor, milisegundos, potencia):
    cronometro = StopWatch()
    motor.dc(potencia)
    cronometro.reset()
    while cronometro.time() < milisegundos:
        await wait(10)
    motor.brake()

async def motor_dc_hasta_tope(motor, potencia, tiempo_max_ms, vel_tope=60, tiempo_tope_ms=40, arranque_ms=120):
    """Empuja con potencia hasta que el motor se FRENA contra su tope (o pasa tiempo_max_ms).
    Termina apenas llega, en vez de esperar siempre el tiempo completo.
    - vel_tope: °/s por debajo de esto = está contra el tope. Calibrar.
    - tiempo_tope_ms: cuánto tiempo seguido debe estar frenado para confirmarlo.
    - arranque_ms: al inicio no revisa (el motor parte desde 0 °/s)."""
    cronometro = StopWatch()
    cronometro_tope = StopWatch()
    frenado = False
    motor.dc(potencia)
    while cronometro.time() < tiempo_max_ms:
        if cronometro.time() > arranque_ms and abs(motor.speed()) < vel_tope:
            if not frenado:
                frenado = True
                cronometro_tope.reset()
            elif cronometro_tope.time() >= tiempo_tope_ms:
                break
        else:
            frenado = False
        await wait(10)
    motor.brake()

# =============================================================================
# CUADRO (servo 20) Y ELEVADOR
# Si el cuadro está ARRIBA, el elevador choca con él al subir o bajar: en ese caso el cuadro
# se corre a CUADRO_ESQUIVA mientras BAJA el elevador y después vuelve a donde estaba.
# Al SUBIR el elevador el cuadro no se mueve (moverlo a la vez le quitaba voltaje a la subida).
# Si el cuadro está ABAJO (agarrando algo) NO se toca, para no soltar lo que lleva.
# =============================================================================
CUADRO_ARRIBA = 65        # Calibrar: ángulo de subir_cuadro
CUADRO_ABAJO = 237        # Calibrar: ángulo de bajar_cuadro
CUADRO_POQUITO = 170      # Calibrar: ángulo de bajar_cuadro_poquito
CUADRO_ESQUIVA = 90       # Calibrar: ángulo al que se corre el cuadro para que pase el elevador
                          #   (por debajo de este ángulo se considera que el cuadro está ARRIBA)

cuadro_angulo = CUADRO_ARRIBA   # Última posición mandada al cuadro (se asume arriba al iniciar)
cuadro_a_devolver = None        # Si bajar_elevador_grados dejó el cuadro corrido: ángulo al que bajar_elevador lo devuelve

async def mover_cuadro(grados, espera):
    """Mueve el cuadro y RECUERDA dónde quedó (para que el elevador sepa si debe esquivarlo)."""
    global cuadro_angulo, cuadro_a_devolver
    await servo(20, grados, espera)
    cuadro_angulo = grados
    cuadro_a_devolver = None    # se movió a mano: ya no hay nada pendiente de devolver

def cuadro_esta_arriba():
    return cuadro_angulo < CUADRO_ESQUIVA

GRADOS_ELEVADOR_POQUITO = 170   # Calibrar: grados que SUBE subir_elevador y que BAJA bajar_elevador_poquito (los dos usan este mismo valor)

async def subir_elevador():
    await motor_dc_por_grados(motorElevador, GRADOS_ELEVADOR_POQUITO, 100)


async def bajar_elevador_poquito(potencia=60):
    # Baja POR GRADOS lo mismo que sube subir_elevador (GRADOS_ELEVADOR_POQUITO), no hasta el tope.
    # Calibrar: potencia (bajando ayuda el peso; más baja = más suave, muy baja = no se mueve)
    global elevador_arriba
    elevador_arriba = False
    if cuadro_esta_arriba():
        # Cuadro arriba: lo corre mientras baja el elevador y luego lo devuelve
        angulo_original = cuadro_angulo
        await multitask(motor_dc_por_grados(motorElevador, GRADOS_ELEVADOR_POQUITO, -abs(potencia)), mover_cuadro(CUADRO_ESQUIVA, 500))
        await mover_cuadro(angulo_original, 100)
    else:
        # Cuadro abajo agarrando algo: no se toca
        await motor_dc_por_grados(motorElevador, GRADOS_ELEVADOR_POQUITO, -abs(potencia))

# --- Subida completa del elevador ---
# Problema: al llegar arriba a potencia alta los engranes BARREN (saltan dientes). Mientras barren el motor
# sigue girando, así que la detección por velocidad (motor_dc_hasta_tope) no ve el tope y sigue empujando.
# Solución: bajar_elevador pone el ángulo en 0 al tocar el tope de ABAJO; con esa referencia la subida
# va rápida hasta cerca de arriba, termina lenta y se detiene por GRADOS antes de forzar los engranes.
ELEVADOR_GRADOS_ARRIBA = 333  # Calibrar con medir_elevador_arriba(): grados desde el tope de abajo hasta arriba.
                             #   Poner unos 10° MENOS que lo medido. 0 = desactivado (sube hasta el tope como antes).
ELEVADOR_MARGEN_LENTO = 80   # Calibrar: últimos grados de la subida que se hacen despacio
ELEVADOR_DC_LENTO = 50       # Calibrar: potencia en ese tramo final (muy baja = no llega con carga)
ELEVADOR_TIEMPO_MAX_MS = 1500
ELEVADOR_VEL_TOPE = 60       # Calibrar: °/s por debajo de esto = el elevador llegó al tope y está esperando
ELEVADOR_TIEMPO_TOPE_MS = 60 # Calibrar: ms seguidos frenado para confirmar el tope (muy bajo = para antes si se traba un momento)
ELEVADOR_ARRANQUE_MS = 150   # Calibrar: ms del inicio en que no revisa el tope (el motor parte desde 0 °/s)

elevador_referenciado = False   # True después de que bajar_elevador tocó el tope de abajo y puso el ángulo en 0
elevador_arriba = False         # True si el elevador está COMPLETAMENTE arriba (subir_elevador_todo o estado_inicial_actuadores)

def estado_inicial_actuadores(elevador_arriba_inicio=True, cuadro_inicio=CUADRO_ARRIBA):
    """Le dice a las funciones cómo ARRANCA el robot (llamar al inicio de la Ruta 1, antes de mover nada).
    - elevador_arriba_inicio=True: el elevador empieza ARRIBA -> subir_elevador_todo no lo vuelve a subir
      (no fuerza los engranes contra el tope) y el ángulo queda como si estuviera en ELEVADOR_GRADOS_ARRIBA.
    - cuadro_inicio: ángulo en el que arranca el cuadro (CUADRO_ARRIBA por defecto) -> el elevador sabe
      que debe esquivarlo al bajar."""
    global elevador_arriba, cuadro_angulo, cuadro_a_devolver
    cuadro_angulo = cuadro_inicio
    cuadro_a_devolver = None
    elevador_arriba = elevador_arriba_inicio
    if elevador_arriba_inicio:
        motorElevador.reset_angle(ELEVADOR_GRADOS_ARRIBA)
        motorElevador.hold()   # lo sostiene arriba desde el arranque
    else:
        motorElevador.reset_angle(0)

async def subir_elevador_todo(potencia=100):
    # Al subir NO se mueve el cuadro: moverlo a la vez le quitaba voltaje a la subida
    global elevador_arriba
    if elevador_arriba:
        # Ya está arriba: no empuja contra el tope (los engranes barrerían)
        motorElevador.hold()
        return
    elevador_arriba = True
    if not elevador_referenciado or ELEVADOR_GRADOS_ARRIBA <= 0:
        # Sin referencia (todavía no se ha bajado) o sin calibrar: sube hasta el tope como antes
        await motor_dc_por_grados (motorElevador,ELEVADOR_GRADOS_ARRIBA-GRADOS_ELEVADOR_POQUITO-ELEVADOR_MARGEN_LENTO,potencia)
        await motor_dc_hasta_tope(motorElevador, potencia-50, ELEVADOR_TIEMPO_MAX_MS)
        motorElevador.hold()
        print("entro aca")
        return
    cronometro = StopWatch()
    cronometro_tope = StopWatch()
    frenado = False
    llego_tope = False
    # Tramo rápido y luego tramo lento hasta el ángulo de arriba. En los dos: si el motor se
    # FRENA (ya llegó al tope antes de ELEVADOR_GRADOS_ARRIBA), termina ahí en vez de esperar el tiempo máximo.
    motorElevador.dc(potencia)
    tramo_lento = False
    while motorElevador.angle() < ELEVADOR_GRADOS_ARRIBA and cronometro.time() < ELEVADOR_TIEMPO_MAX_MS:
        if not tramo_lento and motorElevador.angle() >= ELEVADOR_GRADOS_ARRIBA - ELEVADOR_MARGEN_LENTO:
            tramo_lento = True
            motorElevador.dc(ELEVADOR_DC_LENTO)
            frenado = False
        # Detección de tope (no revisa al arrancar: el motor parte desde 0 °/s)
        if cronometro.time() > ELEVADOR_ARRANQUE_MS and abs(motorElevador.speed()) < ELEVADOR_VEL_TOPE:
            if not frenado:
                frenado = True
                cronometro_tope.reset()
            elif cronometro_tope.time() >= ELEVADOR_TIEMPO_TOPE_MS:
                llego_tope = True
                break
        else:
            frenado = False
        await wait(5)
    motorElevador.hold()
    if llego_tope:
        # Llegó al tope antes del ángulo calibrado: avisa para poder bajar ELEVADOR_GRADOS_ARRIBA
        print("subir_elevador_todo: tope en ángulo", motorElevador.angle(), "(ELEVADOR_GRADOS_ARRIBA =", ELEVADOR_GRADOS_ARRIBA, ")")
    # Diagnóstico: si sale esto, no llegó a ELEVADOR_GRADOS_ARRIBA y se quedó empujando hasta el tiempo máximo
    if cronometro.time() >= ELEVADOR_TIEMPO_MAX_MS:
        print("subir_elevador_todo: TIEMPO MÁXIMO, ángulo", motorElevador.angle(), "de", ELEVADOR_GRADOS_ARRIBA)

async def medir_elevador_arriba():
    """PRUEBA para calibrar ELEVADOR_GRADOS_ARRIBA: baja hasta el tope, sube DESPACIO hasta el tope de arriba
    (despacio los engranes no barren) e imprime los grados. Correrla 2-3 veces y usar el menor valor - 10."""
    await bajar_elevador()
    await motor_dc_hasta_tope(motorElevador, 40, 4000, arranque_ms=300)   # Calibrar 40 si no logra subir
    print("Elevador: grados del tope de abajo al de arriba =", motorElevador.angle())
    motorElevador.hold()

async def bajar_elevador_grados(grados, potencia=40):
    """Baja el elevador SOLO 'grados' (primer tramo de una bajada en dos partes, ej. bajar un poco,
    avanzar y luego bajar_elevador). Si el cuadro está arriba lo corre a CUADRO_ESQUIVA y lo DEJA
    corrido: bajar_elevador lo devuelve a su lugar cuando el elevador termina de bajar.
    Si el cuadro está abajo (agarrando algo) no se toca."""
    global cuadro_a_devolver, elevador_arriba
    elevador_arriba = False
    if cuadro_esta_arriba():
        angulo_original = cuadro_angulo
        await mover_cuadro(CUADRO_ESQUIVA, 300)
        await motor_dc_por_grados(motorElevador, grados, -abs(potencia))
        cuadro_a_devolver = angulo_original   # después del multitask: mover_cuadro lo borra
    else:
        await motor_dc_por_grados(motorElevador, grados, -abs(potencia))

async def bajar_elevador():
    # Baja hasta que se BLOQUEA contra el tope de abajo (termina apenas llega) y luego sube 10° para no quedar forzando
    # Calibrar: potencia -30 y 1000 ms máximo de seguridad
    global elevador_referenciado, cuadro_a_devolver, elevador_arriba
    elevador_arriba = False
    angulo_original = None
    if cuadro_a_devolver is not None:
        # bajar_elevador_grados ya dejó el cuadro corrido: solo baja y al final lo devuelve
        angulo_original = cuadro_a_devolver
        cuadro_a_devolver = None
        await motor_dc_hasta_tope(motorElevador, -40, 1000)
    elif cuadro_esta_arriba():
        # Cuadro arriba: lo corre mientras baja el elevador y luego lo devuelve
        angulo_original = cuadro_angulo
        await mover_cuadro(CUADRO_ESQUIVA, 250)
        await motor_dc_hasta_tope(motorElevador, -40, 1000)
    else:
        # Cuadro abajo agarrando algo: NO se toca
        await motor_dc_hasta_tope(motorElevador, -40, 1000)
    # Tope de abajo = ángulo 0 (se fija ANTES de subir los 10°, para medir siempre desde el tope real)
    motorElevador.reset_angle(0)
    if angulo_original is not None:
        # Dentro de multitask van las funciones SIN await (multitask es quien las espera)
        await multitask (mover_cuadro(angulo_original, 100), motor_dc_por_grados(motorElevador, 6, 100))
    else:
        await motor_dc_por_grados(motorElevador, 6, 100)
    elevador_referenciado = True
    motorElevador.hold()

async def bajar_cuadro(espera=900):
    await mover_cuadro(CUADRO_ABAJO, espera)

async def bajar_cuadro_poquito():
    await mover_cuadro(CUADRO_POQUITO, 700)

async def subir_cuadro(espera=600):
    await mover_cuadro(CUADRO_ARRIBA, espera)
async def cerrar_paletas():
    await servo(19, 260, 600)  # cerrar paletas

async def abrir_paletas():
    await servo(19, 95,600)  # abrir paletas

# Detección del tope al ABRIR la garra. Con los valores por defecto (60 °/s durante 40 ms) la garra
# se daba por "en el tope" antes de tiempo (al arrancar lenta o con roce) y abría muy poco.
GARRA_VEL_TOPE = 20         # Calibrar: °/s por debajo de esto = contra el tope (más bajo = más difícil cortar antes)
GARRA_TIEMPO_TOPE_MS = 150  # Calibrar: tiempo seguido frenada para confirmar el tope (más alto = abre más seguro)
GARRA_ARRANQUE_MS = 250     # Calibrar: tiempo inicial en que no se revisa el tope (la garra parte cerrada/apretada)
GARRA_TIEMPO_MAX_MS = 1500  # Seguridad: corta siempre después de esto

async def abrir_garra(potencia=-100):
    # Abre hasta el tope (termina apenas llega); GARRA_TIEMPO_MAX_MS solo como seguridad
    await motor_dc_hasta_tope(motorgarra, potencia, GARRA_TIEMPO_MAX_MS,
                              vel_tope=GARRA_VEL_TOPE, tiempo_tope_ms=GARRA_TIEMPO_TOPE_MS,
                              arranque_ms=GARRA_ARRANQUE_MS)
    motorgarra.hold()

async def cerrar_garra():
    await motor_dc_por_segundos (motorgarra,1000,100)
    motorgarra.hold()

async def cerrar_garra_poquito():
    await motor_dc_por_grados (motorgarra,45,80)

# =============================================================================
# COMER BLOQUES (logica_de_comer: cada 2 bloques, sin ir a descargar)
# =============================================================================

async def abrir_garra_poquito():
       await motor_dc_por_grados (motorgarra, 18, -90)

