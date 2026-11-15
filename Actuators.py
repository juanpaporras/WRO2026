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
# se corre a CUADRO_ESQUIVA mientras se mueve el elevador y después vuelve a donde estaba.
# Si el cuadro está ABAJO (agarrando algo) NO se toca, para no soltar lo que lleva.
# =============================================================================
CUADRO_ARRIBA = 65        # Calibrar: ángulo de subir_cuadro
CUADRO_ABAJO = 237        # Calibrar: ángulo de bajar_cuadro
CUADRO_POQUITO = 170      # Calibrar: ángulo de bajar_cuadro_poquito
CUADRO_ESQUIVA = 90       # Calibrar: ángulo al que se corre el cuadro para que pase el elevador
                          #   (por debajo de este ángulo se considera que el cuadro está ARRIBA)

cuadro_angulo = CUADRO_ARRIBA   # Última posición mandada al cuadro (se asume arriba al iniciar)

async def mover_cuadro(grados, espera):
    """Mueve el cuadro y RECUERDA dónde quedó (para que el elevador sepa si debe esquivarlo)."""
    global cuadro_angulo
    await servo(20, grados, espera)
    cuadro_angulo = grados

def cuadro_esta_arriba():
    return cuadro_angulo < CUADRO_ESQUIVA

async def subir_elevador():
    if cuadro_esta_arriba():
        # Cuadro arriba: lo corre mientras sube el elevador y luego lo devuelve
        angulo_original = cuadro_angulo
        await multitask(motor_dc_por_grados(motorElevador, 170, 100), mover_cuadro(CUADRO_ESQUIVA, 500))
        await mover_cuadro(angulo_original, 100)
    else:
        # Cuadro abajo: no se toca
        await motor_dc_por_grados(motorElevador, 170, 100)

async def subir_elevador_todo(potencia=100):
    # Sube el elevador hasta que se BLOQUEA contra su tope (termina apenas llega)
    # Calibrar: potencia (más baja = golpea menos el tope) y 1500 ms máximo de seguridad
    if cuadro_esta_arriba():
        # Cuadro arriba: lo corre mientras sube el elevador y luego lo devuelve
        angulo_original = cuadro_angulo
        await multitask(motor_dc_hasta_tope(motorElevador, potencia, 1500), mover_cuadro(CUADRO_ESQUIVA, 500))
        await mover_cuadro(angulo_original, 100)
    else:
        # Cuadro abajo: no se toca
        await motor_dc_hasta_tope(motorElevador, potencia, 1500)
    motorElevador.hold()

async def bajar_elevador():
    # Baja hasta que se BLOQUEA contra el tope de abajo (termina apenas llega) y luego sube 15° para no quedar forzando
    # Calibrar: potencia -50 y 1000 ms máximo de seguridad (antes eran 600 ms fijos)
    if cuadro_esta_arriba():
        # Cuadro arriba: lo corre mientras baja el elevador y luego lo devuelve
        angulo_original = cuadro_angulo
        await multitask (motor_dc_hasta_tope(motorElevador, -40, 1000), mover_cuadro(CUADRO_ESQUIVA, 500))
        await multitask (motor_dc_por_grados(motorElevador, 15, 100), mover_cuadro(angulo_original, 100))
    else:
        # Cuadro abajo agarrando algo: NO se sube (si no, suelta lo que lleva)
        await motor_dc_hasta_tope(motorElevador, -40, 1000)
        await motor_dc_por_grados(motorElevador, 15, 100)
    motorElevador.hold()

async def bajar_cuadro(espera=900):
    await mover_cuadro(CUADRO_ABAJO, espera)

async def bajar_cuadro_poquito():
    await mover_cuadro(CUADRO_POQUITO, 700)

async def subir_cuadro(espera=600):
    await mover_cuadro(CUADRO_ARRIBA, espera)
async def cerrar_paletas():
    await servo(19, 270, 600)  # cerrar paletas

async def abrir_paletas():
    await servo(19, 95,600)  # abrir paletas

async def abrir_garra():
    # Abre hasta el tope (termina apenas llega); 1000 ms solo como seguridad
    await motor_dc_hasta_tope (motorgarra, -90, 1000)
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

