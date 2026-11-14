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

async def subir_elevador():
    await motor_dc_por_grados(motorElevador, 170, 100)

async def bajar_elevador():
    await multitask (motor_dc_por_segundos(motorElevador, 600, -50),servo (20,90,500))
    await multitask (motor_dc_por_grados(motorElevador, 15, 100),servo (20,60,100))
    motorElevador.hold()

async def bajar_cuadro(espera=900):
    await servo (20,237,espera)

async def bajar_cuadro_poquito():
    await servo (20,170,700)

async def subir_cuadro(espera=600):
    await servo (20,65,espera)
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

async def abrir_garra_poquito():
       await motor_dc_por_grados (motorgarra, 18, -90)

