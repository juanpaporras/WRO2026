from Extras import boton
from pybricks.pupdevices import Motor
from pybricks.parameters import Port
from pybricks.tools import wait, StopWatch
from Funciones_ESP32 import servo, mover_servo
# =============================================================================
# INICIALIZACIÓN DE MOTORES DE ACCESORIOS
# =============================================================================

# motorgarras = Motor(Port.C)
motorElevador = Motor(Port.E)
motorgarra = Motor(Port.A)

# =============================================================================
# CONFIGURACIÓN BÁSICA DE ACTUADORES
# =============================================================================

def configurar_actuador(motor):
    motor.control.limits(speed=1000, acceleration=2000, torque=None)
    motor.control.pid(kp=500, ki=10, kd=100)

# =============================================================================
# FUNCIONES CINEMÁTICAS Y DE CONTROL DE ACCESORIOS
# =============================================================================

async def motor_dc_por_grados(motor, grados, potencia):
    angulo_objetivo = motor.angle() + grados
    if grados > 0:
        motor.dc(abs(potencia))
        while motor.angle() < angulo_objetivo:
            await wait(10)
    elif grados < 0:
        motor.dc(-abs(potencia))
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

async def subir_elevador_todo():
    await motor_dc_por_segundos(motorElevador, 700, 90)

async def subir_elevador():
    await motor_dc_por_grados(motorElevador, 185, 100)

async def bajar_elevador():
    await motor_dc_por_segundos(motorElevador, 800, -30)

async def bajar_cuadro():
    await servo (20,237,1000)

async def subir_cuadro():
    await servo (20,65,1000)

async def cerrar_paletas():
    await servo(19, 260,600)  # cerrar paletas

async def abrir_paletas():
    await servo(19, 30,600)  # abrir paletas

async def abrir_garra():
    await motor_dc_por_segundos (motorgarra,1300,-80)
    motorgarra.hold()

async def cerrar_garra():
    await motor_dc_por_segundos (motorgarra,1500,80)
    motorgarra.hold()

async def cerrar_garra_poquito():
    await motor_dc_por_grados (motorgarra,50,80)

