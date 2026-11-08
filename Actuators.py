from Extras import boton
from pybricks.pupdevices import Motor
from pybricks.parameters import Port
from pybricks.tools import wait, StopWatch
from Funciones_ESP32 import servo
# =============================================================================
# INICIALIZACIÓN DE MOTORES DE ACCESORIOS
# =============================================================================

# motorgarras = Motor(Port.C)
motorElevador = Motor(Port.E)
motorcuadro = Motor(Port.A)

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
    await motor_dc_por_segundos(motorElevador, 1000, -60)

async def bajar_cuadro():
    await motor_dc_por_grados(motorcuadro, 400, -80)

async def subir_cuadro():
    await motor_dc_por_segundos(motorcuadro, 1500, 90)

async def cerrar_cuadro():
    await motor_dc_por_grados(motorcuadro, -400, 80)  # calibrar

async def cerrar_paletas():
    await servo(19, 270)  # cerrar paletas

async def abrir_paletas():
    await servo(19, 30)  # abrir paletas

async def abrir_garra():
    await wait (500)
    #await servo(20, 30)  # abrir garra

async def abrir_garra_todo():
    await wait (500)
    #await servo(20, 260)  # abrir garra

async def cerrar_garra():
    await wait (500)
    #await servo(20, 100)  # cerrar garra

async def cerrar_garra_poquito():
    await wait (500)
    #await servo(20, 45)  # cerrar garra un poco
