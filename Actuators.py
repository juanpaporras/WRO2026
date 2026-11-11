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
    await motor_dc_por_segundos(motorcuadro, 1000, -90)  # calibrar

async def subir_cuadro():
    await motor_dc_por_grados(motorcuadro, 300, 90)  # calibrar

async def cerrar_cuadro():
    await motor_dc_por_grados(motorcuadro, -400, 80)  # calibrar

async def cerrar_paletas():
    await servo(19, 260)  # cerrar paletas

async def abrir_paletas():
    await servo(19, 30)  # abrir paletas

async def abrir_garra():
    await servo(20, 150)  # abrir garra
 
# La garra abierta empuja contra su tope (a propósito, para tensar las ligas).
# Si se le pide un ángulo MUY pasado del tope, el servo queda trabado a toda fuerza,
# se calienta y su protección interna lo bloquea: al cerrar solo zumba.
# Por eso se le pide apenas unos grados más allá del tope: hace fuerza, pero poca.
# Probado con prueba_servo.py: a 95 hace los 20 ciclos; a 90 el servo no vence las ligas y se bloquea.
GARRA_TOQUE = 95         # Calibrar: apertura máxima sin que el servo se bloquee (probar 93-94)
GARRA_PRECARGA = 0       # Calibrar: grados extra (más = más riesgo de bloqueo)
GARRA_CERRADA = 230      # Calibrar: garra cerrada

async def abrir_garra_todo():
    await servo(20, GARRA_TOQUE - GARRA_PRECARGA)  # abrir garra contra el tope

async def cerrar_garra():
    # Pasa directo de abierta a cerrada (sin soltar el eje), igual que en prueba_servo.py
    # con SOLTAR = "no", que fue lo que funcionó los 20 ciclos.
    await servo(20, GARRA_CERRADA, espera_ms=350)  # Calibrar: tiempo de cierre

async def cerrar_garra_poquito():
    await wait (500)
    #await servo(20, 45)  # cerrar garra un poco
