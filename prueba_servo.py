# Prueba aislada del servo de la garra (correr en el ESP32 desde Thonny, NO como main.py)
# No usa hub, PUPRemote ni HuskyLens: solo el servo.
#
# MODO = "medir"   -> abre la garra poco a poco (5 grados cada 2 s) e imprime el ángulo.
#                     Anota el número en que la garra LLEGA AL TOPE y deja de abrirse.
# MODO = "apretar" -> abre la garra, te da tiempo de poner el bloque y la cierra
#                     poco a poco (5 grados cada 2 s) imprimiendo el ángulo.
#                     Anota el número en que la garra YA SOSTIENE BIEN el bloque.
# MODO = "ciclos"  -> repite la secuencia del robot: abrir 5 s, (soltar), cerrar.
#
# SOLTAR = forma de "dejar de hacer fuerza" antes de cerrar (solo MODO = "ciclos"):
#   "no"     -> no suelta: pasa directo de abierta a cerrada
#   "duty0"  -> deja el pin en 0
#   "deinit" -> apaga el PWM y lo vuelve a crear al cerrar
from machine import Pin, PWM
import time

MODO = "apretar"    # Editar: "medir", "apretar" o "ciclos"
SOLTAR = "no"

PIN = 20            # pin de la garra
TOQUE = 95          # ángulo de garra abierta (medido)
PRECARGA = 0        # Calibrar: grados extra contra el tope (igual que GARRA_PRECARGA)
CERRAR = 230        # Calibrar: ángulo cerrado (igual que GARRA_CERRADA)
T_ABIERTA = 5       # segundos que se mantiene abierta (igual que en Run)
T_SUELTA_MS = 150   # tiempo sin señal antes de cerrar
T_CERRADA = 2       # segundos que se mantiene cerrada
CICLOS = 20

T_PONER_BLOQUE = 8  # segundos para poner el bloque dentro de la garra (MODO = "apretar")
PASO_APRETAR = 5    # grados por paso al cerrar (MODO = "apretar")
T_PASO = 2          # segundos entre pasos (para revisar si ya aprieta)

pwm = None

def duty(g):
    pulso = 500 + g / 270 * 2000          # 500-2500 us = s0-270 grados
    return int(pulso / 20000 * 65535)

def ir(g):
    global pwm
    if pwm is None:                       # (re)crea el PWM ya con el pulso correcto
        pwm = PWM(Pin(PIN), freq=50, duty_u16=duty(g))
    else:
        pwm.duty_u16(duty(g))

def soltar():
    global pwm
    if pwm is None:
        return
    if SOLTAR == "deinit":
        pwm.deinit()
        pwm = None
        Pin(PIN, Pin.OUT, value=0)        # pin en bajo: el servo no recibe pulsos
    else:
        pwm.duty_u16(0)

def medir():
    ir(CERRAR)
    time.sleep(2)
    for g in range(CERRAR, 20, -5):      # de cerrada hacia abierta, de 5 en 5
        ir(g)
        print("angulo", g)
        time.sleep(2)

def apretar():
    ir(TOQUE - PRECARGA)
    print("Garra abierta. Pon el bloque dentro de la garra...")
    for s in range(T_PONER_BLOQUE, 0, -1):
        print("cerrando en", s)
        time.sleep(1)
    for g in range(TOQUE, CERRAR + 1, PASO_APRETAR):   # de abierta hacia cerrada
        ir(g)
        print("angulo", g)
        time.sleep(T_PASO)
    print("Llegó a CERRAR =", CERRAR)
    time.sleep(2)

def ciclos():
    print("SOLTAR =", SOLTAR)
    for i in range(1, CICLOS + 1):
        print("ciclo", i, "- abrir")
        ir(TOQUE - PRECARGA)
        time.sleep(T_ABIERTA)
        if SOLTAR != "no":
            print("ciclo", i, "- soltar")
            soltar()
            time.sleep_ms(T_SUELTA_MS)
        print("ciclo", i, "- cerrar")
        ir(CERRAR)
        time.sleep(T_CERRADA)
    print("Prueba terminada")

try:
    if MODO == "medir":
        medir()
    elif MODO == "apretar":
        apretar()
    else:
        ciclos()
finally:
    if pwm is not None:
        pwm.duty_u16(0)   # suelta el servo al terminar o al detener con Stop
