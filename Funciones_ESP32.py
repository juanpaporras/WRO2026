from pybricks.parameters import Port
from pybricks.tools import wait
from pupremote import PUPRemoteHub

ROJO = 5

wait(5000)   # deja que el ESP32 arranque
pr = PUPRemoteHub(Port.C)
pr.add_command("robot", to_hub_fmt="13B", from_hub_fmt="hh")   # mismo formato que el ESP32

def _llamar(pin, grados):
    d = pr.call("robot", pin, grados, wait_ms=50)
    return list(d[:12]), d[12]

def mover_servo(pin, grados):
    """pin = GPIO del ESP32, grados 0-270, -1 suelta el eje.
    Devuelve True si el servo se ejecutó bien."""
    _, estado = _llamar(pin, grados)
    return (estado & 2) == 0

def leer_camara():
    """Espera una lectura completa y devuelve (izquierda, derecha, lado_rojo)."""
    while True:
        g, estado = _llamar(0, 0)      # pin 0 = no mover servo
        if estado & 1:
            break
        wait(100)

    fila1, fila2, fila3 = g[0:4], g[4:8], g[8:12]
    izquierda = fila1[0:2] + fila3[0:2] + fila2[0:2]
    derecha   = fila1[2:4] + fila3[2:4] + fila2[2:4]

    if ROJO in izquierda:
        lado_rojo = 0      # 0 = izquierda
    elif ROJO in derecha:
        lado_rojo = 1      # 1 = derecha
    else:
        lado_rojo = -1     # no se vio el rojo
    return izquierda, derecha, lado_rojo

# =====================================
# PROGRAMA PRINCIPAL
# =====================================
izquierda, derecha, lado_rojo = leer_camara()
print("Izquierda:", izquierda)
print("Derecha:", derecha)
print("Lado rojo:", lado_rojo)
