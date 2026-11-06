from pybricks.parameters import Port
from pybricks.tools import wait
from pupremote import PUPRemoteHub

ROJO = 5
NOMBRES = {0: "?", 1: "Amarillo", 2: "Azul", 3: "Verde", 4: "Blanco", 5: "Rojo"}

pr = None   # se crea en iniciar_esp32()

# =====================================
# INICIALIZACIÓN
# =====================================
def iniciar_esp32():
    """Llamar UNA vez ANTES de run_task(...) (no dentro de Run)."""
    global pr
    pr = PUPRemoteHub(Port.C)
    pr.add_command("robot", to_hub_fmt="13B", from_hub_fmt="hh")   # mismo formato que el ESP32

async def proceso_esp32():
    """Tarea de fondo que procesa las llamadas al ESP32.
    Debe correr en paralelo con Run():
        run_task(multitask(Run(), proceso_esp32(), race=True))"""
    await pr.process_async()

# =====================================
# COMUNICACIÓN
# =====================================
async def _llamar(pin, grados):
    d = await pr.call_multitask("robot", pin, grados, wait_ms=50)
    return list(d[:12]), d[12]

async def mover_servo(pin, grados):
    """pin = GPIO del ESP32, grados 0-270, -1 suelta el eje.
    Devuelve True si el servo se ejecutó bien."""
    _, estado = await _llamar(pin, grados)
    return (estado & 2) == 0

async def servo(pin, grados, espera_ms=300):
    """Mueve el servo y espera a que llegue (calibrar espera_ms)."""
    await mover_servo(pin, grados)
    await wait(espera_ms)

# =====================================
# CÁMARA
# =====================================
def color_mas_repetido(lista):
    """Devuelve (color, veces, empate).
    Si hay empate, 'color' es el primero que aparece en la lista
    (arriba, luego abajo, luego centro) y empate = True."""
    cuentas = {}
    for c in lista:
        cuentas[c] = cuentas.get(c, 0) + 1
    maximo = max(cuentas.values())
    empatados = [c for c in cuentas if cuentas[c] == maximo]
    return empatados[0], maximo, len(empatados) > 1

async def leer_camara(timeout_ms=3000):
    """Espera una lectura completa y devuelve:
    (izquierda, derecha, lado_rojo, color_mas, veces, empate)
    color_mas = color más repetido en el lado SIN rojo.
    Si no hay lectura en timeout_ms, devuelve listas vacías."""
    t = 0
    while True:
        g, estado = await _llamar(0, 0)      # pin 0 = no mover servo
        if estado & 1:
            break
        if t >= timeout_ms:
            return [], [], -1, 0, 0, False   # sin lectura
        await wait(100)
        t += 100

    fila1, fila2, fila3 = g[0:4], g[4:8], g[8:12]
    izquierda = fila1[0:2] + fila3[0:2] + fila2[0:2]
    derecha   = fila1[2:4] + fila3[2:4] + fila2[2:4]

    if ROJO in izquierda:
        lado_rojo = 0                  # 0 = izquierda
        lado_sin_rojo = derecha
    elif ROJO in derecha:
        lado_rojo = 1                  # 1 = derecha
        lado_sin_rojo = izquierda
    else:
        return izquierda, derecha, -1, 0, 0, False   # no se vio el rojo

    color_mas, veces, empate = color_mas_repetido(lado_sin_rojo)
    return izquierda, derecha, lado_rojo, color_mas, veces, empate

# Ejemplo de servo (dentro de una función async):
# await servo(19, 30)
# await mover_servo(19, -1)
