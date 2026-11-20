from machine import SoftI2C, Pin, PWM
from pyhuskylens import HuskyLens
from pupremote import PUPRemoteSensor, SPIKE_ULTRASONIC
import asyncio
import sys

# =====================================
# CONFIG
# =====================================
FILAS = 3
COLS = 4
N = FILAS * COLS          # 12 casillas
AREA_MIN = 300            # ignora manchas pequeñas (ajusta según tu distancia)
PINES_PROHIBIDOS = (21, 22)   # los usa la HuskyLens (I2C)

# =====================================
# SERVO
# =====================================
# Velocidad de cada servo en grados por segundo. Pin que no esté aquí = salto directo.
VELOCIDAD = {20: 450}   # Calibrar: más bajo = más lento y suave (el servo da máx ~430 °/s a 4.8 V)
PASO_MS = 10            # cada cuánto avanza la rampa

class ServomotorPosicional:
    def __init__(self, pin_num, freq=50):
        self.pwm = PWM(Pin(pin_num), freq=freq)
        self.pulso_min_us = 500
        self.pulso_max_us = 2500
        self.rango_angular = 270.0
        self.vel = VELOCIDAD.get(pin_num, 0)
        self.actual = None      # None = posición desconocida (recién creado o eje suelto)
        self.objetivo = None
        self.pwm.duty_u16(0)

    def _aplicar(self, g):
        pulso_us = self.pulso_min_us + (g / self.rango_angular) * (self.pulso_max_us - self.pulso_min_us)
        self.pwm.duty_u16(int((pulso_us / 20000.0) * 65535))

    def mover_a_posicion(self, grados):
        g = max(0, min(self.rango_angular, grados))
        self.objetivo = g
        # Sin posición conocida o sin rampa: salta directo. Si no, paso() lo lleva poco a poco.
        if self.actual is None or self.vel <= 0:
            self.actual = g
            self._aplicar(g)

    def paso(self):
        """Avanza la rampa un poco hacia el objetivo (lo llama mover_servos)."""
        if self.actual is None or self.objetivo is None or self.actual == self.objetivo:
            return
        d = self.vel * PASO_MS / 1000
        if abs(self.objetivo - self.actual) <= d:
            self.actual = self.objetivo
        elif self.objetivo > self.actual:
            self.actual += d
        else:
            self.actual -= d
        self._aplicar(self.actual)

    def liberar_eje(self):
        self.pwm.duty_u16(0)
        self.actual = None
        self.objetivo = None

servos = {}   # pin -> servo

async def mover_servos():
    while True:
        for m in servos.values():
            m.paso()
        await asyncio.sleep_ms(PASO_MS)

# =====================================
# HUSKYLENS
# =====================================
i2c = SoftI2C(scl=Pin(21), sda=Pin(22), freq=100000)
DIR_HUSKY = 0x32          # dirección I2C de la HuskyLens (50)
husky_ok = 0              # diagnóstico: 0 = la HuskyLens todavía no aparece en el I2C
husky_error = 0           # diagnóstico: 1 = la última lectura de la HuskyLens dio error
hl = None                 # se crea en conectar_husky() cuando la HuskyLens ya arrancó

def conectar_husky():
    """Crea la HuskyLens SOLO si ya responde en el I2C. Con el ESP32 alimentado por el hub,
    el ESP32 arranca antes que la HuskyLens (tarda unos segundos en encender): si se crea
    antes, la HuskyLens nunca contesta. Devuelve True si quedó conectada."""
    global hl, husky_ok
    encontrados = i2c.scan()
    if DIR_HUSKY not in encontrados:
        husky_ok = 0
        return False
    try:
        hl = HuskyLens(i2c)
        hl.mode_color_recognition()   # quita esta línea si ya lo dejas fijo en la HuskyLens
        husky_ok = 1
        print("HuskyLens conectada, I2C:", encontrados)
        return True
    except Exception as e:
        sys.print_exception(e)
        hl = None
        husky_ok = 0
        return False

# Última cuadrícula completa leída (0 = desconocido)
grid = [0] * N
grid_ok = 0   # 1 = la última lectura de la cámara fue completa

def attr(b, *names):
    for n in names:
        if hasattr(b, n):
            return int(getattr(b, n))
    return 0

bloques_vistos = 0   # diagnóstico: bloques que pasaron el filtro AREA_MIN en la última lectura
bloques_total = 0    # diagnóstico: bloques que mandó la HuskyLens (sin filtrar)

def leer_grid(blocks):
    global bloques_vistos, bloques_total
    bloques_total = len(blocks)
    items = []
    for b in blocks:
        x = attr(b, "x")
        y = attr(b, "y")
        w = attr(b, "width", "w")
        h = attr(b, "height", "h")
        i = attr(b, "ID", "id")
        if w * h >= AREA_MIN and i > 0:
            items.append((x, y, w * h, i))

    bloques_vistos = len(items)
    if len(items) < N:
        return None

    items.sort(key=lambda t: t[2], reverse=True)
    items = items[:N]

    items.sort(key=lambda t: t[1])
    resultado = []
    for r in range(FILAS):
        fila = items[r * COLS:(r + 1) * COLS]
        fila.sort(key=lambda t: t[0])
        for t in fila:
            resultado.append(min(t[3], 255))
    return resultado

async def leer_husky():
    global grid, grid_ok, husky_error, hl
    errores_seguidos = 0
    while True:
        if hl is None:
            # Todavía no arranca la HuskyLens (o se perdió): reintenta cada 0.5 s
            if not conectar_husky():
                await asyncio.sleep_ms(500)
                continue
        try:
            data = hl.get()
            errores_seguidos = 0
            nuevo = leer_grid(data["blocks"])
            husky_error = 0
            if nuevo is not None:
                grid = nuevo
                grid_ok = 1
            else:
                grid_ok = 0      # grid conserva la última lectura buena
        except Exception as e:
            husky_error = 1
            sys.print_exception(e)
            errores_seguidos += 1
            if errores_seguidos >= 10:   # ~1 s fallando: se vuelve a conectar desde cero
                hl = None
                errores_seguidos = 0
        await asyncio.sleep_ms(100)

# =====================================
# COMANDO ÚNICO "robot"
# Recibe (pin, grados). pin <= 0 = no mover ningún servo.
# Responde (16 bytes, máximo de PUPRemote): (12 casillas, estado, pin_eco, grados_eco)
#   pin_eco, grados_eco = ÚLTIMA orden de servo aplicada (el hub la compara
#                         con lo que mandó y reenvía si no coincide)
#   estado bit0 = 1 -> lectura de cámara completa
#   estado bit1 = 1 -> el servo falló
#   estado bits2-5  -> bloques que pasaron el filtro AREA_MIN (0-15)   (diagnóstico)
#   estado bit6 = 1 -> la HuskyLens dio error en la última lectura    (diagnóstico)
#   estado bit7 = 1 -> no se encontró nada en el I2C al arrancar       (diagnóstico)
# =====================================
ultimo_pin = 0
ultimo_grados = 0

async def robot(pin, grados):
    global ultimo_pin, ultimo_grados
    servo_error = 0
    if pin > 0:
        try:
            if pin in PINES_PROHIBIDOS:
                raise ValueError("pin no permitido")
            if pin not in servos:
                servos[pin] = ServomotorPosicional(pin)
            if grados < 0:
                servos[pin].liberar_eje()
            else:
                servos[pin].mover_a_posicion(grados)
            ultimo_pin = pin
            ultimo_grados = grados
        except Exception as e:
            sys.print_exception(e)
            servo_error = 1

    estado = (grid_ok + 2 * servo_error + 4 * min(bloques_vistos, 15)
              + 64 * husky_error + 128 * (1 - husky_ok))
    return tuple(grid) + (estado, ultimo_pin, ultimo_grados)

# =====================================
# PUPREMOTE
# =====================================
pr = PUPRemoteSensor(sensor_id=SPIKE_ULTRASONIC, power=True)
pr.add_command("robot", to_hub_fmt="<14Bh", from_hub_fmt="hh")   # debe ser IGUAL en Funciones_ESP32.py

# =====================================
# MAIN
# =====================================
async def main():
    asyncio.create_task(pr.process_async())
    asyncio.create_task(leer_husky())
    asyncio.create_task(mover_servos())
    while True:
        print("Conectado:", pr.connected())
        print("Fila 1:", grid[0:4])
        print("Fila 2:", grid[4:8])
        print("Fila 3:", grid[8:12])
        print("Camara completa:", grid_ok)
        # Si "filtrados" < 12 la lectura nunca sale completa: bloques pequeños (bajar AREA_MIN)
        # o dos bloques iguales pegados que la HuskyLens junta en uno solo
        print("Bloques HuskyLens:", bloques_total, "| filtrados (area >=", AREA_MIN, "):", bloques_vistos)
        print("-----")
        await asyncio.sleep(1)

try:
    asyncio.run(main())
finally:
    for m in servos.values():
        m.liberar_eje()
    asyncio.new_event_loop()