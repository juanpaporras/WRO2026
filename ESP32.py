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
class ServomotorPosicional:
    def __init__(self, pin_num, freq=50):
        self.pwm = PWM(Pin(pin_num), freq=freq)
        self.pulso_min_us = 500
        self.pulso_max_us = 2500
        self.rango_angular = 270.0
        self.pwm.duty_u16(0)

    def mover_a_posicion(self, grados):
        g = max(0, min(self.rango_angular, grados))
        pulso_us = self.pulso_min_us + (g / self.rango_angular) * (self.pulso_max_us - self.pulso_min_us)
        self.pwm.duty_u16(int((pulso_us / 20000.0) * 65535))

    def liberar_eje(self):
        self.pwm.duty_u16(0)

servos = {}   # pin -> servo

# =====================================
# HUSKYLENS
# =====================================
i2c = SoftI2C(scl=Pin(21), sda=Pin(22), freq=100000)
print("I2C:", i2c.scan())
hl = HuskyLens(i2c)
hl.mode_color_recognition()   # quita esta línea si ya lo dejas fijo en la HuskyLens

# Última cuadrícula completa leída (0 = desconocido)
grid = [0] * N
grid_ok = 0   # 1 = la última lectura de la cámara fue completa

def attr(b, *names):
    for n in names:
        if hasattr(b, n):
            return int(getattr(b, n))
    return 0

def leer_grid(blocks):
    items = []
    for b in blocks:
        x = attr(b, "x")
        y = attr(b, "y")
        w = attr(b, "width", "w")
        h = attr(b, "height", "h")
        i = attr(b, "ID", "id")
        if w * h >= AREA_MIN and i > 0:
            items.append((x, y, w * h, i))

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
    global grid, grid_ok
    while True:
        try:
            data = hl.get()
            nuevo = leer_grid(data["blocks"])
            if nuevo is not None:
                grid = nuevo
                grid_ok = 1
            else:
                grid_ok = 0      # grid conserva la última lectura buena
        except Exception as e:
            sys.print_exception(e)
        await asyncio.sleep_ms(100)

# =====================================
# COMANDO ÚNICO "robot"
# Recibe (pin, grados). pin <= 0 = no mover ningún servo.
# Responde siempre 13 bytes: 12 casillas + estado
#   estado bit0 = 1 -> lectura de cámara completa
#   estado bit1 = 1 -> el servo falló
# =====================================
async def robot(pin, grados):
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
        except Exception as e:
            sys.print_exception(e)
            servo_error = 1

    estado = grid_ok + 2 * servo_error
    return tuple(grid) + (estado,)

# =====================================
# PUPREMOTE
# =====================================
pr = PUPRemoteSensor(sensor_id=SPIKE_ULTRASONIC, power=True)
pr.add_command("robot", to_hub_fmt="13B", from_hub_fmt="hh")

# =====================================
# MAIN
# =====================================
async def main():
    asyncio.create_task(pr.process_async())
    asyncio.create_task(leer_husky())
    while True:
        print("Conectado:", pr.connected())
        print("Fila 1:", grid[0:4])
        print("Fila 2:", grid[4:8])
        print("Fila 3:", grid[8:12])
        print("Camara completa:", grid_ok)
        print("-----")
        await asyncio.sleep(1)

try:
    asyncio.run(main())
finally:
    for m in servos.values():
        m.liberar_eje()
    asyncio.new_event_loop()