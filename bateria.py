from pybricks.hubs import PrimeHub
from pybricks.tools import wait, StopWatch
hub = PrimeHub()

# 1. Inicialización del microcontrolador

# 2. Extracción de datos telemétricos
voltaje_v = hub.battery.voltage() / 1000.0  # Conversión de milivoltios (mV) a voltios (V)
corriente_ma = hub.battery.current()        # Lectura directa en miliamperios (mA)

# 3. Visualización en la terminal de salida
print(f"Voltaje actual: {voltaje_v:.2f} V")
print(f"Consumo actual: {corriente_ma} mA")