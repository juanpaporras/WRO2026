from pybricks.hubs import PrimeHub

# 1. Inicialización del microcontrolador
hub = PrimeHub()

# 2. Extracción de datos telemétricos
voltaje_v = hub.battery.voltage() / 1000.0  # Conversión de milivoltios (mV) a voltios (V)
corriente_ma = hub.battery.current()        # Lectura directa en miliamperios (mA)

# 3. Visualización en la terminal de salida
print(f"Voltaje actual: {voltaje_v:.2f} V")
print(f"Consumo actual: {corriente_ma} mA")