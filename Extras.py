from pybricks.parameters import Button
from pybricks.tools import wait, StopWatch
import math
from Robot_specs import get_robot
# Se toma el hub de Robot_specs (misma instancia) y NO de GyroMovement,
# porque GyroMovement importa Extras (import circular)
_robot = get_robot()
hub = _robot.hub
left_motor = _robot.left_motor
right_motor = _robot.right_motor

async def sonido_advertencia():
    hub.speaker.volume(100)
    contador = 0 
    while contador < 3:
        await hub.speaker.beep(frequency=200, duration=1000)
        await wait(100)
        await hub.speaker.beep(frequency=200, duration=1000)
        await wait(100)
        await hub.speaker.beep(frequency=200, duration=2500)
        contador += 1

async def advertencia_bateria(boton_func):
    await hub.speaker.beep(1000, 50)
    voltaje = (hub.battery.voltage() / 1000.0)
    print(voltaje)

    if float(voltaje) <= 8 or voltaje > 8.35:
        await sonido_advertencia()  
        while voltaje > 8.3:
            left_motor.dc(50)
            right_motor.dc(50)  
            voltaje = (hub.battery.voltage() / 1000.0)
            print(hub.battery.voltage() / 1000.0)
            await wait(1000)
        await boton_func()

async def boton():
    hub.system.set_stop_button(None)
    try:
        while Button.CENTER not in hub.buttons.pressed():
            await hub.speaker.beep(1000, 50)
            await wait(500)
            
        while Button.CENTER in hub.buttons.pressed():
            await wait(10)
    finally:
        hub.system.set_stop_button((Button.CENTER,))
        await hub.speaker.play_notes(['C4/4', 'E4/4', 'G4/4'], tempo=120)
        await wait(500)

async def medir_heading_correction(correccion_actual=360):
    """Mide el valor de heading_correction (grados que marca el hub por cada vuelta REAL).
    Uso:
      1. Alinear el robot contra una referencia (pared, escuadra o línea) y NO moverlo hasta el beep agudo.
      2. Girarlo a mano, PLANO sobre la mesa, varias vueltas completas (ej. 10) en un sentido.
      3. Volver a alinearlo exactamente igual y dar 1 toque al botón -> guarda la medición.
      4. Repetir en el otro sentido (conviene medir horario y antihorario).
      5. Doble toque -> imprime el promedio para poner en HEADING_CORRECTION (GyroMovement.py).
    correccion_actual: el HEADING_CORRECTION que está puesto ahora en GyroMovement.py
    (la medición se pasa a valor "crudo" para que el resultado no dependa de él)."""
    mediciones = []
    cronometro_click = StopWatch()
    hub.system.set_stop_button(None)
    left_motor.stop()
    right_motor.stop()

    print("--------------------------------------------------")
    print(" MEDICIÓN DE heading_correction")
    print(" Alinear el robot y NO moverlo hasta el beep agudo")
    print("--------------------------------------------------")
    espera = 0
    while not hub.imu.ready() and espera < 5000:
        await wait(10)
        espera += 10
    hub.imu.reset_heading(0)
    await hub.speaker.beep(1500, 100)
    print(" -> Girar N vueltas, realinear y 1 TOQUE : guardar medición")
    print(" -> 2 TOQUES (doble clic)              : terminar y ver resultado")

    try:
        while True:
            if Button.CENTER in hub.buttons.pressed():
                while Button.CENTER in hub.buttons.pressed():
                    await wait(10)
                cronometro_click.reset()
                es_doble_toque = False
                while cronometro_click.time() < 400:
                    if Button.CENTER in hub.buttons.pressed():
                        es_doble_toque = True
                        while Button.CENTER in hub.buttons.pressed():
                            await wait(10)
                        break
                    await wait(10)

                if not es_doble_toque:
                    # Valor crudo: lo que marcaría el hub sin corrección (360 por vuelta)
                    crudo = abs(hub.imu.heading()) * correccion_actual / 360
                    vueltas = round(crudo / 360)
                    if vueltas < 1:
                        print("Medición ignorada: menos de 1 vuelta (", crudo, "°)")
                        await hub.speaker.beep(300, 300)
                    else:
                        por_vuelta = crudo / vueltas
                        mediciones.append(por_vuelta)
                        await hub.speaker.beep(1000, 50)
                        print(f"Medición {len(mediciones)}: {crudo:.1f}° en {vueltas} vueltas -> {por_vuelta:.2f}° por vuelta")
                    hub.imu.reset_heading(0)
                else:
                    await hub.speaker.play_notes(['C4/4', 'E4/4', 'G4/4'], tempo=120)
                    print("\n================ RESULTADO heading_correction ================")
                    if not mediciones:
                        print(" No se registraron mediciones.")
                    else:
                        promedio = sum(mediciones) / len(mediciones)
                        print(" Mediciones:", ["{:.2f}".format(m) for m in mediciones])
                        print(f" Poner en GyroMovement.py -> HEADING_CORRECTION = {promedio:.2f}")
                    print("==============================================================")
                    break
            await wait(20)
    finally:
        hub.system.set_stop_button((Button.CENTER,))

async def medir_trayectos_manual():
    diametro_rueda = 62.5
    trayectos_guardados = []
    contador_trayecto = 1
    cronometro_click = StopWatch()
    
    hub.system.set_stop_button(None)
    await hub.speaker.play_notes(['C4/4', 'E4/4', 'G4/4'], tempo=120)
    left_motor.stop()
    right_motor.stop()
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    hub.imu.reset_heading(0)
    
    print("--------------------------------------------------")
    print(" INICIO DE MEDICIÓN (UN BOTÓN) ")
    print(" -> 1 TOQUE RÁPIDO : Guardar trayecto")
    print(" -> 2 TOQUES (Doble clic) : Finalizar y ver resumen")
    print("--------------------------------------------------")
    
    try:
        while True:
            botones = hub.buttons.pressed()
            
            if Button.CENTER in botones:
                while Button.CENTER in hub.buttons.pressed():
                    await wait(10)
                    
                cronometro_click.reset()
                es_doble_toque = False
                
                while cronometro_click.time() < 400:
                    if Button.CENTER in hub.buttons.pressed():
                        es_doble_toque = True
                        while Button.CENTER in hub.buttons.pressed():
                            await wait(10)
                        break
                    await wait(10)
                    
                if not es_doble_toque:
                    grados_izq = left_motor.angle()
                    grados_der = right_motor.angle()
                    grados_motor_promedio = (abs(grados_izq) + abs(grados_der)) / 2
                    distancia_mm = (grados_motor_promedio / 360) * (math.pi * diametro_rueda)
                    grados_chasis = hub.imu.heading()
                    await hub.speaker.beep(1000, 50)
                    print(f"\n--- TRAMO {contador_trayecto} GUARDADO ---")
                    print(f" -> Para seguir_linea_distancia : {distancia_mm:.1f} mm")
                    print(f" -> Para avance (Grados originales) : {grados_motor_promedio:.1f}°")
                    print(f" -> Ángulo IMU (Rumbo)          : {grados_chasis}°")
                    
                    trayectos_guardados.append({
                        "trayecto": contador_trayecto,
                        "mm": distancia_mm,
                        "grados_motor": grados_motor_promedio,
                        "grados_chasis": grados_chasis
                    })
                    contador_trayecto += 1
                    left_motor.reset_angle(0)
                    right_motor.reset_angle(0)
                    hub.imu.reset_heading(0)
                else:
                    await hub.speaker.play_notes(['C4/4', 'E4/4', 'G4/4'], tempo=120)
                    print("\n========================= TABLA RESUMEN =========================")
                    print(" Tramo | Distancia (mm) | Motor (avance) | IMU (Rumbo)")
                    print("-----------------------------------------------------------------")
                    if not trayectos_guardados:
                        print(" No se registraron trayectos.")
                    else:
                        for t in trayectos_guardados:
                            print(f"  {t['trayecto']:02d}   |   {t['mm']:>8.1f} mm   |   {t['grados_motor']:>13.1f}°  | {t['grados_chasis']:>8}°")
                    print("=================================================================")
                    break 
            await wait(20)
    finally:
        hub.system.set_stop_button((Button.CENTER,))