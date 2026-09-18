from pybricks.parameters import Button
from pybricks.tools import wait, StopWatch
import math
from GyroMovement import hub, left_motor, right_motor

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

    if float(voltaje) <= 7.9 or voltaje > 8.3:
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