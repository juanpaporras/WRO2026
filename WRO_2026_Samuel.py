from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Button
from pybricks.tools import wait, StopWatch, run_task, multitask
import math

from subdyk import get_robot
from imports_and_inits import (
    avance_gyro, avance_hasta_linea, retrocede_gyro, 
    left_motor, right_motor, giro, giro_pivot, 
    retrocede_hasta_linea_gyro, sensor_mbc, obtener_error_lpf2,
    giro_arco, diagnostico_espectral_mbc
)

robot = get_robot()
hub = robot.hub
stopwatch = robot.stopwatch

motorgarras = Motor(Port.C)
motorElevador = Motor(Port.E)
motorcuadro = Motor(Port.A)
heading_temporal = 0

def configurar_actuador(motor):
    
    motor.control.limits(speed=1000, acceleration=2000, torque=None)
    
    motor.control.pid(kp=500, ki=10, kd=100)

def obtener_heading_normalizado():
    raw_heading = float(hub.imu.heading())
    normalizado = float(((raw_heading + 180) % 360) - 180)
    return normalizado

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

async def advertencia_bateria():
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
        await boton()

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
                    print(f" -> Para avance_gyro (Grados originales) : {grados_motor_promedio:.1f}°")
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
                    print(" Tramo | Distancia (mm) | Motor (avance_gyro) | IMU (Rumbo)")
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

async def getBlackLineError():
    return await line.reflection() // 10 - 8

async def getBlackLineWidth():
    await sensor_mbc.actualizar_lectura()
    return sensor_mbc.ancho_linea

# ---------------------------------------------------------
# FUNCIONES DE SEGUIMIENTO ÓPTICO (Mantienen lazo en Python)

async def seguir_linea_distancia(distancia_mm, vel_crucero_mm_s=450, accel_dist_mm=50, decel_dist_mm=100, vel_minima_mm_s=80, kp=3, kd=2.1, ki=0.0, freno=True):
    start_dist = robot.drive_base.distance()
    cronometro = StopWatch()
    
    last_error = 0.0
    integral = 0.0
    derivada_filtrada = 0.0
    alfa_filtro = 0.35 
    LIMITE_INTEGRAL = 30.0
    MAX_TURN_RATE = 60.0 
    dt_fijo = 0.010
    
    while True:
        dist_actual = robot.drive_base.distance() - start_dist
        if dist_actual >= distancia_mm:
            break 
            
        dist_faltante = distancia_mm - dist_actual
        if dist_actual < accel_dist_mm:
            escala = dist_actual / accel_dist_mm
            vel_lineal = vel_minima_mm_s + (vel_crucero_mm_s - vel_minima_mm_s) * escala
        elif dist_faltante < decel_dist_mm and not freno:
            escala = dist_faltante / decel_dist_mm
            vel_lineal = vel_minima_mm_s + (vel_crucero_mm_s - vel_minima_mm_s) * escala
        else:
            vel_lineal = vel_crucero_mm_s
            
        await sensor_mbc.actualizar_lectura()
        error = sensor_mbc.error_espacial
        
        P = error
        integral += error * dt_fijo
        integral = max(min(integral, LIMITE_INTEGRAL), -LIMITE_INTEGRAL)
        I = integral
        
        derivada_cruda = (error - last_error) / dt_fijo
        derivada_filtrada = (derivada_filtrada * (1.0 - alfa_filtro)) + (derivada_cruda * alfa_filtro)
        D = derivada_filtrada
        
        if abs(error) <= 0.2:
            error = 0.0
            
        turn_rate = -((kp * P) + (ki * I) + (kd * D))
        turn_rate = max(min(turn_rate, MAX_TURN_RATE), -MAX_TURN_RATE)
        
        robot.drive_base.drive(vel_lineal, turn_rate)
        last_error = error
        await wait(10)

    if freno:
        robot.drive_base.stop()
        await wait(50) 

async def seguir_linea_hasta_interseccion(lado="centro", vel_crucero_mm_s=360, kp=4, kd=2.8, ki=0.0, umbral_interseccion=8, centrado_estricto=False, target_heading=None, montarse=True):
    last_error = 0.0
    integral = 0.0
    derivada_filtrada = 0.0
    alfa_filtro = 0.35
    LIMITE_INTEGRAL = 30.0
    MAX_TURN_RATE = 60.0
    dt_fijo = 0.010 
    
    if lado == "izquierda":
        setpoint = -4.0
        vel_crucero_mm_s = 300
    elif lado == "derecha": 
        setpoint = 4.0 
        vel_crucero_mm_s = 300
    else: 
        setpoint = 0.0  
        
    lecturas_interseccion = 0
    
    while True:
        await sensor_mbc.actualizar_lectura()
        ancho_linea = int(sensor_mbc.ancho_linea)
        if ancho_linea >= umbral_interseccion:
            break 
    
            

        error = sensor_mbc.error_espacial - setpoint
         
            
        P = error

        derivada_cruda = (error - last_error) / dt_fijo
        derivada_filtrada = (derivada_filtrada * (1.0 - alfa_filtro)) + (derivada_cruda * alfa_filtro)
        D = derivada_filtrada
        
        turn_rate = -((kp * P) + (kd * D))
        turn_rate = max(min(turn_rate, MAX_TURN_RATE), -MAX_TURN_RATE)

        robot.drive_base.drive(vel_crucero_mm_s, turn_rate)
        
        last_error = error
        await wait(10)
        
    if montarse:
        await montarse_linea(target_heading)
    else:
        robot.drive_base.stop()
    await wait(150) 
    
    if centrado_estricto:
        heading_lock = target_heading if target_heading is not None else hub.imu.heading()
        
        while True:
            ancho_actual = await getBlackLineWidth()
            if ancho_actual >= umbral_interseccion:
                break 
                
            error_yaw = heading_lock - hub.imu.heading()
            error_yaw = ((error_yaw + 180) % 360) - 180
            
            robot.drive_base.drive(-60, error_yaw * 2.5) 
            await wait(10)
            
        robot.drive_base.stop()
        await wait(100) 
        
        if target_heading is not None:
            while True:
                error_angular = target_heading - hub.imu.heading()
                error_angular = ((error_angular + 180) % 360) - 180
                
                if abs(error_angular) <= 1:
                    break
                
                turn_rate_fino = error_angular * 3.5
                robot.drive_base.drive(0, turn_rate_fino)
                await wait(10)
            
            robot.drive_base.stop
# ---------------------------------------------------------
# FUNCIONES CINEMÁTICAS DE ACTUADORES
# ---------------------------------------------------------
async def motor_dc_por_grados(motor, grados, potencia):
    # STREAMING_CHUNK: Restaurando inyección directa por grados
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
    # STREAMING_CHUNK: Restaurando inyección directa por tiempo
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
    await motor_dc_por_grados(motorcuadro, 400, 80)

async def subir_cuadro():
    await motor_dc_por_segundos(motorcuadro, 1500, -80)

async def cerrar_cuadro():
    await motor_dc_por_segundos(motorcuadro, 800, 90)
    motorcuadro.hold

async def abrir_garra():
    await motor_dc_por_grados(motorgarras, 160, 100)

async def abrir_garra_todo():
    await motor_dc_por_segundos(motorgarras, 1000, 100)

async def cerrar_garra():
    motorgarras.dc(-100)

async def cerrar_garra_poquito():
    await motor_dc_por_grados(motorgarras, -50, 50)

async def dejar_bloques():
    await wait(300)
    await motor_dc_por_grados(motorElevador, -50, 70)
    await motor_dc_por_grados(motorgarras, 160, 100)
    await cerrar_garra_poquito()
    await subir_elevador_todo()
    await motor_dc_por_grados(motorgarras, -105, 50)
    await motor_dc_por_segundos(motorElevador, 500, -60)
    await subir_elevador_todo()
    # await motor_dc_por_segundos(motorgarras, 500, -80)
    # await motor_dc_por_segundos(motorElevador, 500, -90)
    # await subir_elevador_todo()

async def dejar_bloques_2(heading):
    await retrocede_gyro(heading, 50, max_speed=30)
    await motor_dc_por_grados(motorElevador, -120, 70)
    await wait(200)
    await motor_dc_por_grados(motorgarras, -20, 30)
    await avance_gyro(heading, 200, max_speed=40)
    await motor_dc_por_grados(motorgarras, -100, 45)
    await motor_dc_por_segundos(motorgarras, 500, -45)
    await cerrar_garra_poquito()
    await wait(100)
    await subir_elevador_todo()

async def montarse_linea(heading):
    await avance_gyro(heading, 55, max_speed=50,freno=True)
    await giro(heading)
# ---------------------------------------------------------
# RUTAS DE COMPETICIÓN (Máquina de Estados)
# ---------------------------------------------------------
async def Ruta_1_recoger_bloques():
    hub.imu.reset_heading(0)
    await multitask(avance_gyro(0, 365, max_speed=70, freno=True), bajar_elevador())
    await cerrar_garra()
    await wait(800)
    await subir_elevador()
    await retrocede_gyro(0, 160, max_speed=60)
        
async def Ruta_2_recoger_ladrillos_blancos():
    await giro(90)
    await avance_gyro(90, 250,freno=False)
    await seguir_linea_distancia(1300)
    #await seguir_linea_hasta_interseccion("derecha",montarse=False)
    await wait(200)
    await giro_arco(173,130)
    await giro(-90)

    await multitask(retrocede_gyro(-90, 1000, max_speed=50, stall_stop=True),bajar_cuadro())
    await avance_gyro(-90, 40,freno=True)
    await cerrar_cuadro()

async def Ruta_3_llevar_ladrillos_blancos(angulo_giro):
    await giro(-80)
    await seguir_linea_hasta_interseccion("derecha",montarse=False)
    await giro(2)
    await seguir_linea_hasta_interseccion("derecha",centrado_estricto=True,target_heading=0)
    await giro(angulo_giro)
    await wait(100)
    await retrocede_gyro(angulo_giro, 1265,freno=True)
    await giro_pivot(92, pivote_izquierdo=False)
    await subir_cuadro()
    
async def Ruta_4_recoger_verdes():
    await giro_pivot(136, pivote_izquierdo=False)
    await avance_gyro(136, 450,freno=True)
    await giro(90)
    await seguir_linea_hasta_interseccion(montarse=False)
    hub.imu.reset_heading(90)
    await giro(-90)
    await multitask (retrocede_gyro(-90, 515, stall_stop=True,freno=True), bajar_cuadro())
    await cerrar_cuadro()

async def Ruta_5_llevar_bloques():
    await seguir_linea_distancia(450,vel_crucero_mm_s=310)
    await seguir_linea_hasta_interseccion(montarse=False,vel_crucero_mm_s=310)
    heading_temporal = hub.imu.heading()
    await avance_gyro(heading_temporal, 400, max_speed=40, stall_stop=True)
    robot.drive_base.stop()
    await wait(50)
    hub.imu.reset_heading(-90)

async def Ruta_6_llevar_ladrillos_verdes():
    await retrocede_gyro(-90, 600)
    await giro(90)
    await retrocede_gyro(90, 265)
    await multitask (subir_cuadro(),retrocede_gyro(90, 45,freno=True))

async def Ruta_7_recoger_ladrillos_amarillos():
    await seguir_linea_hasta_interseccion(montarse=True,target_heading=90)
    await giro(7)
    await seguir_linea_hasta_interseccion("derecha",montarse=True,target_heading=0)
    await giro(-93)
    await multitask(bajar_cuadro(),retrocede_gyro(-93, 470,freno=True))
    await cerrar_cuadro()

async def Ruta_8_llevar_ladrillos_amarillos():
    await avance_hasta_linea(-120)
    await giro(180)
    await seguir_linea_distancia(200)
    await seguir_linea_hasta_interseccion("derecha",target_heading=180)
    await wait (200)
    await giro(-90)
    await seguir_linea_distancia(400,330)
    await seguir_linea_hasta_interseccion(target_heading=-90)
    await montarse_linea(-90)
    await giro(180)
    await multitask(subir_cuadro(),retrocede_gyro(180, 240,freno=True) )  

async def Ruta_9_Recoger_y_llevar_YACUZY():
    await seguir_linea_hasta_interseccion(montarse=False,vel_crucero_mm_s=210)
    hub.imu.reset_heading(180)
    await giro(0)
    await bajar_cuadro()
    await retrocede_gyro(0, 140,freno=True)
    await cerrar_cuadro()

    
    # await giro_pivot(100,True,"pesado")
    # await retrocede_gyro(87, 386,freno=False)
    # heading_temporal = hub.imu.heading()
    # await avance_gyro(heading_temporal, 250,freno=False)
    # await avance_gyro(80, 200,freno=False)
    # await giro_pivot(0,True,"pesado")

    await avance_gyro(0, 40,freno=False)
    await giro(90)
    await seguir_linea_distancia(480,vel_crucero_mm_s=310)
    await giro_pivot(-5, pivote_izquierdo=True,)
    await giro_pivot(89, pivote_izquierdo=False,)
    await avance_gyro(89, 30,freno=True)
    await subir_cuadro()

async def Ruta_10_recoger_ladrillos_azules():
    await avance_gyro(89, 110)
    await giro(-91)
    await multitask(retrocede_gyro(-91, 1000, max_speed=40, stall_stop=True),bajar_cuadro())
    robot.drive_base.stop()
    await wait(50)
    hub.imu.reset_heading(-90)
    await wait(200)
    await avance_gyro(-90, 40,freno=True)
    await cerrar_cuadro()

async def Ruta_11_Recoger_pala():
    await seguir_linea_hasta_interseccion(target_heading=-90,vel_crucero_mm_s=300)
    await giro(170)
    await multitask( seguir_linea_hasta_interseccion("izquierda",vel_crucero_mm_s=210, montarse=False,centrado_estricto=True,target_heading=180),bajar_elevador())
    await giro_pivot(-95,False)
    await multitask( avance_gyro(-95, 300,freno=True),abrir_garra(), motor_dc_por_grados(motorElevador, 120, 100) )
    await cerrar_garra()
    await wait(600)
    await avance_gyro(-100,210,freno=True)
    await avance_gyro(-90, 1190)
    await retrocede_gyro(-90, 150)
    await avance_gyro(-60, 200)
    await avance_hasta_linea(-60,giro=False)
    await montarse_linea(-60)
    #await avance_gyro(-60, 440)
    await giro(-90)

async def Ruta_12_llevar_pala():
    await seguir_linea_hasta_interseccion(montarse=False)
    await giro(-120)
    await multitask (abrir_garra(),subir_elevador_todo())
    await giro(-90)

async def Ruta_13_recoger_bloques_2():
    await multitask (retrocede_gyro(-90, 260), motor_dc_por_grados(motorgarras, 100, 100))
    await giro_pivot(0,False)
    await avance_hasta_linea(0)
    await avance_gyro(0, 20,40)
    await giro(-90)
    await seguir_linea_hasta_interseccion("izquierda",montarse=False,vel_crucero_mm_s=210)
    await avance_gyro(-90, 268,freno=True,aceleracion=550)
    await giro(0,"medio")
    await bajar_elevador()
    heading_temporal=hub.imu.heading()
    await avance_gyro(heading_temporal, 120, max_speed=70,freno=True)
    await cerrar_garra()
    await wait(800)
    await subir_elevador_todo()

async def Ruta_14_llevar_bloques_2():
    await avance_gyro(50, 350)
    await avance_gyro(80, 350,freno=True)
    await seguir_linea_hasta_interseccion(montarse=False, vel_crucero_mm_s=230)
    heading_temporal=hub.imu.heading()
    await avance_gyro(heading_temporal, 400, max_speed=40, stall_stop=True)
    robot.drive_base.stop()
    await wait(50)
    hub.imu.reset_heading(90)

async def Ruta_15_llevar_azules():
    await retrocede_gyro(90, 600)
    await giro(-90)
    await retrocede_gyro(-90, 265)
    await multitask (subir_cuadro(),retrocede_gyro(-90, 40,freno=True))

# Inicio del Run
async def Run():
    # await giro_pivot(90)
    # await giro_pivot(0)
    # await giro_pivot(180)
    await subir_cuadro()
    await subir_elevador_todo()
    await advertencia_bateria()
    await boton()
    hub.imu.reset_heading(-90)
    await wait(200)
    await Ruta_1_recoger_bloques()
    await Ruta_2_recoger_ladrillos_blancos()
    await Ruta_3_llevar_ladrillos_blancos(137)
    await Ruta_4_recoger_verdes()
    await Ruta_5_llevar_bloques()
    await dejar_bloques()
    await Ruta_6_llevar_ladrillos_verdes()
    await Ruta_7_recoger_ladrillos_amarillos()
    await Ruta_8_llevar_ladrillos_amarillos()
    await Ruta_9_Recoger_y_llevar_YACUZY()
    await Ruta_10_recoger_ladrillos_azules()
    await Ruta_11_Recoger_pala()
    await Ruta_12_llevar_pala()
    await Ruta_13_recoger_bloques_2()
    await Ruta_14_llevar_bloques_2()
    await dejar_bloques()
    await Ruta_15_llevar_azules()
    await medir_trayectos_manual()

run_task(Run())