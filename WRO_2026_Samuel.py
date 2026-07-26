from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Button
from pybricks.tools import wait, StopWatch, run_task, multitask
import math

# Importamos la instancia del robot y todas las funciones de cinemática optimizadas
from subdyk import get_robot
from imports_and_inits import (
    avance_gyro, avance_hasta_linea, retrocede_gyro, 
    left_motor, right_motor, giro, giro_pivot, 
    retrocede_hasta_linea_gyro, sensor_mbc, obtener_error_lpf2,
    giro_arco
)

# Inicialización de hardware superior (Actuadores del Efector Final)
robot = get_robot()
hub = robot.hub
stopwatch = robot.stopwatch

motorgarras = Motor(Port.C)
motorElevador = Motor(Port.E)
motorcuadro = Motor(Port.A)
heading_temporal = 0

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
# ---------------------------------------------------------
async def seguir_linea_distancia(distancia_mm, target_heading_calibracion=None, potencia_crucero=85, potencia_inicial=35, accel_rate=1.1, potencia_final=40, decel_rate=1.3, kp_base=2.0, kd_base=0.5, ki_base=0, accel_deadband=2):
    diametro_rueda = 62.5  
    grados_objetivo = (distancia_mm * 360) / (math.pi * diametro_rueda)

    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    cronometro = StopWatch()
    
    integral_acumulada = 0.0
    last_error = 0.0  
    error_deadband = 0.9
    LIMITE_INTEGRAL = 30.0 
    derivada_filtrada = 0.0
    ALFA_FILTRO = 0.25 
    
    lecturas_paralelo = 0
    gyro_calibrado = False
    UMBRAL_CENTRO = 0.5    
    UMBRAL_DERIVADA = 1.0  
    CICLOS_CAPTURA = 25    
    CICLOS_CONFIRMACION = 35 
    heading_capturado = 0.0
    
    potencia_actual = abs(potencia_inicial)
    potencia_crucero = min(abs(potencia_crucero), 100)
    potencia_final = min(abs(potencia_final), potencia_crucero)
    
    pasos_desaceleracion = (potencia_crucero - potencia_final) / decel_rate
    grados_desaceleracion_estimados = pasos_desaceleracion * 5  
    grados_inicio_desaceleracion = max(0, grados_objetivo - grados_desaceleracion_estimados)

    cronometro.reset()

    while True:
        grados_recorridos = (abs(left_motor.angle()) + abs(right_motor.angle())) / 2.0
        
        if grados_recorridos >= grados_objetivo:
            break

        dt = cronometro.time() / 1000.0
        cronometro.reset()
        if dt <= 0.0: dt = 0.001 

        error = await obtener_error_lpf2()
        derivada_cruda = (error - last_error) / dt
        derivada_filtrada = (derivada_filtrada * (1.0 - ALFA_FILTRO)) + (derivada_cruda * ALFA_FILTRO)

        if target_heading_calibracion is not None and not gyro_calibrado:
            if abs(error) < UMBRAL_CENTRO and abs(derivada_filtrada) < UMBRAL_DERIVADA:
                lecturas_paralelo += 1
                if lecturas_paralelo == CICLOS_CAPTURA:
                    heading_capturado = hub.imu.heading()
                elif lecturas_paralelo >= CICLOS_CONFIRMACION:
                    delta_movimiento = hub.imu.heading() - heading_capturado
                    nuevo_heading = target_heading_calibracion + delta_movimiento
                    hub.imu.reset_heading(nuevo_heading)
                    gyro_calibrado = True 
                lecturas_paralelo = 0 
        
        if grados_recorridos >= grados_inicio_desaceleracion:
            if potencia_actual > potencia_final:
                potencia_actual -= decel_rate
                potencia_actual = max(potencia_actual, potencia_final)
        else:
            if abs(error) < accel_deadband and potencia_actual < potencia_crucero:
                potencia_actual += accel_rate
                potencia_actual = min(potencia_actual, potencia_crucero)
        
        if abs(error) <= error_deadband:
            error = 0.0
            
        P = error
        integral_acumulada += error * dt
        integral_acumulada = max(min(integral_acumulada, LIMITE_INTEGRAL), -LIMITE_INTEGRAL)
        I = integral_acumulada
        D = derivada_filtrada
        
        kp_dinamico = kp_base
        kd_dinamico = kd_base
        ki_dinamico = ki_base
        
        if potencia_actual > 0.9 * potencia_crucero:
            kp_dinamico = kp_base * 0.5
            kd_dinamico = kd_base * 0.5
            
        turn = (kp_dinamico * P) + (ki_dinamico * I) + (kd_dinamico * D)
        limite_turn = potencia_actual * 0.65 
        turn_clamped = max(min(turn, limite_turn), -limite_turn)

        if abs(error) < error_deadband:
            turn_clamped = 0.0
        
        left_motor.dc(potencia_actual - turn_clamped)
        right_motor.dc(potencia_actual + turn_clamped)
        last_error = error
        await wait(10)

    left_motor.hold()
    right_motor.hold()

async def seguir_linea_hasta_interseccion(lado="centro", potencia_crucero=55, potencia_inicial=30, accel_rate=1.5, kp_base=2.0, kd_base=0.4, ki_base=0.0, umbral_interseccion=8):
    last_error = 0.0
    integral_acumulada = 0.0
    derivada_filtrada = 0.0
    ALFA_FILTRO = 0.25 
    LIMITE_INTEGRAL = 30.0
    error_deadband = 0.9
    
    potencia_actual = abs(potencia_inicial)
    cronometro = StopWatch()
    
    if lado == "izquierda": setpoint = -5.0 
    elif lado == "derecha": setpoint = 5.0  
    else: setpoint = 0.0  
        
    lecturas_interseccion_consecutivas = 0
    lecturas_requeridas = 1 
    cronometro.reset()
    
    while True:
        dt = cronometro.time() / 1000.0
        cronometro.reset()
        if dt <= 0.0: dt = 0.001

        ancho_linea = await getBlackLineWidth()
        if ancho_linea >= umbral_interseccion:
            lecturas_interseccion_consecutivas += 1
            if lecturas_interseccion_consecutivas >= lecturas_requeridas:
                break 
        else:
            lecturas_interseccion_consecutivas = 0 
        
        if potencia_actual < potencia_crucero: potencia_actual += accel_rate
        if potencia_actual > potencia_crucero: potencia_actual = potencia_crucero
            
        posicion_actual = await obtener_error_lpf2() 
        error = posicion_actual - setpoint
        
        if abs(error) <= error_deadband:
            error = 0.0
        else:
            error = error - (error_deadband * (error / abs(error)))    
            
        P = error
        integral_acumulada += error * dt
        integral_acumulada = max(min(integral_acumulada, LIMITE_INTEGRAL), -LIMITE_INTEGRAL)
        I = integral_acumulada

        derivada_cruda = (error - last_error) / dt
        derivada_filtrada = (derivada_filtrada * (1.0 - ALFA_FILTRO)) + (derivada_cruda * ALFA_FILTRO)
        D = derivada_filtrada
        
        turn = (kp_base * P) + (ki_base * I) + (kd_base * D)
        limite_turn = potencia_actual * 0.65
        turn_clamped = max(min(turn, limite_turn), -limite_turn)

        left_motor.dc(potencia_actual - turn_clamped)
        right_motor.dc(potencia_actual + turn_clamped)
        
        last_error = error
        await wait(10)
        
    left_motor.hold()
    right_motor.hold()

# ---------------------------------------------------------
# FUNCIONES CINEMÁTICAS DE ACTUADORES
# ---------------------------------------------------------
async def motor_dc_por_grados(motor, grados, potencia):
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
    cronometro = StopWatch()
    motor.dc(potencia)
    while cronometro.time() < milisegundos:
        await wait(10)
    motor.brake()

async def subir_elevador_todo():
    await motor_dc_por_segundos(motorElevador, 600, 90)

async def subir_elevador():
    await motor_dc_por_grados(motorElevador, 200, 100)
    motorElevador.hold()

async def bajar_elevador():
    await motor_dc_por_segundos(motorElevador, 1000, -60)

async def bajar_cuadro():
    await motor_dc_por_grados(motorcuadro, 410, 80)

async def subir_cuadro():
    await motor_dc_por_segundos(motorcuadro, 1500, -80)

async def cerrar_cuadro():
    motorcuadro.dc(80)

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
    await motor_dc_por_grados(motorgarras, 175, 100)
    await cerrar_garra_poquito()
    await subir_elevador_todo()
    await motor_dc_por_grados(motorgarras, -100, 50)
    await motor_dc_por_segundos(motorElevador, 500, -70)
    await subir_elevador_todo()
    await motor_dc_por_segundos(motorgarras, 500, -80)
    await motor_dc_por_segundos(motorElevador, 500, -70)
    await subir_elevador_todo()

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
    await avance_gyro(heading, 60, max_speed=30)

# ---------------------------------------------------------
# RUTAS DE COMPETICIÓN (Máquina de Estados)
# ---------------------------------------------------------
async def Ruta_1_recoger_bloques():
    hub.imu.reset_heading(0)
    await multitask(avance_gyro(0, 365, max_speed=65), bajar_elevador())
    await cerrar_garra()
    await wait(800)
    await subir_elevador()
    await retrocede_gyro(0, 130, max_speed=60)
        
async def Ruta_2_recoger_ladrillos_blancos():
    await giro(90)
    await multitask(avance_gyro(90, 250), bajar_cuadro())
    await multitask(seguir_linea_distancia(1350), cerrar_cuadro())
    await seguir_linea_hasta_interseccion()
    await multitask(giro_pivot(154, pivote_izquierdo=False, tipo_carga="medio"), subir_cuadro())
    await giro_pivot(90, tipo_carga="medio")
    await giro(-90)
    await bajar_cuadro()
    await retrocede_gyro(-90, 1000, max_speed=50, stall_stop=True)
    await avance_gyro(-90, 40)
    await cerrar_cuadro()

async def Ruta_3_llevar_ladrillos_blancos(angulo_giro):
    await seguir_linea_hasta_interseccion("derecha")
    await montarse_linea(-90)
    await giro(0)
    await seguir_linea_hasta_interseccion("derecha")
    await giro(angulo_giro, "medio")
    await wait(100)
    await retrocede_gyro(angulo_giro, 1262)
    await giro_pivot(90, pivote_izquierdo=False, tipo_carga="pesado")
    await subir_cuadro()
    
async def Ruta_4_recoger_verdes():
    await giro_pivot(131, pivote_izquierdo=False, tipo_carga="pesado")
    await avance_gyro(131, 465)
    await giro(90)
    await seguir_linea_distancia(350)
    await giro(-90)
    await bajar_cuadro()
    await retrocede_gyro(-90, 1000, max_speed=50, stall_stop=True)
    await avance_gyro(-90, 40)
    await cerrar_cuadro()

async def Ruta_5_llevar_bloques():
    await seguir_linea_distancia(450, potencia_crucero=70)
    await seguir_linea_hasta_interseccion()
    heading_temporal = hub.imu.heading()
    await avance_gyro(heading_temporal, 10000, max_speed=55, stall_stop=True)
    hub.imu.reset_heading(-90)

async def Ruta_6_llevar_ladrillos_verdes():
    await retrocede_gyro(-90, 600)
    await giro(90)
    await retrocede_gyro(90, 265)
    await subir_cuadro()
    await retrocede_gyro(90, 40)

async def Ruta_7_recoger_ladrillos_amarillos():
    await seguir_linea_hasta_interseccion()
    await montarse_linea(90)
    await giro(0)
    await seguir_linea_hasta_interseccion("derecha")
    await montarse_linea(0)
    await giro(-90)
    await bajar_cuadro()
    await retrocede_gyro(-90, 1000, max_speed=50, stall_stop=True)
    await avance_gyro(-90, 40)
    await cerrar_cuadro()

async def Ruta_8_llevar_ladrillos_amarillos():
    await seguir_linea_hasta_interseccion("izquierda")
    await montarse_linea(-90)
    await giro(180)
    await seguir_linea_distancia(450)
    await seguir_linea_hasta_interseccion("derecha")
    await montarse_linea(180)
    await giro(-90)
    await seguir_linea_distancia(300)
    await seguir_linea_hasta_interseccion()
    await montarse_linea(-90)
    await giro(180)
    await retrocede_gyro(180, 180)
    await subir_cuadro()
    await retrocede_gyro(180, 40)   

async def Ruta_9_Recoger_y_llevar_YACUZY():
    await seguir_linea_hasta_interseccion()
    await giro(0, "medio")
    await bajar_cuadro()
    await retrocede_gyro(0, 140)
    await cerrar_cuadro()
    await seguir_linea_hasta_interseccion()
    await montarse_linea(0)
    await giro(90, "medio")
    await seguir_linea_distancia(450)
    await giro_pivot(-5, pivote_izquierdo=True, tipo_carga="pesado")
    await giro_pivot(90, pivote_izquierdo=False, tipo_carga="pesado")
    await subir_cuadro()

async def Ruta_10_recoger_ladrillos_azules():
    await avance_gyro(90, 140)
    await multitask(giro(-90), bajar_cuadro())
    await retrocede_gyro(-90, 1000, max_speed=50, stall_stop=True)
    await avance_gyro(-90, 40)
    await cerrar_cuadro()

async def Ruta_11_Recoger_pala():
    await seguir_linea_hasta_interseccion()
    await montarse_linea(-90)
    await giro(180)
    await seguir_linea_hasta_interseccion("izquierda")
    await avance_gyro(180, 30, max_speed=30)
    await montarse_linea(180)
    await giro_pivot(-90, pivote_izquierdo=False, tipo_carga="pesado")
    await multitask(avance_gyro(-90, 450), abrir_garra())
    await motor_dc_por_segundos(motorElevador, 600, -30)
    await cerrar_garra()
    await wait(1000)
    await motor_dc_por_grados(motorElevador, 80, 100)
    await avance_gyro(-90, 1430, stall_stop=True)

async def Ruta_12_llevar_pala():
    await giro_pivot(-125, pivote_izquierdo=False)
    await retrocede_gyro(-125, 500)
    await giro(-90)
    await seguir_linea_distancia(350)
    await seguir_linea_hasta_interseccion()
    await giro(-116)
    await multitask(abrir_garra(), subir_elevador())

async def Ruta_13_recoger_bloques_2():
    await retrocede_gyro(-116, 650)
    await boton()
    await giro(0)
    await seguir_linea_hasta_interseccion("izquierda")
    await montarse_linea(0)
    await giro(-90)
    await seguir_linea_hasta_interseccion()
    await avance_gyro(-90, 287)
    await giro(0)
    await bajar_elevador()
    await avance_gyro(0, 120, max_speed=60)
    await cerrar_garra()
    await wait(800)
    await subir_elevador()

# Inicio del Run
async def Run():
    await advertencia_bateria()
    await subir_cuadro()
    await subir_elevador_todo()
    await boton()
    hub.imu.reset_heading(0)
    await wait(10)
    hub.imu.reset_heading(0)
    await wait(200)
    await Ruta_1_recoger_bloques()
    await Ruta_2_recoger_ladrillos_blancos()
    await Ruta_3_llevar_ladrillos_blancos(131)
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
    
    await medir_trayectos_manual()

run_task(Run())