from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from pybricks.tools import run_task, wait, multitask
from imports_and_inits import avance_gyro, avance_hasta_linea,retrocede_gyro,left_motor, right_motor, giro, giro_pivot,avance_hasta_linea
from subdyk import get_robot
hub = PrimeHub()
import math
line = ColorSensor(Port.D)
motorgarras = Motor(Port.C)
motorElevador = Motor(Port.E)
motorcuadro = Motor(Port.A)


async def getBlackLineError():
    # Paréntesis extra para resolver el await antes de la división
    return ((await line.reflection()) / 10) - 8 

async def getBlackLineWidth():
    # Paréntesis extra para resolver el await antes del módulo
    return ((await line.reflection()) % 10)

async def seguir_linea_distancia(distancia_mm, potencia_crucero=85, potencia_inicial=40, accel_rate=2, potencia_final=10, decel_rate=2,kp_base=1.25 , kd_base=2.5):
 
    diametro_rueda = 62.5  
    grados_objetivo = (distancia_mm * 360) / (math.pi * diametro_rueda)

    left_motor.reset_angle(0)
    right_motor.reset_angle(0)

    last_error = 0 
    potencia_actual = abs(potencia_inicial)
    potencia_crucero = min(abs(potencia_crucero), 100)
    potencia_final = min(abs(potencia_final), potencia_crucero)
    last_error = 0  
    error_deadband = 0.05 
    
    pasos_desaceleracion = (potencia_crucero - potencia_final) / decel_rate
    grados_desaceleracion_estimados = pasos_desaceleracion * 5  
    grados_inicio_desaceleracion = max(0, grados_objetivo - grados_desaceleracion_estimados)


    while (abs(left_motor.angle()) + abs(right_motor.angle())) / 2 < grados_objetivo:
        grados_recorridos = (abs(left_motor.angle()) + abs(right_motor.angle())) / 2

        
        if grados_recorridos >= grados_inicio_desaceleracion:
            if potencia_actual > potencia_final:
                potencia_actual -= decel_rate
                potencia_actual = max(potencia_actual, potencia_final)
        else:
            if potencia_actual < potencia_crucero:
                potencia_actual += accel_rate
                potencia_actual = min(potencia_actual, potencia_crucero)
            
        error = await getBlackLineError()
        
        if abs(error) <= error_deadband:
            error = 0
        else:
            error = error - (error_deadband*(error/abs(error)))
            
        derivative = (error - last_error) 

        factor_escala = min (potencia_crucero/ potencia_actual,2)
        
        kp_dinamico = kp_base * factor_escala
        kd_dinamico = kd_base  * factor_escala
        if kp_dinamico > kp_base * 1.25:
            kp_dinamico = kp_dinamico*1.3
        if kd_dinamico > kd_base * 1.25:
            kd_dinamico = kd_dinamico*1.3
  
        turn = (kp_dinamico * error) + (kd_dinamico * derivative)
        
        limite_turn = potencia_actual * 0.4
        turn_clamped = max(min(turn, limite_turn), -limite_turn)

        if abs(error) < error_deadband:
            turn_clamped = 0

        dc_izq = potencia_actual - turn_clamped
        dc_der = potencia_actual + turn_clamped

     
        left_motor.dc(dc_izq)
        right_motor.dc(dc_der)
        
        last_error = error
        await wait(10)
    left_motor.hold()
    right_motor.hold()

async def seguir_linea_hasta_interseccion(potencia_crucero=60, potencia_inicial=40, accel_rate=1 , kp_base=1.25 , kd_base=2.5, umbral_interseccion=7):
 
    last_error = 0  
    potencia_actual = abs(potencia_inicial)
    error_deadband = 0.05 
    
    while (await getBlackLineWidth()) < umbral_interseccion:
        
        if potencia_actual < potencia_crucero:
            potencia_actual += accel_rate

        if potencia_actual > potencia_crucero:
            potencia_actual = potencia_crucero
            
        error = await getBlackLineError()
        
        if abs(error) <= error_deadband:
            error = 0
        else:
            error = error - (error_deadband*(error/abs(error)))    
        derivative = (error - last_error) 

        factor_escala = min (potencia_crucero/ potencia_actual,2)
        
        kp_dinamico = kp_base * factor_escala 
        kd_dinamico = kd_base  * factor_escala
        if kp_dinamico > kp_base * 1.25:
            kp_dinamico = kp_dinamico*1.3
        if kd_dinamico > kd_base * 1.25 :
            kd_dinamico = kd_dinamico*1.3
   
        turn = (kp_dinamico * error) + (kd_dinamico * derivative)
        
        limite_turn = potencia_actual * 0.4
        turn_clamped = max(min(turn, limite_turn), -limite_turn)

        if abs(error) < error_deadband:
            turn_clamped = 0

        dc_izq = potencia_actual - turn_clamped
        dc_der = potencia_actual + turn_clamped

     
        left_motor.dc(dc_izq)
        right_motor.dc(dc_der)
        
        last_error = error

        wait(10)
    left_motor.hold()
    right_motor.hold()

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
    tiempo_objetivo_ms = milisegundos
    motor.dc(potencia)
    while cronometro.time() < tiempo_objetivo_ms:
        await wait(10)
    motor.hold()

async def subir_elevador_todo():
    await motor_dc_por_segundos(motorElevador, 1000, 100)

async def subir_elevador():
    await motor_dc_por_grados(motorElevador, 200, 100)
    motorElevador.hold

async def bajar_elevador():
    await motor_dc_por_segundos(motorElevador, 1000, -100)

async def bajar_cuadro():
    await motor_dc_por_segundos(motorcuadro, 1300, 80)

async def subir_cuadro():
    await motor_dc_por_grados(motorcuadro, -260, 100)

async def abrir_garra():
    await motor_dc_por_segundos(motorgarras, 1500, -100)

async def cerrar_garra():
    await motor_dc_por_segundos(motorgarras, 1500, 100)

async def cerrar_garra_poquito():
    
    await motor_dc_por_grados(motorgarras, 50, 50)

async def dejar_bloques ():
    #bajar la garra hasta el mosaico
    await motor_dc_por_grados(motorElevador, -120, 70)
    await wait (200)
    #abrir toda la garra despacio
    await motor_dc_por_grados(motorgarras, -100, 60)
    await motor_dc_por_segundos(motorgarras, 500, -80)
    await cerrar_garra_poquito()
    await wait (100)
    await subir_elevador_todo()
    #cerrar garra para majar
    await motor_dc_por_grados(motorgarras, 135, 50)
    # baja para majar
    await motor_dc_por_segundos(motorElevador, 600, -100)
    await subir_elevador_todo()
    #cerrar garra para majar
    await motor_dc_por_segundos(motorgarras, 500, 80)
    # baja para majar
    await motor_dc_por_segundos(motorElevador, 600, -100)
    await subir_elevador_todo()

async def montarse_linea ():
    await avance_gyro(0,80,30,40,30,50,1,True,True)

#Prueba de dejar bloques
"""
    await abrir_garra()
    await bajar_elevador()
    await cerrar_garra()
    await subir_elevador()
    await avance_gyro (0,1000,30,40,30,800,1.2,True,True)
    await dejar_bloques()
"""
#Inicio Run
async def Run():
    await abrir_garra()
    await bajar_elevador()
    await cerrar_garra()
    await subir_elevador()
    await avance_hasta_linea(0,30,50,1.2)
    await montarse_linea()
    await giro (90,False)
    await wait (100)
    await seguir_linea_distancia (500)
    await seguir_linea_hasta_interseccion ()
    await giro(-90,False)
    await wait (100)
    await seguir_linea_distancia (55)
    await avance_gyro (-90,1000,30,40,30,800,1.2,False)
    await dejar_bloques()
#final Run
#run_task(seguir_linea_hasta_interseccion ())
run_task(seguir_linea_distancia(700))
#run_task(Run())
