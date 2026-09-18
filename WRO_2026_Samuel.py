from pybricks.tools import wait, run_task, multitask

# Importación de módulos propios organizados
from GyroMovement import hub, left_motor, right_motor, avance, retrocede, giro, giro_pivot
from Line_sensors import seguir_linea_distancia, seguir_linea_hasta_interseccion, avance_hasta_linea
from Actuators import (
    motorgarras, motorElevador, motorcuadro, 
    subir_elevador_todo, subir_elevador, bajar_elevador, 
    bajar_cuadro, subir_cuadro, cerrar_cuadro, 
    abrir_garra, abrir_garra_todo, cerrar_garra, cerrar_garra_poquito, 
    motor_dc_por_grados, motor_dc_por_segundos
)
from Extras import boton, advertencia_bateria, medir_trayectos_manual

heading_temporal = 0

# ---------------------------------------------------------
# RUTINAS LOCALES DE DESCARGA (Mantenidas en Main)
# ---------------------------------------------------------
async def dejar_bloques():   
    await motor_dc_por_grados(motorElevador, -42, 70)
    await motor_dc_por_grados(motorgarras, 90, 80)
    await motor_dc_por_grados(motorgarras, 85, 100)
    await cerrar_garra_poquito()
    await subir_elevador_todo()
    await motor_dc_por_grados(motorgarras, -85, 50)
    await motor_dc_por_segundos(motorElevador, 500, -60)
    await subir_elevador_todo()
    await motor_dc_por_segundos(motorgarras, 500, -80)
    await motor_dc_por_segundos(motorElevador, 500, -90)
    await subir_elevador_todo()

async def dejar_bloques_2():
    await motor_dc_por_grados(motorElevador, -42, 70)
    await motor_dc_por_grados(motorgarras, 90, 80)
    await motor_dc_por_grados(motorgarras, 85, 100)
    await cerrar_garra_poquito()
    await subir_elevador_todo()
    await motor_dc_por_grados(motorgarras, -85, 50)
    await motor_dc_por_segundos(motorElevador, 500, -60)
    await subir_elevador_todo()
    await motor_dc_por_segundos(motorgarras, 500, -80)
    await motor_dc_por_segundos(motorElevador, 500, -90)
    await subir_elevador_todo()

# ---------------------------------------------------------
# RUTAS DE COMPETICIÓN (Máquina de Estados en mm)
# ---------------------------------------------------------
async def Ruta_1_recoger_bloques():
    hub.imu.reset_heading(0)
    await multitask(avance(0, 202), bajar_elevador())
    await cerrar_garra()
    await wait(800)
    await subir_elevador()
    await retrocede(0, 85)
        
async def Ruta_2_recoger_ladrillos_blancos():
    await giro(90)
    await avance(90, 136)
    await seguir_linea_distancia(1250)
    await wait(200)
    await giro(173)
    await giro(-90)
    await multitask(retrocede(-90, 220), bajar_cuadro())
    await cerrar_cuadro()

async def Ruta_3_llevar_ladrillos_blancos(angulo_giro):
    await avance_hasta_linea(-70, distancia_ignorar=82, min_sensores_linea=1)
    await giro(2)
    await seguir_linea_hasta_interseccion("derecha", centrado_estricto=True, target_heading=0)
    await giro(angulo_giro)
    await wait(100)
    await retrocede(angulo_giro, 693)
    await giro_pivot(92, pivote_izquierdo=False)
    await retrocede(90, 16)
    await avance(90, 16)
    await subir_cuadro()
    
async def Ruta_4_recoger_verdes():
    await giro_pivot(136, pivote_izquierdo=False)
    await avance(136, 248)
    await giro(90)
    await seguir_linea_hasta_interseccion(montarse=False)
    hub.imu.reset_heading(90)
    await giro(-90)
    await multitask(retrocede(-90, 200), bajar_cuadro())
    await cerrar_cuadro()

async def Ruta_5_llevar_bloques():
    await seguir_linea_distancia(450)
    await seguir_linea_hasta_interseccion(montarse=False)
    global heading_temporal
    heading_temporal = hub.imu.heading()
    await avance(heading_temporal, 273)
    left_motor.hold()
    right_motor.hold()
    await wait(50)
    hub.imu.reset_heading(-90)

async def Ruta_6_llevar_ladrillos_verdes():
    await multitask(retrocede(-90, 305), cerrar_garra())
    await giro(90)
    await multitask(subir_cuadro(), retrocede(90, 200))

async def Ruta_7_recoger_ladrillos_amarillos():
    await seguir_linea_distancia(400)
    await seguir_linea_hasta_interseccion(montarse=True, target_heading=90, distancia_montaje=110)
    await giro(180)
    await bajar_cuadro()
    await retrocede(180, 200)
    await cerrar_cuadro()

async def Ruta_8_llevar_ladrillos_amarillos():
    await avance_hasta_linea(-110, distancia_ignorar=27)
    await wait(300)
    await giro(180)
    await seguir_linea_distancia(200)
    await seguir_linea_hasta_interseccion("derecha", target_heading=180)
    await wait(100)
    await giro(-90)
    await seguir_linea_distancia(400)
    await seguir_linea_hasta_interseccion(target_heading=-90)
    await giro(180)
    await multitask(subir_cuadro(), retrocede(180, 131))  

async def Ruta_9_Recoger_y_llevar_YACUZY():
    await seguir_linea_hasta_interseccion(montarse=False)
    await giro(0)
    await bajar_cuadro()
    await retrocede(0, 76)
    await motor_dc_por_segundos(motorcuadro, 1000, 100)
    motorcuadro.hold()
    await avance(0, 22)
    await giro(90)
    await seguir_linea_distancia(480)
    hub.imu.reset_heading(90)
    await giro_pivot(-5, pivote_izquierdo=True)
    await giro_pivot(89, pivote_izquierdo=False)
    await avance(89, 49)

async def Ruta_10_recoger_ladrillos_azules():
    await multitask(avance(90, 60), subir_cuadro())
    await giro(-90)
    await multitask(retrocede(-90, 150), bajar_cuadro())
    await cerrar_cuadro()

async def Ruta_11_Recoger_pala():
    await seguir_linea_hasta_interseccion(target_heading=-90, montarse=False)
    await giro(-177)
    await multitask(seguir_linea_hasta_interseccion("izquierda", montarse=False, target_heading=180), abrir_garra_todo())
    
    await multitask(avance(180, 545), subir_elevador_todo())
    hub.imu.reset_heading(180)
    await retrocede(180, 150)
    await multitask(cerrar_garra(), bajar_elevador())
    await giro_pivot(-95, False)
    
    await multitask(avance(-95, 164), abrir_garra(), motor_dc_por_grados(motorElevador, 140, 100))
    await cerrar_garra()
    await avance(-110, 27)
    await avance(-90, 709)
    await retrocede(-90, 82)
    await avance_hasta_linea(-60, distancia_ignorar=164, montarse=True)
    await giro(-90)

async def Ruta_12_llevar_pala():
    await seguir_linea_distancia(100)
    await seguir_linea_hasta_interseccion(montarse=False)
    await giro(-120)
    await multitask(abrir_garra(), subir_elevador_todo())
    await giro(-90)

async def Ruta_13_recoger_bloques_2():
    await multitask(retrocede(-90, 142), motor_dc_por_grados(motorgarras, 40, 100))
    await giro_pivot(0, False)
    await wait(200)
    await avance_hasta_linea(0, distancia_ignorar=0)
    await avance(0, 16)
    await wait(200)
    await giro(-90)
    await wait(100)
    await seguir_linea_hasta_interseccion("izquierda", montarse=False)
    await wait(100)
    await avance(-90, 169)
    await giro(0.3)
    await bajar_elevador()
    global heading_temporal
    heading_temporal = hub.imu.heading()
    await avance(heading_temporal, 71)
    await cerrar_garra()
    await wait(800)
    await subir_elevador()

async def Ruta_14_llevar_bloques_2():
    await avance(50, 191)
    await avance(85, 191)
    await seguir_linea_hasta_interseccion(montarse=False)
    heading_temporal = hub.imu.heading()
    await avance(heading_temporal, 218)
    left_motor.hold()
    right_motor.hold()
    await wait(50)
    hub.imu.reset_heading(90)

async def Ruta_15_llevar_azules():
    await retrocede(90, 324)
    await giro(-90)
    await retrocede(-90, 55)
    await multitask(subir_cuadro(), retrocede(-90, 115))

# Inicio del Run principal
async def Run():
    voltaje = (hub.battery.voltage() / 1000.0)
    print(voltaje)
    if float(voltaje) <= 7.9 or voltaje > 8.3:
        await hub.speaker.beep(1000, 50) 
        
    await multitask(subir_cuadro(), subir_elevador_todo())
    hub.imu.reset_heading(0)
    
    await Ruta_1_recoger_bloques()
    await Ruta_2_recoger_ladrillos_blancos()
    await Ruta_3_llevar_ladrillos_blancos(137.5)
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
    await dejar_bloques_2()
    await Ruta_15_llevar_azules()
    await medir_trayectos_manual()

run_task(Run())