from pybricks.tools import wait, run_task, multitask

# Importación de módulos propios organizados
from GyroMovement import hub, left_motor, right_motor, avance, retrocede, giro, giro_pivot
from Line_sensors import seguir_linea_distancia, seguir_linea_hasta_interseccion, avance_hasta_linea
from Actuators import (
    motorElevador, motorcuadro, 
    subir_elevador_todo, subir_elevador, bajar_elevador, 
    bajar_cuadro, subir_cuadro, cerrar_paletas, abrir_paletas,
    abrir_garra, abrir_garra_todo, cerrar_garra, cerrar_garra_poquito, 
    motor_dc_por_grados, motor_dc_por_segundos
)
from Extras import boton, advertencia_bateria, medir_trayectos_manual
from Funciones_ESP32 import leer_camara, servo

heading_temporal = 0

heading_temporal = 0

bloques_azules = 6
bloques_verdes = 6
bloques_amarillos = 6
bloques_blancos = 6

colores= {1:bloques_amarillos, 2:bloques_azules, 3:bloques_verdes, 4:bloques_blancos}

posicion_actual=0
lado_posicion_actual=0 #0 es izquierda, 1 es derecha
posicion_objetivo=0
lado_posicion_objetivo=0 #0 es derecha, diferente de 0 es izquierda
heading_objetivo=0
#dar valor antes de usar la logica de resolver lecturas
lectura=[]
i=0

llevadas=0 # Poner 0 antes de usar la logica
izquierda, derecha, lado_rojo = [], [], -1

async def lectura_camara():
    global izquierda, derecha, lado_rojo
    izquierda, derecha, lado_rojo = leer_camara()
    
async def logica_de_recoleccion():
    await multitask(giro(-90), abrir_garra_todo())
    if lado_posicion_actual == 0: #Si el robot está en el lado izquierdo, avanza hasta la línea y se monta sobre ella, esta distancia es suficiente para que el robot agarre la primera fila
        await seguir_linea_hasta_interseccion("izquierda", montarse=False, target_heading=-90)
    else:
        await seguir_linea_hasta_interseccion("derecha", montarse=False, target_heading=-90)
    avanzar = (6 - colores[lectura[i]]) // 2 #Calcula cuantas filas de bloques hay que avanzar para llegar al bloque objetivo
    if avanzar >= 1:
        await avance(-90, 20 * avanzar) #Calibrar para que el robot avance a la siguiente fila de bloques
    await cerrar_garra()
    #Calcula cuantas filas de bloques avanzo para retroceder lo mismo
    #Calibrar para que el robot retroceda hasta llegar a la linea
    await retrocede(-90, 20 * avanzar + 20) #Calibrar para que el robot retroceda hasta llegar a media linea para poder girar y avanzar hasta la línea de del siguiente color objetivo
    colores[lectura[i]] -= 1 #Resta uno a la cantidad de bloques del color objetivo

async def logica_de_movimiento(objetivo, lado_posicion_objetivo):
    global heading_objetivo, lado_posicion_actual, posicion_actual
    movimiento = abs(objetivo)
    if objetivo > 0: # Si el objetivo es mayor que 0, gira a la derecha
        await giro(0)
        heading_objetivo=0
    elif objetivo < 0: # Si el objetivo es menor que 0, gira a la izquierda:
        await giro(180)
        heading_objetivo=180

    if objetivo != 0: #si la distancia es diferente de 0, avanza hasta la línea
        while movimiento > 1:
            await avance_hasta_linea(heading_objetivo, distancia_ignorar=10, min_sensores_linea=1, montarse=True, distancia_montaje=10 ) #Valores a ajustar: Distancia a montarse para que pase sobre la linea
            movimiento -= 1

        if (lado_posicion_objetivo==0 and heading_objetivo == 0) or (lado_posicion_objetivo!=0 and heading_objetivo == 180):
            await avance_hasta_linea(heading_objetivo, distancia_ignorar=10, min_sensores_linea=1, montarse=True, distancia_montaje=10 ) #Valores a ajustar: Distancia a montarse para que pase sobre la linea y se alinee con los bloques al otro lado 
        else:
            await avance_hasta_linea(heading_objetivo, distancia_ignorar=10, min_sensores_linea=1) #No se monta, solo avanza hasta la línea y se alinea con los bloques mas cercanos
        await giro(-90)
        if lado_posicion_objetivo==0:
            lado_posicion_actual = 1
        else:
            lado_posicion_actual = 0
        posicion_actual = lectura[i]
    else:
        if lado_posicion_objetivo == lado_posicion_actual: #Si tiene que ir a la derecha y el robot está en el lado izquierdo, gira a la derecha
            await giro(0)
            await avance(0,20) #Calibrar para que el robot se pase de izquierda a derecha
            await giro(-90)
            lado_posicion_actual = 1
            posicion_actual = lectura[i]

        elif lado_posicion_objetivo != 0 and lado_posicion_actual == 1: #Si tiene que ir a la izquierda y el robot está en el lado derecho, gira a la izquierda
            await giro(180)
            await avance(180,20) #Calibrar para que el robot se pase de derecha a izquierda
            await giro(-90)
            lado_posicion_actual = 0
            posicion_actual = lectura[i]

async def llevar_a_descarga():
    global posicion_actual, lado_posicion_actual
    if posicion_actual > 2:
        objetivo = 2 - posicion_actual
        lado_posicion_objetivo = 1 #Diferente de 0, el robot debe ir al lado izquierdo
        orientacion_retroceso = 1 # El robot esta a la abajo y debe retroceder hacia arriba
    else:
        objetivo = 3 - posicion_actual
        lado_posicion_objetivo = 0 #El robot debe ir al lado derecho
        orientacion_retroceso = 0 # El robot esta a la arriba y debe retroceder hacia abajo

    await logica_de_movimiento(objetivo, lado_posicion_objetivo)

    if orientacion_retroceso == 1:
        if llevadas == 2:
            await retrocede(180, 20) #Calibrar para que el robot retroceda un bloque
        elif llevadas == 1:
            await retrocede(180, 40) #Calibrar para que el robot retroceda dos bloques
    else:
        if llevadas == 2:
            await retrocede(0, 20) #Calibrar para que el robot retroceda un bloque
        elif llevadas == 0:
            await retrocede(0, 40) #Calibrar para que el robot retroceda dos bloques
    await giro(90)
    await avance (90, 20) #Calibrar para que el robot avance hasta la zona de descarga
    await abrir_garra_todo()
    await retrocede(90, 20) #Calibrar para que el robot retroceda hasta pasarse de la linea y para girar hacia arriva
    if llevadas != 1 :
        await giro(0)
        await avance_hasta_linea(0, distancia_ignorar=10, min_sensores_linea=1)
        posicion_actual = 3
        lado_posicion_actual = 0 #El robot está en la posición 3 y lado izquierdo
    else:
        await giro(180)
        await avance_hasta_linea(180, distancia_ignorar=10, min_sensores_linea=1)
        posicion_actual = 2
        lado_posicion_actual = 1 #El robot está en la posición 2 y lado derecho

async def agarrar_rojo():
    global posicion_actual, lado_posicion_actual
    objetivo = 4 - posicion_actual
    lado_posicion_objetivo = 1 #Va a la posicion 4 y lado izquierdo porque 1 != 0
    await logica_de_movimiento(objetivo, lado_posicion_objetivo) 
    await giro(90)
    await avance(90, 20) #Calibrar para que el robot avance hasta el rojo
    await multitask(giro(0), abrir_garra_todo())
    await avance(0, 20) #Calibrar para que el robot avance hasta agarrar el rojo 
    await cerrar_garra()
    await retrocede(0, 20) #Calibrar para que el robot retroceda lo mismo que avanzó para agarrar el rojo\
    await giro(90)
    await retrocede(90, 20) #Calibrar para que el robot retroceda hasta pasarse de la linea y para girar hacia abajo
    await giro(180)
    await avance_hasta_linea(180, distancia_ignorar=10, min_sensores_linea=1) 
    posicion_actual = 3
    lado_posicion_actual = 1 #El robot está en la posición 3 y lado derecho

async def resolver_lecturas():
    global i, llevadas
    contador = 0
    for i in range(len(lectura)): #Recorre la lista de lecturas

        while lectura[i] < 5 and colores[lectura[i]] == 0: #Verifica si el color objetivo ya no tiene bloques, si es así, cambia el color objetivo al siguiente color disponible
            if lectura[i] < 4 :
                lectura[i] = lectura[i] + 1
            else:
                lectura[i] = lectura[i] - 1

        if lectura[i] <5: # Si la lectura no es roja

            """
            LOGICA DE MOVIMIENTO HACIA EL COLOR OBJETIVO
            """
            lado_posicion_objetivo = colores[lectura[i]]%2 # Si el bloque es par, el lado objetivo es 0 (DERECHA), si es impar, el lado objetivo es diferente de 0 (IZQUIERDA)

            objetivo = lectura[i] - posicion_actual # Calcula la diferencia entre la posición actual y la posición objetivo, me dice cuanto debo moverme para llegar al color objetivo
            
            await logica_de_movimiento(objetivo, lado_posicion_objetivo)

            """
            LOGICA PARA AVANZAR Y RECOGER EL BLOQUE
            """
            await logica_de_recoleccion()
            contador += 1
            if contador >1:
                """
                LOGICA PARA LLEVAR LOS BLOQUES A LA ZONA DE DESCARGA
                """
                await llevar_a_descarga() 
                llevadas += 1
                contador =0
        else: 
            """
            Logica para cuando la lectura es roja
            """
            await agarrar_rojo()

# ---------------------------------------------------------
# RUTINAS LOCALES DE DESCARGA 
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
# RUTAS DE COMPETICIÓN 
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
    

run_task(Run())