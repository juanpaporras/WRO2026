from pybricks.tools import wait, run_task, multitask

# Importación de módulos propios organizados
from GyroMovement import hub, left_motor, right_motor, avance, retrocede, retrocede_hasta_pared, avance_hasta_pared, giro, giro_pivot, giro_arco
from Line_sensors import sensor_mbc, diagnostico_espectral_mbc,seguir_linea_distancia, seguir_linea_hasta_interseccion, avance_hasta_linea, avance_contando_lineas, retrocede_hasta_linea
from Actuators import (
    motorElevador, motorgarra, 
    subir_elevador_todo, subir_elevador, bajar_elevador, 
    bajar_cuadro, subir_cuadro, cerrar_paletas, abrir_paletas,
    abrir_garra, cerrar_garra, cerrar_garra_poquito,
    motor_dc_por_grados, motor_dc_por_segundos
)
from Extras import boton, advertencia_bateria, medir_trayectos_manual
from Funciones_ESP32 import iniciar_esp32, proceso_esp32, leer_camara, servo
from Robot_specs import get_robot
robot = get_robot()
hub = robot.hub
left_motor = robot.left_motor
right_motor = robot.right_motor

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

# ---------------------------------------------------------
# MEDIDAS APROXIMADAS (mm) - SOLO PARA LA SIMULACIÓN, CALIBRAR EN LA MESA
# Sacadas del tapete WRO 2026 Senior (2362 x 1143 mm).
# Coordenadas: 0 = hacia arriba del tapete, el robot recoge mirando a -90.
# ---------------------------------------------------------
SEPARACION_FILAS = 33   # Distancia entre las 2 filas de bloques de un mismo color (lado izq/der)
PASO_COLUMNA = 45         # Distancia entre columnas de bloques (avanzar una "fila" hacia -90)
VEL_RECOLECCION = 200     # Calibrar: velocidad (mm/s) al entrar a agarrar; más baja = se desliza menos al frenar
EXTRA_RETROCESO = 63   # Calibrar: mm extra que retrocede MÁS de lo que avanzó, para quedar en el medio del carril y girar sin tocar los bloques
DISTANCIA_MONTAJE = 55    # Calibrar: mm que avanza después de la línea cuando tiene que pasarse al otro lado (si es mucho, luego no encuentra la línea al seguirla)
MONTAJE_SIN_CAMBIO = 18   # Calibrar: mm que avanza después de la línea cuando NO tiene que pasarse de lado
DESCARGA_BLOQUE = 49     # Retroceso de "un bloque" en la zona de descarga
DESCARGA_AVANCE = 90      # Del carril hasta la zona de descarga
# Rojo: cuadro gris arriba a la derecha, pasando la calle gris vertical.
# Desde el centro del carril (fila de abajo de los blancos) está a ~300 mm hacia 90 y ~150 mm hacia 0.
ALCANCE_GARRA = 80        # Calibrar: distancia del eje de las ruedas al centro de la garra (garra adelante)
ROJO_AVANCE_90 = 250      # Desde el carril, a la altura de la línea blanca, hasta quedar debajo del rojo (mirando a 90)
ROJO_AVANCE_50 = 200 - ALCANCE_GARRA   # ~70 mm: avance hacia arriba hasta que la garra quede sobre el rojo

async def lectura_camara():
    global izquierda, derecha, lado_rojo
    izquierda, derecha, lado_rojo, _, _, _ = await leer_camara()

ANGULO_BUSQUEDA = 10   # Calibrar: cuánto se desvía de -90 para buscar la línea (-100 desde la derecha, -80 desde la izquierda)
SIGNO_ERROR_MBC = 1    # Si al probar busca la línea cuando NO debe (o al revés), cambiar a -1

DC_BUSQUEDA = 35              # Calibrar: potencia al buscar la línea (lento para no pasarse)
DISTANCIA_MAX_BUSQUEDA = 120  # Calibrar: seguridad (mm); si no encuentra la línea real, deja de buscar
MM_POR_GRADO = (63.0 * 3.141) / 360.0   # Rueda de 63 mm
AVANCE_MIN_BUSQUEDA = 10      # Calibrar: grados de rueda que avanza SIEMPRE tras el giro de búsqueda antes de poder detenerse (10° ≈ 5.5 mm)

async def ve_linea_real():
    """True si el sensor ve la línea Y el error la ubica del lado correcto según
    lado_posicion_actual. Si el error haría corregir hacia el lado contrario, es una
    línea ficticia (gris de la mesa) y cuenta como que NO ve la línea.
    Seguidor: turn_rate = -kp*error -> error > 0 corrige a la IZQUIERDA (motor derecho
    con más potencia), error < 0 corrige a la DERECHA."""
    await sensor_mbc.actualizar_lectura()
    if int(sensor_mbc.ancho_linea) == 0:
        return False
    error = sensor_mbc.error_espacial * SIGNO_ERROR_MBC
    if lado_posicion_actual == 1:
        return error > 0   # derecha de la línea: la línea debe quedar a su izquierda
    else:
        return error < 0   # izquierda de la línea: la línea debe quedar a su derecha

async def buscar_linea_recoleccion():
    """Mirando a -90: si el sensor está completamente fuera de la línea, gira hacia
    el lado donde debe estar la línea, avanza hasta encontrarla y vuelve a -90.
    - Lado derecho (1): la línea queda a su izquierda -> gira a -100.
    - Lado izquierdo (0): la línea queda a su derecha -> gira a -80."""
    if await ve_linea_real():
        return   # ve la línea del lado correcto, no hace falta buscarla

    if lado_posicion_actual == 1:
        heading_busqueda = -90 - ANGULO_BUSQUEDA   # -100
    else:
        heading_busqueda = -90 + ANGULO_BUSQUEDA   # -80

    await giro(heading_busqueda)

    # Avanza recto (giroscopio) hasta ver la línea REAL. No usa avance_hasta_linea porque esa
    # se detiene con cualquier negro, incluida la línea ficticia del gris.
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    while True:
        recorrido = ((left_motor.angle() + right_motor.angle()) / 2.0) * MM_POR_GRADO
        if recorrido >= DISTANCIA_MAX_BUSQUEDA:
            print("No encontró la línea real")
            break
        grados_ruedas = (left_motor.angle() + right_motor.angle()) / 2.0
        if grados_ruedas >= AVANCE_MIN_BUSQUEDA and await ve_linea_real():
            break
        if grados_ruedas >= AVANCE_MIN_BUSQUEDA and int(sensor_mbc.ancho_linea) >= 6:
            # Con un ángulo tan suave puede llegar a la línea vertical (intersección) antes
            # de encontrar la línea: para ahí para no meterse en los bloques.
            print("Llegó a la intersección buscando la línea")
            break
        error_ang = ((heading_busqueda - hub.imu.heading() + 180) % 360) - 180
        correccion = 3.5 * error_ang
        left_motor.dc(max(min(DC_BUSQUEDA + correccion, 100.0), -100.0))
        right_motor.dc(max(min(DC_BUSQUEDA - correccion, 100.0), -100.0))
        await wait(10)
    left_motor.hold()
    right_motor.hold()
    await giro(-90)

async def logica_de_recoleccion():
    await giro(-90)
    await abrir_garra()
    await buscar_linea_recoleccion()   # Si el sensor no ve la línea, la busca según el lado antes de seguirla
    if lado_posicion_actual == 0: #Si el robot está en el lado izquierdo, avanza hasta la línea y se monta sobre ella, esta distancia es suficiente para que el robot agarre la primera fila
        await seguir_linea_hasta_interseccion("izquierda", montarse=False, target_heading=-90, vel_crucero_mm_s=VEL_RECOLECCION)
    else:
        await seguir_linea_hasta_interseccion("derecha", montarse=False, target_heading=-90, vel_crucero_mm_s=VEL_RECOLECCION)  # Distancia real recorrida desde el carril
    avanzar =(6 - colores[lectura[i]]) // 2 #Calcula cuantas filas de bloques hay que avanzar para llegar al bloque objetivo
    # Lado derecho (1) avanza hacia -85, lado izquierdo (0) hacia -95 (Calibrar).
    # Si no avanza columnas se queda en -90. El retroceso usa el mismo ángulo para volver al mismo punto.
    # Con 2 avances (quedan 1 o 2 bloques), el PRIMER avance va recto a -90 y solo el segundo en diagonal.
    heading_avance = -90
    avances_rectos = 0   # cuántas columnas avanzó a -90 (para retrocederlas también a -90)
    if avanzar >= 1:
        heading_avance = -85 if lado_posicion_actual == 1 else -95
        if avanzar >= 2:
            avances_rectos = avanzar - 1
            await avance(-90, PASO_COLUMNA * avances_rectos, 60) #Calibrar: primer avance recto a -90
        await avance(heading_avance, PASO_COLUMNA+10, 60) #Calibrar para que el robot avance a la siguiente fila de bloques
    await cerrar_garra_poquito()
    #Calcula cuantas filas de bloques avanzo para retroceder lo mismo
    #Calibrar para que el robot retroceda hasta llegar a la linea
    # Retrocede en orden inverso: primero el tramo en diagonal, luego el recto a -90 + el extra
    if avanzar >= 1:
        await retrocede(heading_avance, PASO_COLUMNA, max_dc=70, accel_mm=80) #Calibrar: deshace el avance en diagonal
    await retrocede(-90, PASO_COLUMNA * avances_rectos + EXTRA_RETROCESO, max_dc=70, accel_mm=80) #Calibrar (max_dc/accel_mm: arranque más suave para que no cabecee) para que el robot retroceda hasta llegar a media linea para poder girar y avanzar hasta la línea de del siguiente color objetivo
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

    if objetivo != 0: #si la distancia es diferente de 0, avanza contando líneas SIN frenar en las intermedias
        # En la última línea: se monta (pasa al otro lado) o se queda justo en ella
        montarse_final = (lado_posicion_objetivo==0 and heading_objetivo == 0) or (lado_posicion_objetivo!=0 and heading_objetivo == 180)
        # Solo se monta si tiene que pasarse al otro lado de la línea; si no, frena apenas la detecta.
        await avance_contando_lineas(
            heading_objetivo, movimiento,
            distancia_ignorar=50, min_sensores_linea=2,
            montarse=True, distancia_montaje=DISTANCIA_MONTAJE if montarse_final else MONTAJE_SIN_CAMBIO,  # Calibrar: distancia para pasar la línea y alinearse con los bloques del otro lado
            max_dc=80, dc_aproximacion=40  # Calibrar: velocidad crucero y velocidad al llegar a la última línea
        )
        if lado_posicion_objetivo==0:
            lado_posicion_actual = 1
        else:
            lado_posicion_actual = 0
        posicion_actual = lectura[i]
    else:
        if lado_posicion_objetivo == 0 and lado_posicion_actual == 0: #Si tiene que ir a la derecha y el robot está en el lado izquierdo, gira a la derecha
            await giro(5)
            await avance(5, SEPARACION_FILAS) #Calibrar para que el robot se pase de izquierda a derecha
            lado_posicion_actual = 1
            posicion_actual = lectura[i]

        elif lado_posicion_objetivo != 0 and lado_posicion_actual == 1: #Si tiene que ir a la izquierda y el robot está en el lado derecho, gira a la izquierda
            await giro(175)
            await avance(175, SEPARACION_FILAS) #Calibrar para que el robot se pase de derecha a izquierda
            lado_posicion_actual = 0
            posicion_actual = lectura[i]

async def llevar_a_descarga():
    global posicion_actual, lado_posicion_actual
    if posicion_actual > 2:
        objetivo = 2 - posicion_actual
        lado_posicion_objetivo = 0 #Diferente de 0, el robot debe ir al lado izquierdo
        orientacion_retroceso = 1 # El robot esta a la abajo y debe retroceder hacia arriba
    else:
        objetivo = 3 - posicion_actual
        lado_posicion_objetivo = 1 #El robot debe ir al lado derecho
        orientacion_retroceso = 0 # El robot esta a la arriba y debe retroceder hacia abajo

    await logica_de_movimiento(objetivo, lado_posicion_objetivo)

    if orientacion_retroceso == 1:
        if llevadas == 2:
            await retrocede(180, 35, 60) #Calibrar para que el robot retroceda un bloque
            print("llevadas = 2, retrocediendo 1 bloque")
        elif llevadas == 0:
            await retrocede(180, DESCARGA_BLOQUE * 2, 60) #Calibrar para que el robot retroceda dos bloques
            print("llevadas = 0, retrocediendo 2 bloques")
    else:
        if llevadas == 2:
            await retrocede(0, 35, 60) #Calibrar para que el robot retroceda un bloque
            print("llevadas = 2, retrocediendo 1 bloque")
        elif llevadas == 1:
            await retrocede(0, DESCARGA_BLOQUE * 2, 60) #Calibrar para que el robot retroceda dos bloques
            print("llevadas = 1, retrocediendo 2 bloques")
    await giro(90, 80)
    if llevadas == 2:
        await avance_hasta_linea(90, distancia_ignorar=0, min_sensores_linea=4, max_dc=60,esperar_blanco=False) #Calibrar para que el robot avance hasta la línea de descarga
        await seguir_linea_distancia (distancia_mm=DESCARGA_AVANCE, vel_crucero_mm_s=300) #Calibrar para que el robot avance hasta la línea de descarga y se monte sobre ella
    elif lado_posicion_actual == 0:
       await avance_hasta_linea(90, distancia_ignorar=0, min_sensores_linea=4, max_dc=60, montarse=True, distancia_montaje=DESCARGA_AVANCE,esperar_blanco=False) #Calibrar para que el robot avance hasta la línea de descarga y se monte sobre ella
    else:
        await avance_hasta_linea(90, distancia_ignorar=0, min_sensores_linea=4, max_dc=60, montarse=True, distancia_montaje=DESCARGA_AVANCE,esperar_blanco=False) #Calibrar para que el robot avance hasta la línea de descarga y se monte sobre ella
    if llevadas != 2:
        await abrir_garra()
        await retrocede(90, DESCARGA_AVANCE-5) #Calibrar para que el robot retroceda hasta pasarse de la linea y para girar hacia arriva
        await giro (-176)
        await retrocede_hasta_pared(-176, distancia_max = 1000) #Calibrar para que el robot retroceda hasta pasarse de la linea y para girar hacia arriva
        hub.imu.reset_heading(180)
        await avance_hasta_linea(180, distancia_ignorar=40, min_sensores_linea=2, max_dc=50, montarse=True, distancia_montaje=MONTAJE_SIN_CAMBIO) #Calibrar para que el robot avance hasta la línea de descarga
        posicion_actual = 4
        lado_posicion_actual = 1 #El robot está en la posición 4 y lado derecho

async def agarrar_rojo():
    global posicion_actual, lado_posicion_actual
    objetivo = 4 - posicion_actual
    lado_posicion_objetivo = 3 #Va a la posicion 4 y lado izquierdo porque 3 != 0
    await logica_de_movimiento(objetivo, lado_posicion_objetivo) 
    await giro(90)
    await seguir_linea_hasta_interseccion("derecha", target_heading=90, vel_crucero_mm_s=300,montarse=False) #Calibrar para que el robot avance hasta la línea de descarga y se monte sobre ella
    await giro_pivot(57, pivote_izquierdo=True)
    await abrir_garra()
    await avance(57, ROJO_AVANCE_50, max_dc=70) #Calibrar para que el robot avance hasta agarrar el rojo
    await cerrar_garra_poquito()
    await retrocede(57, ROJO_AVANCE_50+ 20) #Calibrar para que el robot retroceda lo mismo que avanzó para agarrar el rojo\
    await giro_pivot(90, pivote_izquierdo=True)
    posicion_actual = 4 #El robot está en la posición 4 y lado izquierdo
    lado_posicion_actual = 0 #El robot está en la posición 4 y lado izquierdo

async def   resolver_lecturas():
    global i, llevadas
    contador = 0
    for i in range(len(lectura)): #Recorre la lista de lecturas
        if llevadas == 3: #Si ya llevó 3 bloques termina la función y no hace nada más
            break
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
             # Espera a que el robot esté en la posición correcta y el botón sea presionado para continuar

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
            contador += 1
            if contador >1:
                """
                Si con el rojo ya completó 2, también lleva a la zona de descarga
                """
                await llevar_a_descarga()
                llevadas += 1
                contador =0


# ---------------------------------------------------------
# RUTINAS LOCALES DE DESCARGA 
# ---------------------------------------------------------
async def dejar_bloques():   
    await motor_dc_por_grados(motorElevador, -42, 70)
    await motor_dc_por_grados(motorgarra, 90, 80)
    await motor_dc_por_grados(motorgarra, 85, 100)
    await cerrar_garra_poquito()
    await subir_elevador_todo()
    await motor_dc_por_grados(motorgarra, -85, 50)
    await motor_dc_por_segundos(motorElevador, 500, -60)
    await subir_elevador_todo()
    await motor_dc_por_segundos(motorgarra, 500, -80)
    await motor_dc_por_segundos(motorElevador, 500, -90)
    await subir_elevador_todo()

async def dejar_bloques_2():
    await motor_dc_por_grados(motorElevador, -42, 70)
    await motor_dc_por_grados(motorgarra, 90, 80)
    await motor_dc_por_grados(motorgarra, 85, 100)
    await cerrar_garra_poquito()
    await subir_elevador_todo()
    await motor_dc_por_grados(motorgarra, -85, 50)
    await motor_dc_por_segundos(motorElevador, 500, -60)
    await subir_elevador_todo()
    await motor_dc_por_segundos(motorgarra, 500, -80)
    await motor_dc_por_segundos(motorElevador, 500, -90)
    await subir_elevador_todo()

# ---------------------------------------------------------
# RUTAS DE COMPETICIÓN 
# ---------------------------------------------------------
async def Ruta_1_Lectura(): # --> RUTA EN LISTA #Se acomoda de la salida y sigue linea hasta interseccion para llegar a leer, la distancia de montaje debe ser para que quede en el centro de la linea, gire se tire hacia atras y choque con el mosaico. Ahi ya hace la lectura
    #await avance(0, 100, 80)
    #await giro(90)

    #await avance(0, 15, 80, freno=False)
    #await giro_pivot(90, pivote_izquierdo=False)

    await multitask(giro_arco(90, 90, freno=False), subir_cuadro())
    await avance(90, 200, 80,freno=False)
    await seguir_linea_distancia(distancia_mm=500, vel_crucero_mm_s=700)
    await seguir_linea_hasta_interseccion("centro", target_heading=90, vel_crucero_mm_s=400, montarse=True, distancia_montaje=41)
    await giro(0)
    await seguir_linea_hasta_interseccion("centro", target_heading=0, vel_crucero_mm_s=200)
    hub.imu.reset_heading(0)
    await giro(-179,80)
    await retrocede_hasta_pared(180, distancia_max=1000, dc=50)
    hub.imu.reset_heading(180)

async def Ruta_2_Recoger_y_llevar_cemento():      # --> RUTA EN LISTA
    await avance(180, 150, 80)
    await seguir_linea_distancia(distancia_mm=75, vel_crucero_mm_s=300)
    await giro(1)
    await bajar_cuadro()
    await giro(90)
    await seguir_linea_distancia(distancia_mm=550, vel_crucero_mm_s=500, freno=False)
    await giro_pivot(0, pivote_izquierdo=True, freno=False)
    await giro_pivot(90, pivote_izquierdo=False)
    await multitask(subir_cuadro(), retrocede(90, 40))
    await seguir_linea_distancia(distancia_mm=50, vel_crucero_mm_s=300)

async def Ruta_3_recoger_ladrillos_azules():  # --> RUTA EN LISTA
    await giro(-90)
    await abrir_paletas() 
    await retrocede(-89, 180)
    await cerrar_paletas()
    await retrocede_hasta_pared(-89, 40)
    hub.imu.reset_heading(-90)
    await avance(-90, 140)
    await seguir_linea_hasta_interseccion("centro", target_heading=-90, vel_crucero_mm_s=400, montarse=False)
    await giro(3)
    await retrocede(0, 250, freno= False)
    await retrocede_hasta_pared(0, 30)
    hub.imu.reset_heading(0)
    await avance(0, 110) 
    await giro_pivot(90)
    await retrocede(90, 200)
    await multitask(bajar_cuadro(), retrocede(90, 680, max_dc= 70))
    await avance(90, 100)
    await giro(120)
    await retrocede(120, 280)
    await giro(90)
    await retrocede(90, 400, freno= False)
    await retrocede_hasta_pared(90, 30)
    hub.imu.reset_heading(90)
    await avance(90, 150)
    await giro_pivot(45, pivote_izquierdo= False)
    await subir_cuadro()
    await giro_pivot(90, pivote_izquierdo= False)
    await giro(0)
    await retrocede_hasta_pared(0, 60)
    await avance_hasta_linea(0)
    hub.imu.reset_heading(0)

async def Ruta_ladrillos_verdes():
    await retrocede(-90, 150, freno=False)
    await giro_arco(180, 300, freno=False, adelante=False, velocidad_mm_s=500)
    await retrocede_hasta_pared(180, 50)
    hub.imu.reset_heading(180)
    await avance_hasta_linea(180, esperar_blanco=False, montarse=True, distancia_montaje=35, max_dc=60)
    await giro(-90)
    await abrir_paletas()
    await retrocede(-90, 205)
    await cerrar_paletas()
    await retrocede_hasta_pared(-90)
    hub.imu.reset_heading(-90)
    await avance(-90, 40)
    await giro(0)
    await retrocede(0, 110, 80)
    await bajar_cuadro()
    await retrocede(0, 110)
    await giro(90)
    await retrocede(91, 600)
    await retrocede_hasta_pared(90, 30)
    hub.imu.reset_heading(90)
    await avance(91, 35, 55)
    await subir_cuadro()
    await retrocede(90, 25)
    await avance(90, 150, freno=False)
    await giro_arco(-90, 200, lado="derecha", velocidad_mm_s=500)
    await seguir_linea_distancia(150, 300)
    await seguir_linea_distancia(200)
    await seguir_linea_hasta_interseccion("centro", 300, montarse=True, distancia_montaje=35)
    await giro(180)
    await retrocede(180, 190)

async def Ruta_blancos():
    await avance(180, 300, freno=False)
    await avance_hasta_linea(180)
    await giro(90)
    await seguir_linea_distancia(1200, freno=False)


    await seguir_linea_hasta_interseccion("centro", vel_crucero_mm_s=450)
    await giro_pivot(145, pivote_izquierdo=False)
    await giro_pivot(90, pivote_izquierdo=True)
    await giro(-90)
    await abrir_paletas()
    await retrocede(-90, 200)
    await cerrar_paletas()
    await retrocede_hasta_pared(-90)
    await avance_hasta_linea(-91)
    await giro(-45)
    await avance(-45, 600)
    await avance_hasta_linea(-45, montarse=True, distancia_montaje=25)
    await giro(-90)
    await seguir_linea_hasta_interseccion("centro", montarse=True, distancia_montaje=25)
    await avance_hasta_pared(-90, 50)


#async def Ruta_3__Comerce_Azules():
# ---------------------------------------------------------
# RUTA DE PRUEBA DE LA LÓGICA (lectura ficticia con rojo)
# Posición inicial: línea de abajo, lado izquierdo (abajo), mirando a 0.
# ---------------------------------------------------------
async def Ruta_prueba_logica():
    await bajar_elevador ()
    global lectura, posicion_actual, lado_posicion_actual, llevadas, colores
    hub.imu.reset_heading(0)

    # 1 = Amarillo, 2 = Azul, 3 = Verde, 4 = Blanco, 5 = Rojo
    # Recorrido esperado: amarillo -> azul -> descarga -> rojo -> verde
    lectura = [1, 2, 4, 5, 4, 4 ]   # Lectura ficticia, cambiarla para probar otros casos

    colores = {1: 6, 2: 6, 3: 6, 4: 6}   # Reinicia la cantidad de bloques por color
    posicion_actual = 0                  # 0 = línea de abajo (inicio)
    lado_posicion_actual = 0             # 0 = lado izquierdo (abajo)
    llevadas = 0

    await resolver_lecturas()

async def prueba_motores():
    while True:

        left_motor.dc(40)
        right_motor.dc(40)


# Inicio del Run principal

async def Run():
    voltaje = (hub.battery.voltage() / 1000.0)
    print(voltaje)
    if float(voltaje) <= 7.9 or voltaje > 8.3:
        await hub.speaker.beep(1000, 50) 
    await hub.speaker.beep(1000, 50)
    hub.imu.reset_heading(0) 
    await seguir_linea_hasta_interseccion("derecha",100)
    await boton ()
    #await multitask (subir_elevador_todo (),subir_cuadro())
    await Ruta_prueba_logica ()
    #await Ruta_blancos()
    #await Ruta_ladrillos_verdes()
    #await prueba_motores()
    # await Ruta_1_Lectura()
    # await Ruta_2_Recoger_y_llevar_cemento()
    # await Ruta_3_recoger_ladrillos_azules()
    #await medir_trayectos_manual()  # Mide los trayectos y los guarda en la memoria del hub
    #await Ruta_prueba_logica()

"""
    while True:
        await motor_dc_por_segundos (right_motor,2000,100)
        await boton ()
        await motor_dc_por_segundos (right_motor,2000,-100)
        await boton ()"""

#iniciar_esp32()   # debe ir ANTES de run_task
# race=True: el programa termina cuando Run() termina (proceso_esp32 corre para siempre)
run_task (Run())
#Wrun_task(multitask(Run(), proceso_esp32(), race=True))