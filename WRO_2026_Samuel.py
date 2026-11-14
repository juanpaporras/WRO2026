from pybricks.tools import wait, run_task, multitask
from urandom import randint, seed

# Importación de módulos propios organizados
from GyroMovement import hub, left_motor, right_motor, avance, retrocede, retrocede_hasta_pared, avance_hasta_pared, giro, giro_pivot, giro_arco, reset_heading_seguro, avance_preciso
from Line_sensors import diagnostico_espectral_mbc, seguir_linea_distancia, seguir_linea_hasta_interseccion, avance_hasta_linea, seguir_linea_contando_lineas, Registro, retroceder_hasta_linea
from Actuators import (
    motorElevador, motorgarra,
    subir_elevador, bajar_elevador,
    bajar_cuadro, subir_cuadro, cerrar_paletas, abrir_paletas,
    abrir_garra, cerrar_garra, cerrar_garra_poquito, abrir_garra_poquito,
    motor_dc_por_grados, motor_dc_por_segundos, bajar_cuadro_poquito
)
from Extras import boton, advertencia_bateria, medir_trayectos_manual
from Funciones_ESP32 import iniciar_esp32, proceso_esp32, leer_camara

# =============================================================================
# ESTADO DE LA LÓGICA (variables que cambian durante la ruta)
# Coordenadas: 0 = hacia arriba del tapete, 90 = zona de descarga, -90 = cuadrícula de bloques.
# Colores / líneas: 1 = Amarillo, 2 = Azul, 3 = Verde, 4 = Blanco, 5 = Rojo (0 = línea de inicio)
# =============================================================================
colores = {1: 6, 2: 6, 3: 6, 4: 6}   # Bloques que quedan de cada color

posicion_actual = 0          # Línea de color donde está el robot (0 = línea de inicio)
lado_posicion_actual = 0     # 0 es izquierda, 1 es derecha
heading_objetivo = 0         # Rumbo del último movimiento de logica_de_movimiento (0 o 180)
lectura = []                 # Dar valor antes de usar la lógica de resolver lecturas
i = 0                        # Índice de la lectura que se está resolviendo
llevadas = 0                 # Poner 0 antes de usar la lógica

# Cámara (lectura_camara)
izquierda, derecha, lado_rojo = [], [], -1

# =============================================================================
# LÓGICA DE MOVIMIENTO (seguir la línea vertical contando líneas)
# =============================================================================
OFFSET_SEGUIDOR = 6           # Calibrar: offset del borde (izquierda = -5, derecha = +5)
VEL_MOVIMIENTO = 450          # Calibrar: velocidad (mm/s) siguiendo la línea vertical
VEL_APROX_MOVIMIENTO = 250    # Calibrar: velocidad (mm/s) desde la penúltima línea hasta la línea objetivo
DISTANCIA_MONTAJE = 78        # Calibrar: mm que avanza después de la línea cuando tiene que pasarse al otro lado (si es mucho, luego no encuentra la línea al seguirla)
MONTAJE_SIN_CAMBIO = 18       # Calibrar: mm que avanza después de la línea cuando NO tiene que pasarse de lado
# Cambiar de lado en la MISMA línea = la diferencia entre los dos montajes (pasa de una fila a la otra)
DISTANCIA_CAMBIO_LADO = DISTANCIA_MONTAJE - MONTAJE_SIN_CAMBIO   # Calibrar: ~60 mm
DC_CAMBIO_LADO = 60           # Calibrar: potencia del avance de cambio de lado

# =============================================================================
# LÓGICA DE RECOLECCIÓN (giro a -90, avance recto, agarrar, retroceder lo mismo)
# =============================================================================
DISTANCIA_PRIMERA_FILA = 110  # Calibrar: mm a -90 desde la línea vertical hasta agarrar un bloque de la 1ª fila
PASO_COLUMNA = 32             # Calibrar: mm extra por cada fila de bloques más al fondo
DC_RECOLECCION = 55           # Calibrar: potencia MÁXIMA del avance a -90 (al llegar al bloque igual baja a min_dc=30 por la rampa de frenado de avance)
DC_RETROCESO_RECOLECCION = 85 # Calibrar: potencia del retroceso con el bloque (si el bloque se suelta al arrancar, bajarla)

# =============================================================================
# LLEVAR A DESCARGA
# =============================================================================
DESCARGA_AVANCE = 120         # Calibrar: del carril hasta la zona de descarga (mirando a 90)
DESCARGA_ADENTRO = 2          # Calibrar: mm hacia adentro (3 izq -> hacia 180, 2 der -> hacia 0) en las descargas con llevadas 0 y 1. Negativo = más hacia AFUERA (lejos de la línea central)
DESCARGA_UN_BLOQUE = 30       # Calibrar: montaje EXTRA en la descarga con llevadas == 2 (junta los 2 últimos con los otros 4)
VEL_REFERENCIA_DESCARGA = 100 # Calibrar: mm/s al retroceder buscando la línea (más lento = detecta en un punto más exacto)
VEL_AJUSTE_DESCARGA = 150     # Calibrar: mm/s del avance fijo después de detectar la línea

# =============================================================================
# AGARRAR ROJO (cuadro gris arriba a la derecha, desde la posición 4 lado izquierdo)
# =============================================================================
ROJO_AVANCE_50 = 120          # Calibrar: mm de avance a 56° hasta que la garra quede sobre el rojo

# =============================================================================
# CÁMARA
# =============================================================================
async def lectura_camara():
    global izquierda, derecha, lado_rojo
    izquierda, derecha, lado_rojo, _, _, _ = await leer_camara()


async def logica_de_recoleccion():
    # Desde la línea vertical: gira a -90, avanza recto (gyro) hasta la fila del bloque,
    # agarra y retrocede EXACTAMENTE lo mismo para volver a la línea vertical.
    await multitask (cerrar_garra_poquito(), giro(-90))
    avanzar =(6 - colores[lectura[i]]) // 2 #Calcula cuantas filas de bloques hay que avanzar para llegar al bloque objetivo
    distancia = DISTANCIA_PRIMERA_FILA + PASO_COLUMNA * avanzar   #Calibrar: PASO_COLUMNA = distancia entre filas de bloques
    await multitask(avance(-90, distancia, DC_RECOLECCION), abrir_garra())
    await cerrar_garra_poquito()
    await retrocede(-90, distancia, max_dc=DC_RETROCESO_RECOLECCION, accel_mm=80) # Mismo valor que avanzó: vuelve a la línea vertical
    colores[lectura[i]] -= 1 #Resta uno a la cantidad de bloques del color objetivo

async def logica_de_movimiento(objetivo, lado_posicion_objetivo, distancia_montaje=None, montaje_extra=0):
    # distancia_montaje=None: usa DISTANCIA_MONTAJE / MONTAJE_SIN_CAMBIO según el lado.
    # Si se da un valor, se monta exactamente esa distancia.
    # montaje_extra: mm que se SUMAN al montaje (ej. descarga con llevadas == 2).
    global heading_objetivo, lado_posicion_actual, posicion_actual
    # Se mueve SIGUIENDO la línea vertical por un costado y contando las líneas de color:
    # mirando a 0 por la izquierda (offset -5), mirando a 180 por la derecha (offset +5).
    movimiento = abs(objetivo)
    if objetivo > 0: # Si el objetivo es mayor que 0, gira a la derecha
        await giro(0)
        heading_objetivo=0
    elif objetivo < 0: # Si el objetivo es menor que 0, gira a la izquierda:
        await giro(180)
        heading_objetivo=180

    if objetivo != 0: #si la distancia es diferente de 0, sigue la línea contando líneas SIN frenar en las intermedias
        # En la última línea: se monta (pasa al otro lado) o se queda justo en ella
        montarse_final = (lado_posicion_objetivo==0 and heading_objetivo == 0) or (lado_posicion_objetivo!=0 and heading_objetivo == 180)
        # Solo se monta si tiene que pasarse al otro lado de la línea; si no, frena apenas la detecta.
        if distancia_montaje is None:
            distancia_montaje = DISTANCIA_MONTAJE if montarse_final else MONTAJE_SIN_CAMBIO
        distancia_montaje += montaje_extra
        lado_seguidor = "izquierda" if heading_objetivo == 0 else "derecha"
        await multitask (seguir_linea_contando_lineas(
            lado_seguidor, movimiento, heading_objetivo,
            offset=OFFSET_SEGUIDOR, distancia_ignorar=50,  # Calibrar distancia_ignorar: no cuenta la línea de donde sale
            montarse=True, distancia_montaje=distancia_montaje,  # Calibrar: distancia para pasar la línea y alinearse con los bloques del otro lado
            vel_crucero_mm_s=VEL_MOVIMIENTO, vel_aproximacion_mm_s=VEL_APROX_MOVIMIENTO
        ), abrir_garra())

        if lado_posicion_objetivo==0:
            lado_posicion_actual = 1
        else:
            lado_posicion_actual = 0
        posicion_actual += objetivo
    else:
        # Misma línea, cambiar de lado: avance recto con gyro de DISTANCIA_CAMBIO_LADO.
        # NO se cuenta la línea: el sensor va ~50 mm delante del eje, así que al girar 180° el
        # sensor ya quedó DEL OTRO LADO de la línea y contaba la siguiente (bajaba/subía una de más).
        if lado_posicion_objetivo == 0 and lado_posicion_actual == 0: #Si tiene que ir a la derecha y el robot está en el lado izquierdo
            await giro(0)
            heading_objetivo = 0
            await multitask (avance(0, DISTANCIA_CAMBIO_LADO + montaje_extra, max_dc=DC_CAMBIO_LADO), abrir_garra()) #Calibrar
            lado_posicion_actual = 1

        elif lado_posicion_objetivo != 0 and lado_posicion_actual == 1: #Si tiene que ir a la izquierda y el robot está en el lado derecho
            await giro(180)
            heading_objetivo = 180
            await multitask (avance(180, DISTANCIA_CAMBIO_LADO + montaje_extra, max_dc=DC_CAMBIO_LADO), abrir_garra()) #Calibrar
            lado_posicion_actual = 0

        elif montaje_extra > 0:
            print("logica_de_movimiento: ya está en la posición y lado, NO se aplica el montaje_extra de", montaje_extra)

async def llevar_a_descarga():
    global posicion_actual, lado_posicion_actual
    # lado_posicion_objetivo: diferente de 0 = lado IZQUIERDO, 0 = lado DERECHO
    # montaje_extra = mm que queda MÁS HACIA ADENTRO (hacia la rama de descarga entre la 2 y la 3):
    # en la 3 (izquierda) hacia 180, en la 2 (derecha) hacia 0.
    if llevadas == 0:
        destino, lado_posicion_objetivo = 3, 1      # posición 3, lado izquierdo
        montaje_extra = DESCARGA_ADENTRO
    elif llevadas == 1:
        destino, lado_posicion_objetivo = 2, 0      # posición 2, lado derecho
        montaje_extra = DESCARGA_ADENTRO
    else:
        # llevadas == 2: va a la más cercana y se monta un bloque extra (al centro, con los otros 4)
        montaje_extra = DESCARGA_UN_BLOQUE
        if posicion_actual >= 3:
            destino, lado_posicion_objetivo = 3, 1  # posición 3, lado izquierdo
        else:
            destino, lado_posicion_objetivo = 2, 0  # posición 2, lado derecho

    # 1) Llega cerca de la línea de descarga (si ya está en esa línea, no se mueve:
    #    la re-referencia de abajo lo deja en el lado correcto venga del lado que venga)
    objetivo = destino - posicion_actual
    if objetivo != 0:
        await logica_de_movimiento(objetivo, lado_posicion_objetivo)
    await cerrar_garra_poquito()   # sujeta los bloques antes de girar y retroceder

    # 2) RE-REFERENCIA (misma posición final venga por donde venga):
    #    mira a 0 (línea 3) o a 180 (línea 2), retrocede despacio hasta que el sensor ve la línea
    #    de color y avanza una distancia fija con control de velocidad.
    #    El sensor va ~50 mm delante del eje, así que en cualquier punto cerca de la posición de
    #    descarga la línea queda DETRÁS del sensor y retrocediendo siempre la encuentra.
    #    "Hacia adentro" es hacia atrás en los dos casos, por eso el extra se RESTA.
    heading_ref = 0 if destino == 3 else 180
    await giro(heading_ref)
    await retroceder_hasta_linea(heading_ref, vel_mm_s=VEL_REFERENCIA_DESCARGA)
    await avance_preciso(MONTAJE_SIN_CAMBIO - montaje_extra, vel_mm_s=VEL_AJUSTE_DESCARGA) #Calibrar
    posicion_actual = destino
    lado_posicion_actual = 0 if lado_posicion_objetivo != 0 else 1

    await giro(90, 80)
    if llevadas == 2:
        # Ya está sobre la línea: no hace falta avance_hasta_linea (no tenía montaje)
        await abrir_garra_poquito ()
        await seguir_linea_distancia (distancia_mm=DESCARGA_AVANCE+100, vel_crucero_mm_s=100) #Calibrar para que el robot avance hasta la línea de descarga y se monte sobre ella
        await retrocede (90,22,50)
        await cerrar_garra()
    elif llevadas == 1:
       montaje_descarga = DESCARGA_AVANCE-20   # Calibrar: lo que se monta en la descarga con llevadas == 1
       await multitask( abrir_garra(),  avance(90, montaje_descarga, max_dc=60)) # Ya está en la línea: avanza directo la distancia del montaje
       await multitask (giro (88), cerrar_garra_poquito())
    else:
        montaje_descarga = DESCARGA_AVANCE      # Calibrar: lo que se monta en la descarga con llevadas == 0
        await multitask (abrir_garra(), avance(90, montaje_descarga, max_dc=60)) #Calibrar: ya está en la línea, avanza directo la distancia del montaje hasta la zona de descarga
        await multitask (giro (92),cerrar_garra_poquito())
    if llevadas != 2:
        await giro(90,40)
        # Retrocede lo MISMO que se montó en cada caso (menos el ajuste de -5)
        await multitask(abrir_garra(),retrocede(90, montaje_descarga-5) )#Calibrar: retrocede lo que avanzó para volver a la línea vertical
        # Ya no va a la pared: queda sobre la línea vertical y sigue con la logica de movimiento
        if llevadas == 0:
            posicion_actual = 3
            lado_posicion_actual = 0 #Posición 3, lado izquierdo
        else:
            posicion_actual = 2
            lado_posicion_actual = 1 #Posición 2, lado derecho

async def agarrar_rojo():
    global posicion_actual, lado_posicion_actual
    objetivo = 4 - posicion_actual
    lado_posicion_objetivo = 3 #Va a la posicion 4 y lado izquierdo porque 3 != 0
    await logica_de_movimiento(objetivo, lado_posicion_objetivo)
    await cerrar_garra_poquito()
    # Ya está sobre la línea vertical (llegó siguiéndola): gira directo hacia el rojo
    await giro(56)
    await multitask (abrir_garra(), avance(56, ROJO_AVANCE_50, max_dc=70)) #Calibrar para que el robot avance hasta agarrar el rojo
    await cerrar_garra_poquito()
    await retrocede(56, ROJO_AVANCE_50+ 10) #Calibrar para que el robot retroceda lo mismo que avanzó para agarrar el rojo
    posicion_actual = 4 #El robot está en la posición 4 y lado izquierdo
    lado_posicion_actual = 0 #El robot está en la posición 4 y lado izquierdo

def color_menos_agarrado():
    """Devuelve el color (1-4) que MÁS bloques tiene todavía (el menos agarrado), porque sus
    bloques quedan en las primeras filas y es más seguro agarrarlos que los del fondo.
    Empate: el más cercano a posicion_actual. None si no queda ningún bloque."""
    mejor = None
    for c in (1, 2, 3, 4):
        if colores[c] == 0:
            continue
        if (mejor is None or colores[c] > colores[mejor]
                or (colores[c] == colores[mejor] and abs(c - posicion_actual) < abs(mejor - posicion_actual))):
            mejor = c
    return mejor

async def   resolver_lecturas():
    global i, llevadas
    contador = 0
    for i in range(len(lectura)): #Recorre la lista de lecturas
        if llevadas == 3: #Si ya llevó 3 bloques termina la función y no hace nada más
            break
        if lectura[i] < 5 and colores[lectura[i]] == 0: #Si el color objetivo ya no tiene bloques, lo cambia por el color MENOS agarrado (bloques en las primeras filas = más seguro)
            reemplazo = color_menos_agarrado()
            if reemplazo is None:
                print("No quedan bloques de ningún color")
                break
            print("Color", lectura[i], "sin bloques -> se reemplaza por", reemplazo)
            lectura[i] = reemplazo

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
async def dejar_bloques(heading):   
    await motor_dc_por_grados(motorElevador, 140, -100)
    await abrir_garra()
    await motor_dc_por_grados (motorgarra,9,80)
    await boton()
    await retrocede (heading,25,50)
    await boton()
    await motor_dc_por_segundos (motorgarra,300,-100)
    await retrocede (heading,17,50)
    await motor_dc_por_grados (motorgarra,9,80)
    await retrocede (heading,4,50)
    await multitask (cerrar_garra_poquito(),motor_dc_por_segundos(motorElevador, 100, -50))
    await retrocede (heading,3,50)
    await abrir_garra()
    await motor_dc_por_grados (motorgarra,9,80)


# ---------------------------------------------------------
# RUTAS DE COMPETICIÓN 
# ---------------------------------------------------------


async def Ruta_1_Lectura(): # --> RUTA EN LISTA #Se acomoda de la salida y sigue linea hasta interseccion para llegar a leer, la distancia de montaje debe ser para que quede en el centro de la linea, gire se tire hacia atras y choque con el mosaico. Ahi ya hace la lectura
    #await avance(0, 100, 80)
    #await giro(90)

    #await avance(0, 15, 80, freno=False)
    #await giro_pivot(90, pivote_izquierdo=False)

    await multitask(giro_arco(90, 90, freno=False), subir_cuadro(), cerrar_paletas())
    await avance(90, 200, 80,freno=False)
    await seguir_linea_distancia(distancia_mm=500, vel_crucero_mm_s=700)
    await seguir_linea_hasta_interseccion("centro", target_heading=90, vel_crucero_mm_s=400, montarse=True, distancia_montaje=41)
    await giro(0)
    await seguir_linea_hasta_interseccion("centro", target_heading=0, vel_crucero_mm_s=200)
    reset_heading_seguro(0)
    await giro(-179,80)
    await retrocede_hasta_pared(180, distancia_max=1000, dc=50)
    reset_heading_seguro(180)

async def Ruta_2_Recoger_y_llevar_cemento():      # --> RUTA EN LISTA
    await avance(180, 150, 80)
    await seguir_linea_distancia(distancia_mm=75, vel_crucero_mm_s=300)
    await multitask (giro(1),bajar_cuadro_poquito())
    await bajar_cuadro(300)
    await giro(90)
    await seguir_linea_distancia(distancia_mm=550, vel_crucero_mm_s=500, freno=False)
    await giro_pivot(0, pivote_izquierdo=True, freno=False)
    await avance(0, 10, freno=False)
    await giro_pivot(90, pivote_izquierdo=False)
    await multitask(bajar_cuadro_poquito(), retrocede(90, 30))
    await multitask (seguir_linea_distancia(distancia_mm=100, vel_crucero_mm_s=250),subir_cuadro(200))

async def Ruta_3_recoger_ladrillos_azules():  # --> RUTA EN LISTA
    await multitask(giro(-89),abrir_paletas() )
    await retrocede(-88, 150, 40,freno=False)
    await multitask(cerrar_paletas(), retrocede(-89, 40, 40))
    await retrocede_hasta_pared(-89, 60)
    reset_heading_seguro(-90)
    await avance(-90, 140)
    await seguir_linea_hasta_interseccion("centro", target_heading=-90, vel_crucero_mm_s=400, montarse=False)
    await giro(3)
    await retrocede(0, 200, freno= False)
    await retrocede_hasta_pared(0, 40)
    reset_heading_seguro(0)
    await avance(0, 120) 
    await giro_pivot(90)
    await retrocede(90, 200)
    await multitask(bajar_cuadro(), retrocede(90, 680, max_dc= 70))
    await avance(90, 100)
    await giro(120)
    await retrocede(120, 280)
    await giro(90)
    await retrocede(90, 350, freno= False)
    await retrocede_hasta_pared(90, 30)
    reset_heading_seguro(90)
    await avance(90, 150)
    await giro_pivot(67.5, pivote_izquierdo= False)
    await subir_cuadro()
    await giro_pivot(90, pivote_izquierdo= False)
    await avance(90, 35)
    await giro(0)
    
    await bajar_elevador()
    
async def Ruta_5_dejar_bloques_y_azules():
    await seguir_linea_hasta_interseccion ("centro", 300)
    await multitask(subir_elevador(), avance_hasta_pared(90, 40))
    reset_heading_seguro(90)
    await dejar_bloques(90)
    await retrocede(90,150)
    await giro(-90)
    await multitask(retrocede(-90, 170,60), abrir_paletas())
    await avance(-90, 150)
    await multitask(cerrar_paletas(), seguir_linea_hasta_interseccion("centro"))

async def Ruta_ladrillos_verdes():
    await retrocede(-90, 150, freno=False)
    await giro_arco(180, 300, freno=False, adelante=False, velocidad_mm_s=500)
    await retrocede_hasta_pared(180, 50)
    reset_heading_seguro(180)
    await avance_hasta_linea(180, esperar_blanco=False, montarse=True, distancia_montaje=35, max_dc=60)
    await giro(-90)
    await abrir_paletas()
    await retrocede(-90, 205)
    await cerrar_paletas()
    await retrocede_hasta_pared(-90)
    reset_heading_seguro(-90)
    await avance(-90, 40)
    await giro(0)
    await retrocede(0, 110, 80)
    await bajar_cuadro()
    await retrocede(0, 110)
    await giro(90)
    await retrocede(91, 600)
    await retrocede_hasta_pared(90, 30)
    reset_heading_seguro(90)
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


# ---------------------------------------------------------
# RUTA DE PRUEBA DE LA LÓGICA (lectura ficticia con rojo)
# Posición inicial: línea de abajo, lado izquierdo (abajo), mirando a 0.
# ---------------------------------------------------------
async def Ruta_prueba_logica():
    global lectura, posicion_actual, lado_posicion_actual, llevadas, colores
    hub.imu.reset_heading(0)

    # 1 = Amarillo, 2 = Azul, 3 = Verde, 4 = Blanco, 5 = Rojo
    # Recorrido esperado: amarillo -> azul -> descarga -> rojo -> verde
    # Lectura ficticia ALEATORIA: un 5 (rojo) en una posición al azar y el resto colores del 1 al 4
    # Semilla con el "ruido" del acelerómetro y la batería para que cada corrida sea distinta
    ax, ay, az = hub.imu.acceleration()
    seed(int(abs(ax) * 1000 + abs(ay) * 100 + abs(az) * 10) + hub.battery.voltage() + hub.battery.current())
    # Un color al azar queda con 0 bloques y la lectura lo pide AL MENOS una vez,
    # para probar el reemplazo por el color menos agarrado (color_menos_agarrado)
    color_vacio = randint(1, 4)
    lectura = [randint(1, 4) for _ in range(6)]
    pos_rojo = randint(0, 5)
    lectura[pos_rojo] = 5
    pos_vacio = randint(0, 4)
    if pos_vacio >= pos_rojo:
        pos_vacio += 1          # posición distinta a la del rojo
    lectura[pos_vacio] = color_vacio
    print("Lectura aleatoria:", lectura, "| color con 0 bloques:", color_vacio)
    # lectura = [4, 5, 2, 3, 3, 2 ]   # Lectura fija, descomentar para repetir un caso concreto

    colores = {1: 6, 2: 6, 3: 6, 4: 6}   # Reinicia la cantidad de bloques por color
    colores[color_vacio] = 0             # Comentar esta línea si usas la lectura fija
    posicion_actual = 0                  # 0 = línea de abajo (inicio)
    lado_posicion_actual = 0             # 0 = lado izquierdo (abajo)
    llevadas = 0

    await resolver_lecturas()

# ---------------------------------------------------------
# PRUEBA DEL SEGUIDOR CON REGISTRO DE DATOS
# Posición inicial: línea de abajo (0), sobre la línea vertical, mirando a 0.
# Sube siguiendo la línea hasta la 4 y baja hasta la 0.
# Al final imprime un RESUMEN corto por tramo (mandar captura de la consola para analizarlo).
# ---------------------------------------------------------
async def Ruta_prueba_seguidor():
    hub.imu.reset_heading(0)
    reg = Registro()
    await seguir_linea_contando_lineas("izquierda", 4, 0, offset=OFFSET_SEGUIDOR,
        montarse=True, distancia_montaje=MONTAJE_SIN_CAMBIO,
        vel_crucero_mm_s=VEL_MOVIMIENTO, vel_aproximacion_mm_s=VEL_APROX_MOVIMIENTO, registro=reg)
    await giro(180)
    await seguir_linea_contando_lineas("derecha", 4, 180, offset=OFFSET_SEGUIDOR,
        montarse=True, distancia_montaje=MONTAJE_SIN_CAMBIO,
        vel_crucero_mm_s=VEL_MOVIMIENTO, vel_aproximacion_mm_s=VEL_APROX_MOVIMIENTO, registro=reg)
    print("Heading final:", hub.imu.heading())
    await reg.resumen()      # Resumen corto (cabe en la consola)

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
    # Espera a que el gyro termine de calibrar su bias inicial (NO mover el robot hasta el beep)
    espera_imu = 0
    while not hub.imu.ready() and espera_imu < 3000:   # máximo 3 s de seguridad
        await wait(10)
        espera_imu += 10
    if not hub.imu.ready():
        print("ADVERTENCIA: el giroscopio no terminó de calibrar (¿se movió el robot al iniciar?)")
    hub.imu.reset_heading(0)
    await hub.speaker.beep(1500, 50)   # beep de "giroscopio listo"
    await boton()
    await advertencia_bateria(boton)   # antes recibía None y fallaba si la batería estaba fuera de rango
    #await Ruta_blancos()
    #await Ruta_ladrillos_verdes()
    #await prueba_motores()
    #await boton()
    # await Ruta_1_Lectura()
    # await Ruta_2_Recoger_y_llevar_cemento()
    # await Ruta_3_recoger_ladrillos_azules()
    # await cerrar_paletas()
    await Ruta_prueba_logica()
    # await Ruta_5_dejar_bloques_y_azules()
    #await medir_trayectos_manual()  # Mide los trayectos y los guarda en la memoria del hub
    #await Ruta_prueba_logica()
    #await Ruta_prueba_seguidor()     # Prueba del seguidor con registro: imprime un resumen al final

iniciar_esp32()   # debe ir ANTES de run_task
#race=True: el programa termina cuando Run() termina (proceso_esp32 corre para siempre)
#run_task (Run())
run_task(multitask(Run(), proceso_esp32(), race=False))
