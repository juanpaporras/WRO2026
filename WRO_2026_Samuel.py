from pybricks.tools import wait, run_task, multitask
from urandom import randint, seed

# Importación de módulos propios organizados
from GyroMovement import hub, left_motor, right_motor, avance, retrocede, retrocede_hasta_pared, avance_hasta_pared, giro, giro_pivot, giro_arco, reset_heading_seguro, avance_preciso, agarrar_derecha,agarrar_izquierda
from Line_sensors import diagnostico_espectral_mbc, seguir_linea_distancia, seguir_linea_hasta_interseccion, avance_hasta_linea, seguir_linea_contando_lineas, Registro, retroceder_hasta_linea, retrocede_hasta_linea
from Actuators import (
    motorElevador, motorgarra,
    subir_elevador, subir_elevador_todo, bajar_elevador,
    bajar_cuadro, subir_cuadro, cerrar_paletas, abrir_paletas,
    abrir_garra, cerrar_garra, cerrar_garra_poquito, abrir_garra_poquito,
    motor_dc_por_grados, motor_dc_por_segundos, bajar_cuadro_poquito,motor_dc_hasta_tope,bajar_elevador_grados
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
ROJO_LECTURA = 5             # Valor del rojo en las listas de la cámara
color_mayoria = None         # Color del lado mayoría (lo calcula reservar_mayoria una sola vez)

# =============================================================================
# LÓGICA DE MOVIMIENTO (seguir la línea vertical contando líneas)
# =============================================================================
OFFSET_SEGUIDOR = 6           # Calibrar: offset del borde (izquierda = -5, derecha = +5)
VEL_MOVIMIENTO = 450          # Calibrar: velocidad (mm/s) siguiendo la línea vertical
VEL_APROX_MOVIMIENTO = 300    # Calibrar: velocidad (mm/s) desde la penúltima línea hasta la línea objetivo
DISTANCIA_MONTAJE = 85        # Calibrar: mm que avanza después de la línea cuando tiene que pasarse al otro lado (si es mucho, luego no encuentra la línea al seguirla)
MONTAJE_SIN_CAMBIO = 18       # Calibrar: mm que avanza después de la línea cuando NO tiene que pasarse de lado
# Cambiar de lado en la MISMA línea = la diferencia entre los dos montajes (pasa de una fila a la otra)
EXTRA_MONTAJE_UNA_LINEA = 3   # Calibrar: mm que se suman al montaje cuando el movimiento es de UNA sola línea (va lento y se queda corto)
DISTANCIA_CAMBIO_LADO = DISTANCIA_MONTAJE - MONTAJE_SIN_CAMBIO   # Calibrar: ~60 mm
DC_CAMBIO_LADO = 60           # Calibrar: potencia del avance de cambio de lado

# =============================================================================
# LÓGICA DE RECOLECCIÓN (giro a -90, avance recto, agarrar, retroceder lo mismo)
# =============================================================================
DISTANCIA_PRIMERA_FILA = 94  # Calibrar: mm a -90 desde la línea vertical hasta agarrar un bloque de la 1ª fila
PASO_COLUMNA = 45            # Calibrar: mm extra por cada fila de bloques más al fondo
DC_RECOLECCION = 55           # Calibrar: potencia MÁXIMA del avance a -90 (al llegar al bloque igual baja a min_dc=30 por la rampa de frenado de avance)
DC_RECOLECCION_RECTO = 40     # Calibrar: potencia del tramo recto a -90 en filas 2 y 3 (sin freno: con más llega al bloque más rápido que en la 1ª fila)
DC_RETROCESO_RECOLECCION = 85 # Calibrar: potencia del retroceso con el bloque (si el bloque se suelta al arrancar, bajarla)
RETROCESO_AGARRE = 120        # Calibrar: mm EXTRA de retroceso cuando lo que sigue es agarrar (logica_de_comer)
# Filas 2 y 3: avanza a -90 hasta quedar a 1 bloque y el último tramo lo hace en diagonal
HEADING_DIAGONAL_DERECHA = -88   # Calibrar: rumbo del último tramo si el robot está del lado DERECHO de la línea
HEADING_DIAGONAL_IZQUIERDA = -92 # Calibrar: rumbo del último tramo si el robot está del lado IZQUIERDO de la línea
PASO_DIAGONAL = PASO_COLUMNA     # Calibrar: mm del tramo en diagonal (en diagonal puede necesitar un poco más que PASO_COLUMNA)
# Retroceso SENSADO a la línea vertical (todas las posiciones menos 4 por la derecha, donde la línea está cortada)
FALTA_ANTES_DE_SENSAR = DISTANCIA_PRIMERA_FILA / 3   # Calibrar: mm que faltan para la línea cuando empieza a buscarla
DC_RETROCESO_LINEA = 40          # Calibrar: potencia del retroceso buscando la línea vertical
MIN_SENSORES_RETROCESO = 6       # Calibrar: sensores que deben ver la línea para detectarla
AVANCE_DESDE_LINEA = 72          # Calibrar: mm que avanza tras detectar la línea para quedar sobre ella (listo para leerla)
DC_AVANCE_DESDE_LINEA = 40       # Calibrar: potencia de ese avance

async def retroceso_a_linea(distancia, extra=0, rampa_inicio=True):
    """Retrocede a -90 hasta la línea vertical: 'distancia' es lo que avanzó recto a -90.
    Retrocede normal el tramo recto menos FALTA_ANTES_DE_SENSAR (1/3 de la 1ª fila), luego busca la línea retrocediendo y:
    - sin extra: avanza AVANCE_DESDE_LINEA para quedar sobre la línea.
    - con extra (sigue agarrar): retrocede extra - AVANCE_DESDE_LINEA desde donde detectó la línea
      (queda en el mismo punto que con el retroceso por distancia + extra)."""
    # freno=False + dc_final: no se detiene, baja la potencia hasta DC_RETROCESO_LINEA y pasa directo a buscar la línea
    await retrocede(-90, distancia - FALTA_ANTES_DE_SENSAR, max_dc=DC_RETROCESO_RECOLECCION, accel_mm=80, rampa_inicio=rampa_inicio, freno=False, dc_final=DC_RETROCESO_LINEA)
    await retrocede_hasta_linea(-90, max_dc=DC_RETROCESO_LINEA, min_sensores_linea=MIN_SENSORES_RETROCESO)
    if extra > 0:
        await retrocede(-90, extra - AVANCE_DESDE_LINEA, max_dc=DC_RETROCESO_RECOLECCION, accel_mm=80)
    else:
        await avance(-90, AVANCE_DESDE_LINEA, DC_AVANCE_DESDE_LINEA)

# =============================================================================
# LLEVAR A MOSAICO IZQUIERDA (antes: descarga con llevadas == 2)
# =============================================================================
DESCARGA_AVANCE = 220         # Calibrar: del carril hasta la zona de descarga (mirando a 90); el mosaico sigue la línea DESCARGA_AVANCE + 100
DESCARGA_UN_BLOQUE = 55       # Calibrar: mm hacia adentro (al centro, entre la línea 2 y la 3) antes de girar a 90
VEL_REFERENCIA_DESCARGA = 100 # Calibrar: mm/s al retroceder buscando la línea (más lento = detecta en un punto más exacto)
VEL_AJUSTE_DESCARGA = 150     # Calibrar: mm/s del avance fijo después de detectar la línea
BUSQUEDA_LINEA_MM = 130        # Calibrar: mm que re_referenciar_lado busca la línea hacia ATRÁS antes de buscarla adelante (menos que lo que falta a la línea vecina, ~155 mm entre líneas)

# =============================================================================
# AGARRAR ROJO (cuadro gris arriba a la derecha, desde la posición 4 lado izquierdo)
# =============================================================================
ROJO_AVANCE_50 = 130          # Calibrar: mm de avance a 56° hasta que la garra quede sobre el rojo

# =============================================================================
# CÁMARA
# =============================================================================
INTENTOS_CAMARA = 3                      # Calibrar: lecturas antes de usar el plan B (cada una espera hasta 3 s)
LECTURA_DEFECTO_IZQ = [1, 1, 2, 5, 3, 4] # Calibrar: lectura de respaldo si la cámara no lee nada
LECTURA_DEFECTO_DER = [1, 1, 2, 3, 4, 3] # Calibrar: lectura de respaldo si la cámara no lee nada

async def lectura_camara():
    """Lee la cámara hasta INTENTOS_CAMARA veces. Devuelve True si la lectura fue completa
    (6 + 6 valores y se vio el rojo). Si no, aplica un plan B para que las rutas no fallen:
    - Se leyeron las 12 casillas pero no el rojo -> lado_rojo = 0 (izquierda con lógica, derecha por mayoría).
    - No se leyó nada -> usa LECTURA_DEFECTO_IZQ / LECTURA_DEFECTO_DER."""
    global izquierda, derecha, lado_rojo
    for intento in range(INTENTOS_CAMARA):
        izquierda, derecha, lado_rojo, _, _, _ = await leer_camara()
        if len(izquierda) == 6 and len(derecha) == 6 and lado_rojo != -1:
            print("Cámara:", izquierda, derecha, "| lado_rojo:", lado_rojo)
            return True
        print("Cámara: lectura incompleta, intento", intento + 1, "de", INTENTOS_CAMARA)

    if len(izquierda) == 6 and len(derecha) == 6:
        lado_rojo = 0
        print("Cámara: no se vio el rojo -> se usa lado_rojo = 0")
    else:
        izquierda, derecha, lado_rojo = list(LECTURA_DEFECTO_IZQ), list(LECTURA_DEFECTO_DER), 0
        print("Cámara: SIN lectura -> se usa la lectura de respaldo")
    return False

async def logica_de_recoleccion(sigue_agarrar=False, fila=None):
    # fila=None: la fila sale de colores[lectura[i]] y se descuenta el bloque (resolver_lecturas).
    # fila=0, 1, 2: usa esa fila directamente y NO toca colores (agarrar_mayoria: el color está reservado en 0).
    # Desde la línea vertical: gira a -90, avanza recto (gyro) hasta la fila del bloque,
    # agarra y retrocede EXACTAMENTE lo mismo para volver a la línea vertical.
    # sigue_agarrar=True: después viene logica_de_comer -> retrocede RETROCESO_AGARRE de más.
    # En la línea 4 NO se suma aquí: logica_de_comer primero baja a la 3 siguiendo la línea
    # (necesita estar sobre ella) y suma el retroceso allá.
    await multitask (cerrar_garra_poquito(), giro(-90,70))
    # 4 por la derecha: la línea vertical está cortada -> retroceso por distancia (como antes)
    sensado = not (posicion_actual == 4 and lado_posicion_actual == 1)
    if fila is None:
        avanzar =(6 - colores[lectura[i]]) // 2 #Calcula cuantas filas de bloques hay que avanzar para llegar al bloque objetivo
    else:
        avanzar = fila
    extra = RETROCESO_AGARRE if (sigue_agarrar and posicion_actual != 4) else 0
    if avanzar == 0:
        # 1ª fila: todo recto a -90
        distancia = DISTANCIA_PRIMERA_FILA
        await multitask(avance(-90, distancia, DC_RECOLECCION), abrir_garra())
        await cerrar_garra_poquito()
        if sensado:
            await retroceso_a_linea(distancia, extra)
        else:
            await retrocede(-90, distancia + extra, max_dc=DC_RETROCESO_RECOLECCION, accel_mm=80) # Mismo valor que avanzó: vuelve a la línea vertical (+ extra si sigue agarrar)
    else:
        # 2ª y 3ª fila: recto a -90 hasta quedar a 1 bloque, y el último tramo en diagonal
        # (-85 desde el lado derecho, -95 desde el izquierdo). El retroceso deshace los dos tramos al revés.
        heading_diagonal = HEADING_DIAGONAL_DERECHA if lado_posicion_actual == 1 else HEADING_DIAGONAL_IZQUIERDA
        distancia_recta = DISTANCIA_PRIMERA_FILA + PASO_COLUMNA * (avanzar - 1)   #Calibrar: PASO_COLUMNA = distancia entre filas de bloques
        await multitask(avance(-90, distancia_recta, DC_RECOLECCION_RECTO, freno=False), abrir_garra())
        await avance(heading_diagonal, PASO_DIAGONAL, DC_RECOLECCION, rampa_inicio=False)
        await cerrar_garra_poquito()
        await retrocede(heading_diagonal, PASO_DIAGONAL, max_dc=DC_RECOLECCION, accel_mm=80, freno=False)
        if sensado:
            await retroceso_a_linea(distancia_recta, extra, rampa_inicio=False)
        else:
            await retrocede(-90, distancia_recta + extra, max_dc=DC_RETROCESO_RECOLECCION, rampa_inicio=False) # vuelve a la línea vertical (+ extra si sigue agarrar)
    if fila is None:
        colores[lectura[i]] -= 1 #Resta uno a la cantidad de bloques del color objetivo

async def logica_de_movimiento(objetivo, lado_posicion_objetivo, distancia_montaje=None, montaje_extra=0):
    # distancia_montaje=None: usa DISTANCIA_MONTAJE / MONTAJE_SIN_CAMBIO según el lado.
    # Si se da un valor, se monta exactamente esa distancia.
    # montaje_extra: mm que se SUMAN al montaje (ej. descarga con llevadas == 2).
    global heading_objetivo, lado_posicion_actual, posicion_actual
    # CASO ESPECIAL: 4 por la DERECHA (lado_posicion_objetivo == 0). No se llega siguiendo la línea:
    # va a 4 por la izquierda, gira a 180, retrocede hasta la pared, resetea el heading a 180,
    # avanza hasta la línea y se monta MONTAJE_SIN_CAMBIO (queda del lado de la pared = derecha).
    ya_en_4_derecha = (objetivo == 0 and posicion_actual == 4 and lado_posicion_actual == 1)
    if posicion_actual + objetivo == 4 and lado_posicion_objetivo == 0 and not ya_en_4_derecha:
        if not (objetivo == 0 and lado_posicion_actual == 0):   # si ya está en 4 izquierda, no se mueve
            await logica_de_movimiento(objetivo, 1)             # 4 por la izquierda (1 != 0 = izquierda)
        await cerrar_garra_poquito()
        await giro(179)
        await retrocede_hasta_pared(179, 50)                    # Calibrar dc
        await wait (200)
        reset_heading_seguro(180)
        montaje = (MONTAJE_SIN_CAMBIO if distancia_montaje is None else distancia_montaje) + montaje_extra
        await avance_hasta_linea(-179, esperar_blanco=False, montarse=True, distancia_montaje=montaje, max_dc=40)   # Calibrar max_dc
        heading_objetivo = 180
        posicion_actual = 4
        lado_posicion_actual = 1
        return
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
        if movimiento == 1:
            # Una sola línea: va despacio y el montaje se queda corto -> se compensa
            distancia_montaje += EXTRA_MONTAJE_UNA_LINEA
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

async def llevar_a_mosaico_izquierda():
    """Lleva los bloques al mosaico izquierdo (lo que antes era la descarga con llevadas == 2).
    No la llama la lógica: se llama en la ruta cuando resolver_lecturas termina."""
    global posicion_actual, lado_posicion_actual
    # lado_posicion_objetivo: diferente de 0 = lado IZQUIERDO, 0 = lado DERECHO
    # Va a la línea de descarga más cercana y se monta DESCARGA_UN_BLOQUE hacia adentro (al centro,
    # entre la línea 2 y la 3).
    montaje_extra = DESCARGA_UN_BLOQUE
    if posicion_actual >= 3:
        destino, lado_posicion_objetivo = 3, 1  # posición 3, lado izquierdo
    else:
        destino, lado_posicion_objetivo = 2, 0  # posición 2, lado derecho

    # 1) Llega cerca de la línea (si ya está en esa línea, no se mueve:
    #    la re-referencia de abajo lo deja en el lado correcto venga del lado que venga)
    objetivo = destino - posicion_actual
    if objetivo != 0:
        await logica_de_movimiento(objetivo, lado_posicion_objetivo)
    await cerrar_garra_poquito()   # sujeta los bloques antes de girar y retroceder

    # 2) RE-REFERENCIA (misma posición final venga por donde venga):
    #    mira a 0 (línea 3) o a 180 (línea 2), retrocede despacio hasta que el sensor ve la línea
    #    de color y avanza una distancia fija con control de velocidad.
    #    El sensor va ~50 mm delante del eje, así que cerca de la posición la línea queda DETRÁS
    #    del sensor y retrocediendo siempre la encuentra.
    #    "Hacia adentro" es hacia atrás en los dos casos, por eso el extra se RESTA.
    heading_ref = 0 if destino == 3 else 180
    await giro(heading_ref)
    await retroceder_hasta_linea(heading_ref, vel_mm_s=VEL_REFERENCIA_DESCARGA)
    await avance_preciso(MONTAJE_SIN_CAMBIO - montaje_extra, vel_mm_s=VEL_AJUSTE_DESCARGA) #Calibrar
    posicion_actual = destino
    lado_posicion_actual = 0 if lado_posicion_objetivo != 0 else 1

    # 3) Hacia el mosaico
    await giro(90, 80)
    await avance(90,30,40)

async def logica_de_comer():
    """Cada 2 bloques el robot se los "come" (sin ir a descargar ni usar la logica de movimiento).
    llevadas == 0 -> agarrar_izquierda | llevadas == 1 -> agarrar_derecha.
    Si está en la línea 4 NO come ahí: baja a la 3 (arriba, lado derecho) y gira a -90 antes de comer."""
    if posicion_actual == 4:
        await logica_de_movimiento(-1, 0)   # 4 -> 3, lado derecho (arriba de la 3, MONTAJE_SIN_CAMBIO)
        await cerrar_garra_poquito()
        await giro(-90)
        await retrocede(-90, RETROCESO_AGARRE, max_dc=DC_RETROCESO_RECOLECCION, accel_mm=80)  # el mismo retroceso extra que en logica_de_recoleccion
    else:
        await multitask(avance(-90,60,40),abrir_garra())
        await cerrar_garra_poquito()
        await retrocede(-90,60,40)
    if llevadas == 0:
        await agarrar_izquierda()
    elif llevadas == 1:
        await agarrar_derecha()

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
    await retrocede(56, ROJO_AVANCE_50) #Calibrar para que el robot retroceda lo mismo que avanzó para agarrar el rojo
    posicion_actual = 4 #El robot está en la posición 4 y lado izquierdo
    lado_posicion_actual = 0 #El robot está en la posición 4 y lado izquierdo
    # Si después toca comer, logica_de_comer baja a la 3 y gira a -90 (no se come en la 4)

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

async def re_referenciar_lado(lado_posicion_objetivo):
    """Deja el robot EXACTAMENTE en el lado pedido de la línea actual, aunque venga corrido.
    Se usa después de comer cuando el siguiente bloque es en la MISMA línea: agarrar_izquierda deja
    el robot más arriba (derecha) y agarrar_derecha más abajo (izquierda) de lo que cree la lógica.
    Rumbo según el lado donde ESTÁ (mirando hacia afuera de la línea):
    lado actual derecha (arriba) -> mira a 0; lado actual izquierda (abajo) -> mira a 180.
    Como el corrimiento puede dejar el sensor de cualquier lado de la línea, la busca en DOS sentidos:
    1) retrocede hasta BUSQUEDA_LINEA_MM (corto, para no llegar a la línea vecina);
    2) si no la vio, la línea está adelante: avanza hasta 2 * BUSQUEDA_LINEA_MM (vuelve al inicio y sigue).
    En los dos casos termina con el sensor sobre la línea y mirando a heading_ref, y luego avanza
    lo mismo que usaría logica_de_movimiento con ese rumbo:
    - el lado pedido está hacia donde mira -> DISTANCIA_MONTAJE (78 mm)
    - el lado pedido está detrás            -> MONTAJE_SIN_CAMBIO (18 mm)"""
    global heading_objetivo, lado_posicion_actual
    heading_ref = 0 if lado_posicion_actual == 1 else 180
    await giro(heading_ref)
    encontrada = await retroceder_hasta_linea(heading_ref, vel_mm_s=VEL_REFERENCIA_DESCARGA, distancia_max=BUSQUEDA_LINEA_MM)
    if not encontrada:
        # No estaba atrás: el sensor quedó antes de la línea -> buscarla hacia ADELANTE
        # (retroceder_hasta_linea con velocidad negativa avanza)
        print("re_referenciar_lado: línea no está atrás, se busca adelante")
        encontrada = await retroceder_hasta_linea(heading_ref, vel_mm_s=-VEL_REFERENCIA_DESCARGA, distancia_max=2 * BUSQUEDA_LINEA_MM)
    if not encontrada:
        print("re_referenciar_lado: NO encontró la línea en ningún sentido (¿robot muy corrido?)")
        await boton()
    # Igual que montarse_final en logica_de_movimiento
    montarse_final = (lado_posicion_objetivo == 0 and heading_ref == 0) or (lado_posicion_objetivo != 0 and heading_ref == 180)
    await avance_preciso(DISTANCIA_MONTAJE-5 if montarse_final else MONTAJE_SIN_CAMBIO, vel_mm_s=VEL_AJUSTE_DESCARGA)   # Calibrar
    heading_objetivo = heading_ref
    lado_posicion_actual = 1 if lado_posicion_objetivo == 0 else 0

async def   resolver_lecturas():
    global i, llevadas
    contador = 0
    recien_comio = False   # True justo después de logica_de_comer (el robot puede quedar corrido)
    for i in range(len(lectura)): #Recorre la lista de lecturas
        # Valor inválido de la cámara (0 = casilla no leída, o cualquier cosa fuera de 1-5): se trata
        # igual que un color sin bloques. Sin esto, colores[0] da KeyError y un valor > 5 iría a agarrar_rojo.
        invalido = lectura[i] not in (1, 2, 3, 4, 5)
        if invalido or (lectura[i] < 5 and colores[lectura[i]] == 0): #Si el color objetivo ya no tiene bloques (o es inválido), lo cambia por el color MENOS agarrado (bloques en las primeras filas = más seguro)
            reemplazo = color_menos_agarrado()
            if reemplazo is None:
                print("No quedan bloques de ningún color")
                break
            print("Color", lectura[i], "inválido" if invalido else "sin bloques", "-> se reemplaza por", reemplazo)
            lectura[i] = reemplazo

        if lectura[i] <5: # Si la lectura no es roja

            """
            LOGICA DE MOVIMIENTO HACIA EL COLOR OBJETIVO
            """
            lado_posicion_objetivo = colores[lectura[i]]%2 # Si el bloque es par, el lado objetivo es 0 (DERECHA), si es impar, el lado objetivo es diferente de 0 (IZQUIERDA)

            objetivo = lectura[i] - posicion_actual # Calcula la diferencia entre la posición actual y la posición objetivo, me dice cuanto debo moverme para llegar al color objetivo
            if recien_comio and objetivo == 0:
                # Misma línea después de comer: la agarra dejó el robot corrido -> re-referencia
                # con la línea de color (sirve para el mismo lado y para cambiar de lado)
                await re_referenciar_lado(lado_posicion_objetivo)
            else:
                await logica_de_movimiento(objetivo, lado_posicion_objetivo)
             # Espera a que el robot esté en la posición correcta y el botón sea presionado para continuar

            """
            LOGICA PARA AVANZAR Y RECOGER EL BLOQUE
            """
            # Sigue agarrar si es el 2do bloque del par (contador == 1) y todavía se come (llevadas < 2)
            await logica_de_recoleccion(sigue_agarrar=(contador == 1 and llevadas < 2))
        else:
            """
            Logica para cuando la lectura es roja
            """
            await agarrar_rojo()

        recien_comio = False
        contador += 1
        if contador > 1:
            """
            LOGICA PARA COMER: cada 2 bloques se los come (sin ir a descargar).
            Con llevadas == 2 (tercer par) termina la lógica: esos 2 quedan en la garra
            y en la ruta se llama llevar_a_mosaico_izquierda().
            """
            if llevadas >= 2:
                break
            await logica_de_comer()
            llevadas += 1
            contador = 0
            recien_comio = True

async def agarrar_mayoria(color_mayoria):
    if color_mayoria is None:   # no hubo color válido en el lado mayoría: no se mueve (evita el error None - posicion_actual)
        print("agarrar_mayoria: sin color mayoría, se salta")
        return
    # 1) Va a la línea del color mayoría, lado DERECHO (objetivo es RELATIVO: cuántas líneas moverse)
    await logica_de_movimiento(color_mayoria - posicion_actual, 0)
    # 2) Agarra el bloque derecho de la 1ª fila (fila=0: no toca colores, el color está reservado)
    await logica_de_recoleccion(fila=0)
    # 3) Cambia al lado IZQUIERDO en la misma línea (avance de DISTANCIA_CAMBIO_LADO, sin comer)
    await logica_de_movimiento(0, 1)
    # 4) Agarra el bloque izquierdo de la 1ª fila y retrocede a la línea vertical (NO se los come)
    await logica_de_recoleccion(fila=0)
    await giro (0)
    await avance(0,DISTANCIA_CAMBIO_LADO/2)
    await multitask (giro(-90),cerrar_garra())
    await multitask(seguir_linea_hasta_interseccion("centro",200,montarse=True,distancia_montaje=200),subir_elevador_todo())
    await bajar_elevador()
    await multitask (retrocede(-90,360,50),bajar_elevador())
  
def sacar_mayoria(izquierda, derecha, lado_rojo_idx=None):
    """Elige el color (1-4) para el lado que se llena por MAYORÍA (el lado SIN rojo).
    El lado mayoría usa los 6 bloques del color elegido, así que ese color ya no queda para
    el lado rojo (el que se resuelve con resolver_lecturas): cada casilla de ese color en el
    lado rojo es un punto perdido.
    Puntaje de cada color = casillas en el lado sin rojo - casillas en el lado rojo.
    (Incluye la regla de "si en el lado rojo hay más, agarrar el segundo color con mayoría",
    pero compara TODOS los colores, no solo el primero y el segundo.)
    Empate: el que más casillas tiene en el lado sin rojo; si sigue empatado, el primero
    que aparece en la lista del lado sin rojo.
    lado_rojo_idx: 0 = izquierda, 1 = derecha (el lado_rojo de la cámara). Si no se da,
    se busca el 5 (si la cámara confundiera un color con rojo en los dos lados, ganaría la izquierda)."""
    if lado_rojo_idx == 0:
        lado_rojo_lista, lado_mayoria = izquierda, derecha
    elif lado_rojo_idx == 1:
        lado_rojo_lista, lado_mayoria = derecha, izquierda
    elif ROJO_LECTURA in izquierda:
        lado_rojo_lista, lado_mayoria = izquierda, derecha
    else:
        lado_rojo_lista, lado_mayoria = derecha, izquierda

    mejor, mejor_puntaje, mejor_casillas = None, None, -1
    for c in lado_mayoria:          # en orden de la lista -> desempate por el primero que aparece
        if c not in (1, 2, 3, 4) or c == mejor:   # salta el rojo y cualquier valor raro de la cámara
            continue
        casillas = lado_mayoria.count(c)
        perdidos = lado_rojo_lista.count(c)
        puntaje = casillas - perdidos
        if mejor is None or puntaje > mejor_puntaje or (puntaje == mejor_puntaje and casillas > mejor_casillas):
            mejor, mejor_puntaje, mejor_casillas = c, puntaje, casillas
    print("Mayoría:", mejor, "| casillas:", mejor_casillas, "| puntaje neto:", mejor_puntaje)
    return mejor

def reservar_mayoria():
    """Calcula el color mayoría UNA vez y lo quita de colores, para que el lado rojo
    (resolver_lecturas) no gaste bloques de ese color: si lo pide, color_menos_agarrado lo reemplaza.
    Se llama al empezar resolver_izquierda/resolver_derecha (sin importar cuál lado va primero)."""
    global color_mayoria
    if color_mayoria is None:
        color_mayoria = sacar_mayoria(izquierda, derecha, lado_rojo)
        if color_mayoria is not None:
            colores[color_mayoria] = 0   # los 6 bloques quedan reservados para el lado mayoría
    return color_mayoria


# ---------------------------------------------------------
# RUTINAS LOCALES DE DESCARGA 
# ---------------------------------------------------------
async def dejar_bloques(heading):
    #agarrarlos)
    await multitask (abrir_garra(),seguir_linea_distancia (distancia_mm=90, vel_crucero_mm_s=200))
    await wait (100)
    await retrocede(heading,16,40)
    await cerrar_garra_poquito()
    await multitask (subir_elevador_todo(), seguir_linea_hasta_interseccion ( vel_crucero_mm_s=120) )#Calibrar para que el robot avance hasta la línea de descarga y se monte sobre ella
    await wait (100)
    reset_heading_seguro(heading)
    # await giro(heading+5,30)
    # await giro(heading-5,30)
    # await retrocede (heading,23)
    # await giro(heading+7,30)
    # await giro(heading-7,30)
    await retrocede (heading,70,40,30,10,10)
    await giro(heading)
    # await bajar_elevador()
    # await abrir_garra()
    # await motor_dc_por_segundos(motorElevador,300,-100)
    # await cerrar_garra_poquito()
    # await seguir_linea_distancia(150,100)
    await bajar_elevador()
    await motor_dc_por_segundos(motorElevador,300,-50)
    await multitask (abrir_garra(-40), motor_dc_por_segundos(motorElevador,300,-30))
    await cerrar_garra()
    await wait (100)
    await subir_elevador()
    await seguir_linea_hasta_interseccion(vel_crucero_mm_s=200)
    await avance_hasta_pared(heading,60,tiempo_stall=200)
    reset_heading_seguro(heading)
    #soltarlos
    await retrocede (heading,10,40,30,10,10)
    await bajar_elevador()
    await motor_dc_por_grados(motorElevador,57,100)
    await motor_dc_por_grados (motorgarra,50,-50)
    await abrir_garra(-90)
    await motor_dc_por_grados (motorgarra,9,80)
    await retrocede (heading,30,50)
    await motor_dc_hasta_tope(motorgarra,40,1000,30,40,120)
    await retrocede (heading,11,50)
    await abrir_garra(-60)
    await subir_elevador()

# ---------------------------------------------------------
# RUTAS DE COMPETICIÓN 
# ---------------------------------------------------------


async def Ruta_1_Lectura(): # --> RUTA EN LISTA #Se acomoda de la salida y sigue linea hasta interseccion para llegar a leer, la distancia de montaje debe ser para que quede en el centro de la linea, gire se tire hacia atras y choque con el mosaico. Ahi ya hace la lectura
    global izquierda, derecha, lado_rojo
    #await avance(0, 100, 80)
    #await giro(90)

    #await avance(0, 15, 80, freno=False)
    #await giro_pivot(90, pivote_izquierdo=False)
    await subir_elevador_todo()
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
    #await lectura_camara()
    # Lectura MANUAL (mientras no se usa la cámara): poner los valores reales del tapete
    izquierda = [2, 2, 3, 5, 4, 1]   # orden: fila1 + fila3 + fila2, igual que leer_camara
    derecha   = [4, 4, 3, 2, 1, 2]
    lado_rojo = 0                    # NÚMERO, no lista: 0 = el 5 está en izquierda, 1 = en derecha
    reservar_mayoria()               # siempre DESPUÉS de dar los valores

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
    await avance(90, 50)
    await giro(0)
    await bajar_elevador()

async def Ruta_4_resolver_izquierda() :
    global lectura, posicion_actual, lado_posicion_actual, llevadas
    posicion_actual = 0                      # OJO: solo si el robot de verdad está en la línea 0
    lado_posicion_actual = 0
    if lado_rojo == 0:
        lectura = list(izquierda)            # copia: resolver_lecturas cambia los colores sin bloques
        llevadas = 0
        await resolver_lecturas()
    else:
        await agarrar_mayoria(color_mayoria)
    await llevar_a_mosaico_izquierda()

async def Ruta_5_dejar_bloques_y_azules():
    global lectura, posicion_actual, lado_posicion_actual, llevadas
    await dejar_bloques(90)
    await retrocede(90,150)
    await giro(-90)
    await multitask(retrocede(-90, 170,60), abrir_paletas())
    await avance(-90, 150)
    await multitask(cerrar_paletas(), seguir_linea_hasta_interseccion("centro"))
    await giro(90)
    await seguir_linea_hasta_interseccion("izquierda",300,montarse=True,distancia_montaje=MONTAJE_SIN_CAMBIO)
    posicion_actual=3
    lado_posicion_actual=0 #izquierda posicion 3 para empezar la logica del lado derecho

async def Ruta_6_resolver_derecha() :
    global lectura, posicion_actual, lado_posicion_actual, llevadas
    reservar_mayoria()
    if lado_rojo == 1:
        lectura = list(derecha)              # copia: resolver_lecturas cambia los colores sin bloques
        llevadas = 0
        await resolver_lecturas()
    else:
        await agarrar_mayoria(color_mayoria)

async def Ruta_7_llevar_bloques():
    global lectura, posicion_actual, lado_posicion_actual, llevadas
    await cerrar_garra_poquito()   # sujeta los bloques antes de que la lógica gire a 180
    # lado_posicion_objetivo 0 = lado DERECHO (queda lado_posicion_actual = 1)
    await logica_de_movimiento (0-posicion_actual,0 , MONTAJE_SIN_CAMBIO+20)
    #Hacer la ruta manualmente para llevar los bloques al lado derecho

async def Ruta_8_ladrillos_verdes():
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
    #No se sabe si se va a usar o no, hay que ver si alcanza el tiempo
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

    #lectura = [3, 5, 3, 1, 3, 4]   # Lectura fija, descomentar para repetir un caso concreto
    colores = {1: 6, 2: 6, 3: 6, 4: 6}   # Reinicia la cantidad de bloques por color
    colores[color_vacio] = 0             # Comentar esta línea si usas la lectura fija
    posicion_actual = 0                  # 0 = línea de abajo (inicio)
    lado_posicion_actual = 0             # 0 = lado izquierdo (abajo)
    llevadas = 0

    await resolver_lecturas()

# ---------------------------------------------------------
# RUTA DE PRUEBA DE AGARRAR MAYORÍA
# Posición inicial: línea de abajo (0), lado izquierdo (abajo), sobre la línea vertical, mirando a 0.
# Calcula el color mayoría con sacar_mayoria y llama agarrar_mayoria (va a la línea de ese color,
# lado derecho, y luego tus movimientos para agarrar los 6 bloques).
# ---------------------------------------------------------
async def Ruta_prueba_mayoria():
    global izquierda, derecha, lado_rojo, color_mayoria, colores, posicion_actual, lado_posicion_actual
    hub.imu.reset_heading(0)

    # Lectura ficticia ALEATORIA: un lado al azar tiene el rojo, el otro solo colores 1-4
    ax, ay, az = hub.imu.acceleration()
    seed(int(abs(ax) * 1000 + abs(ay) * 100 + abs(az) * 10) + hub.battery.voltage() + hub.battery.current())
    izquierda = [randint(1, 4) for _ in range(6)]
    derecha   = [randint(1, 4) for _ in range(6)]
    lado_rojo = randint(0, 1)
    if lado_rojo == 0:
        izquierda[randint(0, 5)] = 5
    else:
        derecha[randint(0, 5)] = 5
    # Lectura fija, descomentar para repetir un caso concreto:
    # izquierda, derecha, lado_rojo = [1, 1, 2, 5, 3, 4], [1, 1, 2, 3, 4, 3], 0   # -> mayoría 3 (verde)
    print("Izquierda:", izquierda, "| Derecha:", derecha, "| lado_rojo:", lado_rojo)

    colores = {1: 6, 2: 6, 3: 6, 4: 6}   # Reinicia la cantidad de bloques por color
    color_mayoria = None                 # para que reservar_mayoria lo calcule de nuevo
    reservar_mayoria()
    # color_mayoria = 3                  # Descomentar para forzar un color concreto (1-4)

    posicion_actual = 0                  # 0 = línea de abajo (inicio)
    lado_posicion_actual = 0             # 0 = lado izquierdo (abajo)

    await agarrar_mayoria(color_mayoria)
    print("Fin prueba mayoría | color:", color_mayoria, "| posicion:", posicion_actual, "| lado:", lado_posicion_actual)

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
    # ---------------- RUTAS DE COMPETICIÓN (en orden) ----------------
    # await Ruta_1_Lectura()
    # await Ruta_2_Recoger_y_llevar_cemento()
    # await Ruta_3_recoger_ladrillos_azules()
    # await Ruta_4_resolver_izquierda()
    # await Ruta_5_dejar_bloques_y_azules()
    # await Ruta_6_resolver_derecha()
    # await Ruta_7_llevar_bloques()
    # await Ruta_8_ladrillos_verdes()
    #await Ruta_blancos()             # No se sabe si se va a usar, hay que ver si alcanza el tiempo

    # ---------------- PRUEBAS (descomentar una y comentar las rutas de arriba) ----------------
    await bajar_elevador()
    await subir_elevador_todo()
    await bajar_elevador()  
    await Ruta_prueba_logica()       # Lógica con lectura aleatoria desde la línea 0, lado izquierdo
    await llevar_a_mosaico_izquierda()
    await dejar_bloques(90)
    # await Ruta_prueba_mayoria()      # Prueba de agarrar_mayoria desde la línea 0, lado izquierdo
    #await Ruta_prueba_seguidor()     # Prueba del seguidor con registro: imprime un resumen al final
    #await medir_trayectos_manual()   # Mide los trayectos y los guarda en la memoria del hub
    #await prueba_motores()           # OJO: bucle infinito

iniciar_esp32()   # debe ir ANTES de run_task
#race=True: el programa termina cuando Run() termina (proceso_esp32 corre para siempre)
#run_task (Run())
run_task(multitask(Run(), proceso_esp32(), race=False))
