import sys
import time
import math
import random
import select
from machine import Pin, PWM, ADC, disable_irq, enable_irq  # type: ignore

# ==========================================================
# CONFIGURACION DE PINES Y PARAMETROS
# ==========================================================

PIN_ENA = 4
PIN_IN1 = 16
PIN_IN2 = 17
PIN_ENC_A = 33
PIN_ENC_B = 25
PIN_VOLTAJE = 32
PIN_SENSOR = 18

INVERTIR_MOTOR = False
SIGNO_ENCODER_GLOBAL = 1

FREC_PWM = 20000
PULSOS_VUELTA_POS = 778.0
# Polaridad real verificada en giro horario (-1 = suma positivo en sentido reloj)
SIGNO_PULSOS_HORARIO = -1

# Sentido horario para movimientos de posición (-1 = derecha / sentido reloj)
SENTIDO_GIRO_POS = -1
SENTIDO_BUSQUEDA_POS = -1
POTENCIA_BUSQUEDA_POS = 38.0
POTENCIA_BUSQUEDA_LENTA = 30.0

# Corrección fina del Cero Absoluto tras detenerse en el sensor IR
OFFSET_CERO_GRADOS = 0.0
SENTIDO_AJUSTE_CERO = -1
POTENCIA_AJUSTE_CERO = 30.0

# Parámetros de Control de Posición
TOLERANCIA_GRADOS = 1.5
POTENCIA_RAPIDA = 42.0
POTENCIA_MEDIA = 36.0
POTENCIA_LENTA = 32.0
ERROR_RAPIDO = 75.0
ERROR_MEDIO = 28.0
TIMEOUT_POSICION = 20000

# ==========================================================
# INICIALIZACION DE HARDWARE
# ==========================================================

in1 = Pin(PIN_IN1, Pin.OUT, value=0)
in2 = Pin(PIN_IN2, Pin.OUT, value=0)
ena = PWM(Pin(PIN_ENA), freq=FREC_PWM, duty_u16=0)

enc_a = Pin(PIN_ENC_A, Pin.IN, Pin.PULL_UP)
enc_b = Pin(PIN_ENC_B, Pin.IN, Pin.PULL_UP)

sensor_ir = Pin(PIN_SENSOR, Pin.IN, Pin.PULL_UP)

adc_voltaje = ADC(Pin(PIN_VOLTAJE))
adc_voltaje.atten(ADC.ATTN_11DB)

# Variables globales
contador = 0
ultimo_pulso_us = 0
min_pulso_us = 40
sentido_enc = 1

posicion_estimada = 0.0
modo_actual = "MODO_VELOCIDAD"
potencia_actual = 0.0

rpm_filtrada = 0.0
rpm_objetivo_suavizada = 0.0

# ==========================================================
# INTERRUPCION Y LECTURA DEL ENCODER (DOBLE FLANCO)
# ==========================================================

def contar_pulso_a(pin):
    global contador, ultimo_pulso_us
    ahora = time.ticks_us()
    if min_pulso_us > 0 and time.ticks_diff(ahora, ultimo_pulso_us) < min_pulso_us:
        return
    ultimo_pulso_us = ahora

    a_val = enc_a.value()
    b_val = enc_b.value()
    if a_val == b_val:
        contador -= sentido_enc * SIGNO_ENCODER_GLOBAL
    else:
        contador += sentido_enc * SIGNO_ENCODER_GLOBAL

enc_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=contar_pulso_a)

def leer_contador():
    estado = disable_irq()
    val = contador
    enable_irq(estado)
    return val

def reiniciar_contador():
    global contador, ultimo_pulso_us
    estado = disable_irq()
    contador = 0
    ultimo_pulso_us = 0
    enable_irq(estado)

def probar_encoder_manual():
    print("[TEST ENCODER] Gira la aguja con la mano durante 4 segundos...")
    reiniciar_contador()
    t0 = time.ticks_ms()
    ultimo = 0
    while time.ticks_diff(time.ticks_ms(), t0) < 4000:
        actual = leer_contador()
        if actual != ultimo:
            print(f"  Pulsos: {actual} | A(33)={enc_a.value()} B(25)={enc_b.value()}")
            ultimo = actual
        time.sleep_ms(50)
    print(f"[TEST ENCODER] Total leido: {leer_contador()} pulsos (A(33)={enc_a.value()}, B(25)={enc_b.value()}).")

# ==========================================================
# CONTROL DEL MOTOR Y SENSORES
# ==========================================================

def mover_motor(potencia):
    potencia = max(-100.0, min(100.0, float(potencia)))

    if INVERTIR_MOTOR:
        potencia = -potencia

    potencia_efectiva = 0.0

    if abs(potencia) > 0.2:
        min_pwm_util = 44.0
        signo = 1.0 if potencia > 0 else -1.0
        potencia_efectiva = signo * (
            min_pwm_util + (abs(potencia) / 100.0) * (100.0 - min_pwm_util)
        )

    duty = int(abs(potencia_efectiva) * 65535 / 100.0)

    if potencia_efectiva > 0:
        in1.value(1)
        in2.value(0)
    elif potencia_efectiva < 0:
        in1.value(0)
        in2.value(1)
    else:
        in1.value(0)
        in2.value(0)

    ena.duty_u16(duty)

def frenar_motor(sentido_actual):
    if sentido_actual != 0:
        mover_motor(-25.0 * sentido_actual)
        time.sleep_ms(16)
    mover_motor(0)

def mover_motor_pos(potencia):
    potencia = max(-100.0, min(100.0, float(potencia)))
    mover_motor(potencia * SENTIDO_GIRO_POS)

def leer_voltaje():
    total = 0.0
    for _ in range(100):
        total += adc_voltaje.read_uv() / 1_000_000.0
        time.sleep_us(137)
    return total / 100.0

def sensor_detecta():
    DETECTA_EN = 0
    for _ in range(2):
        if sensor_ir.value() != DETECTA_EN:
            return False
        time.sleep_us(500)
    return True

# ==========================================================
# LECTURA SERIAL NO BLOQUEANTE
# ==========================================================

poll_serial = select.poll()
poll_serial.register(sys.stdin, select.POLLIN)

def leer_linea_serial():
    if poll_serial.poll(0):
        linea = sys.stdin.readline()
        if linea:
            return linea.strip().upper()
    return None

def limpiar_buffer_serial():
    while poll_serial.poll(0):
        sys.stdin.readline()

# ==========================================================
# RUTINA DE HOMING Y CERO ABSOLUTO (DIRECTA EN UNA PARADA)
# ==========================================================

def ajustar_cero_absoluto():
    if abs(OFFSET_CERO_GRADOS) < 0.5:
        print("[AJUSTE CERO] Sin desplazamiento adicional (0.0 grados).")
        return True

    print("\n[AJUSTE CERO] Corrigiendo posicion de referencia...")

    pulsos_objetivo = round((PULSOS_VUELTA_POS * abs(OFFSET_CERO_GRADOS)) / 360.0)

    if pulsos_objetivo <= 0:
        return True

    sentido_mov = SENTIDO_AJUSTE_CERO if OFFSET_CERO_GRADOS > 0 else -SENTIDO_AJUSTE_CERO

    print(f"[AJUSTE CERO] Desplazamiento: {OFFSET_CERO_GRADOS:.1f} grados")
    print(f"[AJUSTE CERO] Pulsos necesarios: {pulsos_objetivo}")

    cuenta_inicial = leer_contador()
    tiempo_inicio = time.ticks_ms()
    TIMEOUT_AJUSTE = 5000

    mover_motor(POTENCIA_AJUSTE_CERO * sentido_mov)

    while True:
        cuenta_actual = leer_contador()
        pulsos_recorridos = abs(cuenta_actual - cuenta_inicial)

        if pulsos_recorridos >= pulsos_objetivo:
            break

        if time.ticks_diff(time.ticks_ms(), tiempo_inicio) > TIMEOUT_AJUSTE:
            mover_motor(0)
            print("[ERROR] Timeout durante el ajuste del cero.")
            return False

        time.sleep_ms(1)

    frenar_motor(sentido_mov)
    time.sleep_ms(150)
    print("[AJUSTE CERO] Desplazamiento completado.")
    return True

def buscar_y_fijar_cero():
    global posicion_estimada

    print("\n[HOMING] Buscando marca inicial del sensor infrarrojo...")

    mover_motor(SENTIDO_BUSQUEDA_POS * POTENCIA_BUSQUEDA_POS)
    t0 = time.ticks_ms()

    while sensor_detecta():
        if time.ticks_diff(time.ticks_ms(), t0) > 12000:
            mover_motor(0)
            print("[Error] Timeout saliendo del sensor (GPIO 18 fijo en 0).")
            return False
        time.sleep_ms(2)

    t0 = time.ticks_ms()
    while not sensor_detecta():
        if time.ticks_diff(time.ticks_ms(), t0) > 12000:
            mover_motor(0)
            print("[Error] Timeout buscando sensor (GPIO 18 no detecto marca).")
            return False
        time.sleep_ms(1)

    frenar_motor(SENTIDO_BUSQUEDA_POS)
    time.sleep_ms(250)
    reiniciar_contador()

    if not ajustar_cero_absoluto():
        mover_motor(0)
        print("[ERROR] No fue posible establecer el cero absoluto.")
        return False

    time.sleep_ms(200)
    reiniciar_contador()
    posicion_estimada = 0.0

    print("\n[HOMING] CERO ABSOLUTO ESTABLECIDO.")
    print("[HOMING] Posicion de referencia: 0.0 grados.")
    print("[HOMING] Sistema listo para movimientos angulares multivuelta.\n")
    return True

# ==========================================================
# CONTROL DE POSICION CONTINUA MULTIVUELTA (CON MEMORIA)
# ==========================================================

def calcular_grados_absolutos():
    cuenta_actual = leer_contador()
    pulsos_horarios = cuenta_actual * SIGNO_PULSOS_HORARIO
    return (float(pulsos_horarios) / PULSOS_VUELTA_POS) * 360.0

def ir_a_posicion(objetivo):
    global posicion_estimada, SIGNO_PULSOS_HORARIO

    t_inicio = time.ticks_ms()
    t_sin_pulsos = time.ticks_ms()
    ultima_cuenta = leer_contador()

    objetivo = float(objetivo)
    posicion_estimada = calcular_grados_absolutos()
    error_inicial_abs = abs(objetivo - posicion_estimada)
    vueltas = objetivo / 360.0

    print(f"\nPosicion actual: {posicion_estimada:.1f} deg -> Moviendo a objetivo: {objetivo:.1f} deg ({vueltas:.2f} vueltas)")
    time.sleep_ms(40)

    ultimo_sentido = 0
    signo_corregido = False

    while True:
        cmd = leer_linea_serial()
        if cmd:
            if cmd == "X":
                mover_motor(0)
                print(">>> PARO DE EMERGENCIA EN POSICION <<<")
                return
            try:
                nuevo_obj = float(cmd.replace(",", "."))
                if math.isfinite(nuevo_obj):
                    objetivo = nuevo_obj
                    t_inicio = time.ticks_ms()
                    t_sin_pulsos = time.ticks_ms()
                    error_inicial_abs = abs(objetivo - calcular_grados_absolutos())
                    signo_corregido = False
                    print(f"[POSICION] Nuevo objetivo recibido: {objetivo:.1f} deg")
            except ValueError:
                pass

        if time.ticks_diff(time.ticks_ms(), t_inicio) > TIMEOUT_POSICION:
            mover_motor(0)
            print("[ADVERTENCIA] Timeout: no se alcanzo el objetivo.")
            break

        cuenta_ahora = leer_contador()
        dt_sin_pulsos = time.ticks_diff(time.ticks_ms(), t_sin_pulsos)
        if cuenta_ahora != ultima_cuenta:
            ultima_cuenta = cuenta_ahora
            t_sin_pulsos = time.ticks_ms()
        elif dt_sin_pulsos > 1800:
            mover_motor(0)
            print("[ERROR] Seguridad: motor atascado o encoder sin pulsos.")
            break

        posicion_estimada = calcular_grados_absolutos()
        error = objetivo - posicion_estimada
        error_abs = abs(error)

        if error_abs > (error_inicial_abs + 45.0):
            if not signo_corregido:
                SIGNO_PULSOS_HORARIO = -SIGNO_PULSOS_HORARIO
                signo_corregido = True
                posicion_estimada = calcular_grados_absolutos()
                error = objetivo - posicion_estimada
                error_abs = abs(error)
                error_inicial_abs = error_abs
                print("[SEGURIDAD] Polaridad de encoder autocorregida.")
            else:
                mover_motor(0)
                print("[ERROR] Seguridad: el error angular aumento. Verifica cableado del encoder.")
                break

        if error_abs <= TOLERANCIA_GRADOS:
            frenar_motor(ultimo_sentido * SENTIDO_GIRO_POS)
            print("[POSICION] Objetivo alcanzado.")
            break

        if error_abs > ERROR_RAPIDO:
            potencia_mov = POTENCIA_RAPIDA
        elif error_abs > ERROR_MEDIO:
            potencia_mov = POTENCIA_MEDIA
        else:
            potencia_mov = POTENCIA_LENTA

        if dt_sin_pulsos > 200:
            potencia_mov = max(potencia_mov, 52.0)

        if error < 0:
            potencia_mov = -potencia_mov
            ultimo_sentido = -1
        else:
            ultimo_sentido = 1

        mover_motor_pos(potencia_mov)
        time.sleep_ms(12)

    mover_motor(0)
    time.sleep_ms(200)

    posicion_estimada = calcular_grados_absolutos()
    print(f"Posicion final con memoria: {posicion_estimada:.1f} grados (Caratula reloj: {posicion_estimada % 360.0:.1f} grados)")

# ==========================================================
# CONTROL PI Y PERFILES DINAMICOS
# ==========================================================

def controlar_paso_velocidad(
    rpm_objetivo_bruta,
    t_anterior,
    cuenta_anterior,
    correccion,
    tope_sup=150.0,
    tope_inf=-150.0
):
    global rpm_filtrada, rpm_objetivo_suavizada

    ahora = time.ticks_ms()
    dt = time.ticks_diff(ahora, t_anterior) / 1000.0

    if dt >= 0.03:
        t_anterior = ahora
        cuenta_actual = leer_contador()
        pulsos_delta = (cuenta_actual - cuenta_anterior) * SIGNO_PULSOS_HORARIO
        cuenta_anterior = cuenta_actual

        rpm_instantanea = 0.0
        if PULSOS_VUELTA_POS > 0:
            rpm_instantanea = (float(pulsos_delta) / PULSOS_VUELTA_POS) * 60.0 / dt

        rpm_filtrada = (0.45 * rpm_filtrada) + (0.55 * rpm_instantanea)
    else:
        dt = 0.0

    rpm_objetivo_suavizada = rpm_objetivo_bruta

    if abs(rpm_objetivo_bruta) < 1.0:
        correccion = 0.0
        rpm_objetivo_suavizada = 0.0
        rpm_filtrada = 0.0
        mover_motor(0)
    else:
        error = rpm_objetivo_suavizada - rpm_filtrada
        KP = 0.35
        KI = 0.45

        if dt > 0.0:
            correccion += error * KI * dt

        correccion = max(-35.0, min(35.0, correccion))

        potencia_base = (rpm_objetivo_suavizada / 150.0) * 65.0
        potencia_final = potencia_base + (KP * error) + correccion
        potencia_final = max(-100.0, min(100.0, potencia_final))

        mover_motor(-potencia_final)

    print(f"{rpm_objetivo_bruta:.2f},{rpm_filtrada:.2f},0,{tope_sup:.2f},{tope_inf:.2f}")

    return t_anterior, cuenta_anterior, correccion

def ejecutar_perfil_grafica(perfil):
    global rpm_filtrada, rpm_objetivo_suavizada

    mover_motor(0)
    rpm_filtrada = 0.0
    rpm_objetivo_suavizada = 0.0
    time.sleep_ms(200)

    t_sup = 150.0
    t_inf = -150.0

    if perfil in ("L", "E"):
        t_sup = 150.0
        t_inf = 0.0

    print("RPM_Objetivo,RPM_Reales,Cero,TopeSup,TopeInf")

    t_inicio = time.ticks_ms()
    t_anterior = time.ticks_ms()
    cuenta_anterior = leer_contador()
    correccion = 0.0

    NUM_REPETICIONES = 5

    def revisar_interrupcion():
        cmd = leer_linea_serial()
        if cmd in ("X", "Q", "P"):
            mover_motor(0)
            print(">>> Perfil interrumpido por comando de parada <<<")
            return True
        return False

    if perfil == "T":
        rpm_min, rpm_max = 0.0, 120.0
        t_rampa = 1.0
        t_meseta = 1.5
        t_pausa = 0.8
        periodo_total = (2.0 * t_rampa) + t_meseta + t_pausa
        duracion_total = NUM_REPETICIONES * periodo_total

        while True:
            if revisar_interrupcion():
                return
            t = time.ticks_diff(time.ticks_ms(), t_inicio) / 1000.0
            if t >= duracion_total:
                break

            t_fase = t % periodo_total
            if t_fase < t_rampa:
                rpm_obj = rpm_min + (rpm_max - rpm_min) * (t_fase / t_rampa)
            elif t_fase < (t_rampa + t_meseta):
                rpm_obj = rpm_max
            elif t_fase < (2.0 * t_rampa + t_meseta):
                t_bajada = t_fase - (t_rampa + t_meseta)
                rpm_obj = rpm_max - (rpm_max - rpm_min) * (t_bajada / t_rampa)
            else:
                rpm_obj = 0.0

            t_anterior, cuenta_anterior, correccion = controlar_paso_velocidad(
                rpm_obj, t_anterior, cuenta_anterior, correccion, t_sup, t_inf
            )
            time.sleep_ms(40)

    elif perfil == "S":
        periodo_onda = 2.5
        duracion_total = NUM_REPETICIONES * periodo_onda
        while True:
            if revisar_interrupcion():
                return
            t = time.ticks_diff(time.ticks_ms(), t_inicio) / 1000.0
            if t >= duracion_total:
                break
            rpm_obj = 120.0 * math.sin(t * (2.0 * math.pi / periodo_onda))
            t_anterior, cuenta_anterior, correccion = controlar_paso_velocidad(
                rpm_obj, t_anterior, cuenta_anterior, correccion, t_sup, t_inf
            )
            time.sleep_ms(40)

    elif perfil == "R":
        for _ in range(NUM_REPETICIONES):
            t_bloque = time.ticks_ms()
            t_cambio = 0
            rpm_obj = 0.0
            while time.ticks_diff(time.ticks_ms(), t_bloque) < 2000:
                if revisar_interrupcion():
                    return
                if time.ticks_diff(time.ticks_ms(), t_cambio) >= 350:
                    rpm_obj = float(random.randint(-120, 120))
                    if abs(rpm_obj) < 20:
                        rpm_obj = 0.0
                    t_cambio = time.ticks_ms()

                t_anterior, cuenta_anterior, correccion = controlar_paso_velocidad(
                    rpm_obj, t_anterior, cuenta_anterior, correccion, t_sup, t_inf
                )
                time.sleep_ms(40)

            t_pausa = time.ticks_ms()
            while time.ticks_diff(time.ticks_ms(), t_pausa) < 500:
                if revisar_interrupcion():
                    return
                t_anterior, cuenta_anterior, correccion = controlar_paso_velocidad(
                    0.0, t_anterior, cuenta_anterior, correccion, t_sup, t_inf
                )
                time.sleep_ms(40)

    elif perfil == "L":
        t_subida = 4.5
        t_pausa = 1.0
        periodo = t_subida + t_pausa
        repeticiones_lineal = 3
        duracion_total = repeticiones_lineal * periodo
        while True:
            if revisar_interrupcion():
                return
            t = time.ticks_diff(time.ticks_ms(), t_inicio) / 1000.0
            if t >= duracion_total:
                break
            t_fase = t % periodo
            if t_fase < t_subida:
                rpm_obj = 135.0 * (t_fase / t_subida)
            else:
                rpm_obj = 0.0
            t_anterior, cuenta_anterior, correccion = controlar_paso_velocidad(
                rpm_obj, t_anterior, cuenta_anterior, correccion, t_sup, t_inf
            )
            time.sleep_ms(40)

    elif perfil == "C":
        etapas_ciclo = [
            (110.0, 900),
            (0.0, 450),
            (-110.0, 900),
            (0.0, 450),
        ]
        for _ in range(NUM_REPETICIONES):
            for val, duracion_etapa in etapas_ciclo:
                t_etapa = time.ticks_ms()
                while time.ticks_diff(time.ticks_ms(), t_etapa) < duracion_etapa:
                    if revisar_interrupcion():
                        return
                    t_anterior, cuenta_anterior, correccion = controlar_paso_velocidad(
                        val, t_anterior, cuenta_anterior, correccion, t_sup, t_inf
                    )
                    time.sleep_ms(35)

    elif perfil == "E":
        t_subida = 3.5
        t_pausa = 0.8
        periodo = t_subida + t_pausa
        repeticiones_exp = 3
        duracion_total = repeticiones_exp * periodo
        while True:
            if revisar_interrupcion():
                return
            t = time.ticks_diff(time.ticks_ms(), t_inicio) / 1000.0
            if t >= duracion_total:
                break
            t_fase = t % periodo
            if t_fase < t_subida:
                norm = t_fase / t_subida
                rpm_obj = 135.0 * ((math.pow(4.0, norm) - 1.0) / 3.0)
            else:
                rpm_obj = 0.0
            t_anterior, cuenta_anterior, correccion = controlar_paso_velocidad(
                rpm_obj, t_anterior, cuenta_anterior, correccion, t_sup, t_inf
            )
            time.sleep_ms(40)

    elif perfil == "D":
        t_subida = 1.8
        t_pausa = 0.5
        periodo = t_subida + t_pausa
        duracion_total = NUM_REPETICIONES * periodo
        while True:
            if revisar_interrupcion():
                return
            t = time.ticks_diff(time.ticks_ms(), t_inicio) / 1000.0
            if t >= duracion_total:
                break
            t_fase = t % periodo
            rpm_obj = (120.0 * (t_fase / t_subida)) if (t_fase < t_subida) else 0.0
            t_anterior, cuenta_anterior, correccion = controlar_paso_velocidad(
                rpm_obj, t_anterior, cuenta_anterior, correccion, t_sup, t_inf
            )
            time.sleep_ms(40)

    elif perfil == "B":
        duracion_campana = 2.2
        for _ in range(NUM_REPETICIONES):
            t_onda = time.ticks_ms()
            while True:
                if revisar_interrupcion():
                    return
                t = time.ticks_diff(time.ticks_ms(), t_onda) / 1000.0
                if t >= duracion_campana:
                    break
                x = ((t / duracion_campana) * 8.0) - 4.0
                rpm_obj = 120.0 * math.exp(-(x * x) / 2.0)
                if rpm_obj < 2.0:
                    rpm_obj = 0.0
                t_anterior, cuenta_anterior, correccion = controlar_paso_velocidad(
                    rpm_obj, t_anterior, cuenta_anterior, correccion, t_sup, t_inf
                )
                time.sleep_ms(40)

            t_pausa = time.ticks_ms()
            while time.ticks_diff(time.ticks_ms(), t_pausa) < 500:
                if revisar_interrupcion():
                    return
                t_anterior, cuenta_anterior, correccion = controlar_paso_velocidad(
                    0.0, t_anterior, cuenta_anterior, correccion, t_sup, t_inf
                )
                time.sleep_ms(40)
    else:
        print("Perfil no reconocido.")
        return

    mover_motor(0)
    print(">>> Perfil concluido. Reposo (0). <<<")

# ==========================================================
# INICIO Y BUCLE PRINCIPAL
# ==========================================================

def iniciar():
    global modo_actual, potencia_actual

    mover_motor(0)
    time.sleep_ms(500)
    v0 = leer_voltaje()
    print(f"Voltaje leido en sistema: {v0:.2f} V")

    print("\n[INICIO] El sistema se ha encendido o reiniciado.")
    print("[INICIO] Buscando automaticamente el punto de referencia (0°)...")

    if buscar_y_fijar_cero():
        modo_actual = "MODO_POSICION"
        print(f"Posicion actual: {posicion_estimada:.1f} grados -> Ingresa objetivo:")
    else:
        modo_actual = "MODO_VELOCIDAD"
        print("[ADVERTENCIA] No se completo el homing.")

    limpiar_buffer_serial()

    while True:
        linea = leer_linea_serial()

        if linea:
            if linea == "X":
                mover_motor(0)
                potencia_actual = 0.0
                print("\n>>> PARO DE EMERGENCIA ACTIVADO (X) <<<")
                continue

            if linea == "E" and modo_actual != "MODO_GRAFICAS":
                mover_motor(0)
                probar_encoder_manual()
                limpiar_buffer_serial()
                continue

            if modo_actual == "MODO_GRAFICAS":
                if linea in ("Q", "SALIR"):
                    mover_motor(0)
                    modo_actual = "MODO_POSICION"
                    print("Saliendo del Modo Graficas -> Modo Posicion.")
                else:
                    ejecutar_perfil_grafica(linea)
                    print("--- Perfil finalizado ---")

            elif modo_actual == "MODO_POSICION":
                if linea in ("Q", "SALIR"):
                    mover_motor(0)
                    modo_actual = "MODO_VELOCIDAD"
                    print("Saliendo del Modo Posicion -> Modo Velocidad.")
                elif linea in ("C", "CERO", "G"):
                    buscar_y_fijar_cero()
                    limpiar_buffer_serial()
                elif linea == "L":
                    modo_actual = "MODO_GRAFICAS"
                    print("Modo Graficas activado.")
                else:
                    linea = linea.replace(",", ".")
                    try:
                        objetivo = float(linea)
                        if math.isfinite(objetivo):
                            ir_a_posicion(objetivo)
                        else:
                            print("Entrada invalida.")
                    except ValueError:
                        print("Entrada invalida.")

            else:  # MODO_VELOCIDAD
                if linea in ("G", "C", "CERO"):
                    if buscar_y_fijar_cero():
                        modo_actual = "MODO_POSICION"
                    limpiar_buffer_serial()
                elif linea in ("P", "V"):
                    modo_actual = "MODO_VELOCIDAD"
                    potencia_actual = 0.0
                    mover_motor(0)
                    print("Motor detenido. Potencia en 0%.")
                elif linea == "L":
                    modo_actual = "MODO_GRAFICAS"
                    print("Modo Graficas activado.")
                elif linea.startswith("V") and len(linea) > 1:
                    try:
                        val = float(linea[1:].replace(",", "."))
                        if math.isfinite(val):
                            potencia_actual = max(-100.0, min(100.0, val))
                            mover_motor(potencia_actual)
                            print(f"Velocidad/Potencia establecida a: {potencia_actual:.1f}%")
                    except ValueError:
                        print("Comando de velocidad invalido.")
                else:
                    linea = linea.replace(",", ".")
                    try:
                        val = float(linea)
                        if math.isfinite(val):
                            potencia_actual = max(-100.0, min(100.0, val))
                            mover_motor(potencia_actual)
                            print(f"Velocidad/Potencia establecida a: {-potencia_actual:.1f}%")
                        else:
                            print("Comando no reconocido.")
                    except ValueError:
                        print("Comando no reconocido.")

        time.sleep_ms(10)

if __name__ == "__main__":
    iniciar()