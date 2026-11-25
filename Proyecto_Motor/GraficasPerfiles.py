import math
import random


# ==========================================================
# CONFIGURACION ORIGINAL DE LOS PERFILES
# ==========================================================

RPM_MAXIMA = 120.0

LIMITE_SUPERIOR = 150.0
LIMITE_INFERIOR = -150.0

PERFILES = {
    "T": "Trapecio",
    "S": "Senoide",
    "R": "Ruido blanco",
    "L": "Rampa lineal",
    "C": "Onda cuadrada",
    "E": "Exponencial",
    "D": "Diente de sierra",
    "B": "Campana de Gauss",
}


# ==========================================================
# GENERADOR DE PERFILES
# ==========================================================

class GraficasPerfiles:

    def __init__(self):

        self.perfil = None
        self.tiempo_inicio = 0.0
        self.duracion = 0.0

        self.tiempo_ultimo_cambio = 0.0
        self.valor_aleatorio = 0.0

        self.terminado = True

        self.secuencia_cuadrada = [
            0.0,
            110.0,
            0.0,
            -110.0,
            0.0,
            110.0,
            0.0,
            -110.0,
            0.0
        ]

        self.duraciones_cuadrada = [
            1.5,
            1.2,
            1.5,
            1.2,
            1.5,
            1.2,
            1.5,
            1.2,
            1.5
        ]

        self.duracion_total_cuadrada = sum(
            self.duraciones_cuadrada
        )

        self.duracion_campana = 5.0
        self.pausa_campana = 0.5
        self.repeticiones_campana = 4

    # ======================================================
    # INICIAR PERFIL
    # ======================================================

    def iniciar(self, perfil, tiempo_actual=0.0):

        perfil = str(perfil).upper().strip()

        if perfil not in PERFILES:
            raise ValueError(
                f"Perfil no reconocido: {perfil}"
            )

        self.perfil = perfil
        self.tiempo_inicio = tiempo_actual

        self.tiempo_ultimo_cambio = 0.0
        self.valor_aleatorio = 0.0

        self.terminado = False

        duraciones = {
            "T": 22.0,
            "S": 8.0,
            "R": 6.0,
            "L": 4.0,
            "C": self.duracion_total_cuadrada,
            "E": 4.0,
            "D": 10.5,
            "B": (
                self.repeticiones_campana
                * (
                    self.duracion_campana
                    + self.pausa_campana
                )
            )
        }

        self.duracion = duraciones[perfil]

        return self.duracion

    # ======================================================
    # CALCULAR VALOR DEL PERFIL
    # ======================================================

    def calcular(self, tiempo_actual):

        if self.perfil is None or self.terminado:

            return 0.0

        tiempo = tiempo_actual - self.tiempo_inicio

        if tiempo < 0:
            tiempo = 0.0

        if tiempo >= self.duracion:

            self.terminado = True
            return 0.0

        if self.perfil == "T":
            return self._trapecio(tiempo)

        elif self.perfil == "S":
            return self._senoide(tiempo)

        elif self.perfil == "R":
            return self._ruido(tiempo)

        elif self.perfil == "L":
            return self._rampa(tiempo)

        elif self.perfil == "C":
            return self._cuadrada(tiempo)

        elif self.perfil == "E":
            return self._exponencial(tiempo)

        elif self.perfil == "D":
            return self._diente_sierra(tiempo)

        elif self.perfil == "B":
            return self._campana_gauss(tiempo)

        return 0.0

    # ======================================================
    # 1. PERFIL TRAPECIO (T)
    # ======================================================

    def _trapecio(self, tiempo):

        rpm_min = 0.0
        rpm_max = 120.0

        t_rampa = 2.5
        t_meseta = 3.0

        periodo = (
            2 * t_rampa
            + 2 * t_meseta
        )

        fase = tiempo % periodo

        if fase < t_rampa:

            return (
                rpm_min
                + (rpm_max - rpm_min)
                * (fase / t_rampa)
            )

        elif fase < (t_rampa + t_meseta):

            return rpm_max

        elif fase < (2 * t_rampa + t_meseta):

            tiempo_bajada = (
                fase - (t_rampa + t_meseta)
            )

            return (
                rpm_max
                - (rpm_max - rpm_min)
                * (tiempo_bajada / t_rampa)
            )

        return 0.0

    # ======================================================
    # 2. PERFIL SENOIDE (S)
    # ======================================================

    def _senoide(self, tiempo):

        return (
            120.0
            * math.sin(
                tiempo * (2.0 * math.pi / 4.0)
            )
        )

    # ======================================================
    # 3. RUIDO BLANCO (R)
    # ======================================================

    def _ruido(self, tiempo):

        if tiempo - self.tiempo_ultimo_cambio >= 0.3:

            self.valor_aleatorio = random.randint(
                -120,
                120
            )

            if abs(self.valor_aleatorio) < 15:

                self.valor_aleatorio = 0.0

            self.tiempo_ultimo_cambio = tiempo

        return float(self.valor_aleatorio)

    # ======================================================
    # 4. RAMPA LINEAL (L)
    # ======================================================

    def _rampa(self, tiempo):

        return 130.0 * (tiempo / 4.0)

    # ======================================================
    # 5. ONDA CUADRADA (C)
    # ======================================================

    def _cuadrada(self, tiempo):

        tiempo_acumulado = 0.0

        for valor, duracion in zip(
            self.secuencia_cuadrada,
            self.duraciones_cuadrada
        ):

            if tiempo < tiempo_acumulado + duracion:

                return valor

            tiempo_acumulado += duracion

        return 0.0

    # ======================================================
    # 6. PERFIL EXPONENCIAL (E)
    # ======================================================

    def _exponencial(self, tiempo):

        duracion = 4.0

        normalizado = tiempo / duracion

        return (
            120.0
            * (math.pow(2.0, normalizado) - 1.0)
        )

    # ======================================================
    # 7. DIENTE DE SIERRA (D)
    # ======================================================

    def _diente_sierra(self, tiempo):

        t_subida = 2.5
        t_pausa = 1.0

        periodo = t_subida + t_pausa

        fase = tiempo % periodo

        if fase < t_subida:

            return (
                120.0
                * (fase / t_subida)
            )

        return 0.0

    # ======================================================
    # 8. CAMPANA DE GAUSS (B)
    # ======================================================

    def _campana_gauss(self, tiempo):

        periodo = (
            self.duracion_campana
            + self.pausa_campana
        )

        fase = tiempo % periodo

        if fase >= self.duracion_campana:

            return 0.0

        x = (
            (fase / self.duracion_campana) * 8.0
        ) - 4.0

        rpm = (
            120.0
            * math.exp(-(x * x) / 2.0)
        )

        if rpm < 2.0:

            return 0.0

        return rpm

    # ======================================================
    # LIMITES DE REFERENCIA PARA LAS GRAFICAS
    # ======================================================

    def obtener_limites(self):

        if self.perfil == "L":

            return 150.0, 0.0

        return (
            LIMITE_SUPERIOR,
            LIMITE_INFERIOR
        )

    # ======================================================
    # ESTADO DEL PERFIL
    # ======================================================

    def esta_activo(self):

        return (
            self.perfil is not None
            and not self.terminado
        )

    def detener(self):

        self.terminado = True
        self.perfil = None
        self.duracion = 0.0

        return 0.0

    def obtener_nombre(self):

        if self.perfil is None:
            return "Sin perfil"

        return PERFILES.get(
            self.perfil,
            "Desconocido"
        )