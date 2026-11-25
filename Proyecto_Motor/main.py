import sys
import time
from collections import deque

import serial.tools.list_ports
import pyqtgraph as pg

from PySide6.QtCore import Qt, QTimer, QObject, Signal
from PySide6.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QLabel,
    QPushButton,
    QComboBox,
    QSlider,
    QSpinBox,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QGroupBox,
    QTextEdit,
    QMessageBox,
    QScrollArea,
    QSplitter,
    QSizePolicy,
)

from MotorControl import MotorControl


BAUDIOS = 115200
MAX_PUNTOS = 1200


# ==========================================================
# COMUNICACION ENTRE HILOS E INTERFAZ
# ==========================================================

class Puente(QObject):

    nuevos_datos = Signal(object)
    nuevo_mensaje = Signal(str)
    nuevo_estado = Signal(object)


# ==========================================================
# VENTANA PRINCIPAL
# ==========================================================

class VentanaPrincipal(QMainWindow):

    def __init__(self):

        super().__init__()

        self.setWindowTitle("Control de Motor - ESP32")
        self.resize(1450, 900)
        self.setMinimumSize(650, 500)

        self.controlador = MotorControl()

        self.puente = Puente()
        self.puente.nuevos_datos.connect(self.recibir_datos)
        self.puente.nuevo_mensaje.connect(self.registrar)
        self.puente.nuevo_estado.connect(
            self.actualizar_estado_motor
        )

        self.controlador.on_data = self.puente.nuevos_datos.emit
        self.controlador.on_message = self.puente.nuevo_mensaje.emit
        self.controlador.on_status = self.puente.nuevo_estado.emit

        self.tiempo_inicio = time.time()
        self.en_modo_graficas = False
        self.en_modo_posicion = True

        self.datos_tiempo = deque(maxlen=MAX_PUNTOS)
        self.datos_objetivo = deque(maxlen=MAX_PUNTOS)
        self.datos_reales = deque(maxlen=MAX_PUNTOS)
        self.datos_cero = deque(maxlen=MAX_PUNTOS)
        self.datos_tope_sup = deque(maxlen=MAX_PUNTOS)
        self.datos_tope_inf = deque(maxlen=MAX_PUNTOS)

        self.crear_interfaz()
        self.aplicar_estilos()
        self.actualizar_puertos()

        self.temporizador = QTimer()
        self.temporizador.timeout.connect(
            self.actualizar_graficas
        )
        self.temporizador.start(100)

    # ======================================================
    # INTERFAZ
    # ======================================================

    def crear_interfaz(self):

        central = QWidget()
        self.setCentralWidget(central)

        principal = QVBoxLayout(central)
        principal.setContentsMargins(10, 8, 10, 8)
        principal.setSpacing(8)

        # --------------------------------------------------
        # ENCABEZADO
        # --------------------------------------------------

        encabezado = QHBoxLayout()

        titulo = QLabel("CONTROL DE MOTOR")
        titulo.setObjectName("titulo")

        self.estado = QLabel("DESCONECTADO")
        self.estado.setObjectName("estado_desconectado")

        encabezado.addWidget(titulo)
        encabezado.addStretch()
        encabezado.addWidget(self.estado)

        principal.addLayout(encabezado)

        # --------------------------------------------------
        # CONEXION
        # --------------------------------------------------

        grupo_conexion = QGroupBox("CONEXIÓN DEL SISTEMA")
        conexion_layout = QHBoxLayout()

        self.combo_puertos = QComboBox()
        self.combo_puertos.setMinimumWidth(120)

        self.boton_actualizar = QPushButton(
            "Actualizar puertos"
        )
        self.boton_actualizar.clicked.connect(
            self.actualizar_puertos
        )

        self.boton_conectar = QPushButton("Conectar")
        self.boton_conectar.clicked.connect(self.conectar)

        self.boton_desconectar = QPushButton("Desconectar")
        self.boton_desconectar.clicked.connect(
            self.desconectar
        )
        self.boton_desconectar.setEnabled(False)

        conexion_layout.addWidget(QLabel("Puerto COM:"))
        conexion_layout.addWidget(self.combo_puertos, 2)
        conexion_layout.addWidget(self.boton_actualizar)
        conexion_layout.addWidget(self.boton_conectar)
        conexion_layout.addWidget(self.boton_desconectar)

        grupo_conexion.setLayout(conexion_layout)
        principal.addWidget(grupo_conexion)

        # --------------------------------------------------
        # PANEL IZQUIERDO: CONTROLES
        # --------------------------------------------------

        panel_controles = QWidget()
        panel_controles.setMinimumWidth(250)

        controles_layout = QVBoxLayout(panel_controles)
        controles_layout.setContentsMargins(4, 4, 4, 4)
        controles_layout.setSpacing(10)

        # CONTROL DE VELOCIDAD

        grupo_velocidad = QGroupBox("CONTROL DE VELOCIDAD")
        velocidad_layout = QVBoxLayout()

        velocidad_layout.addWidget(
            QLabel("Potencia del motor (%)")
        )

        self.valor_potencia = QLabel("0 %")
        self.valor_potencia.setAlignment(
            Qt.AlignmentFlag.AlignCenter
        )
        self.valor_potencia.setObjectName("valor_grande")

        self.slider_potencia = QSlider(
            Qt.Orientation.Horizontal
        )
        self.slider_potencia.setRange(-100, 100)
        self.slider_potencia.setValue(0)

        self.slider_potencia.valueChanged.connect(
            self.actualizar_valor_potencia
        )

        velocidad_layout.addWidget(self.valor_potencia)
        velocidad_layout.addWidget(self.slider_potencia)

        botones_velocidad = QHBoxLayout()

        self.boton_aplicar = QPushButton("Aplicar potencia")
        self.boton_aplicar.clicked.connect(
            self.aplicar_potencia
        )

        self.boton_parar = QPushButton("DETENER")
        self.boton_parar.setObjectName("boton_parar")
        self.boton_parar.clicked.connect(self.detener_motor)

        botones_velocidad.addWidget(self.boton_aplicar)
        botones_velocidad.addWidget(self.boton_parar)

        velocidad_layout.addLayout(botones_velocidad)
        grupo_velocidad.setLayout(velocidad_layout)

        controles_layout.addWidget(grupo_velocidad)

        # CONTROL DE POSICION

        grupo_posicion = QGroupBox("CONTROL DE POSICIÓN")
        posicion_layout = QVBoxLayout()

        posicion_layout.addWidget(
            QLabel("Ángulo objetivo (grados)")
        )

        self.angulo = QSpinBox()
        self.angulo.setRange(-3600, 3600)
        self.angulo.setValue(90)
        self.angulo.setSuffix(" °")

        self.boton_posicion = QPushButton("Ir a posición")
        self.boton_posicion.clicked.connect(
            self.ir_a_posicion
        )

        self.boton_homing = QPushButton(
            "Buscar cero (Homing)"
        )
        self.boton_homing.clicked.connect(self.homing)

        posicion_layout.addWidget(self.angulo)
        posicion_layout.addWidget(self.boton_posicion)
        posicion_layout.addWidget(self.boton_homing)

        grupo_posicion.setLayout(posicion_layout)
        controles_layout.addWidget(grupo_posicion)

        # PERFILES

        grupo_perfiles = QGroupBox("PERFILES DINÁMICOS")
        perfiles_layout = QVBoxLayout()

        self.combo_perfiles = QComboBox()

        self.combo_perfiles.addItem("Trapecio (T)", "T")
        self.combo_perfiles.addItem("Senoide (S)", "S")
        self.combo_perfiles.addItem("Ruido blanco (R)", "R")
        self.combo_perfiles.addItem("Rampa lineal (L)", "L")
        self.combo_perfiles.addItem("Onda cuadrada (C)", "C")
        self.combo_perfiles.addItem("Exponencial (E)", "E")
        self.combo_perfiles.addItem("Diente de sierra (D)", "D")
        self.combo_perfiles.addItem("Campana de Gauss (B)", "B")

        self.boton_perfil = QPushButton("Ejecutar perfil")
        self.boton_perfil.clicked.connect(
            self.ejecutar_perfil
        )

        perfiles_layout.addWidget(self.combo_perfiles)
        perfiles_layout.addWidget(self.boton_perfil)

        grupo_perfiles.setLayout(perfiles_layout)
        controles_layout.addWidget(grupo_perfiles)

        # PARO DE EMERGENCIA

        self.boton_emergencia = QPushButton(
            "PARO DE EMERGENCIA"
        )
        self.boton_emergencia.setObjectName(
            "boton_emergencia"
        )
        self.boton_emergencia.clicked.connect(
            self.paro_emergencia
        )

        controles_layout.addWidget(self.boton_emergencia)
        controles_layout.addStretch()

        # CONTENEDOR CON DESPLAZAMIENTO

        self.scroll_controles = QScrollArea()
        self.scroll_controles.setWidgetResizable(True)
        self.scroll_controles.setWidget(panel_controles)
        self.scroll_controles.setFrameShape(
            QScrollArea.Shape.NoFrame
        )
        self.scroll_controles.setMinimumWidth(260)

        # --------------------------------------------------
        # PANEL DERECHO: GRAFICAS Y MONITOR
        # --------------------------------------------------

        panel_derecho = QWidget()
        derecho_layout = QVBoxLayout(panel_derecho)
        derecho_layout.setContentsMargins(4, 4, 4, 4)
        derecho_layout.setSpacing(8)

        grupo_grafica = QGroupBox(
            "RESPUESTA DEL MOTOR EN TIEMPO REAL"
        )

        grafica_layout = QVBoxLayout()

        self.grafica = pg.PlotWidget()
        self.grafica.setBackground("#151a23")
        self.grafica.showGrid(
            x=True,
            y=True,
            alpha=0.25
        )

        self.grafica.setLabel(
            "left",
            "Velocidad",
            units="RPM"
        )

        self.grafica.setLabel(
            "bottom",
            "Tiempo",
            units="s"
        )

        self.grafica.addLegend()

        self.curva_objetivo = self.grafica.plot(
            pen=pg.mkPen("#00bfff", width=2),
            name="RPM objetivo"
        )

        self.curva_real = self.grafica.plot(
            pen=pg.mkPen("#00ff88", width=2),
            name="RPM reales"
        )

        self.curva_cero = self.grafica.plot(
            pen=pg.mkPen(
                "#ffffff",
                width=1,
                style=Qt.PenStyle.DotLine
            ),
            name="Cero"
        )

        self.curva_sup = self.grafica.plot(
            pen=pg.mkPen(
                "#ff5555",
                width=1,
                style=Qt.PenStyle.DashLine
            ),
            name="Límite superior"
        )

        self.curva_inf = self.grafica.plot(
            pen=pg.mkPen(
                "#ffaa00",
                width=1,
                style=Qt.PenStyle.DashLine
            ),
            name="Límite inferior"
        )

        grafica_layout.addWidget(self.grafica)
        grupo_grafica.setLayout(grafica_layout)

        derecho_layout.addWidget(grupo_grafica, 4)

        # MONITOR

        grupo_monitor = QGroupBox("MONITOR DEL SISTEMA")
        monitor_layout = QVBoxLayout()

        self.monitor = QTextEdit()
        self.monitor.setReadOnly(True)
        self.monitor.setMinimumHeight(90)
        self.monitor.setMaximumHeight(180)

        monitor_layout.addWidget(self.monitor)
        grupo_monitor.setLayout(monitor_layout)

        derecho_layout.addWidget(grupo_monitor, 1)

        # --------------------------------------------------
        # DIVISOR REDIMENSIONABLE
        # --------------------------------------------------

        self.divisor = QSplitter(
            Qt.Orientation.Horizontal
        )

        self.divisor.addWidget(self.scroll_controles)
        self.divisor.addWidget(panel_derecho)

        self.divisor.setStretchFactor(0, 1)
        self.divisor.setStretchFactor(1, 3)

        self.divisor.setSizes([390, 1100])

        principal.addWidget(self.divisor, 1)

        # --------------------------------------------------
        # INDICADORES INFERIORES
        # --------------------------------------------------

        grupo_estado = QGroupBox("ESTADO DEL MOTOR")
        estado_layout = QGridLayout()

        self.indicador_modo = QLabel("Modo: --")
        self.indicador_objetivo = QLabel("RPM objetivo: 0.0")
        self.indicador_real = QLabel("RPM reales: 0.0")
        self.indicador_potencia = QLabel("Potencia: 0.0 %")
        self.indicador_posicion = QLabel("Posición: 0.0 °")

        self.indicadores = [
            self.indicador_modo,
            self.indicador_objetivo,
            self.indicador_real,
            self.indicador_potencia,
            self.indicador_posicion,
        ]

        for indicador in self.indicadores:
            indicador.setObjectName("indicador")
            indicador.setWordWrap(True)
            indicador.setMinimumWidth(100)
            indicador.setSizePolicy(
                QSizePolicy.Policy.Expanding,
                QSizePolicy.Policy.Preferred
            )

        estado_layout.addWidget(self.indicador_modo, 0, 0)
        estado_layout.addWidget(self.indicador_objetivo, 0, 1)
        estado_layout.addWidget(self.indicador_real, 0, 2)
        estado_layout.addWidget(self.indicador_potencia, 0, 3)
        estado_layout.addWidget(self.indicador_posicion, 0, 4)

        grupo_estado.setLayout(estado_layout)
        principal.addWidget(grupo_estado)

    # ======================================================
    # AJUSTE AUTOMATICO SEGUN TAMAÑO DE PANTALLA
    # ======================================================

    def resizeEvent(self, evento):

        super().resizeEvent(evento)

        ancho = self.width()

        if ancho < 1000:

            self.divisor.setOrientation(
                Qt.Orientation.Vertical
            )

            self.divisor.setSizes([360, 600])

        else:

            self.divisor.setOrientation(
                Qt.Orientation.Horizontal
            )

            if ancho < 1250:
                self.divisor.setSizes([320, ancho - 350])
            else:
                self.divisor.setSizes([390, ancho - 420])

    # ======================================================
    # ESTILOS
    # ======================================================

    def aplicar_estilos(self):

        self.setStyleSheet("""
            QMainWindow {
                background-color: #10141c;
            }

            QWidget {
                color: #e6eaf0;
                font-size: 13px;
            }

            QLabel#titulo {
                font-size: 25px;
                font-weight: bold;
                color: #00bfff;
                padding: 8px;
            }

            QLabel#valor_grande {
                font-size: 28px;
                font-weight: bold;
                color: #00ff88;
                padding: 8px;
            }

            QLabel#estado_desconectado {
                color: #ff5555;
                font-weight: bold;
                padding: 8px;
            }

            QLabel#indicador {
                color: #00d5ff;
                font-weight: bold;
                padding: 7px;
            }

            QGroupBox {
                border: 1px solid #394354;
                border-radius: 8px;
                margin-top: 12px;
                padding: 10px;
                font-weight: bold;
                color: #00bfff;
            }

            QGroupBox::title {
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 5px;
            }

            QPushButton {
                background-color: #263449;
                border: 1px solid #40536c;
                border-radius: 6px;
                padding: 9px;
                color: white;
                font-weight: bold;
            }

            QPushButton:hover {
                background-color: #345273;
            }

            QPushButton:disabled {
                background-color: #202630;
                color: #737d8c;
            }

            QPushButton#boton_parar {
                background-color: #a83232;
            }

            QPushButton#boton_emergencia {
                background-color: #e02020;
                color: white;
                font-size: 15px;
                padding: 14px;
            }

            QComboBox, QSpinBox {
                background-color: #202938;
                border: 1px solid #40536c;
                border-radius: 5px;
                padding: 7px;
            }

            QSlider::groove:horizontal {
                background: #394354;
                height: 7px;
                border-radius: 3px;
            }

            QSlider::handle:horizontal {
                background: #00bfff;
                width: 18px;
                margin: -6px 0;
                border-radius: 9px;
            }

            QTextEdit {
                background-color: #151a23;
                border: 1px solid #394354;
                border-radius: 5px;
                color: #00ff88;
                font-family: Consolas;
            }

            QScrollArea {
                border: none;
                background-color: transparent;
            }

            QScrollBar:vertical {
                background: #151a23;
                width: 10px;
                margin: 0;
            }

            QScrollBar::handle:vertical {
                background: #40536c;
                border-radius: 4px;
                min-height: 25px;
            }

            QScrollBar::add-line:vertical,
            QScrollBar::sub-line:vertical {
                height: 0;
            }
        """)

    # ======================================================
    # PUERTOS SERIALES
    # ======================================================

    def actualizar_puertos(self):

        self.combo_puertos.clear()

        puertos = serial.tools.list_ports.comports()

        for puerto in puertos:

            self.combo_puertos.addItem(
                f"{puerto.device} - {puerto.description}",
                puerto.device
            )

    # ======================================================
    # CONEXION
    # ======================================================

    def conectar(self):

        seleccion = self.combo_puertos.currentData()

        if seleccion:

            conectado = self.controlador.conectar_serial(
                seleccion,
                BAUDIOS
            )

            if not conectado:

                QMessageBox.critical(
                    self,
                    "Error de conexión",
                    "No fue posible conectar con el ESP32."
                )

                return

            self.en_modo_graficas = False
            self.en_modo_posicion = True

            self.estado.setText(
                f"ESP32 CONECTADO: {seleccion}"
            )

            self.estado.setStyleSheet(
                "color: #00ff88; font-weight: bold;"
            )

        else:

            QMessageBox.warning(
                self,
                "Conexión",
                "Selecciona un puerto COM disponible."
            )
            return

        self.boton_conectar.setEnabled(False)
        self.boton_desconectar.setEnabled(True)
        self.combo_puertos.setEnabled(False)

    def desconectar(self):

        self.controlador.desconectar()
        self.en_modo_graficas = False
        self.en_modo_posicion = True

        self.estado.setText("DESCONECTADO")
        self.estado.setStyleSheet(
            "color: #ff5555; font-weight: bold;"
        )

        self.boton_conectar.setEnabled(True)
        self.boton_desconectar.setEnabled(False)
        self.combo_puertos.setEnabled(True)

        self.estado_motor_limpio()

    def estado_motor_limpio(self):

        self.indicador_modo.setText("Modo: --")
        self.indicador_objetivo.setText("RPM objetivo: 0.0")
        self.indicador_real.setText("RPM reales: 0.0")
        self.indicador_potencia.setText("Potencia: 0.0 %")
        self.indicador_posicion.setText("Posición: 0.0 °")

    def closeEvent(self, evento):

        self.controlador.desconectar()
        evento.accept()

    # ======================================================
    # PREPARAR MODO VELOCIDAD EN EL ESP32
    # ======================================================

    def preparar_modo_velocidad(self, comando_final):

        self.en_modo_graficas = False
        self.en_modo_posicion = False

        if not self.controlador.enviar_comando("Q"):
            return

        def salir_de_posicion():

            if not self.controlador.enviar_comando("Q"):
                return

            QTimer.singleShot(
                150,
                lambda: self.controlador.enviar_comando(
                    comando_final
                )
            )

        QTimer.singleShot(
            150,
            salir_de_posicion
        )

    # ======================================================
    # CONTROL DE VELOCIDAD
    # ======================================================

    def actualizar_valor_potencia(self, valor):

        self.valor_potencia.setText(f"{valor} %")

    def aplicar_potencia(self):

        potencia = self.slider_potencia.value()
        potencia_esp32 = -potencia

        self.preparar_modo_velocidad(
            str(potencia_esp32)
        )

    def detener_motor(self):

        self.slider_potencia.setValue(0)
        self.preparar_modo_velocidad("P")

    # ======================================================
    # CONTROL DE POSICION
    # ======================================================

    def ir_a_posicion(self):

        angulo = self.angulo.value()

        if self.en_modo_graficas:
            # Salir de MODO_GRAFICAS hacia MODO_POSICION conservando memoria
            self.en_modo_graficas = False
            self.en_modo_posicion = True
            if self.controlador.enviar_comando("Q"):
                QTimer.singleShot(
                    250,
                    lambda: self.controlador.enviar_comando(str(angulo))
                )
        elif not self.en_modo_posicion:
            # Si estaba en MODO_VELOCIDAD, hacer Homing para recalibrar el 0
            self.en_modo_posicion = True
            if self.controlador.enviar_comando("G"):
                QTimer.singleShot(
                    250,
                    lambda: self.controlador.enviar_comando(str(angulo))
                )
        else:
            # Ya estamos en MODO_POSICION: enviar directamente el ángulo (180, 360, 720, 0...)
            self.controlador.enviar_comando(str(angulo))

    def homing(self):

        self.en_modo_graficas = False
        self.en_modo_posicion = True

        if self.controlador.enviar_comando("G"):

            QTimer.singleShot(
                400,
                lambda: self.controlador.enviar_comando("C")
            )

    # ======================================================
    # PERFILES
    # ======================================================

    def ejecutar_perfil(self):

        perfil = self.combo_perfiles.currentData()

        if not self.en_modo_graficas:
            if self.controlador.enviar_comando("L"):
                self.en_modo_graficas = True
                self.en_modo_posicion = False
                QTimer.singleShot(
                    400,
                    lambda: self.controlador.enviar_comando(perfil)
                )
        else:
            self.controlador.enviar_comando(perfil)

    # ======================================================
    # PARO DE EMERGENCIA
    # ======================================================

    def paro_emergencia(self):

        self.slider_potencia.setValue(0)
        self.controlador.enviar_comando("X")

        self.registrar(
            "PARO DE EMERGENCIA SOLICITADO."
        )

    # ======================================================
    # RECEPCION DE DATOS
    # ======================================================

    def recibir_datos(self, valores):

        if len(valores) != 5:
            return

        tiempo = time.time() - self.tiempo_inicio

        self.datos_tiempo.append(tiempo)
        self.datos_objetivo.append(valores[0])
        self.datos_reales.append(valores[1])
        self.datos_cero.append(valores[2])
        self.datos_tope_sup.append(valores[3])
        self.datos_tope_inf.append(valores[4])

    # ======================================================
    # ACTUALIZACION DE GRAFICAS
    # ======================================================

    def actualizar_graficas(self):

        if not self.datos_tiempo:
            return

        tiempo = list(self.datos_tiempo)

        self.curva_objetivo.setData(
            tiempo,
            list(self.datos_objetivo)
        )

        self.curva_real.setData(
            tiempo,
            list(self.datos_reales)
        )

        self.curva_cero.setData(
            tiempo,
            list(self.datos_cero)
        )

        self.curva_sup.setData(
            tiempo,
            list(self.datos_tope_sup)
        )

        self.curva_inf.setData(
            tiempo,
            list(self.datos_tope_inf)
        )

        self.grafica.setXRange(
            max(0, tiempo[-1] - 25),
            max(25, tiempo[-1]),
            padding=0
        )

    # ======================================================
    # ESTADO DEL MOTOR
    # ======================================================

    def actualizar_estado_motor(self, datos):

        rpm_objetivo = datos.get("rpm_objetivo", 0)
        rpm_real = datos.get("rpm_real", 0)
        potencia = datos.get("potencia", 0)
        posicion = datos.get("posicion", 0)
        modo = datos.get("modo", "desconocido")

        self.indicador_modo.setText(
            f"Modo: {modo.upper()}"
        )

        self.indicador_objetivo.setText(
            f"RPM objetivo: {rpm_objetivo:.1f}"
        )

        self.indicador_real.setText(
            f"RPM reales: {rpm_real:.1f}"
        )

        self.indicador_potencia.setText(
            f"Potencia: {potencia:.1f} %"
        )

        self.indicador_posicion.setText(
            f"Posición: {posicion:.1f} °"
        )

    # ======================================================
    # MONITOR
    # ======================================================

    def registrar(self, mensaje):

        hora = time.strftime("%H:%M:%S")

        self.monitor.append(
            f"[{hora}] {mensaje}"
        )


# ==========================================================
# INICIO DE LA APLICACION
# ==========================================================

if __name__ == "__main__":

    aplicacion = QApplication(sys.argv)

    pg.setConfigOptions(antialias=True)

    ventana = VentanaPrincipal()
    ventana.show()

    sys.exit(aplicacion.exec())