import threading
import time

try:
    import serial
except ImportError:
    serial = None


class MotorControl:
    """
    Controlador del motor mediante comunicación serial con ESP32 por USB.
    """

    def __init__(self):

        self.conexion = None
        self.hilo = None
        self.activo = False

        self.on_data = None
        self.on_message = None
        self.on_status = None

        self.lock = threading.Lock()

        # Estado del motor
        self.rpm_objetivo = 0.0
        self.rpm_real = 0.0
        self.potencia = 0.0

        self.posicion = 0.0
        self.posicion_objetivo = 0.0

        self.modo_motor = "velocidad"

    # ======================================================
    # CALLBACKS
    # ======================================================

    def mensaje(self, texto):

        if self.on_message:
            self.on_message(str(texto))

    def enviar_datos(self, objetivo, real, cero, sup, inf):

        if self.on_data:

            self.on_data([
                float(objetivo),
                float(real),
                float(cero),
                float(sup),
                float(inf)
            ])

    def actualizar_estado(self):

        if self.on_status:

            self.on_status({
                "rpm_objetivo": self.rpm_objetivo,
                "rpm_real": self.rpm_real,
                "potencia": self.potencia,
                "posicion": self.posicion,
                "posicion_objetivo": self.posicion_objetivo,
                "modo": self.modo_motor,
                "conectado": self.esta_conectado()
            })

    # ======================================================
    # CONEXION SERIAL
    # ======================================================

    def conectar_serial(self, puerto, baudios=115200):

        if serial is None:

            self.mensaje(
                "Error: instala pyserial para utilizar el puerto serial."
            )

            return False

        self.desconectar()

        try:

            self.conexion = serial.Serial(
                puerto,
                baudios,
                timeout=0.1
            )

            time.sleep(2)
            self.conexion.reset_input_buffer()

            self.activo = True

            self.hilo = threading.Thread(
                target=self._bucle_serial,
                daemon=True
            )

            self.hilo.start()

            self.mensaje(f"ESP32 conectado en {puerto}.")
            self.actualizar_estado()

            return True

        except Exception as error:

            self.conexion = None
            self.mensaje(f"Error de conexión: {error}")

            return False

    # ======================================================
    # ENVIO DE COMANDOS
    # ======================================================

    def enviar_comando(self, comando):

        comando = str(comando).strip()

        if not comando:
            return False

        try:

            if self.conexion and self.conexion.is_open:

                self.conexion.write(
                    (comando + "\n").encode("utf-8")
                )

                self.mensaje(f"TX: {comando}")

                return True

        except Exception as error:

            self.mensaje(f"Error serial: {error}")
            return False

        self.mensaje("No hay conexión activa.")

        return False

    # ======================================================
    # LECTURA SERIAL DEL ESP32
    # ======================================================

    def _bucle_serial(self):

        while self.activo:

            try:

                if self.conexion and self.conexion.in_waiting:

                    linea = (
                        self.conexion.readline()
                        .decode("utf-8", errors="ignore")
                        .strip()
                    )

                    if linea:
                        self._procesar_linea_serial(linea)

                else:

                    time.sleep(0.01)

            except Exception as error:

                self.mensaje(
                    f"Error de lectura serial: {error}"
                )

                break

    def _procesar_linea_serial(self, linea):

        partes = linea.split(",")

        if len(partes) == 5:

            try:

                valores = [
                    float(x.strip())
                    for x in partes
                ]

                self.enviar_datos(*valores)

                with self.lock:
                    self.rpm_objetivo = valores[0]
                    self.rpm_real = valores[1]

                self.actualizar_estado()

            except ValueError:

                self.mensaje(linea)

        else:

            self.mensaje("RX: " + linea)

    # ======================================================
    # DESCONEXION
    # ======================================================

    def desconectar(self):

        self.activo = False

        hilo_actual = self.hilo

        if hilo_actual and hilo_actual.is_alive():

            if hilo_actual != threading.current_thread():

                hilo_actual.join(timeout=1.0)

        self.hilo = None

        if self.conexion:

            try:
                self.conexion.close()

            except Exception:
                pass

            self.conexion = None

        with self.lock:

            self.rpm_objetivo = 0.0
            self.rpm_real = 0.0
            self.potencia = 0.0
            self.posicion = 0.0
            self.posicion_objetivo = 0.0

        self.mensaje("Controlador desconectado.")

    # ======================================================
    # ESTADO
    # ======================================================

    def esta_conectado(self):

        return (
            self.conexion is not None
            and self.conexion.is_open
        )