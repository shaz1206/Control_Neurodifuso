"""
Interfaz gráfica para todo el proyecto: simulación y HIL desde una sola ventana.

  - Modo "Simulación": el controlador difuso y el motor corren en la PC.
  - Modo "HIL": el controlador corre en el ESP32; el motor se simula en la PC
    y el voltaje también se aplica al motor real.
  - Mamdani / Sugeno / PI clásico, escenario configurable, gráficas en vivo,
    tabla de métricas, comparación de corridas y control manual del motor.

Uso:  python interfaz.py
"""
import csv
import os
import queue
import sys
import threading
import time
import tkinter as tk
from datetime import datetime
from tkinter import messagebox, ttk

import matplotlib
import numpy as np
import serial
import serial.tools.list_ports

matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(AQUI, "..", "fase_a"))
from controlador import RPM_MAX, TS, ControladorDifuso, ControladorPI  # noqa: E402
from motor_dc import MotorDC  # noqa: E402
from simular import DT_PLANTA, metricas  # noqa: E402


CARPETA = os.path.join(AQUI, "resultados")
NOMBRES = {"mamdani": "Mamdani", "sugeno": "Sugeno", "pi": "PI clásico"}
COMANDO_METODO = {"mamdani": b"m\n", "sugeno": b"s\n", "pi": b"i\n"}  # letra en el firmware


class Escenario:
    """Referencia en dos escalones y un par de carga que entra en t_carga."""

    def __init__(self, ref1, ref2, t_cambio, t_carga, par_carga, duracion):
        self.ref1, self.ref2 = ref1, ref2
        self.t_cambio, self.t_carga = t_cambio, t_carga
        self.par_carga, self.duracion = par_carga, duracion

    def referencia(self, t):
        return self.ref1 if t < self.t_cambio else self.ref2

    def carga(self, t):
        return self.par_carga if t >= self.t_carga else 0.0


def esperar_respuesta(ser, prefijo, timeout=2.0):
    """Como esperar_linea, pero si falla explica qué fue lo que sí llegó."""
    vistas = []
    limite = time.time() + timeout
    while time.time() < limite:
        linea = ser.readline().decode(errors="ignore").strip()
        if not linea:
            continue
        if linea.startswith(prefijo):
            return linea
        vistas.append(linea)
    raise TimeoutError(diagnosticar(vistas, prefijo))


def diagnosticar(vistas, prefijo):
    if any("Brownout" in l for l in vistas):
        return ("El ESP32 se REINICIÓ por caída de voltaje (brownout).\n\n"
                "El motor le está robando energía o metiendo ruido. Revisa la fuente, "
                "el GND común y pon un capacitor en el motor.")
    if any(l.startswith("# t_ms") or "rst:" in l or "ets " in l for l in vistas):
        return ("El ESP32 se REINICIÓ a mitad de la prueba.\n\n"
                "Casi siempre es ruido o caída de voltaje cuando arranca el motor.")
    if any(l.count(",") == 4 and l[:1].isdigit() for l in vistas):
        return ("El ESP32 salió del modo HIL (está mandando datos normales).\n\n"
                "Probablemente se reinició. Vuelve a presionar Ejecutar.")
    ultimas = " | ".join(vistas[-4:]) or "(no llegó nada)"
    return (f"El ESP32 no respondió con '{prefijo}'.\n\nÚltimo que llegó: {ultimas}\n\n"
            "¿Se desconectó el USB o está cargado el firmware correcto?")


def entrar_hil(ser):
    """El comando 'h' alterna el modo; si el ESP32 ya estaba en HIL, se repite."""
    ser.reset_input_buffer()
    ser.write(b"h\n")
    if "OFF" in esperar_respuesta(ser, "# HIL"):
        ser.write(b"h\n")
        esperar_respuesta(ser, "# HIL ON")


def correr(esc, metodo, ser=None, tiempo_real=True, al_paso=None, detener=None):
    """Lazo cerrado con el motor simulado.

    ser = None -> el controlador corre en Python (simulación).
    ser = puerto abierto -> el controlador corre en el ESP32 (HIL).
    """
    motor = MotorDC()
    sub = int(round(TS / DT_PLANTA))
    n = int(round(esc.duracion / TS))
    t = np.arange(n) * TS
    r, y, u = np.zeros(n), np.zeros(n), np.zeros(n)

    if ser is None:
        ctrl = ControladorPI() if metodo == "pi" else ControladorDifuso(metodo)
    else:
        entrar_hil(ser)
        ser.write(COMANDO_METODO[metodo])

    ref_enviada = None
    k = 0
    inicio = time.perf_counter()
    try:
        for k in range(n):
            if detener is not None and detener.is_set():
                break
            if tiempo_real:  # ir al ritmo real: 10 ms por paso
                falta = t[k] - (time.perf_counter() - inicio)
                if falta > 0.002:
                    time.sleep(falta - 0.001)
                while time.perf_counter() - inicio < t[k]:
                    pass
            r[k] = esc.referencia(t[k])
            y[k] = motor.rpm
            if ser is None:
                u[k] = ctrl.calcular(r[k], y[k])
            else:
                if r[k] != ref_enviada:
                    ser.write(f"r{r[k]:.1f}\n".encode())
                    ref_enviada = r[k]
                ser.write(f"y{y[k]:.3f}\n".encode())
                u[k] = float(esperar_respuesta(ser, "u")[1:])
            for _ in range(sub):
                motor.paso(u[k], DT_PLANTA, esc.carga(t[k]))
            if al_paso is not None:
                al_paso(t[k], r[k], y[k], u[k])
        else:
            k = n
    finally:
        if ser is not None:
            ser.write(b"h\n")  # salir de HIL (el ESP32 apaga el motor)
    return t[:k], r[:k], y[:k], u[:k]


if sys.platform == "win32":  # texto nítido en pantallas con escalado (125 %, 150 %…)
    try:
        import ctypes
        ctypes.windll.shcore.SetProcessDpiAwareness(1)
    except (AttributeError, OSError):
        pass


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Control difuso de motor CD — Mamdani / Sugeno")
        ancho = min(1400, self.winfo_screenwidth() - 40)
        alto = min(900, self.winfo_screenheight() - 90)
        self.geometry(f"{ancho}x{alto}+10+10")
        self.minsize(900, 560)

        self.ser = None
        self.cola = queue.Queue()
        self.detener = threading.Event()
        self.hilo = None
        self.corridas = []      # resultados terminados (para tabla y CSV)
        self.actual = None      # corrida en curso: buffers + líneas
        self.esc_graficado = None

        self._panel()
        self._tabla()      # antes que las gráficas: así siempre le queda espacio
        self._graficas()
        self._refrescar_puertos()
        self._actualizar_estado()
        self.protocol("WM_DELETE_WINDOW", self.cerrar)
        self.after(50, self._revisar_cola)

    # ------------------------------------------------------------ interfaz
    def _panel(self):
        # Panel izquierdo con barra de desplazamiento por si la pantalla es baja.
        contenedor = ttk.Frame(self)
        contenedor.pack(side=tk.LEFT, fill=tk.Y)
        lienzo = tk.Canvas(contenedor, highlightthickness=0, width=300)
        barra = ttk.Scrollbar(contenedor, orient=tk.VERTICAL, command=lienzo.yview)
        lienzo.configure(yscrollcommand=barra.set)
        barra.pack(side=tk.RIGHT, fill=tk.Y)
        lienzo.pack(side=tk.LEFT, fill=tk.Y)
        panel = ttk.Frame(lienzo, padding=8)
        lienzo.create_window((0, 0), window=panel, anchor="nw")

        def ajustar(_):
            lienzo.configure(scrollregion=lienzo.bbox("all"), width=panel.winfo_reqwidth())
        panel.bind("<Configure>", ajustar)
        lienzo.bind_all("<MouseWheel>",
                        lambda e: lienzo.yview_scroll(int(-e.delta / 120), "units")
                        if str(e.widget).startswith(str(lienzo)) else None)

        # Conexión
        caja = ttk.LabelFrame(panel, text="Conexión con el ESP32", padding=6)
        caja.pack(fill=tk.X, pady=4)
        self.puerto = tk.StringVar()
        self.combo = ttk.Combobox(caja, textvariable=self.puerto, width=14, state="readonly")
        self.combo.grid(row=0, column=0, padx=2)
        ttk.Button(caja, text="↻", width=3, command=self._refrescar_puertos).grid(row=0, column=1)
        self.b_conectar = ttk.Button(caja, text="Conectar", command=self.conectar)
        self.b_conectar.grid(row=0, column=2, padx=2)
        self.estado = tk.StringVar()
        ttk.Label(caja, textvariable=self.estado, foreground="#555").grid(
            row=1, column=0, columnspan=3, sticky="w", pady=(4, 0))

        # Modo
        caja = ttk.LabelFrame(panel, text="Modo", padding=6)
        caja.pack(fill=tk.X, pady=4)
        self.modo = tk.StringVar(value="sim")
        ttk.Radiobutton(caja, text="Simulación (solo PC)", variable=self.modo,
                        value="sim").pack(anchor="w")
        ttk.Radiobutton(caja, text="HIL (ESP32 + motor físico)", variable=self.modo,
                        value="hil").pack(anchor="w")
        self.tiempo_real = tk.BooleanVar(value=True)
        ttk.Checkbutton(caja, text="Tiempo real (10 ms por paso)",
                        variable=self.tiempo_real).pack(anchor="w", pady=(4, 0))

        # Controlador
        caja = ttk.LabelFrame(panel, text="Controlador", padding=6)
        caja.pack(fill=tk.X, pady=4)
        self.metodo = tk.StringVar(value="mamdani")
        for clave in ("mamdani", "sugeno", "pi"):
            texto = NOMBRES[clave] + (" (para comparar)" if clave == "pi" else "")
            ttk.Radiobutton(caja, text=texto, variable=self.metodo, value=clave).pack(anchor="w")

        # Escenario
        caja = ttk.LabelFrame(panel, text="Escenario", padding=6)
        caja.pack(fill=tk.X, pady=4)
        self.campos = {}
        for fila, (clave, texto, valor) in enumerate([
            ("ref1", "Referencia 1 [rpm]", "1500"),
            ("ref2", "Referencia 2 [rpm]", "800"),
            ("t_cambio", "Cambio de ref. en [s]", "1.5"),
            ("t_carga", "Entra la carga en [s]", "2.5"),
            ("par_carga", "Par de carga [N·m]", "0.03"),
            ("duracion", "Duración [s]", "4.0"),
        ]):
            ttk.Label(caja, text=texto).grid(row=fila, column=0, sticky="w")
            var = tk.StringVar(value=valor)
            ttk.Entry(caja, textvariable=var, width=8).grid(row=fila, column=1, pady=1)
            self.campos[clave] = var

        # Acciones
        caja = ttk.LabelFrame(panel, text="Acciones", padding=6)
        caja.pack(fill=tk.X, pady=4)
        self.b_ejecutar = ttk.Button(caja, text="▶  Ejecutar", command=self.ejecutar)
        self.b_ejecutar.pack(fill=tk.X, pady=1)
        self.b_comparar = ttk.Button(caja, text="Comparar todos los métodos",
                                     command=self.comparar)
        self.b_comparar.pack(fill=tk.X, pady=1)
        self.b_detener = ttk.Button(caja, text="■  Detener", command=self.detener.set,
                                    state=tk.DISABLED)
        self.b_detener.pack(fill=tk.X, pady=1)
        ttk.Button(caja, text="Limpiar gráficas", command=self.limpiar).pack(fill=tk.X, pady=1)
        ttk.Button(caja, text="Guardar resultados", command=self.guardar).pack(fill=tk.X, pady=1)

        # Manual
        caja = ttk.LabelFrame(panel, text="Motor manual (lazo abierto)", padding=6)
        caja.pack(fill=tk.X, pady=4)
        self.voltaje = tk.DoubleVar(value=0.0)
        self.etq_v = ttk.Label(caja, text="0.0 V")
        self.etq_v.pack()
        ttk.Scale(caja, from_=-12, to=12, variable=self.voltaje, orient=tk.HORIZONTAL,
                  command=lambda _: self.etq_v.config(text=f"{self.voltaje.get():+.1f} V")
                  ).pack(fill=tk.X)
        fila = ttk.Frame(caja)
        fila.pack(fill=tk.X, pady=(4, 0))
        self.b_aplicar = ttk.Button(fila, text="Aplicar", command=self.aplicar_voltaje)
        self.b_aplicar.pack(side=tk.LEFT, expand=True, fill=tk.X)
        self.b_parar = ttk.Button(fila, text="Parar motor", command=self.parar_motor)
        self.b_parar.pack(side=tk.LEFT, expand=True, fill=tk.X)

    def _graficas(self):
        derecha = ttk.Frame(self, padding=(0, 8, 8, 0))
        derecha.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self.fig = Figure(figsize=(9, 5.6), tight_layout=True)
        self.ax_w = self.fig.add_subplot(2, 1, 1)
        self.ax_u = self.fig.add_subplot(2, 1, 2, sharex=self.ax_w)
        self.canvas = FigureCanvasTkAgg(self.fig, master=derecha)
        NavigationToolbar2Tk(self.canvas, derecha).update()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self._ejes_vacios()

    def _tabla(self):
        abajo = ttk.Frame(self, padding=(0, 0, 8, 8))
        abajo.pack(side=tk.BOTTOM, fill=tk.X)
        columnas = ("corrida", "sp", "tr", "ts", "ess", "iae", "caida")
        titulos = ("Corrida", "Sobrepaso %", "t subida [s]", "t establ. [s]",
                   "Error final [rpm]", "IAE", "Caída por carga [rpm]")
        self.tabla = ttk.Treeview(abajo, columns=columnas, show="headings", height=5)
        for c, texto in zip(columnas, titulos):
            self.tabla.heading(c, text=texto)
            self.tabla.column(c, width=150 if c == "corrida" else 115, anchor=tk.CENTER)
        self.tabla.pack(fill=tk.X)

    def _ejes_vacios(self):
        for ax in (self.ax_w, self.ax_u):
            ax.clear()
            ax.grid(alpha=0.3)
        self.ax_w.set_ylabel("Velocidad [rpm]")
        self.ax_w.set_title("Control de velocidad del motor CD")
        self.ax_u.set_ylabel("Voltaje [V]")
        self.ax_u.set_xlabel("Tiempo [s]")
        self.linea_ref = None
        self.esc_graficado = None
        self.canvas.draw_idle()

    def _preparar_ejes(self, esc):
        """Dibuja la referencia y fija los límites (se hace antes de correr)."""
        t = np.arange(0, esc.duracion, TS)
        r = [esc.referencia(x) for x in t]
        if self.linea_ref is None:
            (self.linea_ref,) = self.ax_w.plot(t, r, "k--", label="Referencia")
        else:
            self.linea_ref.set_data(t, r)
        self.ax_w.set_xlim(0, esc.duracion)
        tope = max(esc.ref1, esc.ref2, *(float(np.max(c["y"])) for c in self.corridas)) \
            if self.corridas else max(esc.ref1, esc.ref2)
        self.ax_w.set_ylim(min(0, esc.ref1, esc.ref2) - 50, tope * 1.2 + 50)
        self.ax_u.set_ylim(-1, 12.5)
        self.esc_graficado = esc
        self.ax_w.legend(loc="lower right")
        self.canvas.draw_idle()

    # ------------------------------------------------------------ conexión
    def _refrescar_puertos(self):
        puertos = [p.device for p in serial.tools.list_ports.comports()]
        self.combo["values"] = puertos
        if puertos and self.puerto.get() not in puertos:
            self.puerto.set(puertos[-1])

    def conectar(self):
        if self.ser is not None:
            self._cerrar_puerto()
            self._actualizar_estado()
            return
        if not self.puerto.get():
            messagebox.showwarning("Sin puerto", "Conecta el ESP32 por USB y presiona ↻.")
            return
        try:
            self.ser = serial.Serial(self.puerto.get(), 115200, timeout=1)
        except serial.SerialException as e:
            messagebox.showerror("No se pudo abrir el puerto",
                                 f"{e}\n\n¿Está abierto el Monitor Serie de Arduino?")
            return
        self.estado.set("Conectando… (el ESP32 se reinicia)")
        self.config(cursor="watch")
        self.update()
        time.sleep(2.0)
        self.ser.reset_input_buffer()
        self.ser.write(b"p\n")
        self.config(cursor="")
        self._actualizar_estado()

    def _cerrar_puerto(self):
        try:
            self.ser.write(b"p\n")
            self.ser.close()
        except serial.SerialException:
            pass
        self.ser = None

    def _actualizar_estado(self, ocupado=False):
        conectado = self.ser is not None
        self.estado.set(f"Conectado a {self.ser.port}" if conectado else "Desconectado")
        self.b_conectar.config(text="Desconectar" if conectado else "Conectar",
                               state=tk.DISABLED if ocupado else tk.NORMAL)
        normal = tk.DISABLED if ocupado else tk.NORMAL
        self.b_ejecutar.config(state=normal)
        self.b_comparar.config(state=normal)
        self.b_detener.config(state=tk.NORMAL if ocupado else tk.DISABLED)
        manual = tk.NORMAL if (conectado and not ocupado) else tk.DISABLED
        self.b_aplicar.config(state=manual)
        self.b_parar.config(state=manual)

    # ------------------------------------------------------------ manual
    def aplicar_voltaje(self):
        self.ser.write(f"v{self.voltaje.get():.2f}\n".encode())

    def parar_motor(self):
        self.voltaje.set(0.0)
        self.etq_v.config(text="0.0 V")
        self.ser.write(b"p\n")

    # ------------------------------------------------------------ corridas
    def _leer_escenario(self):
        try:
            v = {k: float(var.get().replace(",", ".")) for k, var in self.campos.items()}
        except ValueError:
            messagebox.showerror("Escenario", "Todos los campos deben ser números.")
            return None
        if not 0.2 <= v["duracion"] <= 20:
            messagebox.showerror("Escenario", "La duración debe estar entre 0.2 y 20 s.")
            return None
        if max(abs(v["ref1"]), abs(v["ref2"])) > RPM_MAX:
            messagebox.showerror("Escenario",
                                 f"El motor simulado llega a {RPM_MAX:.0f} rpm como máximo.")
            return None
        return Escenario(**v)

    def ejecutar(self):
        self._lanzar([self.metodo.get()])

    def comparar(self):
        self._lanzar(["mamdani", "sugeno", "pi"])

    def _lanzar(self, metodos):
        if self.hilo is not None and self.hilo.is_alive():
            return
        esc = self._leer_escenario()
        if esc is None:
            return
        modo = self.modo.get()
        if modo == "hil":
            if self.ser is None:
                messagebox.showwarning("HIL", "Primero conecta el ESP32.")
                return
        self.detener.clear()
        self._preparar_ejes(esc)
        self._actualizar_estado(ocupado=True)
        self.hilo = threading.Thread(
            target=self._trabajo, args=(esc, modo, metodos, self.tiempo_real.get()),
            daemon=True)
        self.hilo.start()

    def _trabajo(self, esc, modo, metodos, tiempo_real):
        """Corre en un hilo aparte para que la ventana no se congele."""
        for metodo in metodos:
            if self.detener.is_set():
                break
            nombre = f"{'HIL' if modo == 'hil' else 'Sim'} {NOMBRES[metodo]}"
            self.cola.put(("nueva", nombre))
            try:
                datos = correr(esc, metodo, ser=self.ser if modo == "hil" else None,
                               tiempo_real=tiempo_real, detener=self.detener,
                               al_paso=lambda *p: self.cola.put(("paso", *p)))
            except Exception as e:  # puerto desconectado, ESP32 sin responder, etc.
                self.cola.put(("error", str(e)))
                break
            self.cola.put(("fin", nombre, datos, esc))
        self.cola.put(("listo",))

    def _revisar_cola(self):
        """Pasa los datos del hilo de trabajo a las gráficas (solo aquí se dibuja)."""
        try:
            while True:
                msg = self.cola.get_nowait()
                tipo = msg[0]
                if tipo == "nueva":
                    color = f"C{len(self.corridas) % 10}"
                    (lw,) = self.ax_w.plot([], [], color=color, lw=2, label=msg[1])
                    (lu,) = self.ax_u.step([], [], color=color, where="post", label=msg[1])
                    self.actual = {"t": [], "y": [], "u": [], "lw": lw, "lu": lu}
                    self.ax_w.legend(loc="lower right")
                    self.ax_u.legend(loc="upper right")
                elif tipo == "paso" and self.actual is not None:
                    _, t, r, y, u = msg
                    self.actual["t"].append(t)
                    self.actual["y"].append(y)
                    self.actual["u"].append(u)
                elif tipo == "fin":
                    self._dibujar_actual()  # última actualización antes de cerrarla
                    self._registrar(*msg[1:])
                elif tipo == "error":
                    messagebox.showerror("Error durante la corrida", msg[1])
                elif tipo == "listo":
                    self._dibujar_actual()
                    self.actual = None
                    self._actualizar_estado()
        except queue.Empty:
            pass
        finally:
            self._dibujar_actual()
            self.after(50, self._revisar_cola)  # siempre se vuelve a programar

    def _dibujar_actual(self):
        a = self.actual
        if a is not None and a["t"]:
            a["lw"].set_data(a["t"], a["y"])
            a["lu"].set_data(a["t"], a["u"])
            self.canvas.draw_idle()

    def _registrar(self, nombre, datos, esc):
        t, r, y, u = datos
        if len(t) < 10:
            return
        self.corridas.append({"nombre": nombre, "t": t, "r": r, "y": y, "u": u})
        fila = [nombre]
        if t[-1] >= esc.t_cambio - TS:  # hubo escalón completo: se pueden medir métricas
            m = metricas(t, r, y, t0=0.0, t1=esc.t_cambio)
            fila += [f"{m['sobrepaso_%']:.2f}", f"{m['t_subida_s']:.3f}",
                     f"{m['t_establ_s']:.3f}", f"{m['error_final_rpm']:.1f}", f"{m['IAE']:.1f}"]
        else:
            fila += ["—"] * 5
        despues = t >= esc.t_carga
        if esc.par_carga > 0 and despues.any():
            fila.append(f"{np.max(r[despues] - y[despues]):.1f}")
        else:
            fila.append("—")
        self.tabla.insert("", tk.END, values=fila)

    def limpiar(self):
        if self.hilo is not None and self.hilo.is_alive():
            return
        self.corridas.clear()
        self.tabla.delete(*self.tabla.get_children())
        self._ejes_vacios()

    def guardar(self):
        if not self.corridas:
            messagebox.showinfo("Guardar", "Todavía no hay corridas terminadas.")
            return
        os.makedirs(CARPETA, exist_ok=True)
        sello = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.fig.savefig(os.path.join(CARPETA, f"interfaz_{sello}.png"), dpi=130)
        for c in self.corridas:
            archivo = f"interfaz_{sello}_{c['nombre'].replace(' ', '_')}.csv"
            with open(os.path.join(CARPETA, archivo), "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(["t_s", "ref_rpm", "rpm", "u_V"])
                w.writerows(zip(c["t"], c["r"], c["y"], c["u"]))
        messagebox.showinfo("Guardar", f"Gráfica y CSV guardados en:\n{CARPETA}")

    def cerrar(self):
        self.detener.set()
        if self.hilo is not None:
            self.hilo.join(timeout=2)
        if self.ser is not None:
            self._cerrar_puerto()
        self.destroy()


if __name__ == "__main__":
    App().mainloop()
