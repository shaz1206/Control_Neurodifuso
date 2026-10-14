"""
Abre la interfaz web (fase_b/interfaz_web/index.html) en el navegador.

La conexión USB desde la página usa Web Serial, que solo funciona en Chrome o
Edge y con la página servida desde localhost. Este script levanta ese servidor
local y abre el navegador. Ciérrenlo con Ctrl+C.

Uso:  python interfaz_web.py
"""
import functools
import http.server
import socketserver
import webbrowser

PUERTO = 8765
CARPETA = __file__.replace("interfaz_web.py", "interfaz_web")


def main():
    manejador = functools.partial(http.server.SimpleHTTPRequestHandler, directory=CARPETA)
    manejador.log_message = lambda *a: None  # sin una línea por cada archivo servido
    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.TCPServer(("127.0.0.1", PUERTO), manejador) as servidor:
        url = f"http://localhost:{PUERTO}/"
        print(f"Interfaz en {url}  (ábranla en Chrome o Edge; Ctrl+C para cerrar)")
        webbrowser.open(url)
        try:
            servidor.serve_forever()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
