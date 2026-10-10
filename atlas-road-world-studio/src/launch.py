#!/usr/bin/env python3
"""ATLAS World Studio: embedded desktop workspace with a private local backend."""
import os
import io
import runpy
import socket
import subprocess
import sys
from pathlib import Path

FROZEN = bool(getattr(sys, "frozen", False))
ROOT = Path(getattr(sys, "_MEIPASS", Path(__file__).resolve().parent))
DEFAULT_DATA = Path(os.environ.get("LOCALAPPDATA", str(Path.home()))) / "ATLAS World Studio" if FROZEN else ROOT.parent / "output"
DATA = Path(os.environ.get("ATLAS_DATA_DIR", str(DEFAULT_DATA)))


def serve():
    for name in ("stdout", "stderr"):
        stream = getattr(sys, name)
        if stream is None:
            setattr(sys, name, open(os.devnull, "w", encoding="utf-8"))
        elif hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="replace")
        elif hasattr(stream, "buffer"):
            setattr(sys, name, io.TextIOWrapper(stream.buffer, encoding="utf-8", errors="replace", write_through=True))
    if str(ROOT) not in sys.path:
        sys.path.insert(0, str(ROOT))
    os.environ["ATLAS_DATA_DIR"] = str(DATA)
    runpy.run_path(str(ROOT / "server.py"), run_name="__main__")


def listening(port):
    try:
        with socket.create_connection(("127.0.0.1", port), timeout=0.2):
            return True
    except OSError:
        return False


def desktop():
    from PySide6.QtCore import QTimer, QUrl, Qt
    from PySide6.QtGui import QIcon
    from PySide6.QtWidgets import QApplication, QMainWindow, QMessageBox, QLabel
    from PySide6.QtWebEngineWidgets import QWebEngineView

    app = QApplication(sys.argv)
    app.setApplicationName("ATLAS World Studio")
    app.setOrganizationName("ATLAS")
    icon = QIcon(str(ROOT / "static" / "brand" / "atlas.ico"))
    app.setWindowIcon(icon)
    if os.name == "nt":
        import ctypes
        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID("ATLAS.WorldStudio")
    DATA.mkdir(parents=True, exist_ok=True)
    # The current backend listens on a fixed port. Never kill an unrelated process.
    port = 8743
    existing = listening(port)
    process = None
    if not existing:
        log = open(DATA / "atlas-server.log", "a", encoding="utf-8")
        cmd = [sys.executable, "--serve"] if FROZEN else [sys.executable, str(ROOT / "launch.py"), "--serve"]
        env = os.environ.copy()
        env.update({"ATLAS_DATA_DIR": str(DATA), "PYTHONUTF8": "1", "PYTHONIOENCODING": "utf-8:replace"})
        options = {"cwd": str(DATA), "stdout": log, "stderr": subprocess.STDOUT, "env": env}
        if os.name == "nt":
            options["creationflags"] = subprocess.CREATE_NO_WINDOW
        try:
            process = subprocess.Popen(cmd, **options)
        except OSError as exc:
            QMessageBox.critical(None, "ATLAS failed to launch", str(exc))
            return 1
        finally:
            log.close()

    class MainWindow(QMainWindow):
        def __init__(self):
            super().__init__()
            self.setWindowTitle("ATLAS World Studio")
            self.setWindowIcon(icon)
            self.resize(1480, 900)
            self.setMinimumSize(900, 600)
            self.view = QWebEngineView(self)
            self.setCentralWidget(self.view)
            self.statusBar().setStyleSheet("QStatusBar { background: #15191b; color: #a7b0b0; border-top: 1px solid #333b3d; font-size: 11px; }")
            self.statusBar().showMessage("Starting ATLAS local engine…")
            self.elapsed = 0
            self.timer = QTimer(self)
            self.timer.timeout.connect(self.check_server)
            self.timer.start(250)

        def check_server(self):
            self.elapsed += 250
            if listening(port):
                self.timer.stop()
                self.statusBar().showMessage("ATLAS ready")
                self.view.setUrl(QUrl(f"http://127.0.0.1:{port}/"))
            elif process is not None and process.poll() is not None:
                self.timer.stop()
                QMessageBox.critical(self, "ATLAS engine stopped", f"The backend exited unexpectedly. Check:\n{DATA / 'atlas-server.log'}")
            elif self.elapsed >= 30000:
                self.timer.stop()
                QMessageBox.warning(self, "ATLAS startup timed out", f"The engine has not started after 30 seconds. Check:\n{DATA / 'atlas-server.log'}")

        def closeEvent(self, event):
            self.timer.stop()
            self.view.setUrl(QUrl("about:blank"))
            if process is not None and process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
            super().closeEvent(event)

    window = MainWindow()
    window.show()
    if os.name == "nt":
        # Windows 10/11 native title bar; older systems keep the default frame.
        try:
            dark = ctypes.c_int(1)
            for attribute in (20, 19):
                result = ctypes.windll.dwmapi.DwmSetWindowAttribute(
                    ctypes.c_void_p(int(window.winId())), attribute,
                    ctypes.byref(dark), ctypes.sizeof(dark))
                if result == 0:
                    break
        except (AttributeError, OSError):
            pass
    return app.exec()


if __name__ == "__main__":
    if "--serve" in sys.argv:
        serve()
    else:
        sys.exit(desktop())
