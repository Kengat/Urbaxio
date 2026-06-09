"""Urbaxio Qt shell — frameless window with the companion app's top menu bar.

Run:
    C:\\dev\\Urbaxio\\.venv\\Scripts\\python.exe -m qt_shell.main
or:
    C:\\dev\\Urbaxio\\.venv\\Scripts\\python.exe qt_shell\\main.py
"""
from __future__ import annotations

import sys
from pathlib import Path

from PySide6 import QtCore, QtGui, QtQuickWidgets, QtWidgets

# Allow running both as a module (-m qt_shell.main) and as a script.
try:
    from .controller import QtStateController
except ImportError:  # pragma: no cover - script execution fallback
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from qt_shell.controller import QtStateController

HERE = Path(__file__).resolve().parent
QML_DIR = HERE / "qml"
FONT_DIR = HERE / "assets" / "fonts"


class ChromeWidget(QtQuickWidgets.QQuickWidget):
    def __init__(self, source: Path, controller: QtCore.QObject, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self.setResizeMode(QtQuickWidgets.QQuickWidget.ResizeMode.SizeRootObjectToView)
        self.setClearColor(QtGui.QColor("#050505"))
        self.setAttribute(QtCore.Qt.WidgetAttribute.WA_AlwaysStackOnTop, False)
        self.rootContext().setContextProperty("controller", controller)
        self.setSource(QtCore.QUrl.fromLocalFile(str(source)))


class ShellWindow(QtWidgets.QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.controller = QtStateController(self)

        self.setWindowTitle("Urbaxio")
        self.setWindowFlags(
            QtCore.Qt.WindowType.FramelessWindowHint | QtCore.Qt.WindowType.Window
        )
        self.resize(1600, 980)
        self.setMinimumSize(1024, 640)
        self.setStyleSheet("QMainWindow{background:#050505; color:#FAFAFA;}")
        self._drag_pos: QtCore.QPoint | None = None

        root = QtWidgets.QWidget()
        self.setCentralWidget(root)
        root_layout = QtWidgets.QVBoxLayout(root)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(0)

        # Top menu bar — copied 1:1 from the companion app.
        self.menu_bar_view = ChromeWidget(QML_DIR / "MainMenu.qml", self.controller)
        self.menu_bar_view.setFixedHeight(32)
        root_layout.addWidget(self.menu_bar_view)

        # Empty body so the window has a real frame below the menu.
        body = QtWidgets.QWidget()
        body.setStyleSheet("background:#0A0A0C;")
        root_layout.addWidget(body, 1)


def _load_fonts() -> None:
    loaded: set[str] = set()
    for font_path in FONT_DIR.glob("*.ttf"):
        fid = QtGui.QFontDatabase.addApplicationFont(str(font_path))
        if fid >= 0:
            for fam in QtGui.QFontDatabase.applicationFontFamilies(fid):
                loaded.add(fam)
    if loaded:
        print(f"[fonts] loaded: {', '.join(sorted(loaded))}")


def main() -> int:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    _load_fonts()
    app.setFont(QtGui.QFont("Outfit", 10))
    window = ShellWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
