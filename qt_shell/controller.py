"""State controller + dropdown-menu popup system for the Urbaxio Qt shell.

This is a faithful copy of the top-menu behaviour from the Gaussian Points
companion app: the frameless-window controls (minimize / maximize / close),
title-bar dragging, and the full File/Edit/View/Tools/Window/Help dropdowns
with hovers, icons, shortcuts and submenus.
"""
from __future__ import annotations

from pathlib import Path
from typing import Any

from PySide6 import QtCore, QtGui, QtSvg, QtWidgets

ASSETS_DIR = Path(__file__).resolve().parent / "assets"


class PopupMenuItemFrame(QtWidgets.QFrame):
    hovered = QtCore.Signal(object, object)
    activated = QtCore.Signal(object)

    def __init__(self, controller: "QtStateController", spec: dict[str, Any], parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self._controller = controller
        self.spec = spec
        self._enabled = bool(spec.get("enabled", True))
        self._hovered = False
        self.setCursor(QtGui.QCursor(QtCore.Qt.CursorShape.PointingHandCursor if self._enabled else QtCore.Qt.CursorShape.ArrowCursor))
        self.setMouseTracking(True)
        self.setObjectName("menuItemFrame")
        self.setFixedHeight(28)

        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(12, 5, 12, 5)
        layout.setSpacing(10)

        self.icon_label = QtWidgets.QLabel()
        self.icon_label.setFixedSize(14, 14)
        self.icon_label.setAttribute(QtCore.Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        layout.addWidget(self.icon_label, 0, QtCore.Qt.AlignmentFlag.AlignVCenter)

        self.text_label = QtWidgets.QLabel(str(spec.get("label") or ""))
        self.text_label.setAttribute(QtCore.Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        self.text_label.setStyleSheet("background:transparent;")
        text_font = QtGui.QFont("Outfit")
        text_font.setPixelSize(12)
        text_font.setWeight(QtGui.QFont.Weight.Medium)
        self.text_label.setFont(text_font)
        layout.addWidget(self.text_label, 1, QtCore.Qt.AlignmentFlag.AlignVCenter)

        self.shortcut_label = QtWidgets.QLabel(str(spec.get("shortcut") or ""))
        self.shortcut_label.setAttribute(QtCore.Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        shortcut_font = QtGui.QFont("JetBrains Mono")
        shortcut_font.setPixelSize(9)
        shortcut_font.setWeight(QtGui.QFont.Weight.Medium)
        self.shortcut_label.setFont(shortcut_font)
        layout.addWidget(self.shortcut_label, 0, QtCore.Qt.AlignmentFlag.AlignVCenter)

        self.arrow_label = QtWidgets.QLabel()
        self.arrow_label.setAttribute(QtCore.Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        self.arrow_label.setFixedSize(14, 14)
        self.arrow_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.arrow_label, 0, QtCore.Qt.AlignmentFlag.AlignVCenter)

        self._update_visual()

    def set_hovered(self, hovered: bool) -> None:
        hovered = bool(hovered) and self._enabled
        if self._hovered == hovered:
            return
        self._hovered = hovered
        self._update_visual()

    def _update_visual(self) -> None:
        bg = "rgba(255,255,255,0.08)" if self._hovered and self._enabled else "transparent"
        self.setStyleSheet(f"QFrame#menuItemFrame{{background:{bg}; border:none; border-radius:6px;}}")
        text_color = "#FFFFFF" if self._hovered else "#D4D4D8"
        if not self._enabled:
            text_color = "#52525B"
        self.text_label.setStyleSheet(f"background:transparent; color:{text_color};")
        shortcut_color = "#71717A" if self.spec.get("shortcut") and self._enabled else "#3F3F46" if self.spec.get("shortcut") else "transparent"
        self.shortcut_label.setStyleSheet(f"background:transparent; color:{shortcut_color};")
        icon_name = str(self.spec.get("icon") or "")
        if icon_name:
            pixmap = self._controller._menu_icon_pixmap(
                icon_name,
                "#00F0FF" if self._hovered else "#A1A1AA" if self._enabled else "#52525B",
                14,
            )
            self.icon_label.setPixmap(pixmap)
        else:
            self.icon_label.clear()
        if self.spec.get("submenu"):
            arrow_pixmap = self._controller._menu_icon_pixmap(
                "chevron-right",
                "#FFFFFF" if self._hovered else "#71717A" if self._enabled else "#3F3F46",
                14,
            )
            self.arrow_label.setPixmap(arrow_pixmap)
        else:
            self.arrow_label.clear()

    def enterEvent(self, event: QtCore.QEvent) -> None:
        self.set_hovered(True)
        if self._enabled:
            self.hovered.emit(self, self.spec)
        super().enterEvent(event)

    def leaveEvent(self, event: QtCore.QEvent) -> None:
        self.set_hovered(False)
        super().leaveEvent(event)

    def mouseReleaseEvent(self, event: QtGui.QMouseEvent) -> None:
        if self._enabled and event.button() == QtCore.Qt.MouseButton.LeftButton:
            self.activated.emit(self.spec)
        super().mouseReleaseEvent(event)


class PopupMenuWindow(QtWidgets.QWidget):
    def __init__(
        self,
        controller: "QtStateController",
        menu_name: str,
        items: list[dict[str, Any]],
        *,
        root_menu_name: str,
        parent_popup: "PopupMenuWindow | None" = None,
    ) -> None:
        window_parent = controller._app_window()
        super().__init__(window_parent)
        self._controller = controller
        self._menu_name = menu_name
        self._root_menu_name = root_menu_name
        self._parent_popup = parent_popup
        self._child_popup: PopupMenuWindow | None = None
        self._item_widgets: list[PopupMenuItemFrame] = []

        self.setWindowFlags(
            QtCore.Qt.WindowType.Tool
            | QtCore.Qt.WindowType.FramelessWindowHint
            | QtCore.Qt.WindowType.NoDropShadowWindowHint
        )
        self.setAttribute(QtCore.Qt.WidgetAttribute.WA_TranslucentBackground, True)
        self.setAttribute(QtCore.Qt.WidgetAttribute.WA_ShowWithoutActivating, True)

        root_layout = QtWidgets.QVBoxLayout(self)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(0)

        self.card = QtWidgets.QFrame()
        self.card.setObjectName("menuCard")
        self.card.setStyleSheet(
            "QFrame#menuCard{"
            "background:rgba(14,14,18,242);"
            "border:1px solid rgba(255,255,255,0.10);"
            "border-radius:12px;"
            "}"
        )
        shadow = QtWidgets.QGraphicsDropShadowEffect(self.card)
        shadow.setBlurRadius(50)
        shadow.setOffset(0, 20)
        shadow.setColor(QtGui.QColor(0, 0, 0, 204))
        self.card.setGraphicsEffect(shadow)
        root_layout.addWidget(self.card)

        card_layout = QtWidgets.QVBoxLayout(self.card)
        card_layout.setContentsMargins(6, 6, 6, 6)
        card_layout.setSpacing(0)

        self.setMinimumWidth(268 if parent_popup is None else 190)

        for spec in items:
            if spec.get("type") == "separator":
                separator = QtWidgets.QFrame()
                separator.setFixedHeight(14)
                sep_layout = QtWidgets.QVBoxLayout(separator)
                sep_layout.setContentsMargins(12, 6, 12, 6)
                sep_layout.setSpacing(0)
                line = QtWidgets.QFrame()
                line.setFixedHeight(1)
                line.setStyleSheet("background:rgba(255,255,255,0.10); border:none;")
                sep_layout.addWidget(line)
                card_layout.addWidget(separator)
                continue

            item = PopupMenuItemFrame(controller, spec, self.card)
            item.hovered.connect(self._on_item_hovered)
            item.activated.connect(self._on_item_activated)
            self._item_widgets.append(item)
            card_layout.addWidget(item)

        self.adjustSize()

    def open_at(self, global_pos: QtCore.QPoint) -> None:
        self._controller.cancelMenuClose()
        self.adjustSize()
        self.move(global_pos)
        self.show()
        self.raise_()

    def contains_global_pos(self, global_pos: QtCore.QPoint) -> bool:
        if self.isVisible() and self.frameGeometry().contains(global_pos):
            return True
        if self._child_popup and self._child_popup.contains_global_pos(global_pos):
            return True
        return False

    def close_chain(self) -> None:
        if self._child_popup is not None:
            self._child_popup.close_chain()
            self._child_popup.deleteLater()
            self._child_popup = None
        self.hide()

    def enterEvent(self, event: QtCore.QEvent) -> None:
        self._controller.cancelMenuClose()
        super().enterEvent(event)

    def leaveEvent(self, event: QtCore.QEvent) -> None:
        self._controller.scheduleMenuClose()
        super().leaveEvent(event)

    def _open_submenu(self, spec: dict[str, Any], source_item: PopupMenuItemFrame) -> None:
        submenu = spec.get("submenu")
        if not submenu:
            self._close_submenu()
            return
        submenu_name = str(spec.get("label") or "submenu")
        if self._child_popup is not None and self._child_popup._menu_name == submenu_name:
            point = source_item.mapToGlobal(QtCore.QPoint(source_item.width() - 4, 0))
            self._child_popup.open_at(point)
            return
        self._close_submenu()
        child = PopupMenuWindow(
            self._controller,
            submenu_name,
            list(submenu),
            root_menu_name=self._root_menu_name,
            parent_popup=self,
        )
        self._child_popup = child
        point = source_item.mapToGlobal(QtCore.QPoint(source_item.width() - 4, 0))
        child.open_at(point)

    def _close_submenu(self) -> None:
        if self._child_popup is None:
            return
        self._child_popup.close_chain()
        self._child_popup.deleteLater()
        self._child_popup = None

    def _on_item_hovered(self, source_item: PopupMenuItemFrame, spec: dict[str, Any]) -> None:
        submenu = spec.get("submenu")
        if submenu:
            self._open_submenu(spec, source_item)
        else:
            self._close_submenu()

    def _on_item_activated(self, spec: dict[str, Any]) -> None:
        submenu = spec.get("submenu")
        if submenu:
            return
        self._controller._execute_menu_action(str(spec.get("action") or ""))


class QtStateController(QtCore.QObject):
    activeMenuNameChanged = QtCore.Signal()
    menuPopupVisibleChanged = QtCore.Signal()

    def __init__(self, parent: QtCore.QObject | None = None) -> None:
        super().__init__(parent)
        self._active_menu_name = ""
        self._menu_popup_visible = False
        self._menu_definitions: dict[str, list[dict[str, Any]]] = {}
        self._menu_root_popup: PopupMenuWindow | None = None
        self._menu_icon_cache: dict[tuple[str, str, int, float], QtGui.QPixmap] = {}
        self._menu_hot_zone = QtCore.QRect()
        self._menu_close_timer = QtCore.QTimer(self)
        self._menu_close_timer.setSingleShot(True)
        self._menu_close_timer.setInterval(160)
        self._menu_close_timer.timeout.connect(self._close_menus_if_pointer_outside)

    # --- Properties consumed by MainMenu.qml ---
    @QtCore.Property(str, notify=activeMenuNameChanged)
    def activeMenuName(self) -> str:
        return self._active_menu_name

    @QtCore.Property(bool, notify=menuPopupVisibleChanged)
    def menuPopupVisible(self) -> bool:
        return self._menu_popup_visible

    # --- Frameless window controls ---
    @QtCore.Slot()
    def minimizeWindow(self) -> None:
        w = self._app_window()
        if w:
            w.showMinimized()

    @QtCore.Slot()
    def maximizeWindow(self) -> None:
        w = self._app_window()
        if w:
            if w.isMaximized():
                w.showNormal()
            else:
                w.showMaximized()

    @QtCore.Slot()
    def closeWindow(self) -> None:
        w = self._app_window()
        if w:
            w.close()

    @QtCore.Slot(float, float)
    def startDrag(self, gx: float, gy: float) -> None:
        w = self._app_window()
        if w:
            w._drag_pos = QtCore.QPoint(int(gx), int(gy))

    @QtCore.Slot(float, float)
    def updateDrag(self, gx: float, gy: float) -> None:
        w = self._app_window()
        if w and getattr(w, "_drag_pos", None) is not None:
            delta = QtCore.QPoint(int(gx), int(gy)) - w._drag_pos
            w.move(w.pos() + delta)
            w._drag_pos = QtCore.QPoint(int(gx), int(gy))

    @QtCore.Slot()
    def endDrag(self) -> None:
        w = self._app_window()
        if w:
            w._drag_pos = None

    def _app_window(self) -> QtWidgets.QWidget | None:
        p = self.parent()
        while p and not isinstance(p, QtWidgets.QMainWindow):
            p = p.parent()
        return p

    # --- Menu hot-zone / auto-close machinery ---
    def _set_menu_popup_visible(self, visible: bool) -> None:
        visible = bool(visible)
        if self._menu_popup_visible == visible:
            return
        self._menu_popup_visible = visible
        app = QtWidgets.QApplication.instance()
        if app is not None:
            if visible:
                app.installEventFilter(self)
            else:
                app.removeEventFilter(self)
        self.menuPopupVisibleChanged.emit()

    def _set_active_menu_name(self, menu_name: str) -> None:
        menu_name = menu_name or ""
        if self._active_menu_name == menu_name:
            return
        self._active_menu_name = menu_name
        self.activeMenuNameChanged.emit()

    @QtCore.Slot(float, float, float, float)
    def setMenuHotZone(self, gx: float, gy: float, width: float, height: float) -> None:
        self._menu_hot_zone = QtCore.QRect(int(gx), int(gy), max(1, int(width)), max(1, int(height)))

    @QtCore.Slot()
    def cancelMenuClose(self) -> None:
        self._menu_close_timer.stop()

    @QtCore.Slot()
    def scheduleMenuClose(self) -> None:
        if self._menu_popup_visible:
            self._menu_close_timer.start()

    def _close_menus_if_pointer_outside(self) -> None:
        if not self._menu_popup_visible:
            return
        global_pos = QtGui.QCursor.pos()
        if self._menu_hot_zone.contains(global_pos):
            return
        if self._menu_root_popup is not None and self._menu_root_popup.contains_global_pos(global_pos):
            return
        self.closeAllMenus()

    def eventFilter(self, obj: QtCore.QObject, event: QtCore.QEvent) -> bool:
        if not self._menu_popup_visible or self._menu_root_popup is None:
            return super().eventFilter(obj, event)
        if event.type() == QtCore.QEvent.Type.KeyPress:
            key_event = event if isinstance(event, QtGui.QKeyEvent) else None
            if key_event is not None and key_event.key() == QtCore.Qt.Key.Key_Escape:
                self.closeAllMenus()
                return True
        if event.type() == QtCore.QEvent.Type.MouseButtonPress:
            mouse_event = event if isinstance(event, QtGui.QMouseEvent) else None
            if mouse_event is not None:
                global_pos = mouse_event.globalPosition().toPoint()
                if not self._menu_root_popup.contains_global_pos(global_pos):
                    self.closeAllMenus()
        return super().eventFilter(obj, event)

    @QtCore.Slot(str, float, float)
    def showMenu(self, menu_name: str, gx: float, gy: float) -> None:
        self._ensure_menus()
        self.cancelMenuClose()
        menu_key = (menu_name or "").lower()
        menu_items = self._menu_definitions.get(menu_key)
        if menu_items is None:
            return
        if self._menu_root_popup is not None:
            self._menu_root_popup.close_chain()
            self._menu_root_popup.deleteLater()
            self._menu_root_popup = None
        self._menu_root_popup = PopupMenuWindow(
            self,
            menu_key,
            list(menu_items),
            root_menu_name=menu_key,
            parent_popup=None,
        )
        self._set_active_menu_name(menu_key)
        self._set_menu_popup_visible(True)
        self._menu_root_popup.open_at(QtCore.QPoint(int(gx), int(gy)))

    @QtCore.Slot()
    def closeAllMenus(self) -> None:
        self._menu_close_timer.stop()
        if self._menu_root_popup is not None:
            self._menu_root_popup.close_chain()
            self._menu_root_popup.deleteLater()
            self._menu_root_popup = None
        self._set_menu_popup_visible(False)
        self._set_active_menu_name("")

    # --- Top-bar buttons (Community Gallery / Sign In) ---
    @QtCore.Slot()
    def showCommunityGalleryDialog(self) -> None:
        print("[shell] Community Gallery clicked")

    @QtCore.Slot()
    def showSignInDialog(self) -> None:
        print("[shell] Sign In clicked")

    # --- Icon tinting ---
    def _icon_path(self, name: str) -> Path | None:
        if not name:
            return None
        path = ASSETS_DIR / "icons" / f"{name}.svg"
        return path if path.exists() else None

    def _menu_icon_pixmap(self, name: str, color_hex: str, size: int) -> QtGui.QPixmap:
        app = QtWidgets.QApplication.instance()
        screen = app.primaryScreen() if app is not None else None
        dpr = max(1.0, float(screen.devicePixelRatio() if screen is not None else 1.0))
        cache_key = (name, color_hex, int(size), round(dpr, 2))
        cached = self._menu_icon_cache.get(cache_key)
        if cached is not None:
            return cached

        icon_path = self._icon_path(name)
        if icon_path is None:
            return QtGui.QPixmap()

        pixel_size = max(1, int(round(int(size) * dpr)))
        image = QtGui.QImage(pixel_size, pixel_size, QtGui.QImage.Format.Format_ARGB32_Premultiplied)
        image.fill(QtCore.Qt.GlobalColor.transparent)
        renderer = QtSvg.QSvgRenderer(str(icon_path))
        painter = QtGui.QPainter(image)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing, True)
        painter.setRenderHint(QtGui.QPainter.RenderHint.SmoothPixmapTransform, True)
        painter.scale(dpr, dpr)
        logical_rect = QtCore.QRectF(0, 0, int(size), int(size))
        renderer.render(painter, logical_rect)
        painter.setCompositionMode(QtGui.QPainter.CompositionMode.CompositionMode_SourceIn)
        painter.fillRect(logical_rect, QtGui.QColor(color_hex))
        painter.end()
        pixmap = QtGui.QPixmap.fromImage(image)
        pixmap.setDevicePixelRatio(dpr)
        self._menu_icon_cache[cache_key] = pixmap
        return pixmap

    # --- Menu definitions (1:1 with the companion app) ---
    def _menu_separator(self) -> dict[str, Any]:
        return {"type": "separator"}

    def _menu_item(
        self,
        label: str,
        *,
        icon: str = "",
        shortcut: str = "",
        action: str = "",
        submenu: list[dict[str, Any]] | None = None,
        enabled: bool = True,
    ) -> dict[str, Any]:
        return {
            "type": "item",
            "label": label,
            "icon": icon,
            "shortcut": shortcut,
            "action": action,
            "submenu": list(submenu or []),
            "enabled": bool(enabled),
        }

    def _ensure_menus(self) -> None:
        if self._menu_definitions:
            return

        export_menu = [
            self._menu_item("Export as .ply", icon="download", action="exportPly"),
            self._menu_item("Export to SketchUp", icon="arrow-up-right", action="exportSketchUp"),
            self._menu_item("Open Export Folder", icon="folder", action="openExportFolder"),
            self._menu_separator(),
            self._menu_item("Export Sequence", icon="film", enabled=False),
        ]
        appearance_menu = [
            self._menu_item("Dark Mode", enabled=False),
            self._menu_item("Light Mode", enabled=False),
            self._menu_separator(),
            self._menu_item("High Contrast", enabled=False),
        ]
        camera_menu = [
            self._menu_item("Fly Mode", enabled=False),
            self._menu_item("Orbit Mode", action="resetViewport"),
            self._menu_item("Walk Mode", enabled=False),
        ]
        transform_menu = [
            self._menu_item("Translate", action="toolMove"),
            self._menu_item("Rotate", action="toolTransform"),
            self._menu_item("Scale", action="toolTransform"),
        ]
        workspace_layout_menu = [
            self._menu_item("Default", enabled=False),
            self._menu_item("Training Focused", enabled=False),
            self._menu_item("Inspection Focused", enabled=False),
        ]

        self._menu_definitions = {
            "file": [
                self._menu_item("New Project...", icon="folder", shortcut="CTRL+N", action="newProject"),
                self._menu_item("Open Project...", icon="folder", shortcut="CTRL+O", action="importScene"),
                self._menu_separator(),
                self._menu_item("Save", icon="copy", shortcut="CTRL+S", action="saveState"),
                self._menu_item("Save As...", shortcut="CTRL+SHIFT+S", enabled=False),
                self._menu_separator(),
                self._menu_item("Export", icon="arrow-up-right", submenu=export_menu),
                self._menu_separator(),
                self._menu_item("Exit", shortcut="ALT+F4", action="exitApp"),
            ],
            "edit": [
                self._menu_item("Undo", icon="rotate-ccw", shortcut="CTRL+Z", action="undoPreviewTransform"),
                self._menu_item("Redo", icon="rotate-ccw", shortcut="CTRL+Y", action="redoPreviewTransform"),
                self._menu_separator(),
                self._menu_item("Cut", shortcut="CTRL+X", enabled=False),
                self._menu_item("Copy", icon="copy", shortcut="CTRL+C", enabled=False),
                self._menu_item("Paste", shortcut="CTRL+V", enabled=False),
                self._menu_separator(),
                self._menu_item("Preferences", icon="settings", shortcut="CTRL+,", action="preferences"),
            ],
            "view": [
                self._menu_item("Reset Viewport", icon="maximize", shortcut="SPACE", action="resetViewport"),
                self._menu_separator(),
                self._menu_item("Appearance", submenu=appearance_menu),
                self._menu_item("Show grid", icon="activity", action="toggleViewportGrid"),
                self._menu_item("Toggle Bounding Box", icon="box-select", action="toolClip"),
            ],
            "tools": [
                self._menu_item("Camera Tools", icon="camera", submenu=camera_menu),
                self._menu_item("Transform", icon="move", submenu=transform_menu),
                self._menu_separator(),
                self._menu_item("Point Selection", icon="mouse-pointer-2", action="toolSelect"),
                self._menu_item("Color Picker", icon="pipette", action="toolColor"),
            ],
            "window": [
                self._menu_item("Project Explorer", action="toolProjects"),
                self._menu_item("Properties", enabled=False),
                self._menu_item("Console", icon="terminal", enabled=False),
                self._menu_separator(),
                self._menu_item("Workspace Layout", submenu=workspace_layout_menu),
            ],
            "help": [
                self._menu_item("Documentation", enabled=False),
                self._menu_item("Tutorials", enabled=False),
                self._menu_separator(),
                self._menu_item("About Gaussian Studio", icon="box", action="about"),
            ],
        }

    def _execute_menu_action(self, action_key: str) -> None:
        # The shell only reproduces the menu UI; actions are stubs except Exit.
        if action_key == "exitApp":
            self.closeAllMenus()
            window = self._app_window()
            if window is not None:
                window.close()
            return
        if action_key:
            print(f"[shell] menu action: {action_key}")
        self.closeAllMenus()
