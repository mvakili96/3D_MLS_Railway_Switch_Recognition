"""Optional Open3D desktop viewer. GUI imports are isolated from the core CLI."""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from threading import Event
import time
import textwrap
import sys

import numpy as np

from .models import InitialLocation
from .pipeline import initial_location_from_picks
from .points import Point3D
from .viewer_model import (
    DEFAULT_LAS, DEFAULT_CROP_RADIUS, DEFAULT_TRACKING_POINT_LIMIT, MAX_CROP_RADIUS,
    VANCOUVER_SEED, ViewerSession, load_preview, load_tracking_crop,
)


def _require_desktop():
    # Open3D's native window creation exits the process on an inaccessible X
    # display. Check first so headless/SSH users receive an actionable CLI error.
    if sys.platform.startswith("linux"):
        import ctypes
        from ctypes.util import find_library

        library = find_library("X11")
        if library is None:
            raise RuntimeError("The viewer requires desktop X11/XWayland libraries.")
        x11 = ctypes.CDLL(library)
        x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
        x11.XOpenDisplay.restype = ctypes.c_void_p
        x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
        display = x11.XOpenDisplay(None)
        if not display:
            raise RuntimeError("Cannot access the graphical display. Run the viewer from a desktop terminal with a working DISPLAY.")
        x11.XCloseDisplay(display)


def launch_viewer(path=None, *, radius=DEFAULT_CROP_RADIUS, preview_points=150_000,
                  max_tracking_points=DEFAULT_TRACKING_POINT_LIMIT, random_seed=1,
                  trace_branches=True, _test_driver=None):
    """Open the desktop app. _test_driver is an internal GUI acceptance hook."""
    _require_desktop()
    try:
        import open3d as o3d
        from open3d.visualization import gui, rendering
    except ImportError as error:
        raise RuntimeError('Install GUI support: python -m pip install -e ".[viewer]"') from error

    class Viewer:
        def __init__(self):
            self.app = gui.Application.instance
            self.app.initialize()
            self.window = self.app.create_window("Railway MLS — Vancouver demo", 1280, 920)
            self.closed = Event()
            self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="railway-viewer")
            self.busy = False
            self.playing = False
            self.preview = self.session = self.frame = None
            self.picking = False
            self.picks = []
            self.pick_generation = 0
            self.pick_pending = False
            self.radius = radius
            self.origin = np.zeros(3)
            self.last_progress = 0.0
            self.last_step = 0.0
            self.last_camera = None
            self.overlays = set()
            self.scene = gui.SceneWidget()
            self.scene.scene = rendering.Open3DScene(self.window.renderer)
            self.scene.scene.set_background([0.035, 0.047, 0.065, 1])
            self.scene.scene.show_axes(True)
            self.scene.set_on_mouse(self.on_mouse)
            self.window.add_child(self.scene)
            em = self.window.theme.font_size
            self.panel = gui.ScrollableVert(0.45 * em, gui.Margins(em, em, em, em))
            self.window.add_child(self.panel)
            self.panel.add_child(gui.Label("RAILWAY MLS"))
            self.panel.add_child(gui.Label("Interactive track and switch demo"))
            self.file_label = gui.Label("Open a LAS/LAZ file to begin.")
            self.panel.add_child(self.file_label)
            self.open_button = self.button("Open LAS / LAZ...", self.open_dialog)
            self.panel.add_fixed(0.5 * em)
            self.panel.add_child(gui.Label("1  Choose a starting position"))
            self.panel.add_child(gui.Label("Ctrl-click two cloud points to set\nposition and direction."))
            self.pick_button = self.button("Pick two points", self.start_picking)
            self.example_button = self.button("Use Vancouver source example", self.use_example)
            self.panel.add_child(gui.Label("Tracking crop radius (m)"))
            self.radius_edit = gui.NumberEdit(gui.NumberEdit.DOUBLE)
            self.radius_edit.set_limits(2.0, MAX_CROP_RADIUS)
            self.radius_edit.double_value = radius
            self.panel.add_child(self.radius_edit)
            self.panel.add_child(gui.Label("Tracking uses every source-selected\npoint inside the cyan region."))
            self.branch_checkbox = gui.Checkbox("Trace other switch paths")
            self.branch_checkbox.checked = trace_branches
            self.panel.add_child(self.branch_checkbox)
            self.panel.add_fixed(0.3 * em)
            self.panel.add_child(gui.Label("2  Confirm the fitted seed"))
            self.edits = []
            pose_panel = gui.CollapsableVert("Edit position / direction", 0.25 * em,
                                             gui.Margins(0, 0, 0, 0))
            pose_panel.set_is_open(False)
            for name, integer in (("X", False), ("Y", False), ("Z", False),
                                  ("Angle (degrees)", True), ("Direction mode", True)):
                row = gui.Horiz(0.3 * em)
                label = gui.Label(name)
                row.add_child(label)
                edit = gui.NumberEdit(gui.NumberEdit.INT if integer else gui.NumberEdit.DOUBLE)
                edit.decimal_precision = 6
                if name == "Angle (degrees)":
                    edit.set_limits(0, 360)
                if name == "Direction mode":
                    edit.set_limits(1, 4)
                    edit.int_value = 4
                row.add_child(edit)
                self.edits.append(edit)
                pose_panel.add_child(row)
            self.panel.add_child(pose_panel)
            self.confirm_button = self.button("Confirm start", self.confirm)
            self.panel.add_fixed(0.4 * em)
            self.panel.add_child(gui.Label("3  Follow the track windows"))
            row = gui.Horiz(0.3 * em)
            self.play_button = self.button("Play", self.play, row)
            self.pause_button = self.button("Pause", self.pause, row)
            self.step_button = self.button("Step", self.step, row)
            self.panel.add_child(row)
            self.reset_button = self.button("Reset / pick again", self.reset)
            self.extend_button = self.button("Restart with larger region", self.extend_region)
            row = gui.Horiz(0.3 * em)
            self.button("Top view", lambda: self.camera(top=True), row)
            self.button("3D view", lambda: self.camera(top=False), row)
            self.panel.add_child(row)
            self.panel.add_fixed(0.4 * em)
            self.panel.add_child(gui.Label("Green / red: rail windows\nGold: main · Purple: branch · Cyan: crop"))
            self.status_label = gui.Label("Ready to open a point cloud.")
            self.panel.add_child(self.status_label)
            self.detail_label = gui.Label("")
            self.panel.add_child(self.detail_label)
            self.window.set_on_layout(self.layout)
            self.window.set_on_tick_event(self.tick)
            self.window.set_on_close(self.close)
            self.update_controls()
            chosen = path if path is not None else (DEFAULT_LAS if DEFAULT_LAS.is_file() else None)
            if chosen is not None:
                self.open_file(chosen)

        def button(self, label, callback, parent=None):
            widget = gui.Button(label)
            widget.set_on_clicked(callback)
            (parent if parent is not None else self.panel).add_child(widget)
            return widget

        def layout(self, _):
            rect = self.window.content_rect
            width = min(345, rect.width * 0.4)
            self.panel.frame = gui.Rect(rect.x, rect.y, width, rect.height)
            self.scene.frame = gui.Rect(rect.x + width, rect.y, rect.width - width, rect.height)

        def post(self, callback):
            if not self.closed.is_set():
                def guarded():
                    if not self.closed.is_set():
                        callback()
                self.app.post_to_main_thread(self.window, guarded)

        def progress(self, message):
            now = time.monotonic()
            if now - self.last_progress >= 0.3:
                self.last_progress = now
                self.post(lambda value=message: self.set_status(value))

        def set_status(self, text):
            self.status_label.text = textwrap.fill(text, width=35)
            self.window.set_needs_layout()
            self.window.post_redraw()

        def submit(self, work, completed):
            if self.busy:
                return
            self.busy = True
            self.update_controls()
            future = self.executor.submit(work)

            def finish(future):
                try:
                    result = future.result()
                    error = None
                except Exception as exc:
                    result, error = None, str(exc)

                def apply():
                    self.busy = False
                    if error is not None:
                        self.playing = False
                        self.set_status("Could not continue.")
                        self.detail_label.text = error
                    else:
                        try:
                            completed(result)
                        except Exception as exc:
                            self.playing = False
                            self.set_status("Could not update the view.")
                            self.detail_label.text = str(exc)
                    self.update_controls()
                    self.window.post_redraw()
                self.post(apply)
            future.add_done_callback(finish)

        def update_controls(self):
            loaded = self.preview is not None
            phase = self.frame.phase if self.frame else "empty"
            self.open_button.enabled = not self.busy and not self.playing
            self.pick_button.enabled = loaded and not self.busy and not self.playing
            self.example_button.enabled = (loaded and self.preview.info.contains(VANCOUVER_SEED.point)
                                           and not self.busy and not self.playing)
            self.confirm_button.enabled = not self.busy and phase == "preview"
            ready = phase in ("ready", "tracking")
            self.play_button.enabled = ready and not self.busy and not self.playing
            self.step_button.enabled = ready and not self.busy and not self.playing
            self.pause_button.enabled = self.playing
            self.reset_button.enabled = loaded and not self.busy
            self.extend_button.enabled = (not self.busy and not self.playing and self.session is not None
                                          and self.frame is not None and self.frame.boundary == "crop"
                                          and self.session.crop.radius < MAX_CROP_RADIUS)
            self.radius_edit.enabled = not self.busy and not self.playing and phase == "empty"
            self.branch_checkbox.enabled = not self.busy and not self.playing and phase == "empty"
            for edit in self.edits:
                edit.enabled = not self.busy and phase == "preview"

        def open_dialog(self):
            dialog = gui.FileDialog(gui.FileDialog.OPEN, "Open point cloud", self.window.theme)
            dialog.add_filter(".las .laz", "LAS / LAZ point clouds")
            if self.preview:
                dialog.set_path(str(self.preview.info.path.parent))
            elif Path("Vancouver").is_dir():
                dialog.set_path(str(Path("Vancouver").resolve()))
            dialog.set_on_cancel(self.window.close_dialog)

            def selected(filename):
                self.window.close_dialog()
                self.open_file(filename)
            dialog.set_on_done(selected)
            self.window.show_dialog(dialog)

        def open_file(self, filename):
            self.reset()
            self.set_status("Loading cloud preview…")
            self.detail_label.text = "Tracking will use a crop around your chosen seed."
            self.submit(lambda: load_preview(filename, max_points=preview_points,
                                             progress=self.progress, cancel=self.closed.is_set),
                        self.show_preview)

        def show_preview(self, preview):
            self.preview = preview
            self.origin = preview.info.origin
            self.scene.scene.clear_geometry()
            cloud = o3d.geometry.PointCloud()
            cloud.points = o3d.utility.Vector3dVector(preview.xyz - self.origin)
            height = preview.xyz[:, 2]
            lo, hi = np.percentile(height, [2, 95])
            t = np.clip((height - lo) / max(hi - lo, 0.01), 0, 1)
            cloud.colors = o3d.utility.Vector3dVector(
                np.column_stack((0.30 + 0.42 * t, 0.48 + 0.30 * t, 0.58 + 0.27 * t)))
            material = rendering.MaterialRecord()
            material.shader, material.point_size = "defaultUnlit", 2.0 * self.window.scaling
            self.scene.scene.add_geometry("cloud", cloud, material)
            self.file_label.text = (f"{preview.info.path.name} · {preview.info.point_count:,} records\n"
                                    f"Preview: {len(preview.xyz):,} points")
            self.camera(top=True)
            self.set_status("Choose two points or use the source example.")

        def camera(self, top=False, center=None, extent=None):
            if not self.preview:
                return
            bounds = self.preview.info.bounds
            center = ((bounds.minimum.as_array() + bounds.maximum.as_array()) / 2
                      if center is None else np.asarray(center))
            local = center - self.origin
            span = max(bounds.maximum.x - bounds.minimum.x,
                       bounds.maximum.y - bounds.minimum.y, 2) if extent is None else extent
            # A local rendering origin avoids float32 loss at projected world coordinates.
            box = o3d.geometry.AxisAlignedBoundingBox(local - span / 2, local + span / 2)
            self.scene.setup_camera(50, box, local)
            eye = local + (np.array([0, 0, 1.6]) if top else np.array([0, -1.1, 0.9])) * span
            self.scene.scene.camera.look_at(local, eye, [0, 1, 0] if top else [0, 0, 1])
            self.last_camera = (top, center.copy(), span)
            self.scene.force_redraw()
            self.window.post_redraw()

        def clear_overlay(self):
            for name in self.overlays | {"pick0", "pick1"}:
                if self.scene.scene.has_geometry(name):
                    self.scene.scene.remove_geometry(name)
            self.overlays.clear()

        def line(self, name, xyz, color, *, closed=False, width=3):
            if self.scene.scene.has_geometry(name):
                self.scene.scene.remove_geometry(name)
            if len(xyz) < 2:
                return
            pairs = [[i, i + 1] for i in range(len(xyz) - 1)]
            if closed:
                pairs.append([len(xyz) - 1, 0])
            geometry = o3d.geometry.LineSet(o3d.utility.Vector3dVector(xyz - self.origin),
                                          o3d.utility.Vector2iVector(pairs))
            geometry.paint_uniform_color(color)
            material = rendering.MaterialRecord()
            material.shader, material.line_width = "unlitLine", width * self.window.scaling
            self.scene.scene.add_geometry(name, geometry, material)
            self.overlays.add(name)

        def start_picking(self):
            self.reset()
            self.picking = True
            self.set_status("Ctrl-click the first cloud point.")

        def on_mouse(self, event):
            if (not self.picking or self.busy or event.type != gui.MouseEvent.Type.BUTTON_DOWN
                    or not event.is_button_down(gui.MouseButton.LEFT)
                    or not event.is_modifier_down(gui.KeyModifier.CTRL)):
                return gui.Widget.EventCallbackResult.IGNORED
            if self.pick_pending:
                return gui.Widget.EventCallbackResult.HANDLED
            frame = self.scene.frame
            x, y = int(event.x - frame.x), int(event.y - frame.y)
            width, height = int(frame.width), int(frame.height)
            if not (0 <= x < width and 0 <= y < height):
                return gui.Widget.EventCallbackResult.IGNORED

            self.pick_pending = True
            generation = self.pick_generation

            def depth_ready(image):
                data = np.asarray(image)
                depth = float(data[y, x]) if y < data.shape[0] and x < data.shape[1] else 1.0

                def apply_pick():
                    if generation != self.pick_generation or self.closed.is_set():
                        return
                    self.pick_pending = False
                    if depth >= 1 or not np.isfinite(depth):
                        self.set_status("Click directly on a visible cloud point.")
                        return
                    local = self.scene.scene.camera.unproject(x, y, depth, width, height)
                    world = np.asarray(local, dtype=np.float64).reshape(3) + self.origin
                    self.add_pick(Point3D(*world))
                self.post(apply_pick)
            self.scene.scene.scene.render_to_depth_image(depth_ready)
            self.scene.force_redraw()
            self.window.post_redraw()
            return gui.Widget.EventCallbackResult.HANDLED

        def add_pick(self, point):
            if not self.picking or self.busy:
                return
            # Resolve display-depth rounding to an actual preview record. This
            # returns original double XYZ and avoids selecting axes/background.
            distance = np.sum((self.preview.xyz - point.as_array()) ** 2, axis=1)
            nearest = int(np.argmin(distance))
            if distance[nearest] > 0.2**2:
                self.set_status("Click directly on a visible cloud point.")
                return
            point = Point3D(*self.preview.xyz[nearest])
            self.picks.append(point)
            sphere = o3d.geometry.TriangleMesh.create_sphere(0.055)
            sphere.translate(point.as_array() - self.origin)
            material = rendering.MaterialRecord()
            material.shader, material.base_color = "defaultUnlit", [1, 0.8, 0.2, 1]
            self.scene.scene.add_geometry(f"pick{len(self.picks)-1}", sphere, material)
            if len(self.picks) == 1:
                self.set_status("Ctrl-click the second point to set direction.")
            else:
                self.picking = False
                if np.linalg.norm((self.picks[1] - self.picks[0]).as_array()) < 0.02:
                    self.set_status("Choose two different points. Pick again to retry.")
                    return
                self.prepare(initial_location_from_picks(*self.picks))

        def use_example(self):
            self.reset()
            self.prepare(VANCOUVER_SEED)

        def prepare(self, pose, *, confirmation=None):
            radius = self.radius_edit.double_value
            branches = self.branch_checkbox.checked
            self.set_status("Loading the tracking region…")
            filename = self.preview.info.path

            def work():
                crop = load_tracking_crop(filename, pose.point, radius=radius,
                                          max_points=max_tracking_points,
                                          progress=self.progress, cancel=self.closed.is_set)
                self.progress(f"Building the grid for {len(crop.cloud):,} points…")
                session = ViewerSession(crop, random_seed=random_seed, trace_branches=branches)
                return session, session.preview(pose)

            def prepared(result):
                self.session, frame = result
                self.apply_frame(frame)
                edited_pose = confirmation or frame.pose
                if edited_pose:
                    for edit, value in zip(self.edits, (*edited_pose.point.as_array(),
                                                        edited_pose.angle, edited_pose.mode)):
                        edit.double_value = value
                self.camera(top=True, center=pose.point.as_array(), extent=radius * 2.7)
            self.submit(work, prepared)

        def confirm(self):
            pose = InitialLocation(Point3D(*(e.double_value for e in self.edits[:3])),
                                   self.edits[3].int_value, self.edits[4].int_value)
            self.set_status("Confirming starting windows…")
            self.submit(lambda: self.session.confirm(pose), self.apply_frame)

        def extend_region(self):
            if self.busy or self.session is None or self.session.preview_seed is None:
                return
            radius = min(MAX_CROP_RADIUS, self.session.crop.radius * 2)
            pose, confirmation = self.session.preview_seed, self.session.confirmed_seed
            self.reset()
            self.radius_edit.double_value = radius
            self.prepare(pose, confirmation=confirmation)

        def apply_frame(self, frame):
            self.frame = frame
            self.clear_overlay()
            colors = ([0.25, 1, 0.52], [1, 0.32, 0.32])
            for i, xyz in enumerate(frame.windows):
                self.line(f"window{i}", xyz, colors[i], closed=True, width=4)
            path_sets = [trace.paths for trace in frame.traces] or [frame.paths]
            for i, paths in enumerate(path_sets):
                center_color = [1, 0.78, 0.15] if i == 0 else [0.76, 0.5, 1]
                for role, xyz, color in zip(("rail0", "rail1", "center"), paths,
                                           (*colors, center_color)):
                    self.line(f"path{i}_{role}", xyz, color, width=4 if role == "center" else 2)
            crop = self.session.crop
            x, y, z = crop.center.as_array()
            r = crop.radius
            outline = np.array([[x-r,y-r,z], [x+r,y-r,z], [x+r,y+r,z], [x-r,y+r,z]])
            self.line("crop", outline, [0.1, 0.8, 0.95], closed=True, width=2)
            details = f"{len(crop.cloud):,} tracking points\nCrop: {2*r:g} × {2*r:g} m"
            for trace in frame.traces:
                details += f"\n{trace.name}: {trace.accepted_steps} steps · {trace.distance:.2f} m"
            if frame.phase == "preview":
                message = "Check the starting windows, then Confirm start."
            elif frame.phase == "ready":
                message = "Ready. Press Play or Step."
            elif frame.phase == "tracking":
                if frame.status == "branches_pending":
                    message = "Returning to the switch region for the next path…"
                elif frame.status == "branch_initialized":
                    message = f"{frame.active_path} initialized at the switch."
                else:
                    message = (f"{'Tracking' if self.playing else 'Paused'} {frame.active_path} · "
                               f"{frame.accepted_steps} steps · {frame.distance:.2f} m")
            elif frame.phase == "stopped":
                self.playing = False
                message = (f"Stopped · {len(frame.traces)} paths · {frame.total_steps} total steps · "
                           f"{frame.status.replace('_', ' ')}")
                if frame.boundary == "crop":
                    details += "\nSearch reached the crop edge.\nRestart with a larger region to trace farther."
                elif frame.boundary == "file":
                    details += "\nSearch reached this LAS tile's edge.\nMore data may require another tile."
                else:
                    details += "\nThe tracker reached a source stop condition."
            else:
                message = f"Initialization failed: {frame.status.replace('_', ' ')}"
            if frame.notes:
                details += "\n" + "\n".join(frame.notes)
            self.detail_label.text = details
            self.set_status(message)
            self.scene.force_redraw()

        def play(self):
            self.playing = True
            self.update_controls()

        def pause(self):
            self.playing = False
            self.set_status("Pausing after the current step…" if self.busy else "Paused. Press Play or Step.")
            self.update_controls()

        def step(self):
            if self.busy or self.session is None:
                return
            self.last_step = time.monotonic()
            self.submit(self.session.step, self.apply_frame)

        def reset(self):
            if self.busy:
                return
            self.playing = self.picking = False
            self.session = self.frame = None
            self.picks = []
            self.pick_generation += 1
            self.pick_pending = False
            self.clear_overlay()
            self.detail_label.text = ""
            self.set_status("Choose two points or use the source example.")
            self.update_controls()

        def tick(self):
            if self.closed.is_set():
                return False
            if _test_driver:
                _test_driver(self)
            if self.playing and not self.busy and time.monotonic() - self.last_step >= 0.4:
                self.step()
            return False

        def close(self):
            self.playing = False
            self.closed.set()
            self.executor.shutdown(wait=False, cancel_futures=True)
            return True

    viewer = Viewer()
    viewer.app.run()
