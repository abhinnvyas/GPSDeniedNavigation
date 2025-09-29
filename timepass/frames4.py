# ===== Refactored GUI - Triple Sensor Payload (with threading and FPS logging) =====
# Requires: Python 3.x, tkinter, pillow, opencv-python, pyserial, gstreamer (for day stream)
# Save as frames3.py and run.

import gi
gi.require_version("Gst", "1.0")
gi.require_version("GstApp", "1.0")
from gi.repository import Gst
import numpy as np
import cv2
import serial
import serial.tools.list_ports
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox
from collections import deque
import subprocess
import os
import signal
import queue # NEW: Import for thread-safe queue

Gst.init(None)

# --- Safe Import for ImageTk ---
try:
    from PIL import Image, ImageTk
    PIL_AVAILABLE = True
except Exception:
    PIL_AVAILABLE = False



# ===================== CROSSHAIR (SIMPLE '+') =====================
def overlay_crosshair(frame):
    """
    Draw a simple red '+' crosshair at the center of the frame.
    Returns modified frame (same dtype).
    """
    if frame is None or frame.size == 0:
        return frame

    out = frame.copy()
    h, w = out.shape[:2]
    cx, cy = w // 2, h // 2

    color = (0, 0, 255)  # Red (BGR)
    thickness = 2
    size = max(10, min(w, h) // 20)

    # Horizontal line
    cv2.line(out, (cx - size, cy), (cx + size, cy), color, thickness, cv2.LINE_AA)
    # Vertical line
    cv2.line(out, (cx, cy - size), (cx, cy + size), color, thickness, cv2.LINE_AA)

    return out

# =================== LRF COMMANDS ===================
STOP_MEASUREMENT        = bytes([0x55, 0xAA, 0x8E, 0xFF, 0xFF, 0xFF, 0xFF, 0x8A])
CONTINUOUS_MEASUREMENT  = bytes([0x55, 0xAA, 0x89, 0xFF, 0xFF, 0xFF, 0xFF, 0x85])
SINGLE_MEASUREMENT      = bytes([0x55, 0xAA, 0x88, 0xFF, 0xFF, 0xFF, 0xFF, 0x84])

# ================= THERMAL COMMANDS =================
def build_sumcheck(val, func, opcode):
    val = int(val)
    ranges = {
        "brightness": (10, 250),
        "contrast": (10, 250),
        "reference_image": (0, 128),
        "DDE_mode": (1, 3),
        "DDE_strength": (1, 128),
        "zoom": (1, 16),
    }
    if func in ranges:
        mn, mx = ranges[func]
        if not (mn <= val <= mx):
            raise ValueError(f"{func} must be between {mn} and {mx}")
    checksum = opcode + val
    return f"{checksum & 0xFF:02X} {(checksum >> 8) & 0xFF:02X}"


def checksum_response(data):
    return sum(data) & 0xFFFF


def send_to_ir_camera(ser, hex_data, response_len=16, read_timeout=1):
    try:
        ser.timeout = read_timeout
        data = [int(x, 16) for x in hex_data.split()]
        ser.write(bytes(data))
        time.sleep(0.1)
        response = ser.read(response_len)
        if len(response) < 8:
            return 0
        data_length = response[2]
        chk_lo = response[6]
        chk_hi = response[7]
        recv_chk = (chk_hi << 8) | chk_lo
        calc = list(response)
        calc[6] = 0
        calc[7] = 0
        if recv_chk == checksum_response(calc):
            expected = 8 + data_length
            if len(response) >= expected:
                return response[8:expected]
            else:
                return []
        return 0
    except serial.SerialException:
        return 0

THERMAL_FUNCTION_GROUPS = {
    "color": {
        "response_len": 10,
        "functions": {
            "rainbow": {"data": "68 11 03 01 00 00 80 00 02 01 00"},
            "green":   {"data": "68 11 03 01 00 00 81 00 02 01 01"},
            "metel":   {"data": "68 11 03 01 00 00 82 00 02 01 02"},
            "white":   {"data": "68 11 03 01 00 00 83 00 02 01 03"},
            "black":   {"data": "68 11 03 01 00 00 84 00 02 01 04"},
            "save":    {"data": "68 11 01 01 00 00 7E 00 03"},
        },
    },
    "hotspot": {
        "response_len": 12,
        "functions": {
            "on":  {"data": "68 10 03 01 00 00 80 00 02 01 01"},
            "off": {"data": "68 10 03 01 00 00 7F 00 02 01 00"},
        },
    },
    "brightness": {
        "response_len": 12,
        "parameterized": True,
        "opcode": 0x93,
        "build_data": lambda v: f"68 24 04 01 00 00 {build_sumcheck(v, 'brightness', 0x93)} 02 00 {int(v):02X} 00",
    },
    "contrast": {
        "response_len": 12,
        "parameterized": True,
        "opcode": 0x94,
        "build_data": lambda v: f"68 24 04 01 00 00 {build_sumcheck(v, 'contrast', 0x94)} 02 01 {int(v):02X} 00",
    },
    "denoise": {
        "response_len": 12,
        "parameterized": True,
        "opcode": 0x98,
        "build_data": lambda v: f"68 24 04 01 00 00 {build_sumcheck(v,'denoise',0x98)} 02 05 {int(v):02X} 00",
    },
    "vstripe": {
        "response_len": 12,
        "parameterized": True,
        "opcode": 0x9A,
        "build_data": lambda v: f"68 24 04 01 00 00 {build_sumcheck(v,'vstripe',0x9A)} 02 07 {int(v)&1:02X} 00",
    },
    "zoom": {
        "response_len": 12,
        "parameterized": True,
        "opcode": 0x96,
        "build_data": lambda v: f"68 24 04 01 00 00 {build_sumcheck(v, 'zoom', 0x96)} 04 01 {int(v):02X} 00",
    },
}

THERMAL_INFO_REQUESTS = [
    {"label": "Chip model: ",   "data": "68 01 00 01 00 00 6A 00",        "response_len": 10},
    {"label": "Chip ID: ",      "data": "68 35 00 01 00 00 9E 00",        "response_len": 16},
    {"label": "Firmware ver: ", "data": "68 10 02 01 00 00 7C 00 01 00",  "response_len": 12},
    {"label": "Hardware ver: ", "data": "68 10 02 01 00 00 7D 00 01 01",  "response_len": 12},
]

# ===================== NEW: Camera Streamer Thread Class =====================
class CameraStreamer:
    def __init__(self, source_type, source_id, frame_queue, fps=30):
        self.source_type = source_type  # 'cv2' or 'gstreamer'
        self.source_id = source_id
        self.frame_queue = frame_queue
        self.stop_event = threading.Event()
        self.thread = None
        self.fps = fps
        self.pipeline = None
        self.sink = None
        self.day_paused = False # Flag for the Day camera pause/resume feature

    def start(self):
        if self.thread and self.thread.is_alive():
            print("Streamer already running.")
            return

        self.stop_event.clear()
        self.day_paused = False
        if self.source_type == 'cv2':
            self.thread = threading.Thread(target=self._run_cv2_stream, daemon=True)
            self.thread.start()
        elif self.source_type == 'gstreamer':
            self.thread = threading.Thread(target=self._run_gstreamer_stream, daemon=True)
            self.thread.start()

    def stop(self):
        self.stop_event.set()
        if self.thread:
            # It's good practice to try to set the pipeline to NULL from the main thread if possible
            # to release resources, but joining the thread is the main goal here.
            self.thread.join(timeout=2) # Wait for thread to finish

    def _run_cv2_stream(self):
        cap = cv2.VideoCapture(self.source_id)
        if not cap.isOpened():
            print(f"Failed to open OpenCV source {self.source_id}")
            return
        
        # FIX: Ensure 640x480 resolution and 30 FPS for the thermal camera
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)

        while not self.stop_event.is_set():
            ret, frame = cap.read()
            if not ret or frame is None:
                continue
            
            # Put the frame in the queue
            # This is the only work done in the background thread for CV2
            if not self.day_paused:
                self.frame_queue.put(frame.copy())
            
            # Control frame rate
            time.sleep(1/self.fps)
            
        cap.release()
        print("CV2 stream thread stopped.")

    def _run_gstreamer_stream(self):
        try:
            self.pipeline = Gst.parse_launch(self.source_id)
            self.sink = self.pipeline.get_by_name("sink")
            if not self.sink:
                raise RuntimeError("GStreamer pipeline must contain an appsink named 'sink'")

            self.sink.set_property("emit-signals", True)
            self.sink.set_property("max-buffers", 1)
            self.sink.set_property("drop", True)
            self.sink.connect("new-sample", self._on_gstreamer_sample)

            self.pipeline.set_state(Gst.State.PLAYING)
            
            # Wait for the pipeline to stop
            bus = self.pipeline.get_bus()
            while not self.stop_event.is_set():
                msg = bus.timed_pop_filtered(Gst.SECOND, Gst.MessageType.ANY)
                if msg:
                    # Handle GStreamer messages, e.g., errors
                    if msg.type == Gst.MessageType.ERROR:
                        print("GStreamer Error:", msg.parse_error())
                        self.stop_event.set()
            
            self.pipeline.set_state(Gst.State.NULL)
            self.sink = None
            self.pipeline = None
            
        except Exception as e:
            print(f"GStreamer stream error: {e}")
            self.stop_event.set()
            
        print("GStreamer stream thread stopped.")

    def _on_gstreamer_sample(self, sink):
        if self.stop_event.is_set() or self.day_paused:
            return Gst.FlowReturn.OK

        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.OK
        
        # Get frame data from sample
        buf = sample.get_buffer()
        data = buf.extract_dup(0, buf.get_size())
        caps = sample.get_caps()
        
        try:
            w = caps.get_structure(0).get_value("width")
            h = caps.get_structure(0).get_value("height")
        except Exception:
            # Default to 640x480 if caps fails
            w, h = 640, 480
        
        # Determine format and reshape
        arr = np.frombuffer(data, np.uint8)
        
        # FIX: The pipeline is now configured to output RGB, which is 3 channels.
        expected_size_rgb = h * w * 3

        if arr.size == expected_size_rgb:
            # Correct size for RGB (3 channels)
            arr = arr.reshape((h, w, 3))
        elif arr.size == (h * w):
            # This handles unexpected grayscale data
            arr = arr.reshape((h, w))
        else:
            # This handles the common ValueError from the original log.
            # We skip the frame and report the issue.
            print(f"ValueError: cannot reshape array of size {arr.size} into expected shape ({h},{w},3) or ({h},{w}). Skipping frame.")
            return Gst.FlowReturn.OK
        
        # Put the raw frame array into the queue
        self.frame_queue.put(arr.copy())
        
        return Gst.FlowReturn.OK

# ===================== UNIFIED SINGLE-PAGE GUI =====================
class TriplePayloadGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Entangled Photons EO/IR")
        self.root.geometry("1500x900")

        # ---- State ----
        self.lrf_ser = None
        self.lrf_running = False
        self.lrf_last_distance = None
        self.lrf_read_thread = None

        self.thermal_ser = None
        self.thermal_connected = False
        self.thermal_streaming = False
        # FIX: Initialize day_streaming flag to resolve AttributeError
        self.day_streaming = False 
        
        self.thermal_size = (640, 480)
        self.thermal_palette = "white"

        # NEW: Day and Thermal Camera Streamer objects
        self.day_queue = queue.Queue(maxsize=2)
        self.thermal_queue = queue.Queue(maxsize=2)
        self.day_streamer = None
        self.thermal_streamer = None
        
        self.fullscreen_mode = False
        self.day_paused = False
        self.day_colour_running = False
        self.crosshair_enabled = False
        self.day_zoom_level = tk.DoubleVar(value=1.0)
        self.thermal_zoom_var = tk.IntVar(value=1)

        # NEW: FPS Counters and Timers

        self.day_pipeline = None
        self.day_frames_count = 0
        self.day_start_time = time.time()
        self.day_streaming_mode = "1" # 1 for alone, 2 for together
        self.thermal_frames_count = 0
        self.thermal_start_time = time.time()

        self.SENSOR_MAX_WIDTH = 4112
        self.SENSOR_MAX_HEIGHT = 3008

            # Target stream settings
        self.ROI_WIDTH = 640
        self.ROI_HEIGHT = 480
        self.TARGET_FPS = 30

        # Build UI
        self._build_layout()
        if PIL_AVAILABLE:
            black_frame = np.zeros((480, 640, 3), dtype=np.uint8)
            try:
                self.day_black_img = ImageTk.PhotoImage(image=Image.fromarray(black_frame))
                self.day_video_label.config(image=self.day_black_img)
                self.day_video_label.imgtk = self.day_black_img
                self.thermal_video_label.config(image=self.day_black_img)
                self.thermal_video_label.imgtk = self.day_black_img
            except Exception:
                pass

        # Bind close
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        # Start the GUI update loops
        self._update_day_video_loop()
        self._update_thermal_video_loop()

    # ----------------- NEW: GUI update loops -----------------
    def _update_day_video_loop(self):
        """
        Pulls a frame from the Day queue and updates the display.
        This runs on the main GUI thread and is non-blocking.
        """
        try:
            frame = self.day_queue.get_nowait()
            self.day_frames_count += 1
            
            elapsed_time = time.time() - self.day_start_time
            if elapsed_time >= 1.0:
                fps = self.day_frames_count / elapsed_time
                print(f"{self.day_streaming_mode}: Day Camera FPS: {fps:.2f}")
                self.day_frames_count = 0
                self.day_start_time = time.time()

            # --- Frame Processing (on GUI thread, but it's fast now) ---
            if frame.size == 0:
                self.root.after(30, self._update_day_video_loop)
                return

            # Apply digital zoom
            h, w = frame.shape[:2]
            zoom_level = self.day_zoom_level.get()
            if zoom_level > 1.0:
                zoom_factor = 1.0 / zoom_level
                zoom_w, zoom_h = int(w * zoom_factor), int(h * zoom_factor)
                cx, cy = w // 2, h // 2
                x1, y1 = cx - zoom_w // 2, cy - zoom_h // 2
                x2, y2 = cx + zoom_w // 2, cy + zoom_h // 2
                frame = frame[y1:y2, x1:x2]

            # Determine target widget and size
            if self.fullscreen_mode == "day_thermal":
                widget_w, widget_h = max(10, self.day_video_frame.winfo_width()), max(10, self.day_video_frame.winfo_height())
                target_label = self.day_video_label
            elif self.fullscreen_mode == "thermal_day":
                widget_w, widget_h = 320, 240
                target_label = self.day_overlay_label
            else:
                widget_w, widget_h = max(10, self.day_video_frame.winfo_width()), max(10, self.day_video_frame.winfo_height())
                target_label = self.day_video_label
                
            # Convert to BGR for crosshair and resize
            if frame.ndim == 2: # Grayscale
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            else: # RGB or BGR (Gstreamer's RGB is converted to BGR for OpenCV functions)
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) 
            
            resized_frame = cv2.resize(frame_bgr, (widget_w, widget_h), interpolation=cv2.INTER_AREA)

            if self.crosshair_enabled:
                resized_frame = overlay_crosshair(resized_frame)
            
            # Convert back to RGB for PIL
            resized_rgb = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2RGB)
            
            img = Image.fromarray(resized_rgb)
            imgtk = ImageTk.PhotoImage(image=img)
            target_label.imgtk = imgtk
            target_label.config(image=imgtk)

        except queue.Empty:
            # Queue is empty, nothing to do.
            pass
        except Exception as e:
            self._set_status(f"Day video update error: {e}")

        self.root.after(30, self._update_day_video_loop)

    def _update_thermal_video_loop(self):
        """
        Pulls a frame from the Thermal queue and updates the display.
        This also runs on the main GUI thread.
        """
        try:
            frame = self.thermal_queue.get_nowait()
            self.thermal_frames_count += 1

            elapsed_time = time.time() - self.thermal_start_time
            if elapsed_time >= 1.0:
                fps = self.thermal_frames_count / elapsed_time
                print(f"Thermal Camera FPS: {fps:.2f}")
                self.thermal_frames_count = 0
                self.thermal_start_time = time.time()

            # --- Frame Processing (on GUI thread, fast now) ---
            if frame.size == 0:
                self.root.after(30, self._update_thermal_video_loop)
                return

            # Apply color map
            if self.thermal_palette == "white":
                show = cv2.applyColorMap(frame, cv2.COLORMAP_BONE)
            elif self.thermal_palette == "black":
                show = cv2.applyColorMap(frame, cv2.COLORMAP_OCEAN)
            elif self.thermal_palette == "rainbow":
                show = cv2.applyColorMap(frame, cv2.COLORMAP_JET)
            elif self.thermal_palette == "green":
                show = cv2.applyColorMap(frame, cv2.COLORMAP_SUMMER)
            elif self.thermal_palette == "metel":
                show = cv2.applyColorMap(frame, cv2.COLORMAP_HOT)
            else:
                show = frame
                
            if self.crosshair_enabled:
                show = overlay_crosshair(show)
                
            rgb = cv2.cvtColor(show, cv2.COLOR_BGR2RGB)

            if self.fullscreen_mode == "day_thermal":
                widget_w, widget_h = 320, 240
                target_label = self.thermal_overlay_label
            else:
                widget_w, widget_h = max(10, self.thermal_video_frame.winfo_width()), max(10, self.thermal_video_frame.winfo_height())
                target_label = self.thermal_video_label
            
            resized = cv2.resize(rgb, (widget_w, widget_h), interpolation=cv2.INTER_AREA)
            img = Image.fromarray(resized)
            imgtk = ImageTk.PhotoImage(image=img)
            
            target_label.imgtk = imgtk
            target_label.config(image=imgtk)

        except queue.Empty:
            pass
        except Exception as e:
            self._set_status(f"Thermal video update error: {e}")

        self.root.after(30, self._update_thermal_video_loop)

    # ----------------- Layout helpers (unchanged) -----------------
    def _make_scrollable_frame(self, parent):
        canvas = tk.Canvas(parent, highlightthickness=0)
        vsb = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        inner = ttk.Frame(canvas)
        inner.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=inner, anchor="nw")
        canvas.configure(yscrollcommand=vsb.set)
        canvas.pack(side="left", fill="both", expand=True)
        vsb.pack(side="right", fill="y")
        return (canvas, inner)

    # ===================== UI LAYOUT (unchanged) =====================
    def _build_layout(self):
        # ... (unchanged layout code from original file) ...
        self.header = ttk.Frame(self.root, padding=(8, 6))
        self.header.pack(fill="x")

        # logo (left corner) - keep same logo file if present
        if PIL_AVAILABLE:
            self.logo_img = None
            for candidate in ["./icon.png", "/mnt/data/gui.jpg", "logo.png"]:
                try:
                    if os.path.exists(candidate):
                        img = Image.open(candidate).convert("RGBA")
                        img.thumbnail((120, 120))
                        self.logo_img = ImageTk.PhotoImage(img)
                        break
                except Exception:
                    pass
            self.logo_label = None # Store the label for later hiding
            if self.logo_img:
                self.logo_label = ttk.Label(self.header, image=self.logo_img)
                self.logo_label.pack(side="left", padx=(0, 10))

        style = ttk.Style()
        style.configure("Header.TLabel", foreground="black")
        style.configure("Tiny.TButton", font=("Segoe UI", 8), padding=[0, 0, 0, 0])

        self.title_label = ttk.Label(
            self.header,
            text="Entangled Photons EO/IR",
            font=("Segoe UI",50, "bold"),
            style="Header.TLabel"
       )
        self.title_label.pack(expand=True, anchor="center")

        # Create the exit fullscreen button but don't pack it yet
        self.exit_fs_button = ttk.Button(self.root, text="Exit Fullscreen", command=self._exit_fullscreen)
        # New variable for the day overlay label
        self.day_overlay_label = None

        # -------- Main body: streams + stacked settings on right --------
        self.body = ttk.Frame(self.root, padding=6)
        self.body.pack(fill="both", expand=True)
        # configure 3 columns: day | thermal | settings
        self.body.grid_rowconfigure(0, weight=1)
        self.body.grid_columnconfigure(0, weight=1)
        self.body.grid_columnconfigure(1, weight=1)
        self.body.grid_columnconfigure(2, weight=0)

        # ---- Stream-01 (Day) ----
        self.s1_group = ttk.Labelframe(self.body, text="STREAM-01 (Day Camera)")
        self.s1_group.grid(row=0, column=0, sticky="nsew", padx=(4,4), pady=4)
        self.s1_group.grid_rowconfigure(0, weight=1)
        self.s1_group.grid_columnconfigure(0, weight=1)

        self.day_video_frame = tk.Frame(self.s1_group, bg="black")
        self.day_video_frame.grid(row=0, column=0, sticky="nsew", padx=6, pady=6)
        self.day_video_label = tk.Label(self.day_video_frame, bg="black")
        self.day_video_label.place(relx=0.5, rely=0.5, anchor="center")
        self.day_video_frame.grid_propagate(False)

        # ---- Stream-02 (Thermal) ----
        self.s2_group = ttk.Labelframe(self.body, text="STREAM-02 (Thermal Camera)")
        self.s2_group.grid(row=0, column=1, sticky="nsew", padx=(4,4), pady=4)
        self.s2_group.grid_rowconfigure(0, weight=1)
        self.s2_group.grid_columnconfigure(0, weight=1)

        self.thermal_video_frame = tk.Frame(self.s2_group, bg="black")
        self.thermal_video_frame.grid(row=0, column=0, sticky="nsew", padx=6, pady=6)
        self.thermal_video_label = tk.Label(self.thermal_video_frame, bg="black")
        self.thermal_video_label.place(relx=0.5, rely=0.5, anchor="center")
        self.thermal_video_frame.grid_propagate(False)

        # Range overlay (previously LRF)
        self.lrf_overlay = tk.Label(self.thermal_video_frame,
                                    text="Range: --.- m",
                                    font=("Segoe UI", 12, "bold"),
                                    fg="cyan", bg="black")
        self.lrf_overlay.place(relx=1.0, rely=0.0, anchor="ne", x=-8, y=8)

        # ---- Right column: stacked settings with scrolls ----
        self.right = ttk.Frame(self.body)
        self.right.grid(row=0, column=2, sticky="nsew", padx=(6,4), pady=4)
        self.right.grid_rowconfigure(0, weight=0)  #DAY smaller
        self.right.grid_rowconfigure(1, weight=6)  #THERMAL bigger
        self.right.grid_columnconfigure(0, weight=1)

        # Day settings (scrollable)
        day_settings_frame = ttk.Labelframe(self.right, text="DAY")
        day_settings_frame.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
        day_canvas, day_inner = self._make_scrollable_frame(day_settings_frame)

        # Minimal day settings (use same variable names)
        self.day_exposure = tk.DoubleVar(value=5.0)
        self.day_gain     = tk.DoubleVar(value=1.0)
        self.day_wb       = tk.IntVar(value=4500)
        self.day_zoom_level = tk.DoubleVar(value=1.0)

        ttk.Label(day_inner, text="Exposure (ms)").grid(row=0, column=0, sticky="w", padx=6, pady=(6,0))
        ttk.Scale(day_inner, from_=0.1, to=30.0, orient="horizontal", variable=self.day_exposure,
                  command=lambda _=None: self._apply_day_setting("Exposure", self.day_exposure.get())).grid(row=0, column=1, sticky="ew", padx=6, pady=(6,0))
        ttk.Label(day_inner, text="Gain").grid(row=1, column=0, sticky="w", padx=6, pady=(6,0))
        ttk.Scale(day_inner, from_=0.0, to=16.0, orient="horizontal", variable=self.day_gain,
                  command=lambda _=None: self._apply_day_setting("Gain", self.day_gain.get())).grid(row=1, column=1, sticky="ew", padx=6, pady=(6,0))
        ttk.Label(day_inner, text="White Balance (K)").grid(row=2, column=0, sticky="w", padx=6, pady=(6,0))
        ttk.Scale(day_inner, from_=2800, to=8000, orient="horizontal", variable=self.day_wb,
                  command=lambda _=None: self._apply_day_setting("WB", self.day_wb.get())).grid(row=2, column=1, sticky="ew", padx=6, pady=(6,8))
        
        # Add a new digital zoom slider for the day camera
        ttk.Label(day_inner, text="Digital Zoom (1-4x)").grid(row=3, column=0, sticky="w", padx=6, pady=(6,0))
        ttk.Scale(day_inner, from_=1.0, to=4.0, orient="horizontal", variable=self.day_zoom_level,
                  command=lambda _=None: self._set_status("Day Zoom: " + str(self.day_zoom_level.get()))).grid(row=3, column=1, sticky="ew", padx=6, pady=(6,8))
        
        day_inner.grid_columnconfigure(1, weight=1)

        # ====== Colour Stream toggle button ======
        self.colour_btn_text = tk.StringVar(value="Start Colour Stream")
        self.colour_btn = ttk.Button(day_inner, textvariable=self.colour_btn_text, command=self.toggle_day_colour_stream)
        self.colour_btn.grid(row=4, column=0, columnspan=2, sticky="ew", padx=6, pady=(4,6))

        # Add Crosshair button under Colour Stream
        self.crosshair_btn_text = tk.StringVar(value="Crosshair: OFF")
        ttk.Button(day_inner, textvariable=self.crosshair_btn_text, command=self.toggle_crosshair).grid(row=5, column=0, columnspan=2, sticky="ew", padx=6, pady=(4,10))

        # Thermal settings (scrollable)
        thermal_settings_frame = ttk.Labelframe(self.right, text="THERMAL")
        thermal_settings_frame.grid(row=1, column=0, sticky="nsew", padx=4, pady=4)
        th_canvas, th_inner = self._make_scrollable_frame(thermal_settings_frame)
        th_inner.grid_columnconfigure(1, weight=1)

        # Palette
        pal_box = ttk.Labelframe(th_inner, text="Pseudo-color")
        pal_box.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(6,4), padx=6)
        self.thermal_palette_combo = ttk.Combobox(pal_box, width=12,
                                                  values=["white","black","rainbow","green","metel"], state="readonly")
        self.thermal_palette_combo.set("white")
        self.thermal_palette_combo.grid(row=0, column=0, padx=4, pady=6)
        ttk.Button(pal_box, text="Apply", command=self.thermal_apply_palette).grid(row=0, column=1, padx=4)
        ttk.Button(pal_box, text="Save",  command=lambda: self.thermal_send_group("color","save")).grid(row=0, column=2, padx=4)

        # Hotspot & Zoom
        hz = ttk.Labelframe(th_inner, text="Hotspot & Zoom")
        hz.grid(row=1, column=0, columnspan=2, sticky="ew", pady=6, padx=6)
        ttk.Button(hz, text="Hotspot ON", command=lambda: self.thermal_send_group("hotspot","on")).grid(row=0, column=0, padx=4, pady=4)
        ttk.Button(hz, text="Hotspot OFF", command=lambda: self.thermal_send_group("hotspot","off")).grid(row=0, column=1, padx=4, pady=4)
        ttk.Label(hz, text="Zoom (1-16x)").grid(row=1, column=0, sticky="w", padx=4)
        self.thermal_zoom_var = tk.IntVar(value=1)
        ttk.Scale(hz, from_=1, to=16, orient="horizontal", variable=self.thermal_zoom_var,
                  command=self.thermal_apply_zoom).grid(row=1, column=1, padx=6, sticky="ew")
        hz.grid_columnconfigure(1, weight=1)

        # Image display (brightness/contrast)
        disp = ttk.Labelframe(th_inner, text="Image Display")
        disp.grid(row=2, column=0, columnspan=2, sticky="ew", pady=6, padx=6)
        ttk.Label(disp, text="Brightness").grid(row=0, column=0, sticky="w")
        self.thermal_bright = tk.IntVar(value=128)
        ttk.Scale(disp, from_=10, to=250, orient="horizontal", variable=self.thermal_bright,
                  command=lambda _=None: self.thermal_apply_brightness()).grid(row=0, column=1, padx=6, sticky="ew")
        ttk.Label(disp, text="Contrast").grid(row=1, column=0, sticky="w", pady=(6,0))
        self.thermal_contrast = tk.IntVar(value=128)
        ttk.Scale(disp, from_=10, to=250, orient="horizontal", variable=self.thermal_contrast,
                  command=lambda _=None: self.thermal_apply_contrast()).grid(row=1, column=1, padx=6, sticky="ew", pady=(6,0))
        # Denoise
        ttk.Label(disp, text="Denoise (0–15)").grid(row=2, column=0, sticky="w", pady=(6,0))
        self.thermal_denoise = tk.IntVar(value=10)
        ttk.Scale(disp, from_=0, to=15, orient="horizontal", variable=self.thermal_denoise,
                  command=lambda _=None: self.thermal_apply_denoise()).grid(row=2, column=1, padx=6, sticky="ew", pady=(6,0))

        # Vertical Stripe
        ttk.Label(disp, text="Vertical Stripe (0=on,1=off)").grid(row=3, column=0, sticky="w", pady=(6,0))
        self.thermal_vstripe = tk.IntVar(value=0)
        ttk.Scale(disp, from_=0, to=1, orient="horizontal", variable=self.thermal_vstripe,
                  command=lambda _=None: self.thermal_apply_vstripe()).grid(row=3, column=1, padx=6, sticky="ew", pady=(6,0))

        # Info / custom
        misc = ttk.Labelframe(th_inner, text="Info / Custom")
        misc.grid(row=4, column=0, columnspan=2, sticky="ew", pady=6, padx=6)
        ttk.Button(misc, text="Read Camera Info", command=self.thermal_read_info).grid(row=0, column=0, padx=4, pady=4, sticky="w")
        self.thermal_custom_entry = ttk.Entry(misc, width=34)
        self.thermal_custom_entry.insert(0, "68 01 00 01 00 00 6A 00")
        self.thermal_custom_entry.grid(row=1, column=0, padx=4, pady=4, sticky="w")
        ttk.Button(misc, text="Send Hex", command=self.thermal_send_custom).grid(row=1, column=1, padx=4, pady=4)
        
        # New Fullscreen Buttons
        fullscreen_ctrl = ttk.Frame(th_inner)
        fullscreen_ctrl.grid(row=5, column=0, columnspan=2, sticky="ew", pady=6, padx=6)
        
        # Configure the grid for the fullscreen_ctrl frame
        fullscreen_ctrl.grid_columnconfigure(0, weight=1)
        fullscreen_ctrl.grid_columnconfigure(1, weight=1)

        # TButton style 
        ttk.Button(fullscreen_ctrl, text="Day", style="Tiny.TButton", command=lambda: self._show_fullscreen("day")).grid(row=0, column=0, sticky="ew")
        ttk.Button(fullscreen_ctrl, text="Thermal", style="Tiny.TButton", command=lambda: self._show_fullscreen("thermal")).grid(row=0, column=1, sticky="ew")
        ttk.Button(fullscreen_ctrl, text="Day+Thermal", style="Tiny.TButton", command=lambda: self._show_fullscreen("day_thermal")).grid(row=1, column=0, columnspan=2, sticky="ew")
        ttk.Button(fullscreen_ctrl, text="Thermal+Day", style="Tiny.TButton", command=lambda: self._show_fullscreen("thermal_day")).grid(row=2, column=0, columnspan=2, sticky="ew")

        # -------- Bottom row: All control buttons (unified) --------
        self.controls = ttk.Frame(self.root, padding=6)
        self.controls.pack(fill="x")

        # Day controls (left-most in bottom bar)
        day_ctrl = ttk.Frame(self.controls)
        day_ctrl.pack(side="left", padx=6)
        ttk.Label(day_ctrl, text="DAY").pack(side="left", padx=(0,6))

        # === MODIFIED to use new methods ===
        ttk.Button(day_ctrl, text="Start", command=self.day_start_stream).pack(side="left", padx=4)
        ttk.Button(day_ctrl, text="Stop", command=self.day_stop_stream).pack(side="left", padx=4)
        # === END MODIFIED ===

        # Range / LRF controls
        lrf_ctrl = ttk.Frame(self.controls)
        lrf_ctrl.pack(side="left", padx=8)
        ttk.Label(lrf_ctrl, text="Range").pack(side="left", padx=(0,6))
        ttk.Label(lrf_ctrl, text="Port").pack(side="left", padx=(6,2))
        self.lrf_port_combo = ttk.Combobox(lrf_ctrl, width=12, values=self._list_ports())
        self.lrf_port_combo.set("/dev/serial0" if "/dev/serial0" in self.lrf_port_combo["values"] else (self.lrf_port_combo["values"][0] if self.lrf_port_combo["values"] else ""))
        self.lrf_port_combo.pack(side="left")
        ttk.Label(lrf_ctrl, text="Baud").pack(side="left", padx=(6,2))
        self.lrf_baud_combo = ttk.Combobox(lrf_ctrl, width=8, values=["9600","14400","19200","38400","57600","115200","128000","230400"])
        self.lrf_baud_combo.set("115200")
        self.lrf_baud_combo.pack(side="left", padx=(0,6))
        ttk.Button(lrf_ctrl, text="Start Range", command=self.lrf_start).pack(side="left", padx=4)
        ttk.Button(lrf_ctrl, text="Stop Range", command=self.lrf_stop).pack(side="left", padx=4)

        # Thermal UART controls (in bottom bar)
        th_ctrl = ttk.Frame(self.controls)
        th_ctrl.pack(side="left", padx=12)
        ttk.Label(th_ctrl, text="THERMAL").pack(side="left", padx=(0,6))
        ttk.Button(th_ctrl, text="Start", command=self.thermal_start_stream).pack(side="left", padx=4)
        ttk.Button(th_ctrl, text="Stop", command=self.thermal_stop_stream).pack(side="left", padx=4)
        ttk.Label(th_ctrl, text="UART").pack(side="left", padx=(8,2))
        self.thermal_port_combo = ttk.Combobox(th_ctrl, width=12, values=self._list_ports())
        self.thermal_port_combo.set("/dev/ttyUSB0" if "/dev/ttyUSB0" in self.thermal_port_combo["values"] else (self.thermal_port_combo["values"][0] if self.thermal_port_combo["values"] else ""))
        self.thermal_port_combo.pack(side="left")
        ttk.Label(th_ctrl, text="Baud").pack(side="left", padx=(6,2))
        self.thermal_baud_combo = ttk.Combobox(th_ctrl, width=8, values=["115200","57600","38400","19200","9600"])
        self.thermal_baud_combo.set("115200")
        self.thermal_baud_combo.pack(side="left", padx=(0,6))
        ttk.Button(th_ctrl, text="UART Connect", command=self.thermal_connect_uart).pack(side="left", padx=4)
        ttk.Button(th_ctrl, text="UART Disconnect", command=self.thermal_disconnect_uart).pack(side="left", padx=4)

        # Right-side: Refresh Ports button
        ttk.Button(self.controls, text="Refresh Ports", command=self._refresh_ports).pack(side="right", padx=8)

        # -------- Status bar --------
        self.footer = ttk.Frame(self.root, padding=(8, 4))
        self.footer.pack(fill="x")
        self.status_var = tk.StringVar(value="Ready.")
        ttk.Label(self.footer, textvariable=self.status_var).pack(side="left")

    # ===================== New Fullscreen Logic (updated) =====================
    def _show_fullscreen(self, mode):
        self._set_status(f"Switching to fullscreen {mode} mode...")
        self.fullscreen_mode = mode

        # Stop all streams and rangefinder
        self.lrf_stop()
        self.day_stop_stream()
        self.thermal_stop_stream()
        
        # Clean up any existing overlays
        if hasattr(self, 'thermal_overlay_label') and self.thermal_overlay_label:
            self.thermal_overlay_label.destroy()
            self.thermal_overlay_label = None
        if hasattr(self, 'day_overlay_label') and self.day_overlay_label:
            self.day_overlay_label.destroy()
            self.day_overlay_label = None
        if hasattr(self, 'lrf_fullscreen_overlay_label') and self.lrf_fullscreen_overlay_label:
            self.lrf_fullscreen_overlay_label.destroy()
            self.lrf_fullscreen_overlay_label = None

        self.root.attributes('-fullscreen', True)
        self.header.pack_forget()
        self.controls.pack_forget()
        self.footer.pack_forget()
        self.body.pack_forget()
        self.body.pack(fill="both", expand=True)
        self.s1_group.grid_forget()
        self.s2_group.grid_forget()
        self.right.grid_forget()

        if mode == "day":
            self.s1_group.grid(row=0, column=0, sticky="nsew", padx=0, pady=0)
            self.s1_group.config(text="DAY CAMERA (FULLSCREEN)")
            self.body.grid_columnconfigure(0, weight=1)
            self.body.grid_columnconfigure(1, weight=0)
            self.body.grid_columnconfigure(2, weight=0)
            self.day_start_stream()
        elif mode == "thermal":
            self.s2_group.grid(row=0, column=0, sticky="nsew", padx=0, pady=0)
            self.s2_group.config(text="THERMAL CAMERA (FULLSCREEN)")
            self.body.grid_columnconfigure(0, weight=1)
            self.body.grid_columnconfigure(1, weight=0)
            self.body.grid_columnconfigure(2, weight=0)
            self.thermal_start_stream()
        elif mode == "day_thermal":
            self.s1_group.grid(row=0, column=0, sticky="nsew", padx=0, pady=0)
            self.s1_group.config(text="DAY CAMERA (FULLSCREEN)")
            self.body.grid_columnconfigure(0, weight=1)
            self.body.grid_columnconfigure(1, weight=0)
            self.body.grid_columnconfigure(2, weight=0)
            self.day_start_stream()
            self.thermal_start_overlay_stream()
        elif mode == "thermal_day":
            self.s2_group.grid(row=0, column=0, sticky="nsew", padx=0, pady=0)
            self.s2_group.config(text="THERMAL CAMERA (FULLSCREEN)")
            self.body.grid_columnconfigure(0, weight=1)
            self.body.grid_columnconfigure(1, weight=0)
            self.body.grid_columnconfigure(2, weight=0)
            self.thermal_start_stream()
            self.day_start_overlay_stream()
            self.lrf_start()

        self.fullscreen_exit_frame = tk.Frame(self.root, bg="black")
        self.fullscreen_exit_frame.pack(side="bottom", fill="x", anchor="se")
        self.exit_fs_button.pack(side="right", padx=10, pady=4)
        self.root.update_idletasks()

    def _exit_fullscreen(self):
        self._set_status("Exiting fullscreen mode...")
        self.fullscreen_mode = False
        self.root.attributes('-fullscreen', False)
        self.day_stop_stream()
        self.thermal_stop_stream()
        self.lrf_stop()

        if hasattr(self, 'thermal_overlay_label') and self.thermal_overlay_label:
            self.thermal_overlay_label.destroy()
            self.thermal_overlay_label = None
        if hasattr(self, 'day_overlay_label') and self.day_overlay_label:
            self.day_overlay_label.destroy()
            self.day_overlay_label = None
        if hasattr(self, 'lrf_fullscreen_overlay_label') and self.lrf_fullscreen_overlay_label:
            self.lrf_fullscreen_overlay_label.destroy()
            self.lrf_fullscreen_overlay_label = None

        self.fullscreen_exit_frame.pack_forget()
        self.exit_fs_button.pack_forget()
        self.fullscreen_exit_frame.destroy()
        
        self.body.pack_forget()
        self.header.pack(fill="x")
        self.body.pack(fill="both", expand=True)
        self.s1_group.grid_forget()
        self.s2_group.grid_forget()
        self.right.grid_forget()
        self.body.grid_columnconfigure(0, weight=1)
        self.body.grid_columnconfigure(1, weight=1)
        self.body.grid_columnconfigure(2, weight=0)
        self.s1_group.grid(row=0, column=0, sticky="nsew", padx=(4,4), pady=4)
        self.s1_group.config(text="STREAM-01 (Day Camera)")
        self.s2_group.grid(row=0, column=1, sticky="nsew", padx=(4,4), pady=4)
        self.s2_group.config(text="STREAM-02 (Thermal Camera)")
        self.right.grid(row=0, column=2, sticky="nsew", padx=(6,4), pady=4)
        self.controls.pack(fill="x")
        self.footer.pack(fill="x")
        self.root.update_idletasks()
        
    def thermal_start_overlay_stream(self):
        # Create the thermal overlay label
        self.thermal_overlay_label = tk.Label(self.day_video_frame, bg="black", borderwidth=2, relief="solid")
        self.thermal_overlay_label.place(relx=1.0, rely=0.0, anchor="ne", x=-20, y=20)
        self.thermal_start_stream()

    def day_start_overlay_stream(self):
        self.day_overlay_label = tk.Label(self.thermal_video_frame, bg="black", borderwidth=2, relief="solid")
        self.day_overlay_label.place(relx=1.0, rely=0.0, anchor="ne", x=-20, y=20)
        self.lrf_fullscreen_overlay_label = tk.Label(self.thermal_video_frame,
                                    text="Range: --.- m",
                                    font=("Segoe UI", 12, "bold"),
                                    fg="cyan", bg="black")
        self.lrf_fullscreen_overlay_label.place(relx=1.0, rely=0.25, anchor="ne", x=-20, y=20)
        self.day_start_stream()


    # ===================== Helpers (unchanged) =====================
    def _set_status(self, msg):
        self.status_var.set(msg)
        self.root.update_idletasks()

    def _list_ports(self):
        return [p.device for p in serial.tools.list_ports.comports()]

    def _refresh_ports(self):
        values = self._list_ports()
        for combo in (getattr(self, "lrf_port_combo", None), getattr(self, "thermal_port_combo", None)):
            if combo:
                current = combo.get()
                combo["values"] = values
                if current not in values and values:
                    combo.set(values[0])
        self._set_status("Ports refreshed.")

    # ===================== LRF (Range) =====================
    def lrf_start(self):
        if not self.lrf_ser or not getattr(self.lrf_ser, "is_open", False):
            try:
                port = self.lrf_port_combo.get().strip()
                baud = int(self.lrf_baud_combo.get())
                self.lrf_ser = serial.Serial(port, baudrate=baud, timeout=1)
                self._set_status(f"Range connected: {port} @ {baud}")
            except Exception as e:
                messagebox.showerror("Range Error", f"Failed to open Range: {e}")
                return
        try:
            self.lrf_ser.write(CONTINUOUS_MEASUREMENT)
            self.lrf_running = True
            if not self.lrf_read_thread or not self.lrf_read_thread.is_alive():
                self.lrf_read_thread = threading.Thread(target=self._lrf_read_loop, daemon=True)
                self.lrf_read_thread.start()
            self._set_status("Range continuous started.")
        except Exception as e:
            messagebox.showerror("Range Error", f"Failed to start: {e}")

    def lrf_stop(self):
        self.lrf_running = False
        try:
            if self.lrf_ser and self.lrf_ser.is_open:
                self.lrf_ser.write(STOP_MEASUREMENT)
                self.lrf_ser.close()
            self._set_status("Range stopped & disconnected.")
        except Exception as e:
            self._set_status(f"Range stop error: {e}")

    def _lrf_read_loop(self):
        while self.lrf_running and self.lrf_ser and self.lrf_ser.is_open:
            try:
                if self.lrf_ser.in_waiting >= 8:
                    data = self.lrf_ser.read(8)
                    if len(data) == 8:
                        if data[4] == 0x00:
                            pass
                        else:
                            distance = (data[5] * 256 + data[6]) / 10.0
                            self.lrf_last_distance = distance
                            self.root.after(0, self._update_lrf_overlay)
                else:
                    time.sleep(0.05)
            except Exception:
                time.sleep(0.1)

    def _update_lrf_overlay(self):
        if self.lrf_last_distance is None:
            txt = "Range: --.- m"
        else:
            txt = f"Range: {self.lrf_last_distance:.1f} m"
        try:
            if hasattr(self, 'lrf_fullscreen_overlay_label') and self.fullscreen_mode == 'thermal_day':
                self.lrf_fullscreen_overlay_label.config(text=txt)
            else:
                self.lrf_overlay.config(text=txt)
        except Exception:
            pass

    # ===================== THERMAL (video + UART controls) =====================
    def thermal_connect_uart(self):
        if self.thermal_connected and self.thermal_ser and self.thermal_ser.is_open:
            self._set_status("Thermal UART already connected.")
            return
        try:
            port = self.thermal_port_combo.get().strip()
            baud = int(self.thermal_baud_combo.get())
            self.thermal_ser = serial.Serial(port, baudrate=baud, timeout=1)
            self.thermal_connected = True
            self._set_status(f"Thermal UART connected: {port} @ {baud}")
        except Exception as e:
            messagebox.showerror("Thermal UART Error", str(e))

    def thermal_disconnect_uart(self):
        try:
            if self.thermal_ser and self.thermal_ser.is_open:
                self.thermal_ser.close()
            self.thermal_connected = False
            self._set_status("Thermal UART disconnected.")
        except Exception as e:
            self._set_status(f"UART disconnect error: {e}")

    def _thermal_serial_send(self, hex_str, expect_len, label=None):
        if not (self.thermal_connected and self.thermal_ser and self.thermal_ser.is_open):
            self._set_status("Connect thermal UART first.")
            return
        def worker():
            resp = send_to_ir_camera(self.thermal_ser, hex_str, response_len=expect_len)
            if resp != 0:
                msg = ' '.join(f'{x:02X}' for x in resp[::-1])
            else:
                msg = "(no/invalid response)"
            self.root.after(0, lambda: self._set_status((label + " " if label else "") + msg))
        threading.Thread(target=worker, daemon=True).start()

    def thermal_apply_palette(self):
        name = self.thermal_palette_combo.get()
        key = {"white":"white","black":"black","rainbow":"rainbow","green":"green","metel":"metel"}[name]
        data = THERMAL_FUNCTION_GROUPS["color"]["functions"][key]["data"]
        self._thermal_serial_send(data, THERMAL_FUNCTION_GROUPS["color"]["response_len"], f"Palette {name}:")
        self.thermal_palette = name

    def thermal_apply_brightness(self):
        v = self.thermal_bright.get()
        data = THERMAL_FUNCTION_GROUPS["brightness"]["build_data"](v)
        self._thermal_serial_send(data, THERMAL_FUNCTION_GROUPS["brightness"]["response_len"], "Brightness:")

    def thermal_apply_contrast(self):
        v = self.thermal_contrast.get()
        data = THERMAL_FUNCTION_GROUPS["contrast"]["build_data"](v)
        self._thermal_serial_send(data, THERMAL_FUNCTION_GROUPS["contrast"]["response_len"], f"Contrast {v}:")

    def thermal_apply_denoise(self):
        v = self.thermal_denoise.get()
        data = THERMAL_FUNCTION_GROUPS["denoise"]["build_data"](v)
        self._thermal_serial_send(data, THERMAL_FUNCTION_GROUPS["denoise"]["response_len"], f"Denoise {v}:")

    def thermal_apply_vstripe(self):
        v = self.thermal_vstripe.get()
        data = THERMAL_FUNCTION_GROUPS["vstripe"]["build_data"](v)
        self._thermal_serial_send(data, THERMAL_FUNCTION_GROUPS["vstripe"]["response_len"], f"VStripe {v}:")

    def thermal_apply_zoom(self, event=None):
        v = int(self.thermal_zoom_var.get())
        data = THERMAL_FUNCTION_GROUPS["zoom"]["build_data"](v)
        self._thermal_serial_send(data, THERMAL_FUNCTION_GROUPS["zoom"]["response_len"], f"Zoom {v}x:")

    def thermal_send_group(self, group, function):
        data = THERMAL_FUNCTION_GROUPS[group]["functions"][function]["data"]
        self._thermal_serial_send(data, THERMAL_FUNCTION_GROUPS[group]["response_len"], f"{group} {function}:")

    def thermal_read_info(self):
        self._set_status("Reading thermal camera info…")
        for item in THERMAL_INFO_REQUESTS:
            self._thermal_serial_send(item["data"], item["response_len"], item["label"])

    def thermal_send_custom(self):
        hex_str = self.thermal_custom_entry.get().strip()
        try:
            _ = bytes.fromhex(hex_str)  # validate
        except Exception as e:
            self._set_status(f"Invalid hex: {e}")
            return
        self._thermal_serial_send(hex_str, 16, "Custom:")

    def thermal_start_stream(self):
        if self.thermal_streaming and self.thermal_streamer and self.thermal_streamer.thread.is_alive():
            self._set_status("Thermal stream already running.")
            return
        
        self.thermal_streaming = True
        self.thermal_streamer = CameraStreamer('cv2', 0, self.thermal_queue, fps=30)
        self.thermal_streamer.start()
        # Check if day camera is also streaming
        if self.day_streaming:
            self.day_streaming_mode = "2" # Set label for Day camera
        self._set_status("Thermal stream started.")

    def thermal_stop_stream(self):
        if self.thermal_streamer:
            self.thermal_streamer.stop()
        self.thermal_streaming = False
        # If thermal is stopped, but day camera is still running, set label to 1
        if self.day_streaming:
            self.day_streaming_mode = "1"
        self._set_status("Thermal stream stopped.")

    # ===================== DAY CAMERA (updated) =====================
    def day_start_stream(self):
        if self.day_pipeline:
            self._set_status("Day stream already running.")
            return

        # Calculate Offsets for perfect centering
        OFFSET_X = (self.SENSOR_MAX_WIDTH - self.ROI_WIDTH) // 2
        OFFSET_Y = (self.SENSOR_MAX_HEIGHT - self.ROI_HEIGHT) // 2
        
        # --- GStreamer Pipeline Construction ---
        # We use 'aravissrc' (assuming it's installed/working on RPi 4 for GigE)
        # and set the GenICam feature properties (OffsetX, OffsetY, Width, Height, framerate).
        
        # Note: GStreamer framerate is set as 'numerator/denominator' (e.g., 30/1).
        pipeline_str = (
            f"aravissrc name=day_src "
            f"OffsetX={OFFSET_X} OffsetY={OFFSET_Y} "
            f"Width={self.ROI_WIDTH} Height={self.ROI_HEIGHT} "
            f"framerate={self.TARGET_FPS}/1 ! "  
            "videoconvert ! video/x-raw,format=RGB ! appsink name=day_sink"
        )

        self._set_status(
            f"Starting Day stream at {self.ROI_WIDTH}x{self.ROI_HEIGHT} @ {self.TARGET_FPS} fps "
            f"(Centering Offset: {OFFSET_X},{OFFSET_Y})..."
        )
        
        try:
            self.day_pipeline = Gst.parse_launch(pipeline_str)
            self.day_app_sink = self.day_pipeline.get_by_name("day_sink")
            
            # Configure app_sink
            self.day_app_sink.set_property("emit-signals", True)
            self.day_app_sink.set_property("max-buffers", 1)
            self.day_app_sink.set_property("drop", True)
            self.day_app_sink.connect("new-sample", self._day_new_sample)
            
            self.day_pipeline.set_state(Gst.State.PLAYING)
            
            self.day_paused = False
            self.day_fps_start_time = time.time()
            self.day_frame_count = 0
            self.day_fps_log.clear()

            # Start the thread to update the UI with frames
            self.day_update_thread = threading.Thread(target=self._day_update_frame, daemon=True)
            self.day_update_thread.start()
            
        except Exception as e:
            self._set_status(f"Error starting Day stream: {e}")
            self.day_pipeline = None

    def day_stop_stream(self):
        if self.day_streamer:
            self.day_streamer.stop()
        self.day_streaming = False
        self.day_colour_running = False
        self.day_queue.queue.clear() # Clear any remaining frames
        self._set_status("Day camera stream stopped.")

    def toggle_day_colour_stream(self):
        if self.day_colour_running:
            self.day_stop_stream()
            self.colour_btn_text.set("Start Colour Stream")
            return

        self.day_stop_stream() # Ensure previous stream is stopped
        
        # FIX: Robust GStreamer pipeline to ensure consistent frame size:
        # Puts the video/x-raw filter AFTER videoconvert to force the exact output format.
        pipeline_str = ("aravissrc ! videoconvert ! video/x-raw,format=RGB,width=640,height=480,framerate=30/1 ! appsink name=sink")
        self.day_streamer = CameraStreamer('gstreamer', pipeline_str, self.day_queue, fps=30)
        self.day_streamer.start()
        self.day_colour_running = True
        self.day_streaming = True
        self.colour_btn_text.set("Stop Colour Stream")
        # Check if thermal camera is also streaming
        if self.thermal_streaming:
            self.day_streaming_mode = "2"
        else:
            self.day_streaming_mode = "1"
        self._set_status("Day camera (Colour) streaming started.")
    
    # === MODIFIED ===
    def day_toggle_pause(self):
        if self.day_streamer:
            self.day_streamer.day_paused = not self.day_streamer.day_paused
            self.day_paused = self.day_streamer.day_paused # Sync state
            if self.day_paused:
                self._set_status("Day camera paused.")
            else:
                self._set_status("Day camera resumed.")
    
    def day_start_or_resume(self):
        if self.day_paused:
            self.day_toggle_pause()
        else:
            self.day_start_stream()
    # === END MODIFIED ===

    # Placeholder for day control application
    def _apply_day_setting(self, name, value):
        self._set_status(f"Day setting {name}: {value:.2f}")

    # ===================== CROSSHAIR TOGGLE (unchanged) =====================
    def toggle_crosshair(self):
        self.crosshair_enabled = not self.crosshair_enabled
        self.crosshair_btn_text.set("Crosshair: ON" if self.crosshair_enabled else "Crosshair: OFF")
        self._set_status("Crosshair " + ("enabled" if self.crosshair_enabled else "disabled"))

    # ===================== CLEANUP (updated) =====================
    def on_close(self):
        try:
            self.lrf_stop()
            self.thermal_stop_stream()
            self.day_stop_stream()

            if self.lrf_ser and self.lrf_ser.is_open:
                self.lrf_ser.close()
            if self.thermal_ser and self.thermal_ser.is_open:
                self.thermal_ser.close()

            self.root.destroy()
        except Exception as e:
            print(f"Cleanup error: {e}")

# ===================== MAIN =====================
if __name__ == "__main__":
    root = tk.Tk()
    try:
        root.call("tk", "scaling", 1.2)
    except Exception:
        pass
    app = TriplePayloadGUI(root)
    root.mainloop()
