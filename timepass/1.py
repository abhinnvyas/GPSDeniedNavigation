# === DAY CAMERA STREAM METHODS (MODIFIED FOR CENTERING) ===

    # Define the maximum sensor resolution for centering calculation.
    # *** IMPORTANT ***: CHANGE THESE VALUES if your camera is NOT 4112x3008!
    # Common Max Resolutions:
    # 0.4 MP: 728x544 | 5.0 MP: 2464x2056 | 12.3 MP: 4112x3008 (DEFAULT)
    SENSOR_MAX_WIDTH = 4112
    SENSOR_MAX_HEIGHT = 3008

    # Target stream settings
    ROI_WIDTH = 640
    ROI_HEIGHT = 480
    TARGET_FPS = 30

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

    # === END MODIFIED ===