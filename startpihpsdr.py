#!/usr/bin/env python3
"""
Simple Tkinter GUI launcher for sbitx_ctrl + pihpsdr
- Starts sbitx_ctrl in background
- Launches pihpsdr when user clicks Start
- Kills sbitx_ctrl cleanly when pihpsdr exits or when user clicks Stop/Quit
"""

import tkinter as tk
from tkinter import ttk, messagebox
import subprocess
import os
import signal
import time
import threading
import sys

# ────────────────────────────────────────────────
# CONFIGURATION
# ────────────────────────────────────────────────

# Icon
ICON_PATH = "/home/pi/pihpsdr/release/pihpsdr/piHPSDR_logo.png"

CTRL_DIR  = "/home/pi/sbitx-soapy/sbitx-core_mod"
CTRL_CMD  = ["./sbitx_ctrl"]          # add arguments if needed: ["./sbitx_ctrl", "--option"]

PIHPSDR_DIR = "/home/pi/pihpsdr"
PIHPSDR_CMD = ["./pihpsdr"]           # add arguments if needed

# ────────────────────────────────────────────────

class SbitxLauncher(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("sBitx pihpsdr Launcher")
        self.geometry("380x180")
        self.resizable(False, False)

        self.ctrl_process = None
        self.pihpsdr_process = None
        self.running = False

        self._build_gui()
        self.protocol("WM_DELETE_WINDOW", self.on_closing)

    def _build_gui(self):
        main = ttk.Frame(self, padding="20 15")
        main.pack(fill=tk.BOTH, expand=True)

        # Status
        self.status_var = tk.StringVar(value="Ready – waiting to start")
        ttk.Label(main, textvariable=self.status_var, font=("Helvetica", 11)).pack(pady=(0,15))

        # Buttons
        btn_frame = ttk.Frame(main)
        btn_frame.pack(fill=tk.X, pady=10)

        self.start_btn = ttk.Button(btn_frame, text="Start sbitxCTRL + pihpsdr", command=self.start_both)
        self.start_btn.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)

        self.stop_btn = ttk.Button(btn_frame, text="Stop / Cleanup", command=self.stop_both, state="disabled")
        self.stop_btn.pack(side=tk.RIGHT, padx=5, fill=tk.X, expand=True)

        ttk.Separator(main, orient="horizontal").pack(fill=tk.X, pady=12)

        ttk.Button(main, text="Quit", command=self.on_closing).pack(pady=5)

    def start_both(self):
        if self.running:
            return

        self.running = True
        self.start_btn.config(state="disabled")
        self.stop_btn.config(state="normal")
        self.status_var.set("Starting sbitx_ctrl...")

        try:
            # 1. Launch sbitx_ctrl
            os.chdir(CTRL_DIR)
            self.ctrl_process = subprocess.Popen(
                CTRL_CMD,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid          # so we can kill the whole process group
            )
            time.sleep(2.0)  # give it time to initialize

            if self.ctrl_process.poll() is not None:
                raise RuntimeError("sbitx_ctrl failed to start")

            self.status_var.set("sbitx_ctrl running • starting pihpsdr...")

            # 2. Launch pihpsdr
            os.chdir(PIHPSDR_DIR)
            self.pihpsdr_process = subprocess.Popen(
                PIHPSDR_CMD,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
            )

            self.status_var.set("Running • close pihpsdr window when finished")

            # Watch pihpsdr in background thread
            threading.Thread(target=self._watch_pihpsdr, daemon=True).start()

        except Exception as e:
            messagebox.showerror("Launch Error", str(e))
            self.cleanup()
            self.status_var.set("Failed to start")

    def _watch_pihpsdr(self):
        """Background watcher: when pihpsdr exits → clean up"""
        if not self.pihpsdr_process:
            return

        self.pihpsdr_process.wait()
        if self.running:
            self.after(0, lambda: self.status_var.set("pihpsdr closed → cleaning up..."))
            self.after(300, self.cleanup)

    def stop_both(self):
        self.cleanup()
        self.status_var.set("Stopped")

    def cleanup(self):
        self.running = False

        # Kill pihpsdr if still alive
        if self.pihpsdr_process and self.pihpsdr_process.poll() is None:
            try:
                os.killpg(os.getpgid(self.pihpsdr_process.pid), signal.SIGTERM)
                time.sleep(0.8)
                if self.pihpsdr_process.poll() is None:
                    os.killpg(os.getpgid(self.pihpsdr_process.pid), signal.SIGKILL)
            except:
                pass

        # Kill sbitx_ctrl
        if self.ctrl_process and self.ctrl_process.poll() is None:
            try:
                os.killpg(os.getpgid(self.ctrl_process.pid), signal.SIGTERM)
                time.sleep(0.6)
                if self.ctrl_process.poll() is None:
                    os.killpg(os.getpgid(self.ctrl_process.pid), signal.SIGKILL)
            except:
                pass

        self.ctrl_process = None
        self.pihpsdr_process = None

        self.start_btn.config(state="normal")
        self.stop_btn.config(state="disabled")

    def on_closing(self):
        if self.running:
            if messagebox.askyesno("Quit", "Programs are running.\nStop everything and quit?"):
                self.cleanup()
                self.destroy()
        else:
            self.destroy()


if __name__ == "__main__":
    app = SbitxLauncher()
    app.mainloop()
