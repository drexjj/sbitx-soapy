#!/usr/bin/env python3
import configparser
import subprocess
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

CARD = "audioinjectorpi"

def amixer_get(control: str) -> int | None:
    # Reads first integer like: Front Left: Playback 70 [55%] [-3.00dB] [on]
    try:
        out = subprocess.check_output(["amixer", "-c", CARD, "sget", control], text=True, stderr=subprocess.STDOUT)
    except subprocess.CalledProcessError as e:
        return None
    for line in out.splitlines():
        if "Playback" in line or "Capture" in line:
            # find first token that is an int right after "Playback"/"Capture"
            parts = line.replace(":", " ").replace("[", " ").split()
            for i, p in enumerate(parts):
                if p in ("Playback", "Capture") and i + 1 < len(parts):
                    try:
                        return int(parts[i + 1])
                    except ValueError:
                        continue
    return None

def amixer_set(control: str, value: int) -> None:
    subprocess.check_call(["amixer", "-q", "-c", CARD, "sset", control, str(value)])

class BandEditor(ttk.Frame):
    def __init__(self, parent, name: str, data: dict):
        super().__init__(parent)
        self.name = name

        self.rx_capture = tk.IntVar(value=int(data.get("rx_capture", "21")))
        self.tx_capture = tk.IntVar(value=int(data.get("tx_capture", "26")))
        self.tx_master  = tk.IntVar(value=int(data.get("tx_master",  "80")))

        self._row("RX Capture (0–31)", self.rx_capture, 0, 31, 0)
        self._row("TX Capture (0–31)", self.tx_capture, 0, 31, 1)
        self._row("TX Master  (0–127)", self.tx_master, 0, 127, 2)

        btns = ttk.Frame(self)
        btns.grid(row=3, column=0, columnspan=3, sticky="ew", pady=(8,0))
        ttk.Button(btns, text="Apply RX", command=self.apply_rx).pack(side="left", padx=4)
        ttk.Button(btns, text="Apply TX", command=self.apply_tx).pack(side="left", padx=4)
        ttk.Button(btns, text="Read ALSA → fields", command=self.read_alsa).pack(side="left", padx=4)

        self.columnconfigure(1, weight=1)

    def _row(self, label, var, mn, mx, r):
        ttk.Label(self, text=label).grid(row=r, column=0, sticky="w", padx=6, pady=4)
        s = ttk.Scale(self, from_=mn, to=mx, orient="horizontal",
                      command=lambda _=None, v=var: v.set(int(float(s.get()))))
        s.set(var.get())
        s.grid(row=r, column=1, sticky="ew", padx=6)
        sp = ttk.Spinbox(self, from_=mn, to=mx, textvariable=var, width=6)
        sp.grid(row=r, column=2, sticky="e", padx=6)

    def apply_rx(self):
        try:
            amixer_set("Capture", int(self.rx_capture.get()))
        except Exception as e:
            messagebox.showerror("Apply RX failed", str(e))

    def apply_tx(self):
        try:
            amixer_set("Capture", int(self.tx_capture.get()))
            amixer_set("Master", int(self.tx_master.get()))
        except Exception as e:
            messagebox.showerror("Apply TX failed", str(e))

    def read_alsa(self):
        cap = amixer_get("Capture")
        mas = amixer_get("Master")
        if cap is not None:
            # decide whether to update RX or TX capture field; we update both for convenience
            self.rx_capture.set(cap)
            self.tx_capture.set(cap)
        if mas is not None:
            self.tx_master.set(mas)

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("sbitx ini band trim editor")
        self.geometry("700x320")

        self.cfg = configparser.ConfigParser()
        self.path = "/etc/sbitx/alsamix_bands.ini"

        top = ttk.Frame(self)
        top.pack(fill="x", padx=8, pady=6)
        ttk.Button(top, text="Open INI…", command=self.open_ini).pack(side="left")
        ttk.Button(top, text="Save", command=self.save_ini).pack(side="left", padx=6)
        self.path_lbl = ttk.Label(top, text=self.path)
        self.path_lbl.pack(side="left", padx=10)

        self.nb = ttk.Notebook(self)
        self.nb.pack(fill="both", expand=True, padx=8, pady=8)

        self.editors = {}
        self.load_ini(self.path)

    def load_ini(self, path):
        self.cfg.read(path)
        self.path = path
        self.path_lbl.configure(text=path)

        for tab in list(self.editors.values()):
            tab.destroy()
        self.editors.clear()

        # create tabs for all band sections (skip global if present)
        for sec in self.cfg.sections():
            if sec.lower() == "global":
                continue
            data = dict(self.cfg[sec])
            ed = BandEditor(self.nb, sec, data)
            self.nb.add(ed, text=sec)
            self.editors[sec] = ed

    def open_ini(self):
        p = filedialog.askopenfilename(filetypes=[("INI files","*.ini"),("All","*.*")])
        if p:
            self.load_ini(p)

    def save_ini(self):
        # write back editor values
        for sec, ed in self.editors.items():
            if sec not in self.cfg:
                self.cfg[sec] = {}
            self.cfg[sec]["rx_capture"] = str(ed.rx_capture.get())
            self.cfg[sec]["tx_capture"] = str(ed.tx_capture.get())
            self.cfg[sec]["tx_master"]  = str(ed.tx_master.get())

        try:
            with open(self.path, "w") as f:
                self.cfg.write(f)
            messagebox.showinfo("Saved", f"Saved to {self.path}")
        except Exception as e:
            messagebox.showerror("Save failed", str(e))

if __name__ == "__main__":
    App().mainloop()
