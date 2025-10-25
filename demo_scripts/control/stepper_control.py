import time
import tkinter as tk
from tkinter import ttk, messagebox
import serial
from serial.tools import list_ports

BAUD = 115200
PORT_HINTS = ("arduino", "usbmodem", "usbserial", "wch", "ch340")

def find_default_port():
    for p in list_ports.comports():
        desc = (p.description or "").lower()
        if any(h in desc for h in PORT_HINTS):
            return p.device
    return None

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("28BYJ-48 Controller")
        self.ser = None

        # Port picker
        self.ports_var = tk.StringVar(value="")
        self.status_var = tk.StringVar(value="Disconnected")

        row = ttk.Frame(self, padding=10)
        row.pack(fill="x")

        ttk.Label(row, text="Port:").pack(side="left")
        self.port_combo = ttk.Combobox(row, textvariable=self.ports_var, width=24, state="readonly")
        self.refresh_ports()
        self.port_combo.pack(side="left", padx=6)

        ttk.Button(row, text="Refresh", command=self.refresh_ports).pack(side="left")
        ttk.Button(row, text="Connect", command=self.connect).pack(side="left", padx=6)
        ttk.Button(row, text="Disconnect", command=self.disconnect).pack(side="left")

        # Control buttons
        btns = ttk.Frame(self, padding=(10, 6))
        btns.pack(fill="x")
        ttk.Button(btns, text="−10°", command=lambda: self.send("-")).pack(side="left", padx=4)
        ttk.Button(btns, text="+10°", command=lambda: self.send("+")).pack(side="left", padx=4)

        # Optional: move arbitrary angle / rpm
        advanced = ttk.Frame(self, padding=(10, 6))
        advanced.pack(fill="x")
        self.angle_entry = ttk.Entry(advanced, width=8)
        self.angle_entry.insert(0, "45")
        ttk.Label(advanced, text="Angle:").pack(side="left")
        self.angle_entry.pack(side="left")
        ttk.Button(advanced, text="MOVE", command=self.move_angle).pack(side="left", padx=6)

        self.rpm_entry = ttk.Entry(advanced, width=6)
        self.rpm_entry.insert(0, "10")
        ttk.Label(advanced, text="RPM:").pack(side="left", padx=(12,0))
        self.rpm_entry.pack(side="left")
        ttk.Button(advanced, text="SET", command=self.set_rpm).pack(side="left", padx=6)

        ttk.Label(self, textvariable=self.status_var, foreground="gray").pack(pady=(4,10))

    def refresh_ports(self):
        ports = [p.device for p in list_ports.comports()]
        self.port_combo["values"] = ports
        guess = find_default_port()
        if guess and guess in ports:
            self.ports_var.set(guess)
        elif ports and not self.ports_var.get():
            self.ports_var.set(ports[0])

    def connect(self):
        port = self.ports_var.get()
        if not port:
            messagebox.showerror("Error", "No serial port selected.")
            return
        try:
            self.ser = serial.Serial(port, BAUD, timeout=2)
            time.sleep(2)  # allow Uno to auto-reset after opening the port
            self.ser.reset_input_buffer()
            self.status_var.set(f"Connected: {port}")
        except Exception as e:
            self.ser = None
            messagebox.showerror("Connection failed", str(e))

    def disconnect(self):
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        self.status_var.set("Disconnected")

    def send(self, text):
        if not self.ser:
            messagebox.showwarning("Not connected", "Connect to a serial port first.")
            return
        try:
            self.ser.write((text + "\n").encode("ascii"))
            self.ser.flush()
            reply = self.ser.readline().decode("ascii", errors="ignore").strip()
            self.status_var.set(f"Sent {text!r} → {reply or '(no reply)'}")
        except Exception as e:
            self.status_var.set(f"Error: {e}")

    def move_angle(self):
        try:
            deg = float(self.angle_entry.get())
        except ValueError:
            messagebox.showerror("Invalid angle", "Enter a number (degrees).")
            return
        self.send(f"MOVE {deg}")

    def set_rpm(self):
        try:
            rpm = int(self.rpm_entry.get())
        except ValueError:
            messagebox.showerror("Invalid RPM", "Enter an integer RPM.")
            return
        self.send(f"RPM {rpm}")

if __name__ == "__main__":
    app = App()
    app.mainloop()
