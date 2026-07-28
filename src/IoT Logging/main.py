"""
Serial dashboard for the ATmega328P solar tracker.

Reads UART telemetry lines from the chip over a serial port, parses the
known message formats from the firmware, and serves a live-updating
dashboard at http://localhost:5000 showing the latest sensor/motor state
plus a scrolling, timestamped log.

Install dependencies:
    pip install pyserial flask

Usage:
    python serial_dashboard.py --port /dev/tty.usbserial-XXXX --baud 115200
    (Windows: --port COM5, Linux: --port /dev/ttyUSB0 or /dev/ttyACM0)

Tip: match baud rate to souce code's (115200)
"""

import argparse
import re
import threading
import time
from collections import deque
from datetime import datetime

import serial
from flask import Flask, jsonify, render_template_string

app = Flask(__name__)

# Rolling log of the last N lines, each as (timestamp_str, raw_line)
LOG_MAXLEN = 300
log_lines = deque(maxlen=LOG_MAXLEN)
log_lock = threading.Lock()

# Latest parsed values, keyed by field name
latest = {
    "battery_raw": None,
    "battery_filt": None,
    "base_raw": None,
    "base_filt": None,
    "panel_raw": None,
    "panel_filt": None,
    "pot_raw": None,
    "pot_filt": None,
    "base_motor": None,
    "panel_motor": None,
    "right_hall": None,
    "left_hall": None,
    "last_update": None,
    "connected": False,
}
latest_lock = threading.Lock()

# Regex patterns matching the firmware's sprintf/UART_Transmit_Ptr messages
PATTERNS = {
    "battery": re.compile(r"Battery ADC = (\d+), battery filtADC = (-?\d+)"),
    "base_adc": re.compile(r"Base LDR ADC = (\d+), base filtADC = (-?\d+)"),
    "panel_adc": re.compile(r"Panel LDR ADC = (\d+), panel filtADC = (-?\d+)"),
    "pot_adc": re.compile(r"Pot ADC = (\d+), pot filtADC = (\d+)"),
    "base_motor": re.compile(r"Base motor is (CW|CCW|OFF)"),
    "panel_motor": re.compile(r"Panel motor is (CW|CCW|OFF)"),
    "right_hall": re.compile(r"Right Hall effect is (HIGH|LOW)"),
    "left_hall": re.compile(r"Left Hall effect is (HIGH|LOW)"),
}


def parse_line(line: str):
    """Update `latest` in place based on a single line of firmware output."""
    with latest_lock:
        if m := PATTERNS["battery"].search(line):
            latest["battery_raw"] = int(m.group(1))
            latest["battery_filt"] = int(m.group(2))
        elif m := PATTERNS["base_adc"].search(line):
            latest["base_raw"] = int(m.group(1))
            latest["base_filt"] = int(m.group(2))
        elif m := PATTERNS["panel_adc"].search(line):
            latest["panel_raw"] = int(m.group(1))
            latest["panel_filt"] = int(m.group(2))
        elif m := PATTERNS["pot_adc"].search(line):
            latest["pot_raw"] = int(m.group(1))
            latest["pot_filt"] = int(m.group(2))
        elif m := PATTERNS["base_motor"].search(line):
            latest["base_motor"] = m.group(1)
        elif m := PATTERNS["panel_motor"].search(line):
            latest["panel_motor"] = m.group(1)
        elif m := PATTERNS["right_hall"].search(line):
            latest["right_hall"] = m.group(1)
        elif m := PATTERNS["left_hall"].search(line):
            latest["left_hall"] = m.group(1)

        latest["last_update"] = datetime.now().strftime("%H:%M:%S.%f")[:-3]


def serial_reader(port: str, baud: int):
    """Background thread: continuously read lines from the serial port."""
    while True:
        try:
            with serial.Serial(port, baud, timeout=1) as ser:
                with latest_lock:
                    latest["connected"] = True
                print(f"Connected to {port} @ {baud} baud")
                while True:
                    raw = ser.readline()
                    if not raw:
                        continue
                    try:
                        line = raw.decode("utf-8", errors="replace").strip()
                    except Exception:
                        continue
                    if not line:
                        continue

                    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    with log_lock:
                        log_lines.append((ts, line))

                    parse_line(line)
        except serial.SerialException as e:
            with latest_lock:
                latest["connected"] = False
            print(f"Serial error ({e}), retrying in 2s...")
            time.sleep(2)


@app.route("/")
def index():
    return render_template_string(PAGE_TEMPLATE)


@app.route("/api/data")
def api_data():
    with latest_lock:
        data = dict(latest)
    with log_lock:
        log = list(log_lines)[::-1]  # newest first
    return jsonify({"latest": data, "log": log})


PAGE_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>Solar Tracker Telemetry</title>
<style>
    :root {
        --bg: #0f1419;
        --card-bg: #1a2028;
        --border: #2a3441;
        --text: #e6edf3;
        --dim: #8b949e;
        --accent: #4fd1c5;
        --cw: #63d16a;
        --ccw: #f0a850;
        --off: #6b7480;
        --hall-high: #4fd1c5;
        --hall-low: #e0616b;
    }
    * { box-sizing: border-box; }
    body {
        margin: 0;
        font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
        background: var(--bg);
        color: var(--text);
        padding: 24px;
    }
    h1 { font-size: 1.4rem; margin: 0 0 4px 0; }
    .status {
        font-size: 0.85rem;
        color: var(--dim);
        margin-bottom: 20px;
    }
    .status .dot {
        display: inline-block;
        width: 8px; height: 8px;
        border-radius: 50%;
        margin-right: 6px;
        background: var(--off);
    }
    .status .dot.on { background: var(--cw); }
    .grid {
        display: grid;
        grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
        gap: 14px;
        margin-bottom: 24px;
    }
    .card {
        background: var(--card-bg);
        border: 1px solid var(--border);
        border-radius: 10px;
        padding: 16px 18px;
    }
    .card .label {
        font-size: 0.75rem;
        color: var(--dim);
        text-transform: uppercase;
        letter-spacing: 0.05em;
        margin-bottom: 8px;
    }
    .card .value {
        font-size: 1.6rem;
        font-weight: 600;
    }
    .card .subvalue {
        font-size: 0.85rem;
        color: var(--dim);
        margin-top: 2px;
    }
    .pill {
        display: inline-block;
        padding: 3px 10px;
        border-radius: 999px;
        font-size: 0.85rem;
        font-weight: 600;
    }
    .pill.CW { background: rgba(99,209,106,0.15); color: var(--cw); }
    .pill.CCW { background: rgba(240,168,80,0.15); color: var(--ccw); }
    .pill.OFF { background: rgba(107,116,128,0.15); color: var(--off); }
    .pill.HIGH { background: rgba(79,209,197,0.15); color: var(--hall-high); }
    .pill.LOW { background: rgba(224,97,107,0.15); color: var(--hall-low); }
    .log-section h2 {
        font-size: 1rem;
        color: var(--dim);
        font-weight: 600;
        margin-bottom: 10px;
    }
    .log-box {
        background: var(--card-bg);
        border: 1px solid var(--border);
        border-radius: 10px;
        height: 340px;
        overflow-y: auto;
        font-family: "SF Mono", Consolas, monospace;
        font-size: 0.82rem;
    }
    .log-row {
        display: flex;
        padding: 4px 14px;
        border-bottom: 1px solid var(--border);
    }
    .log-row .ts {
        color: var(--accent);
        margin-right: 14px;
        flex-shrink: 0;
        width: 100px;
    }
    .log-row .msg { color: var(--text); }
</style>
</head>
<body>
<h1>Solar Tracker Telemetry</h1>
<div class="status">
    <span id="statusDot" class="dot"></span>
    <span id="statusText">Connecting...</span>
    &nbsp;&middot;&nbsp; last update: <span id="lastUpdate">--</span>
</div>

<div class="grid">
    <div class="card">
        <div class="label">Battery</div>
        <div class="value" id="batteryFilt">--</div>
        <div class="subvalue">raw: <span id="batteryRaw">--</span></div>
    </div>
    <div class="card">
        <div class="label">Base LDR</div>
        <div class="value" id="baseFilt">--</div>
        <div class="subvalue">raw: <span id="baseRaw">--</span></div>
    </div>
    <div class="card">
        <div class="label">Panel LDR</div>
        <div class="value" id="panelFilt">--</div>
        <div class="subvalue">raw: <span id="panelRaw">--</span></div>
    </div>
    <div class="card">
        <div class="label">Pot</div>
        <div class="value" id="potFilt">--</div>
        <div class="subvalue">raw: <span id="potRaw">--</span></div>
    </div>
    <div class="card">
        <div class="label">Base Motor</div>
        <div class="value"><span id="baseMotor" class="pill OFF">--</span></div>
    </div>
    <div class="card">
        <div class="label">Panel Motor</div>
        <div class="value"><span id="panelMotor" class="pill OFF">--</span></div>
    </div>
    <div class="card">
        <div class="label">Right Hall</div>
        <div class="value"><span id="rightHall" class="pill">--</span></div>
    </div>
    <div class="card">
        <div class="label">Left Hall</div>
        <div class="value"><span id="leftHall" class="pill">--</span></div>
    </div>
</div>

<div class="log-section">
    <h2>Live Log</h2>
    <div class="log-box" id="logBox"></div>
</div>

<script>
function setText(id, val) {
    document.getElementById(id).textContent = (val === null || val === undefined) ? '--' : val;
}

function setPill(id, val) {
    const el = document.getElementById(id);
    el.textContent = val || '--';
    el.className = 'pill ' + (val || '');
}

async function poll() {
    try {
        const res = await fetch('/api/data');
        const data = await res.json();
        const l = data.latest;

        document.getElementById('statusDot').className = 'dot' + (l.connected ? ' on' : '');
        document.getElementById('statusText').textContent = l.connected ? 'Connected' : 'Disconnected';
        setText('lastUpdate', l.last_update);

        setText('batteryFilt', l.battery_filt);
        setText('batteryRaw', l.battery_raw);
        setText('baseFilt', l.base_filt);
        setText('baseRaw', l.base_raw);
        setText('panelFilt', l.panel_filt);
        setText('panelRaw', l.panel_raw);
        setText('potFilt', l.pot_filt);
        setText('potRaw', l.pot_raw);

        setPill('baseMotor', l.base_motor);
        setPill('panelMotor', l.panel_motor);
        setPill('rightHall', l.right_hall);
        setPill('leftHall', l.left_hall);

        const logBox = document.getElementById('logBox');
        logBox.innerHTML = data.log.map(([ts, msg]) =>
            `<div class="log-row"><span class="ts">${ts}</span><span class="msg">${msg}</span></div>`
        ).join('');
    } catch (e) {
        document.getElementById('statusText').textContent = 'Server unreachable';
    }
}

setInterval(poll, 400);
poll();
</script>
</body>
</html>
"""


def main():
    parser = argparse.ArgumentParser(description="Solar tracker serial dashboard")
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/tty.usbserial-XXXX or COM5")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default 115200)")
    parser.add_argument("--http-port", type=int, default=5000, help="Local web server port (default 5000)")
    args = parser.parse_args()

    reader_thread = threading.Thread(
        target=serial_reader, args=(args.port, args.baud), daemon=True
    )
    reader_thread.start()

    print(f"Dashboard running at http://localhost:{args.http_port}")
    app.run(host="0.0.0.0", port=args.http_port, debug=False)


if __name__ == "__main__":
    main()