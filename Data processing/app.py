import os
import csv
from datetime import datetime
from collections import deque
import threading

import dash
from dash import dcc, html, Input, Output, State, ctx, no_update
import dash_bootstrap_components as dbc
import plotly.graph_objects as go
import pandas as pd
import serial
import serial.tools.list_ports

# ===================== CONFIG & STATE =====================
MAX_POINTS = 500
MAX_LOG_LINES = 200

data_buffer = deque(maxlen=MAX_POINTS)
log_buffer = deque(maxlen=MAX_LOG_LINES)

serial_obj = None
serial_thread = None
stop_thread = False

data_lock = threading.Lock()
log_lock = threading.Lock()
serial_lock = threading.Lock()
file_lock = threading.Lock()
state_lock = threading.Lock()

motor_status = "OFF"      # OFF / ON
connection_status_text = "Disconnected"

# Logging files
SESSION_FOLDER = None
TXT_LOG_PATH = None
CSV_LOG_PATH = None


# ===================== FILE / LOG HELPERS =====================
def make_session_files():
    """
    Create a fresh logging folder and files for this app run.
    """
    global SESSION_FOLDER, TXT_LOG_PATH, CSV_LOG_PATH

    now_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    SESSION_FOLDER = os.path.join(os.getcwd(), f"session_logs_{now_str}")
    os.makedirs(SESSION_FOLDER, exist_ok=True)

    TXT_LOG_PATH = os.path.join(SESSION_FOLDER, "serial_log.txt")
    CSV_LOG_PATH = os.path.join(SESSION_FOLDER, "plot_data.csv")

    with file_lock:
        with open(TXT_LOG_PATH, "w", encoding="utf-8") as f:
            f.write(f"Session started: {datetime.now().isoformat()}\n")

        with open(CSV_LOG_PATH, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["pc_time", "ms", "time_s", "current_a", "thrust_g", "raw_line"])


def add_log(message: str):
    """
    Add a message to the on-screen console log buffer.
    """
    with log_lock:
        log_buffer.append(message)


def write_txt_log(tag: str, message: str):
    """
    Write any serial/system event to the TXT log file.
    """
    if not TXT_LOG_PATH:
        return

    timestamp = datetime.now().isoformat(timespec="seconds")
    line = f"[{timestamp}] [{tag}] {message}\n"

    with file_lock:
        with open(TXT_LOG_PATH, "a", encoding="utf-8") as f:
            f.write(line)


def write_csv_row(ms, time_s, current_a, thrust_g, raw_line):
    """
    Write parsed telemetry to CSV.
    """
    if not CSV_LOG_PATH:
        return

    pc_time = datetime.now().isoformat(timespec="seconds")

    with file_lock:
        with open(CSV_LOG_PATH, "a", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow([pc_time, ms, time_s, current_a, thrust_g, raw_line])


def get_available_ports():
    return [port.device for port in serial.tools.list_ports.comports()]


def make_empty_figure(title: str):
    fig = go.Figure()
    fig.update_layout(
        title=title,
        margin=dict(l=20, r=20, t=40, b=30),
        height=320,
        template="plotly_dark",
        paper_bgcolor="rgba(0,0,0,0)",
        plot_bgcolor="rgba(0,0,0,0)",
        xaxis_title="Time (s)",
        yaxis_title=title,
    )
    return fig


def set_motor_status(value: str):
    global motor_status
    with state_lock:
        motor_status = value


def get_motor_status():
    with state_lock:
        return motor_status


def set_connection_status(value: str):
    global connection_status_text
    with state_lock:
        connection_status_text = value


def get_connection_status():
    with state_lock:
        return connection_status_text


# ===================== SERIAL SEND =====================
def send_command(cmd: str):
    """
    Send command to ESP32, also log it to TXT file.
    """
    global serial_obj

    with serial_lock:
        if serial_obj and serial_obj.is_open:
            try:
                serial_obj.write(f"{cmd}\n".encode("utf-8"))
                add_log(f"> Sent: {cmd}")
                write_txt_log("TX", cmd)
            except Exception as e:
                add_log(f"> Failed to send '{cmd}': {e}")
                write_txt_log("ERROR", f"Failed TX '{cmd}': {e}")
        else:
            add_log("> Error: Not connected to serial port.")
            write_txt_log("ERROR", "Attempted TX while disconnected")


# ===================== SERIAL THREAD =====================
def serial_reader():
    """
    Background serial thread:
    - reads every line
    - stores all raw serial lines to TXT
    - parses CSV telemetry to graph + CSV file
    """
    global serial_obj, stop_thread

    add_log("Serial reader thread started.")
    write_txt_log("SYSTEM", "Serial reader thread started")

    while True:
        if stop_thread:
            break

        with serial_lock:
            ser = serial_obj

        if ser is None or not ser.is_open:
            break

        try:
            if ser.in_waiting > 0:
                raw_line = ser.readline()
                line = raw_line.decode("utf-8", errors="ignore").strip()

                if not line:
                    continue

                # Save every received line to TXT
                write_txt_log("RX", line)

                # Expected CSV:
                # ms,raw,v,currentA,lc_raw,lc_grams
                parts = line.split(",")

                if len(parts) == 6:
                    try:
                        ms = int(parts[0].strip())
                        current_a = float(parts[3].strip())

                        lc_str = parts[5].strip().upper()
                        thrust_g = None if lc_str in ("NA", "", "NONE", "NAN") else float(parts[5].strip())

                        time_s = ms / 1000.0

                        row = {
                            "Time (s)": time_s,
                            "Current (A)": current_a,
                            "Thrust (g)": thrust_g,
                        }

                        with data_lock:
                            data_buffer.append(row)

                        write_csv_row(ms, time_s, current_a, thrust_g, line)

                    except ValueError:
                        add_log(line)
                else:
                    add_log(line)

        except serial.SerialException as e:
            add_log(f"Serial connection lost: {e}")
            write_txt_log("ERROR", f"Serial connection lost: {e}")
            break
        except Exception as e:
            add_log(f"Serial Error: {e}")
            write_txt_log("ERROR", f"Serial Error: {e}")
            break

    add_log("Serial reader thread stopped.")
    write_txt_log("SYSTEM", "Serial reader thread stopped")


# ===================== APP SETUP =====================
make_session_files()

app = dash.Dash(__name__, external_stylesheets=[dbc.themes.CYBORG])
app.title = "ESP32 Test Stand"


# ===================== LAYOUT =====================
sidebar = dbc.Card(
    [
        dbc.CardBody(
            [
                html.H4("Connection", className="card-title text-info"),

                dbc.Row(
                    [
                        dbc.Col(
                            dbc.Select(
                                id="dropdown-port",
                                options=[],
                                value=None,
                                className="mb-2",
                            ),
                            width=8,
                        ),
                        dbc.Col(
                            dbc.Button(
                                "Refresh",
                                id="btn-refresh-ports",
                                color="secondary",
                                className="w-100 mb-2",
                            ),
                            width=4,
                        ),
                    ]
                ),

                dbc.Select(
                    id="dropdown-baud",
                    options=[{"label": str(b), "value": b} for b in [115200, 9600, 38400, 57600]],
                    value=115200,
                    className="mb-3",
                ),

                dbc.Row(
                    [
                        dbc.Col(dbc.Button("Connect", id="btn-connect", color="success", className="w-100")),
                        dbc.Col(dbc.Button("Disconnect", id="btn-disconnect", color="danger", className="w-100")),
                    ]
                ),

                html.Div(id="connection-status", className="mt-2 text-warning"),

                html.Hr(),

                html.H4("Motor Control", className="card-title text-info"),
                dbc.ButtonGroup(
                    [
                        dbc.Button("ON", id="btn-on", color="primary"),
                        dbc.Button("OFF", id="btn-off", color="secondary"),
                        dbc.Button("STOP", id="btn-stop", color="danger"),
                    ],
                    className="w-100 mb-3",
                ),

                html.Label("Throttle Speed (%)"),
                dbc.Input(
                    id="input-speed",
                    type="number",
                    min=0,
                    max=100,
                    step=1,
                    value=0,
                    className="mb-2",
                ),
                dbc.Button("Set Speed", id="btn-set-speed", color="info", className="w-100 mt-2 mb-3"),

                html.Hr(),

                html.H4("Load Cell", className="card-title text-info"),
                dbc.Button("TARE (Zero Scale)", id="btn-tare", color="warning", className="w-100 mb-2"),
                dbc.Input(id="input-cal", type="number", value=100.0, step=1.0, className="mb-2"),
                dbc.Button("CALIBRATE", id="btn-cal", color="info", className="w-100 mb-3"),

                html.Hr(),

                dbc.Button("Clear Graphs", id="btn-clear-graphs", color="outline-light", className="w-100 mb-2"),
                dbc.Button("Clear Logs", id="btn-clear-logs", color="outline-warning", className="w-100"),

                html.Div(id="dummy-output", style={"display": "none"}),
            ]
        )
    ],
    className="h-100",
)

main_content = html.Div(
    [
        html.H2("📊 Live thrust and current Dashboard", className="mb-4"),

        dbc.Row(
            [
                dbc.Col(
                    dbc.Card(
                        dbc.CardBody(
                            [html.H5("Live Current"), html.H3(id="val-current", className="text-danger")]
                        )
                    )
                ),
                dbc.Col(
                    dbc.Card(
                        dbc.CardBody(
                            [html.H5("Live Thrust"), html.H3(id="val-thrust", className="text-success")]
                        )
                    )
                ),
                dbc.Col(
                    dbc.Card(
                        dbc.CardBody(
                            [html.H5("Data Points"), html.H3(id="val-points", className="text-info")]
                        )
                    )
                ),
                dbc.Col(
                    dbc.Card(
                        dbc.CardBody(
                            [html.H5("Motor Status"), html.H3(id="val-motor-status", className="text-warning")]
                        )
                    )
                ),
            ],
            className="mb-4",
        ),

        dbc.Row(
            [
                dbc.Col(dcc.Graph(id="graph-current"), width=6),
                dbc.Col(dcc.Graph(id="graph-thrust"), width=6),
            ]
        ),

        html.H4("Console Output", className="mt-4"),
        dbc.Textarea(
            id="console-log",
            style={"width": "100%", "height": "220px", "backgroundColor": "#111", "color": "#0f0"},
            readOnly=True,
        ),

        html.Div(id="log-path-display", className="mt-3 text-info"),
        html.Div(id="csv-path-display", className="mt-1 text-info"),

        dcc.Interval(id="update-interval", interval=500, n_intervals=0),
    ]
)

app.layout = dbc.Container(
    [
        dbc.Row(
            [
                dbc.Col(sidebar, width=3),
                dbc.Col(main_content, width=9),
            ],
            className="mt-4",
        )
    ],
    fluid=True,
)


# ===================== CALLBACKS =====================
@app.callback(
    Output("dropdown-port", "options"),
    Output("dropdown-port", "value"),
    Input("btn-refresh-ports", "n_clicks"),
    Input("update-interval", "n_intervals"),
    State("dropdown-port", "value"),
)
def refresh_ports(_, __, current_value):
    ports = get_available_ports()
    options = [{"label": p, "value": p} for p in ports]

    if current_value in ports:
        value = current_value
    else:
        value = ports[0] if ports else None

    return options, value


@app.callback(
    Output("connection-status", "children"),
    Input("btn-connect", "n_clicks"),
    Input("btn-disconnect", "n_clicks"),
    State("dropdown-port", "value"),
    State("dropdown-baud", "value"),
    prevent_initial_call=True,
)
def manage_connection(btn_conn, btn_disconn, port, baud):
    global serial_obj, serial_thread, stop_thread

    trigger = ctx.triggered_id

    if trigger == "btn-connect":
        if not port:
            set_connection_status("No serial port selected.")
            return "No serial port selected."

        with serial_lock:
            if serial_obj and serial_obj.is_open:
                msg = f"Already connected to {serial_obj.port}."
                set_connection_status(msg)
                return msg

            try:
                serial_obj = serial.Serial(port, int(baud), timeout=1)
                stop_thread = False
                serial_thread = threading.Thread(target=serial_reader, daemon=True)
                serial_thread.start()

                msg = f"Connected to {port}"
                set_connection_status(msg)
                add_log(f"Connected to {port} @ {baud} baud")
                write_txt_log("CONNECTED", f"{port} @ {baud}")
                return msg

            except Exception as e:
                add_log(f"Connection failed: {e}")
                write_txt_log("ERROR", f"Connection failed: {e}")
                serial_obj = None
                msg = f"Error: {e}"
                set_connection_status(msg)
                return msg

    elif trigger == "btn-disconnect":
        with serial_lock:
            if serial_obj and serial_obj.is_open:
                try:
                    stop_thread = True
                    serial_obj.close()
                    add_log("Disconnected from serial port.")
                    write_txt_log("DISCONNECTED", "Serial port closed")
                except Exception as e:
                    add_log(f"Disconnect error: {e}")
                    write_txt_log("ERROR", f"Disconnect error: {e}")
                finally:
                    serial_obj = None

                set_connection_status("Disconnected.")
                return "Disconnected."

            set_connection_status("Already disconnected.")
            return "Already disconnected."

    return no_update


@app.callback(
    Output("dummy-output", "children"),
    Input("btn-on", "n_clicks"),
    Input("btn-off", "n_clicks"),
    Input("btn-stop", "n_clicks"),
    Input("btn-set-speed", "n_clicks"),
    Input("btn-tare", "n_clicks"),
    Input("btn-cal", "n_clicks"),
    Input("btn-clear-graphs", "n_clicks"),
    Input("btn-clear-logs", "n_clicks"),
    State("input-speed", "value"),
    State("input-cal", "value"),
    prevent_initial_call=True,
)
def handle_commands(btn_on, btn_off, btn_stop, btn_spd, btn_tare, btn_cal, btn_clear_graphs, btn_clear_logs, speed, cal_weight):
    trigger = ctx.triggered_id

    if trigger == "btn-on":
        send_command("ON")
        set_motor_status("ON")

    elif trigger == "btn-off":
        send_command("OFF")
        set_motor_status("OFF")

    elif trigger == "btn-stop":
        send_command("STOP")
        set_motor_status("OFF")

    elif trigger == "btn-set-speed":
        try:
            speed = int(speed)
            if not (0 <= speed <= 100):
                add_log("> Error: Speed must be between 0 and 100.")
                write_txt_log("ERROR", f"Invalid SPEED value: {speed}")
            else:
                send_command(f"SPEED {speed}")
        except (TypeError, ValueError):
            add_log("> Error: Invalid speed value.")
            write_txt_log("ERROR", f"Invalid speed input: {speed}")

    elif trigger == "btn-tare":
        send_command("TARE")

    elif trigger == "btn-cal":
        try:
            cal_weight = float(cal_weight)
            send_command(f"CAL={cal_weight}")
        except (TypeError, ValueError):
            add_log("> Error: Invalid calibration weight.")
            write_txt_log("ERROR", f"Invalid CAL input: {cal_weight}")

    elif trigger == "btn-clear-graphs":
        with data_lock:
            data_buffer.clear()
        add_log("> Local graph buffer cleared.")
        write_txt_log("SYSTEM", "Graph buffer cleared")

    elif trigger == "btn-clear-logs":
        with log_lock:
            log_buffer.clear()
        write_txt_log("SYSTEM", "Console log buffer cleared")

    return ""


@app.callback(
    Output("graph-current", "figure"),
    Output("graph-thrust", "figure"),
    Output("val-current", "children"),
    Output("val-thrust", "children"),
    Output("val-points", "children"),
    Output("val-motor-status", "children"),
    Output("console-log", "value"),
    Output("log-path-display", "children"),
    Output("csv-path-display", "children"),
    Input("update-interval", "n_intervals"),
)
def update_dashboard(n):
    with log_lock:
        logs = "\n".join(log_buffer)

    with data_lock:
        rows = list(data_buffer)

    motor = get_motor_status()

    if rows:
        df = pd.DataFrame(rows)

        latest_current = df["Current (A)"].dropna().iloc[-1] if df["Current (A)"].notna().any() else 0.0
        latest_thrust = df["Thrust (g)"].dropna().iloc[-1] if df["Thrust (g)"].notna().any() else None

        curr_val = f"{latest_current:.2f} A"
        thrust_val = f"{latest_thrust:.2f} g" if latest_thrust is not None else "N/A"
        pts = str(len(df))

        fig_curr = go.Figure()
        fig_curr.add_trace(
            go.Scatter(
                x=df["Time (s)"],
                y=df["Current (A)"],
                mode="lines",
                name="Current",
            )
        )
        fig_curr.update_layout(
            title="Current (A)",
            margin=dict(l=20, r=20, t=40, b=30),
            height=320,
            template="plotly_dark",
            paper_bgcolor="rgba(0,0,0,0)",
            plot_bgcolor="rgba(0,0,0,0)",
            xaxis_title="Time (s)",
            yaxis_title="Current (A)",
        )

        fig_thrust = go.Figure()
        fig_thrust.add_trace(
            go.Scatter(
                x=df["Time (s)"],
                y=df["Thrust (g)"],
                mode="lines+markers",
                name="Thrust",
                connectgaps=False,
            )
        )
        fig_thrust.update_layout(
            title="Thrust (g)",
            margin=dict(l=20, r=20, t=40, b=30),
            height=320,
            template="plotly_dark",
            paper_bgcolor="rgba(0,0,0,0)",
            plot_bgcolor="rgba(0,0,0,0)",
            xaxis_title="Time (s)",
            yaxis_title="Thrust (g)",
        )

    else:
        curr_val = "0.00 A"
        thrust_val = "N/A"
        pts = "0"
        fig_curr = make_empty_figure("Current (A)")
        fig_thrust = make_empty_figure("Thrust (g)")

    log_path_text = f"TXT log file: {TXT_LOG_PATH}"
    csv_path_text = f"CSV data file: {CSV_LOG_PATH}"

    return fig_curr, fig_thrust, curr_val, thrust_val, pts, motor, logs, log_path_text, csv_path_text


# ===================== RUN =====================
if __name__ == "__main__":
    app.run(debug=False, port=8050)