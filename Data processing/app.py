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
run_lock = threading.Lock()

motor_status = "OFF"      # OFF / ON / RUN xx%
connection_status_text = "Disconnected"
connection_started_dt = None

# Logging files
SESSION_FOLDER = None
TXT_LOG_PATH = None
CSV_LOG_PATH = None

# Run tracking
run_history = []
current_run = None
run_counter = 0
motor_enabled_state = False
commanded_speed_percent = 0


# ===================== FILE / LOG HELPERS =====================
def make_session_files():
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
    with log_lock:
        log_buffer.append(message)


def write_txt_log(tag: str, message: str):
    if not TXT_LOG_PATH:
        return

    timestamp = datetime.now().isoformat(timespec="seconds")
    line = f"[{timestamp}] [{tag}] {message}\n"

    with file_lock:
        with open(TXT_LOG_PATH, "a", encoding="utf-8") as f:
            f.write(line)


def write_csv_row(ms, time_s, current_a, thrust_g, raw_line):
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
        title=dict(text=title, font=dict(color="white")),
        margin=dict(l=20, r=20, t=40, b=30),
        height=320,
        paper_bgcolor="rgba(0,0,0,0)",
        plot_bgcolor="rgba(0,0,0,0)",
        xaxis_title="Time (s)",
        yaxis_title=title,
        font=dict(color="#e0e0e0"),
        xaxis=dict(gridcolor="rgba(255,255,255,0.1)", zerolinecolor="rgba(255,255,255,0.1)"),
        yaxis=dict(gridcolor="rgba(255,255,255,0.1)", zerolinecolor="rgba(255,255,255,0.1)"),
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


def format_elapsed_seconds(total_seconds):
    if total_seconds is None or total_seconds < 0:
        return "00:00:00"

    total_seconds = int(total_seconds)
    hours = total_seconds // 3600
    minutes = (total_seconds % 3600) // 60
    seconds = total_seconds % 60
    return f"{hours:02d}:{minutes:02d}:{seconds:02d}"


def get_connection_elapsed_str():
    with state_lock:
        if connection_started_dt is None:
            return "00:00:00"
        return format_elapsed_seconds((datetime.now() - connection_started_dt).total_seconds())


def get_active_run_elapsed_str():
    with run_lock:
        if current_run is None:
            return "00:00:00"
        return format_elapsed_seconds((datetime.now() - current_run["start_dt"]).total_seconds())


# ===================== RUN TRACKING HELPERS =====================
def fmt_dt(dt_obj):
    return dt_obj.strftime("%Y-%m-%d %H:%M:%S")


def finalize_current_run(reason: str):
    global current_run

    with run_lock:
        if current_run is None:
            return

        end_dt = datetime.now()
        samples = current_run["samples"]
        thrust_samples = current_run["thrust_samples"]
        ratio_samples = current_run["ratio_samples"]

        avg_current = (current_run["current_sum"] / samples) if samples > 0 else None
        avg_thrust = (current_run["thrust_sum"] / thrust_samples) if thrust_samples > 0 else None
        avg_ratio = (current_run["ratio_sum"] / ratio_samples) if ratio_samples > 0 else None

        run_history.append(
            {
                "run_id": current_run["run_id"],
                "status": "Completed",
                "start_time": current_run["start_time"],
                "start_cmd": current_run["start_cmd"],
                "end_time": fmt_dt(end_dt),
                "end_reason": reason,
                "duration_s": (end_dt - current_run["start_dt"]).total_seconds(),
                "samples": samples,
                "avg_current": avg_current,
                "avg_thrust": avg_thrust,
                "avg_ratio": avg_ratio,
            }
        )

        current_run = None


def start_new_run(trigger_cmd: str):
    global current_run, run_counter

    with run_lock:
        if current_run is not None:
            return

        run_counter += 1
        now_dt = datetime.now()
        current_run = {
            "run_id": run_counter,
            "start_dt": now_dt,
            "start_time": fmt_dt(now_dt),
            "start_cmd": trigger_cmd,
            "samples": 0,
            "current_sum": 0.0,
            "thrust_sum": 0.0,
            "ratio_sum": 0.0,
            "thrust_samples": 0,
            "ratio_samples": 0,
        }


def update_run_metrics(current_a, thrust_g):
    with run_lock:
        if current_run is None:
            return

        current_run["samples"] += 1
        current_run["current_sum"] += current_a

        if thrust_g is not None:
            current_run["thrust_sum"] += thrust_g
            current_run["thrust_samples"] += 1

            if current_a > 0:
                current_run["ratio_sum"] += (thrust_g / current_a)
                current_run["ratio_samples"] += 1


def parse_speed_percent(cmd: str):
    parts = cmd.strip().split(maxsplit=1)
    if len(parts) != 2 or parts[0].upper() != "SPEED":
        return None

    try:
        value = int(float(parts[1].strip()))
    except ValueError:
        return None

    return max(0, min(100, value))


def track_command_for_runs(cmd: str):
    global motor_enabled_state, commanded_speed_percent

    cleaned = cmd.strip()
    upper = cleaned.upper()

    if upper == "ON":
        motor_enabled_state = True
        commanded_speed_percent = 0
        return

    if upper == "OFF":
        motor_enabled_state = False
        commanded_speed_percent = 0
        finalize_current_run("OFF")
        return

    if upper == "STOP":
        commanded_speed_percent = 0
        finalize_current_run("STOP")
        return

    speed_val = parse_speed_percent(cleaned)
    if speed_val is None:
        return

    commanded_speed_percent = speed_val

    if speed_val > 0:
        # Treat every positive SPEED command as a fresh active run.
        # This resets the active timer if a previous run is still active.
        motor_enabled_state = True
        finalize_current_run("New SPEED command")
        start_new_run(f"SPEED {speed_val}")
    else:
        commanded_speed_percent = 0
        finalize_current_run("SPEED 0")
        
#Handle time out:
def handle_command_timeout():
    global motor_enabled_state, commanded_speed_percent

    motor_enabled_state = False
    commanded_speed_percent = 0
    finalize_current_run("Command timeout")
    set_motor_status("OFF")


def build_run_snapshot(run_dict: dict, status: str = "Completed"):
    return {
        "run_id": run_dict["run_id"],
        "status": status,
        "start_time": run_dict["start_time"],
        "start_cmd": run_dict["start_cmd"],
        "end_time": run_dict.get("end_time", "-"),
        "end_reason": run_dict.get("end_reason", "-"),
        "duration_s": run_dict.get("duration_s"),
        "samples": run_dict.get("samples", 0),
        "avg_current": run_dict.get("avg_current"),
        "avg_thrust": run_dict.get("avg_thrust"),
        "avg_ratio": run_dict.get("avg_ratio"),
    }


def get_run_rows_for_display():
    with run_lock:
        rows = [build_run_snapshot(run) for run in run_history]

        if current_run is not None:
            live_samples = current_run["samples"]
            live_thrust_samples = current_run["thrust_samples"]
            live_ratio_samples = current_run["ratio_samples"]

            rows.append(
                {
                    "run_id": current_run["run_id"],
                    "status": "ACTIVE",
                    "start_time": current_run["start_time"],
                    "start_cmd": current_run["start_cmd"],
                    "end_time": "-",
                    "end_reason": "-",
                    "duration_s": (datetime.now() - current_run["start_dt"]).total_seconds(),
                    "samples": live_samples,
                    "avg_current": (current_run["current_sum"] / live_samples) if live_samples > 0 else None,
                    "avg_thrust": (current_run["thrust_sum"] / live_thrust_samples) if live_thrust_samples > 0 else None,
                    "avg_ratio": (current_run["ratio_sum"] / live_ratio_samples) if live_ratio_samples > 0 else None,
                }
            )

        return rows


# ===================== SERIAL SEND =====================
def send_command(cmd: str):
    global serial_obj

    with serial_lock:
        if serial_obj and serial_obj.is_open:
            try:
                serial_obj.write(f"{cmd}\n".encode("utf-8"))
                add_log(f"> Sent: {cmd}")
                write_txt_log("TX", cmd)
                track_command_for_runs(cmd)
            except Exception as e:
                add_log(f"> Failed to send '{cmd}': {e}")
                write_txt_log("ERROR", f"Failed TX '{cmd}': {e}")
        else:
            add_log("> Error: Not connected to serial port.")
            write_txt_log("ERROR", "Attempted TX while disconnected")


# ===================== SERIAL THREAD =====================
def serial_reader():
    global serial_obj, stop_thread, connection_started_dt

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

                write_txt_log("RX", line)

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

                        update_run_metrics(current_a, thrust_g)
                        write_csv_row(ms, time_s, current_a, thrust_g, line)

                    except ValueError:
                        add_log(line)
                else:
                    add_log(line)

                    # Stop active timer immediately on failsafe timeout message
                    if line.strip() == "Motor disabled due to command timeout.":
                        handle_command_timeout()

        except serial.SerialException as e:
            add_log(f"Serial connection lost: {e}")
            write_txt_log("ERROR", f"Serial connection lost: {e}")
            finalize_current_run("Serial lost")
            set_motor_status("OFF")
            set_connection_status("Disconnected.")
            with state_lock:
                connection_started_dt = None
            break

        except Exception as e:
            add_log(f"Serial Error: {e}")
            write_txt_log("ERROR", f"Serial Error: {e}")
            finalize_current_run("Serial error")
            set_motor_status("OFF")
            set_connection_status("Disconnected.")
            with state_lock:
                connection_started_dt = None
            break

    add_log("Serial reader thread stopped.")
    write_txt_log("SYSTEM", "Serial reader thread stopped")


# ===================== APP SETUP =====================
make_session_files()

ASSETS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets")
os.makedirs(ASSETS_DIR, exist_ok=True)

app = dash.Dash(
    __name__,
    external_stylesheets=[dbc.themes.DARKLY],
    suppress_callback_exceptions=True,
    assets_folder=ASSETS_DIR,
)
app.title = "ESP32 Test Stand"

app.index_string = """
<!DOCTYPE html>
<html>
    <head>
        {%metas%}
        <title>{%title%}</title>
        {%favicon%}
        {%css%}
    </head>
    <body>
        {%app_entry%}
        <footer>
            {%config%}
            {%scripts%}
            {%renderer%}
        </footer>
    </body>
</html>
"""


def make_navbar():
    return dbc.Navbar(
        dbc.Container(
            [
                dbc.NavbarBrand("ESP32 Test Stand", className="fw-bold"),
                dbc.Nav(
                    [
                        dbc.NavLink("Dashboard", href="/", active="exact"),
                        dbc.NavLink("Run Summary", href="/summary", active="exact"),
                    ],
                    pills=True,
                    className="ms-auto",
                ),
            ],
            fluid=True,
        ),
        color="dark",
        dark=True,
        sticky="top",
    )


def build_sidebar():
    return html.Div(
        [
            html.H4("🔌 Connection", className="text-info mb-3"),
            dbc.Row(
                [
                    dbc.Col(
                        dbc.Select(
                            id="dropdown-port",
                            options=[],
                            value=None,
                            className="glass-input mb-2",
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
                options=[{"label": f"{b} baud", "value": b} for b in [115200, 9600, 38400, 57600]],
                value=115200,
                className="glass-input mb-3",
            ),
            dbc.Row(
                [
                    dbc.Col(dbc.Button("Connect", id="btn-connect", color="success", className="w-100")),
                    dbc.Col(dbc.Button("Disconnect", id="btn-disconnect", color="danger", className="w-100")),
                ]
            ),
            html.Div(id="connection-status", className="mt-2 text-warning fw-bold"),

            html.Hr(style={"borderColor": "rgba(255,255,255,0.2)"}),

            html.H4("⚙️ Motor Control", className="text-info mb-3"),
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
                className="glass-input mb-2",
            ),
            dbc.Button("Set Speed", id="btn-set-speed", color="info", className="w-100 mt-2 mb-3"),

            html.Hr(style={"borderColor": "rgba(255,255,255,0.2)"}),

            html.H4("⚖️ Load Cell", className="text-info mb-3"),
            dbc.Button("TARE (Zero Scale)", id="btn-tare", color="warning", className="w-100 mb-2"),
            dbc.Input(id="input-cal", type="number", value=100.0, step=1.0, className="glass-input mb-2"),
            dbc.Button("CALIBRATE", id="btn-cal", color="info", className="w-100 mb-4"),

            html.Hr(style={"borderColor": "rgba(255,255,255,0.2)"}),

            html.H4("🧹 Data Management", className="text-info mb-3"),
            dbc.Button(
                "Clear Graphs & Data",
                id="btn-clear-graphs",
                color="outline-danger",
                className="w-100 mb-2 fw-bold",
            ),
            dbc.Button("Clear Logs", id="btn-clear-logs", color="outline-warning", className="w-100"),

            html.Div(id="dummy-output", style={"display": "none"}),
        ],
        className="glass-panel h-100",
    )


def build_dashboard_page():
    return dbc.Container(
        [
            dbc.Row(
                [
                    dbc.Col(build_sidebar(), width=3),
                    dbc.Col(
                        html.Div(
                            [
                                html.H2("📊 Live Telemetry Dashboard", className="mb-4 fw-bold"),

                                dbc.Row(
                                    [
                                        dbc.Col(
                                            dbc.Card(
                                                dbc.CardBody(
                                                    [
                                                        html.H6("Live Current", className="text-muted"),
                                                        html.H3(id="val-current", className="text-danger"),
                                                    ]
                                                ),
                                                className="glass-card text-center",
                                            )
                                        ),
                                        dbc.Col(
                                            dbc.Card(
                                                dbc.CardBody(
                                                    [
                                                        html.H6("Live Thrust", className="text-muted"),
                                                        html.H3(id="val-thrust", className="text-success"),
                                                    ]
                                                ),
                                                className="glass-card text-center",
                                            )
                                        ),
                                        dbc.Col(
                                            dbc.Card(
                                                dbc.CardBody(
                                                    [
                                                        html.H6("Data Points", className="text-muted"),
                                                        html.H3(id="val-points", className="text-info"),
                                                    ]
                                                ),
                                                className="glass-card text-center",
                                            )
                                        ),
                                        dbc.Col(
                                            dbc.Card(
                                                dbc.CardBody(
                                                    [
                                                        html.H6("Motor Status", className="text-muted"),
                                                        html.H3(id="val-motor-status", className="text-warning"),
                                                    ]
                                                ),
                                                className="glass-card text-center",
                                            )
                                        ),
                                    ],
                                    className="mb-3",
                                ),

                                dbc.Row(
                                    [
                                        dbc.Col(
                                            dbc.Card(
                                                dbc.CardBody(
                                                    [
                                                        html.H6("Connection Timer", className="text-muted"),
                                                        html.H3(id="val-connection-timer", className="text-primary"),
                                                    ]
                                                ),
                                                className="glass-card text-center",
                                            )
                                        ),
                                        dbc.Col(
                                            dbc.Card(
                                                dbc.CardBody(
                                                    [
                                                        html.H6("Active Timer", className="text-muted"),
                                                        html.H3(id="val-active-timer", className="text-info"),
                                                    ]
                                                ),
                                                className="glass-card text-center",
                                            )
                                        ),
                                    ],
                                    className="mb-4",
                                ),

                                dbc.Row(
                                    [
                                        dbc.Col(html.Div(dcc.Graph(id="graph-current"), className="glass-panel mb-3"), width=6),
                                        dbc.Col(html.Div(dcc.Graph(id="graph-thrust"), className="glass-panel mb-3"), width=6),
                                    ]
                                ),

                                html.Div(
                                    [
                                        html.H5("💻 Console Output", className="mb-3"),
                                        dbc.Textarea(
                                            id="console-log",
                                            className="glass-textarea",
                                            style={"width": "100%", "height": "200px"},
                                            readOnly=True,
                                        ),
                                        html.Div(id="log-path-display", className="mt-3 text-muted small"),
                                        html.Div(id="csv-path-display", className="mt-1 text-muted small"),
                                    ],
                                    className="glass-panel mt-2",
                                ),
                            ]
                        ),
                        width=9,
                    ),
                ],
                className="mt-4 pb-4",
            )
        ],
        fluid=True,
    )


def build_summary_page():
    return dbc.Container(
        [
            dbc.Row(
                [
                    dbc.Col(
                        html.Div(
                            [
                                html.H2("🧾 Run Summary", className="fw-bold mb-4"),
                                dbc.Row(
                                    [
                                        dbc.Col(
                                            dbc.Card(
                                                dbc.CardBody(
                                                    [
                                                        html.H6("Completed Runs", className="text-muted"),
                                                        html.H3(id="summary-total-runs", className="text-info"),
                                                    ]
                                                ),
                                                className="glass-card text-center mb-3",
                                            ),
                                            md=3,
                                        ),
                                        dbc.Col(
                                            dbc.Card(
                                                dbc.CardBody(
                                                    [
                                                        html.H6("Active Run", className="text-muted"),
                                                        html.H3(id="summary-active-status", className="text-warning"),
                                                    ]
                                                ),
                                                className="glass-card text-center mb-3",
                                            ),
                                            md=3,
                                        ),
                                        dbc.Col(
                                            dbc.Card(
                                                dbc.CardBody(
                                                    [
                                                        html.H6("Latest Run Start", className="text-muted"),
                                                        html.H5(id="summary-last-run", className="text-success"),
                                                    ]
                                                ),
                                                className="glass-card text-center mb-3",
                                            ),
                                            md=3,
                                        ),
                                        dbc.Col(
                                            dbc.Card(
                                                dbc.CardBody(
                                                    [
                                                        html.H6("Latest Avg T/I", className="text-muted"),
                                                        html.H3(id="summary-latest-eff", className="text-primary"),
                                                    ]
                                                ),
                                                className="glass-card text-center mb-3",
                                            ),
                                            md=3,
                                        ),
                                    ]
                                ),

                                html.Div(
                                    [
                                        html.H5("Active Run Detail", className="mb-3"),
                                        html.Div(id="summary-active-detail"),
                                    ],
                                    className="glass-panel mb-4",
                                ),

                                html.Div(
                                    [
                                        html.H5("Run Table", className="mb-3"),
                                        html.Div(
                                            [
                                                dbc.Table(
                                                    [
                                                        html.Thead(
                                                            html.Tr(
                                                                [
                                                                    html.Th("#"),
                                                                    html.Th("Status"),
                                                                    html.Th("Run Command Time"),
                                                                    html.Th("Trigger Cmd"),
                                                                    html.Th("End Time"),
                                                                    html.Th("End Reason"),
                                                                    html.Th("Duration (s)"),
                                                                    html.Th("Samples"),
                                                                    html.Th("Avg Current (A)"),
                                                                    html.Th("Avg Thrust (g)"),
                                                                    html.Th("Avg T/I (g/A)"),
                                                                ]
                                                            )
                                                        ),
                                                        html.Tbody(id="summary-table-body"),
                                                    ],
                                                    bordered=False,
                                                    hover=True,
                                                    responsive=True,
                                                    class_name="summary-table",
                                                )
                                            ],
                                            style={"overflowX": "auto"},
                                        ),
                                    ],
                                    className="glass-panel",
                                ),
                            ],
                            className="mt-4 pb-4",
                        ),
                        width=12,
                    )
                ]
            )
        ],
        fluid=True,
    )


app.layout = html.Div(
    [
        dcc.Location(id="url"),
        make_navbar(),
        dcc.Interval(id="update-interval", interval=500, n_intervals=0),
        html.Div(id="page-content"),
    ]
)


# ===================== PAGE ROUTING =====================
@app.callback(Output("page-content", "children"), Input("url", "pathname"))
def render_page(pathname):
    if pathname == "/summary":
        return build_summary_page()
    return build_dashboard_page()


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
    global serial_obj, serial_thread, stop_thread, connection_started_dt

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

                with state_lock:
                    connection_started_dt = datetime.now()

                msg = f"Connected to {port}"
                set_connection_status(msg)
                add_log(f"Connected to {port} @ {baud} baud")
                write_txt_log("CONNECTED", f"{port} @ {baud}")
                return msg

            except Exception as e:
                add_log(f"Connection failed: {e}")
                write_txt_log("ERROR", f"Connection failed: {e}")
                serial_obj = None
                with state_lock:
                    connection_started_dt = None
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
                    finalize_current_run("Disconnect")
                    set_motor_status("OFF")
                except Exception as e:
                    add_log(f"Disconnect error: {e}")
                    write_txt_log("ERROR", f"Disconnect error: {e}")
                finally:
                    serial_obj = None
                    with state_lock:
                        connection_started_dt = None

                set_connection_status("Disconnected.")
                return "Disconnected."

            with state_lock:
                connection_started_dt = None
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
                if speed > 0:
                    set_motor_status(f"RUN {speed}%")
                else:
                    set_motor_status("ON")
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
    Output("val-connection-timer", "children"),
    Output("val-active-timer", "children"),
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
    connection_timer = get_connection_elapsed_str()
    active_timer = get_active_run_elapsed_str()

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
                line=dict(color="#ff4d4d", width=2),
            )
        )
        fig_curr.update_layout(
            title=dict(text="Current (A)", font=dict(color="white")),
            margin=dict(l=20, r=20, t=40, b=30),
            height=320,
            paper_bgcolor="rgba(0,0,0,0)",
            plot_bgcolor="rgba(0,0,0,0)",
            xaxis_title="Time (s)",
            yaxis_title="Current (A)",
            font=dict(color="#e0e0e0"),
            xaxis=dict(gridcolor="rgba(255,255,255,0.1)", zerolinecolor="rgba(255,255,255,0.1)"),
            yaxis=dict(gridcolor="rgba(255,255,255,0.1)", zerolinecolor="rgba(255,255,255,0.1)"),
        )

        fig_thrust = go.Figure()
        fig_thrust.add_trace(
            go.Scatter(
                x=df["Time (s)"],
                y=df["Thrust (g)"],
                mode="lines+markers",
                name="Thrust",
                connectgaps=False,
                line=dict(color="#00ffcc", width=2),
                marker=dict(size=4),
            )
        )
        fig_thrust.update_layout(
            title=dict(text="Thrust (g)", font=dict(color="white")),
            margin=dict(l=20, r=20, t=40, b=30),
            height=320,
            paper_bgcolor="rgba(0,0,0,0)",
            plot_bgcolor="rgba(0,0,0,0)",
            xaxis_title="Time (s)",
            yaxis_title="Thrust (g)",
            font=dict(color="#e0e0e0"),
            xaxis=dict(gridcolor="rgba(255,255,255,0.1)", zerolinecolor="rgba(255,255,255,0.1)"),
            yaxis=dict(gridcolor="rgba(255,255,255,0.1)", zerolinecolor="rgba(255,255,255,0.1)"),
        )

    else:
        curr_val = "0.00 A"
        thrust_val = "N/A"
        pts = "0"
        fig_curr = make_empty_figure("Current (A)")
        fig_thrust = make_empty_figure("Thrust (g)")

    log_path_text = f"TXT log file: {TXT_LOG_PATH}"
    csv_path_text = f"CSV data file: {CSV_LOG_PATH}"

    return (
        fig_curr,
        fig_thrust,
        curr_val,
        thrust_val,
        pts,
        motor,
        connection_timer,
        active_timer,
        logs,
        log_path_text,
        csv_path_text,
    )


@app.callback(
    Output("summary-total-runs", "children"),
    Output("summary-active-status", "children"),
    Output("summary-last-run", "children"),
    Output("summary-latest-eff", "children"),
    Output("summary-active-detail", "children"),
    Output("summary-table-body", "children"),
    Input("update-interval", "n_intervals"),
)
def update_summary_page(n):
    rows = get_run_rows_for_display()

    completed_rows = [row for row in rows if row["status"] == "Completed"]
    active_rows = [row for row in rows if row["status"] == "ACTIVE"]

    total_runs = str(len(completed_rows))
    active_status = "YES" if active_rows else "NO"

    latest_run = rows[-1]["start_time"] if rows else "-"
    latest_eff = "-"
    if rows and rows[-1]["avg_ratio"] is not None:
        latest_eff = f"{rows[-1]['avg_ratio']:.2f} g/A"

    if active_rows:
        active = active_rows[-1]
        active_detail = html.Ul(
            [
                html.Li(f"Run command time: {active['start_time']}"),
                html.Li(f"Trigger command: {active['start_cmd']}"),
                html.Li(f"Duration: {active['duration_s']:.2f} s"),
                html.Li(f"Samples: {active['samples']}"),
                html.Li(
                    "Average current: "
                    + (f"{active['avg_current']:.3f} A" if active["avg_current"] is not None else "N/A")
                ),
                html.Li(
                    "Average thrust: "
                    + (f"{active['avg_thrust']:.3f} g" if active["avg_thrust"] is not None else "N/A")
                ),
                html.Li(
                    "Average thrust/current: "
                    + (f"{active['avg_ratio']:.3f} g/A" if active["avg_ratio"] is not None else "N/A")
                ),
            ]
        )
    else:
        active_detail = html.Div("No active run right now.", className="text-muted")

    if rows:
        body_rows = []
        for row in rows:
            body_rows.append(
                html.Tr(
                    [
                        html.Td(row["run_id"]),
                        html.Td(row["status"]),
                        html.Td(row["start_time"]),
                        html.Td(row["start_cmd"]),
                        html.Td(row["end_time"]),
                        html.Td(row["end_reason"]),
                        html.Td(f"{row['duration_s']:.2f}" if row["duration_s"] is not None else "-"),
                        html.Td(row["samples"]),
                        html.Td(f"{row['avg_current']:.3f}" if row["avg_current"] is not None else "-"),
                        html.Td(f"{row['avg_thrust']:.3f}" if row["avg_thrust"] is not None else "-"),
                        html.Td(f"{row['avg_ratio']:.3f}" if row["avg_ratio"] is not None else "-"),
                    ]
                )
            )
    else:
        body_rows = [
            html.Tr(
                [
                    html.Td("No run data yet.", colSpan=11, className="text-center text-muted")
                ]
            )
        ]

    return total_runs, active_status, latest_run, latest_eff, active_detail, body_rows


if __name__ == "__main__":
    app.run(debug=True)