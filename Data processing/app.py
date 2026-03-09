import dash
from dash import dcc, html, Input, Output, State, ctx
import dash_bootstrap_components as dbc
import plotly.graph_objects as go
import serial
import serial.tools.list_ports
import threading
import pandas as pd
from collections import deque

# ===================== CONFIG & STATE =====================
# Global variables for background thread & data sharing
MAX_POINTS = 200
data_buffer = deque(maxlen=MAX_POINTS)
log_buffer = deque(maxlen=20)
serial_obj = None
stop_thread = False

# ===================== SERIAL THREAD =====================
def serial_reader():
    global serial_obj, stop_thread, data_buffer, log_buffer
    while not stop_thread and serial_obj and serial_obj.is_open:
        try:
            if serial_obj.in_waiting > 0:
                line = serial_obj.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                
                # Check if line is our CSV: ms,raw,v,currentA,lc_raw,lc_grams
                parts = line.split(',')
                if len(parts) == 6 and parts[0].isdigit():
                    try:
                        ms = int(parts[0])
                        current_a = float(parts[3])
                        lc_grams = float(parts[5]) if parts[5] != "NA" else 0.0 
                        
                        data_buffer.append({
                            'Time (s)': ms / 1000.0,
                            'Current (A)': current_a,
                            'Thrust (g)': lc_grams
                        })
                    except ValueError:
                        log_buffer.append(line)
                else:
                    log_buffer.append(line)
        except Exception as e:
            log_buffer.append(f"Serial Error: {e}")
            break

def send_command(cmd):
    global serial_obj, log_buffer
    if serial_obj and serial_obj.is_open:
        try:
            serial_obj.write(f"{cmd}\n".encode('utf-8'))
            log_buffer.append(f"> Sent: {cmd}")
        except Exception as e:
            log_buffer.append(f"Failed to send: {e}")
    else:
        log_buffer.append("> Error: Not connected to serial port.")

# ===================== DASH APP SETUP =====================
app = dash.Dash(__name__, external_stylesheets=[dbc.themes.CYBORG])
app.title = "ESP32 Test Stand"

# Get available ports
available_ports = [port.device for port in serial.tools.list_ports.comports()]
default_port = available_ports[0] if available_ports else ""

# ===================== LAYOUT =====================
sidebar = dbc.Card([
    dbc.CardBody([
        html.H4("⚙️ Connection", className="card-title text-info"),
        dbc.Select(id="dropdown-port", options=[{"label": p, "value": p} for p in available_ports], value=default_port, className="mb-2"),
        dbc.Select(id="dropdown-baud", options=[{"label": str(b), "value": b} for b in [115200, 9600, 38400, 57600]], value=115200, className="mb-3"),
        dbc.Row([
            dbc.Col(dbc.Button("Connect", id="btn-connect", color="success", className="w-100")),
            dbc.Col(dbc.Button("Disconnect", id="btn-disconnect", color="danger", className="w-100")),
        ]),
        html.Div(id="connection-status", className="mt-2 text-warning"),
        
        html.Hr(),
        
        html.H4("🚁 Motor Control", className="card-title text-info"),
        dbc.ButtonGroup([
            dbc.Button("ON", id="btn-on", color="primary"),
            dbc.Button("OFF", id="btn-off", color="secondary"),
            dbc.Button("STOP", id="btn-stop", color="danger"),
        ], className="w-100 mb-3"),
        
        html.Label("Throttle Speed (%)"),
        dcc.Slider(id="slider-speed", min=0, max=100, step=1, value=0, marks={0: '0', 50: '50', 100: '100'}),
        dbc.Button("Set Speed", id="btn-set-speed", color="info", className="w-100 mt-2 mb-3"),
        
        html.Hr(),
        
        html.H4("⚖️ Load Cell", className="card-title text-info"),
        dbc.Button("TARE (Zero Scale)", id="btn-tare", color="warning", className="w-100 mb-2"),
        dbc.Input(id="input-cal", type="number", value=100.0, step=1.0, className="mb-2"),
        dbc.Button("CALIBRATE", id="btn-cal", color="info", className="w-100 mb-3"),
        
        html.Hr(),
        dbc.Button("Clear Graphs", id="btn-clear-graphs", color="outline-light", className="w-100"),
        
        # Hidden div to capture command callback outputs without interfering with UI
        html.Div(id="dummy-output", style={"display": "none"})
    ])
], className="h-100")

main_content = html.Div([
    html.H2("📊 Live Telemetry Dashboard", className="mb-4"),
    
    dbc.Row([
        dbc.Col(dbc.Card(dbc.CardBody([html.H5("Live Current"), html.H3(id="val-current", className="text-danger")]))),
        dbc.Col(dbc.Card(dbc.CardBody([html.H5("Live Thrust"), html.H3(id="val-thrust", className="text-success")]))),
        dbc.Col(dbc.Card(dbc.CardBody([html.H5("Data Points"), html.H3(id="val-points", className="text-info")]))),
    ], className="mb-4"),
    
    dbc.Row([
        dbc.Col(dcc.Graph(id="graph-current"), width=6),
        dbc.Col(dcc.Graph(id="graph-thrust"), width=6),
    ]),
    
    html.H4("Console Output", className="mt-4"),
    dbc.Textarea(id="console-log", style={"width": "100%", "height": "200px", "backgroundColor": "#111", "color": "#0f0"}, readOnly=True),
    
    # The heartbeat of the dashboard (updates every 500ms)
    dcc.Interval(id="update-interval", interval=500, n_intervals=0)
])

app.layout = dbc.Container([
    dbc.Row([
        dbc.Col(sidebar, width=3),
        dbc.Col(main_content, width=9)
    ], className="mt-4")
], fluid=True)

# ===================== CALLBACKS =====================

@app.callback(
    Output("connection-status", "children"),
    Input("btn-connect", "n_clicks"),
    Input("btn-disconnect", "n_clicks"),
    State("dropdown-port", "value"),
    State("dropdown-baud", "value"),
    prevent_initial_call=True
)
def manage_connection(btn_conn, btn_disconn, port, baud):
    global serial_obj, stop_thread
    trigger = ctx.triggered_id
    
    if trigger == "btn-connect":
        if not serial_obj or not serial_obj.is_open:
            try:
                serial_obj = serial.Serial(port, int(baud), timeout=1)
                stop_thread = False
                thread = threading.Thread(target=serial_reader, daemon=True)
                thread.start()
                return f"Connected to {port}"
            except Exception as e:
                return f"Error: {e}"
        return "Already connected."
        
    elif trigger == "btn-disconnect":
        if serial_obj:
            stop_thread = True
            serial_obj.close()
            serial_obj = None
            return "Disconnected."
    return ""

@app.callback(
    Output("dummy-output", "children"),
    Input("btn-on", "n_clicks"),
    Input("btn-off", "n_clicks"),
    Input("btn-stop", "n_clicks"),
    Input("btn-set-speed", "n_clicks"),
    Input("btn-tare", "n_clicks"),
    Input("btn-cal", "n_clicks"),
    Input("btn-clear-graphs", "n_clicks"),
    State("slider-speed", "value"),
    State("input-cal", "value"),
    prevent_initial_call=True
)
def handle_commands(btn_on, btn_off, btn_stop, btn_spd, btn_tare, btn_cal, btn_clear, speed, cal_weight):
    global data_buffer
    trigger = ctx.triggered_id
    
    if trigger == "btn-on": send_command("ON")
    elif trigger == "btn-off": send_command("OFF")
    elif trigger == "btn-stop": send_command("STOP")
    elif trigger == "btn-set-speed": send_command(f"SPEED {speed}")
    elif trigger == "btn-tare": send_command("TARE")
    elif trigger == "btn-cal": send_command(f"CAL={cal_weight}")
    elif trigger == "btn-clear-graphs": data_buffer.clear()
    
    return ""

@app.callback(
    Output("graph-current", "figure"),
    Output("graph-thrust", "figure"),
    Output("val-current", "children"),
    Output("val-thrust", "children"),
    Output("val-points", "children"),
    Output("console-log", "value"),
    Input("update-interval", "n_intervals")
)
def update_dashboard(n):
    global data_buffer, log_buffer
    
    # Process Logs
    logs = "\n".join(list(log_buffer)[::-1]) # Reverse to show newest at bottom (or remove [::-1] for top)
    
    # Process Data
    if len(data_buffer) > 0:
        df = pd.DataFrame(data_buffer)
        curr_val = f"{df['Current (A)'].iloc[-1]:.2f} A"
        thrust_val = f"{df['Thrust (g)'].iloc[-1]:.2f} g"
        pts = str(len(df))
        
        # Build Figures
        fig_curr = go.Figure(go.Scatter(x=df['Time (s)'], y=df['Current (A)'], mode='lines', line=dict(color='#FF4B4B', width=2)))
        fig_thrust = go.Figure(go.Scatter(x=df['Time (s)'], y=df['Thrust (g)'], mode='lines', line=dict(color='#00D4B2', width=2)))
    else:
        curr_val, thrust_val, pts = "0.00 A", "0.00 g", "0"
        fig_curr, fig_thrust = go.Figure(), go.Figure()

    # Styling for both figures
    for fig in [fig_curr, fig_thrust]:
        fig.update_layout(
            margin=dict(l=20, r=20, t=30, b=20),
            height=300,
            template="plotly_dark",
            paper_bgcolor='rgba(0,0,0,0)',
            plot_bgcolor='rgba(0,0,0,0)'
        )
    
    fig_curr.update_layout(title="Current (Amps)")
    fig_thrust.update_layout(title="Thrust (Grams)")

    return fig_curr, fig_thrust, curr_val, thrust_val, pts, logs

# ===================== RUN =====================
if __name__ == '__main__':
    # debug=False is recommended when using background threading for serial ports 
    # to avoid the Flask reloader opening the COM port twice.
    app.run(debug=False, port=8050)