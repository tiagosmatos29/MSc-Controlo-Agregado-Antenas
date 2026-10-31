import zmq
import struct
import serial
import time
import operator
import csv
import math
from datetime import datetime

#############################################
#            SETTINGS
#############################################
ZMQ_ADDRESS = "tcp://127.0.0.1:5555" # IP Localhost
ARDUINO_PORT = "COM5"  # Porta COM do Arduino
BAUD_RATE = 115200

# Parâmetros de Varrimento
MIN_ANGLE = -90
MAX_ANGLE = 90
STEP = 5

# Parâmetro de Comunicação
MAIN_BEAM_DIRECTION = 0.0 # Direção do feixe principal

# Temporização
SETTLE_TIME_STEER = 0.15    # Tempo para os Phase Shifters estabilizarem
SETTLE_TIME_READ = 0.1      # Tempo para o ZMQ atualizar
PAUSE_BETWEEN_ANGLES = 0.0  # Pausa durante o varrimento

# Tempo de Receção GPS
PAUSE_BETWEEN_CYCLES = 15.0

# Tracking Inteligente
TRACKING_WINDOW_DEG = 25       # Graus a varrer em torno da última posição conhecida
last_jammer_angle = None       # Memória de direção
last_jammer_power = None       # Memória de potência

# Calibração de RF
MAX_SIGNAL_DROP_DB = 5.0       # Ajustado para 5 dB devido ao elevado ganho do LNA
BACKGROUND_NOISE_THRESHOLD = -35.0  # Limiar de ruído de fundo determinado experimentalmente (+- -35 dB)

# Proteções Matemáticas e Físicas
GUARD_BAND_DEG = 10
MIN_SEPARATION_STEYSKAL_DEG = 15

# Configuração do Ficheiro de Registo
START_DATETIME = datetime.now().strftime('%Y%m%d_%H%M%S')
LOG_FILENAME = f"jammer_log_{START_DATETIME}.csv"

#############################################
#             INITIALIZATION
#############################################
context = zmq.Context()
zmq_socket = context.socket(zmq.SUB)

zmq_socket.setsockopt(zmq.CONFLATE, 1)
zmq_socket.setsockopt(zmq.RCVHWM, 10000)

zmq_socket.connect(ZMQ_ADDRESS)
zmq_socket.setsockopt_string(zmq.SUBSCRIBE, "")

print(f"Connecting to Arduino on port {ARDUINO_PORT}...")
arduino = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
time.sleep(2)
arduino.reset_input_buffer()

# Criar o ficheiro CSV e escrever o cabeçalho
print(f"Creating log file: {LOG_FILENAME}")
with open(LOG_FILENAME, mode='w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(["Timestamp", "Cycle", "Tracking Mode", "DOA (degrees)", "Original Power (dB)", "Power w/ Null (dB)", "Attenuation (dB)"])

#############################################
#            AUXILIARY FUNCTIONS
#############################################
def read_arduino_response(print_output=True):
    """Lê mensagens do Arduino. Oculta o texto se print_output=False."""
    while arduino.in_waiting > 0:
        try:
            response = arduino.readline().decode('utf-8', errors='ignore').strip()
            if response and print_output:
                print(f"   -> [Arduino] {response}")
        except Exception:
            break

def set_beam_angle(angle, print_output=False):
    """Navega no menu do Arduino e envia o ângulo alvo para Beamforming"""
    arduino.write(b'2')
    time.sleep(0.05)
    command = f"{angle}\n"
    arduino.write(command.encode())
    time.sleep(SETTLE_TIME_STEER)

    # Limpar o buffer durante o varrimento
    read_arduino_response(print_output)

def execute_independent_null_steering(main_beam_angle, null_angle):
    """Aciona o Independent Null Steering (Steyskal, opção 6) no Arduino"""
    print(f"\nSending Null Steering (Steyskal) command to Arduino...")
    print(f" -> Main Beam: {main_beam_angle}º")
    print(f" -> Target Null: {null_angle}º")

    arduino.write(b'6')
    time.sleep(0.05)
    main_command = f"{main_beam_angle}\n"
    arduino.write(main_command.encode())
    time.sleep(0.1)
    null_command = f"{null_angle}\n"
    arduino.write(null_command.encode())
    time.sleep(0.5)

    read_arduino_response(print_output=True)

def reset_to_main_direction():
    """Repõe o feixe para a direção principal útil ao desligar."""
    print(f"\n[RESET] Resetting beam to {MAIN_BEAM_DIRECTION}º before exiting...")
    try:
        set_beam_angle(MAIN_BEAM_DIRECTION, print_output=True)
        print("[RESET] Beam reset successfully.")
    except Exception as e:
        print(f"[RESET] Failed to reset beam: {e}")

def get_latest_rssi():
    """Lê a mensagem ZMQ mais recente e devolve a média do seu lote."""
    latest_batch = None
    while True:
        try:
            raw_data = zmq_socket.recv(flags=zmq.NOBLOCK)
            n_floats = len(raw_data) // 4
            if n_floats > 0:
                latest_batch = struct.unpack(f'<{n_floats}f', raw_data[:n_floats * 4])
        except zmq.Again:
            break
    if latest_batch is None:
        return -100.0
    return sum(latest_batch) / len(latest_batch)

def log_data(cycle, mode, doa, orig_power, null_power, attenuation):
    """Guarda os dados do ciclo atual no ficheiro CSV."""
    with open(LOG_FILENAME, mode='a', newline='') as f:
        writer = csv.writer(f)
        timestamp = datetime.now().strftime('%H:%M:%S')
        writer.writerow([timestamp, cycle, mode, doa, orig_power, null_power, attenuation])

#############################################
#            MAIN CYCLE ENGINE
#############################################
def execute_cycle(cycle_number, previous_angle, previous_power):
    print(f"\n################################################")
    print(f"#                CYCLE {cycle_number}")
    print(f"################################################")

    if previous_angle is None:
        start_angle = MIN_ANGLE
        end_angle = MAX_ANGLE
        tracking_mode = "FULL SWEEP"
        print(f" -> Mode: [{tracking_mode}] (Searching entire 180º area)")
    else:
        # Arredondar para garantir que o rastreamento respeita a grelha de 5 graus
        base_angle = round(previous_angle / STEP) * STEP
        start_angle = max(MIN_ANGLE, base_angle - TRACKING_WINDOW_DEG)
        end_angle = min(MAX_ANGLE, base_angle + TRACKING_WINDOW_DEG)
        tracking_mode = "QUICK TRACKING"
        print(f" -> Mode: [{tracking_mode}] (Searching around {base_angle}º)")

    print(f"    Scanning range: {start_angle}º to {end_angle}º, step of {STEP}º")

    sweep_history = {}

    for angle in range(start_angle, end_angle + 1, STEP):
        set_beam_angle(angle, print_output=False)
        time.sleep(SETTLE_TIME_READ)

        current_rssi = get_latest_rssi()
        sweep_history[angle] = current_rssi
        print(f"[SWEEPING] Angle: {angle:3d}º | RSSI: {current_rssi:6.2f} dB")
        time.sleep(PAUSE_BETWEEN_ANGLES)

    candidates = {
        ang: rssi for ang, rssi in sweep_history.items()
        if abs(ang - MAIN_BEAM_DIRECTION) >= GUARD_BAND_DEG
    }

    if not candidates:
        print("\n[!] No candidates found outside the guard band. Forcing Full Sweep...")
        log_data(cycle_number, tracking_mode, "N/A", "N/A", "N/A", "N/A")
        # Nenhum candidato: repor feixe para a direção principal
        set_beam_angle(MAIN_BEAM_DIRECTION)
        return None, None

    # Encontrar o pico bruto
    raw_peak_angle, peak_rssi = max(candidates.items(), key=operator.itemgetter(1))

    # Refinamento do DOA: Centro de Massa Ponderado
    sum_weights = 0
    sum_angles = 0

    # Apenas o pico bruto e os ângulos imediatamente adjacentes (-5º e +5º)
    neighbors = [raw_peak_angle - STEP, raw_peak_angle, raw_peak_angle + STEP]

    for ang in neighbors:
        if ang in candidates:
            rssi = candidates[ang]
            # Garante que o vizinho pertence à "ponta" do lóbulo (diferença inferior a 3dB)
            if rssi >= peak_rssi - 3.0:
                linear_weight = 10 ** (rssi / 10) # Converter dB para escala linear
                sum_weights += linear_weight
                sum_angles += ang * linear_weight

    if sum_weights > 0:
        interferer_angle = round(sum_angles / sum_weights, 1) # Precisão decimal
    else:
        interferer_angle = raw_peak_angle

    # Teste 1: Ruído de Fundo Absoluto
    if peak_rssi < BACKGROUND_NOISE_THRESHOLD:
        print(f"\n[!] Power ({peak_rssi:.2f} dB) is at room noise level.")
        print(f"[!] Jammer off or undetectable. Clearing memory...")
        log_data(cycle_number, tracking_mode, "N/A", "N/A", "N/A", "N/A")
        set_beam_angle(MAIN_BEAM_DIRECTION)
        return None, None

    # Teste 2: Queda Relativa
    if previous_power is not None:
        if peak_rssi < (previous_power - MAX_SIGNAL_DROP_DB):
            print(f"\n[!] Abrupt signal drop (from {previous_power:.2f} to {peak_rssi:.2f} dB).")
            print(f"[!] The read peak is just a side lobe. The jammer escaped the window!")
            log_data(cycle_number, tracking_mode, "N/A", "N/A", "N/A", "N/A")
            set_beam_angle(MAIN_BEAM_DIRECTION)
            return None, None

    # Teste 3: Deteção de Fronteiras (Usa o pico bruto para evitar falsos negativos devido ao refinamento)
    if previous_angle is not None:
        if raw_peak_angle == start_angle or raw_peak_angle == end_angle:
            print(f"\n[!] Peak detected at the exact boundary of the window ({raw_peak_angle}º).")
            print(f"[!] Real signal must be further out. Forcing Full Sweep to reacquire!")
            log_data(cycle_number, tracking_mode, "N/A", "N/A", "N/A", "N/A")
            set_beam_angle(MAIN_BEAM_DIRECTION)
            return None, None

    print("\n==============================================")
    print("                JAMMER DETECTED                ")
    print("==============================================")
    print(f"Direction (DOA) : {interferer_angle}º")
    print(f"Max Power       : {peak_rssi:.2f} dB")
    print("==============================================")

    # Aplicar Null Steering para atenuar a fonte interferente
    separation = abs(interferer_angle - MAIN_BEAM_DIRECTION)

    if separation < MIN_SEPARATION_STEYSKAL_DEG:
        print(f"\n[!] WARNING: Jammer ({interferer_angle}º) is too close to GPS ({MAIN_BEAM_DIRECTION}º)!")
        print(f"[!] Initiating defensive 'Beam Shifting' to keep Steyskal stable...")

        if interferer_angle >= MAIN_BEAM_DIRECTION:
            shifted_beam = interferer_angle - MIN_SEPARATION_STEYSKAL_DEG
        else:
            shifted_beam = interferer_angle + MIN_SEPARATION_STEYSKAL_DEG

        # Arredondamento extra de segurança para os prints e hardware
        shifted_beam = round(shifted_beam, 1)
        print(f" -> Main beam shifted from {MAIN_BEAM_DIRECTION}º to {shifted_beam}º.")
        execute_independent_null_steering(shifted_beam, float(interferer_angle))
    else:
        execute_independent_null_steering(MAIN_BEAM_DIRECTION, float(interferer_angle))

    # Verificar a atenuação obtida após o Null Steering
    print("\nWaiting for hardware stabilization...")
    time.sleep(0.5)
    rssi_after_null = get_latest_rssi()
    total_attenuation = peak_rssi - rssi_after_null

    print("\n==============================================")
    print("                VERIFICATION                  ")
    print("==============================================")
    print(f"Original Jammer Power : {peak_rssi:.2f} dB")
    print(f"Power after Null      : {rssi_after_null:.2f} dB")
    print(f"Attenuation achieved  : {total_attenuation:.2f} dB")
    print("==============================================")

    # Registar no CSV
    log_data(cycle_number, tracking_mode, interferer_angle, round(peak_rssi, 2), round(rssi_after_null, 2), round(total_attenuation, 2))

    return interferer_angle, peak_rssi

#############################################
#             MAIN CYCLE
#############################################
print("\n--- Starting continuous Smart Jammer Tracking ---")
print("    Press Ctrl+C to stop.\n")

cycle = 0

try:
    while True:
        cycle += 1

        last_jammer_angle, last_jammer_power = execute_cycle(cycle, last_jammer_angle, last_jammer_power)

        print(f"\n[INFO] Holding Null and listening to GPS for {PAUSE_BETWEEN_CYCLES} seconds...")
        time.sleep(PAUSE_BETWEEN_CYCLES)

except KeyboardInterrupt:
    print("\n\n[STOP] Interrupted by user. Shutting down...")

finally:
    reset_to_main_direction() # Repor o feixe para a direção principal antes de fechar

    # Limpar buffers e fechar ligações
    arduino.close()
    zmq_socket.close()
    context.term()
    print("[STOP] Serial and ZMQ connections closed cleanly.")