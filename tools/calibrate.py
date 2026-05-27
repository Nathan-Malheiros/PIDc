"""
Ferramenta de calibracao visual da tela resistiva
==================================================
- Janela pygame com alvo visual em cada ponto
- Estabilizacao de 0.4s + coleta de 1.2s com mediana (robusto a ruido)
- Indicador de estabilidade em tempo real
- Ao terminar: atualiza touch_screen.h, recompila e reflasha automaticamente

Uso:
    python calibrate.py COM11
    python calibrate.py COM11 115200
"""

import sys
import time
import threading
import re
import subprocess
import os
import statistics
import collections
import pygame
import serial
import serial.tools.list_ports

# Dimensoes fisicas da tela (mm)
SCREEN_W_MM = 187
SCREEN_H_MM = 141

SCALE    = 4
WIN_W    = int(SCREEN_W_MM * SCALE)
WIN_H    = int(SCREEN_H_MM * SCALE)
FOOTER_H = 120

# Paleta
BG       = (25, 25, 35)
GRID     = (45, 45, 60)
BORDER   = (80, 80, 110)
TARGET_C = (255, 70, 70)
DONE_C   = (60, 210, 90)
TEXT_C   = (210, 210, 220)
HINT_C   = (150, 150, 80)
RAW_C    = (90, 195, 255)
YELLOW   = (255, 200, 55)
RED_C    = (220, 60, 60)
ORANGE_C = (220, 140, 40)

STAB_GOOD   = 15   # desvio padrao abaixo disso = estavel (verde)
STAB_OK     = 40   # abaixo = aceitavel (amarelo), acima = ruidoso (vermelho)

STABILIZE_S = 0.4  # segundos ignorados apos pressionar Espaco (transitorio)
COLLECT_S   = 1.2  # segundos de coleta efetiva

POINTS = [
    ("tl", "CANTO SUPERIOR ESQUERDO",  0.05, 0.05),
    ("tr", "CANTO SUPERIOR DIREITO",   0.95, 0.05),
    ("bl", "CANTO INFERIOR ESQUERDO",  0.05, 0.95),
    ("br", "CANTO INFERIOR DIREITO",   0.95, 0.95),
    ("cx", "CENTRO",                   0.50, 0.50),
]

TOOLS_DIR   = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(TOOLS_DIR)
HEADER_PATH = os.path.join(PROJECT_DIR, "main", "touch_screen.h")
IDF_EXPORT  = r"C:\esp\v6.0.1\esp-idf\export.ps1"

# ---------------------------------------------------------------------------
# Leitura serial em background
# ---------------------------------------------------------------------------

latest_raw = None
_lock      = threading.Lock()
_running   = True

def serial_reader(ser):
    global latest_raw, _running
    while _running:
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line.startswith("CAL:"):
                parts = line[4:].split(",")
                with _lock:
                    latest_raw = (int(parts[0]), int(parts[1]))
        except Exception:
            pass

def get_raw():
    with _lock:
        return latest_raw

# ---------------------------------------------------------------------------
# Helpers de desenho
# ---------------------------------------------------------------------------

def mm_to_px(xn, yn):
    return (int(xn * WIN_W), int(yn * WIN_H))

def draw_screen_area(surf, font_sm=None):
    pygame.draw.rect(surf, (15, 15, 22), (0, 0, WIN_W, WIN_H))
    for gx in range(0, WIN_W + 1, WIN_W // 4):
        pygame.draw.line(surf, GRID, (gx, 0), (gx, WIN_H))
    for gy in range(0, WIN_H + 1, WIN_H // 4):
        pygame.draw.line(surf, GRID, (0, gy), (WIN_W, gy))
    pygame.draw.rect(surf, BORDER, (0, 0, WIN_W, WIN_H), 2)

    # Indicador do cabo FPC — lado ESQUERDO da tela
    cable_y0 = WIN_H // 2 - 40
    cable_y1 = WIN_H // 2 + 40
    pygame.draw.rect(surf, (50, 190, 80), (0, cable_y0, 8, cable_y1 - cable_y0), border_radius=3)
    if font_sm:
        lbl = font_sm.render("CABO", True, (50, 190, 80))
        surf.blit(lbl, (10, WIN_H // 2 - lbl.get_height() // 2))

def draw_confirmed(surf, confirmed):
    for key in confirmed:
        for k, lbl, xn, yn in POINTS:
            if k == key:
                px, py = mm_to_px(xn, yn)
                pygame.draw.circle(surf, DONE_C,          (px, py), 11)
                pygame.draw.circle(surf, (255, 255, 255), (px, py), 11, 2)

def draw_target(surf, px, py):
    t = pygame.time.get_ticks()
    r = int(14 + 5 * abs((t % 900) / 450.0 - 1))
    pygame.draw.circle(surf, TARGET_C, (px, py), r, 3)
    pygame.draw.circle(surf, TARGET_C, (px, py), 5)
    pygame.draw.line(surf, TARGET_C, (px - 22, py), (px + 22, py), 2)
    pygame.draw.line(surf, TARGET_C, (px, py - 22), (px, py + 22), 2)

def stability_info(recent):
    """Retorna (texto, cor) com base no desvio padrao das ultimas leituras."""
    if len(recent) < 5:
        return "...", HINT_C
    std_x = statistics.stdev(s[0] for s in recent)
    std_y = statistics.stdev(s[1] for s in recent)
    std   = max(std_x, std_y)
    if std < STAB_GOOD:
        return f"ESTAVEL  std={std:.0f}", DONE_C
    elif std < STAB_OK:
        return f"OK       std={std:.0f}", YELLOW
    else:
        return f"RUIDOSO  std={std:.0f}", RED_C

def draw_footer(surf, label, step, total, raw, recent,
                collecting, stabilizing, progress,
                font_lg, font_md, font_sm):
    fy = WIN_H
    pygame.draw.rect(surf, (18, 18, 28), (0, fy, WIN_W, FOOTER_H))
    pygame.draw.line(surf, BORDER, (0, fy), (WIN_W, fy), 1)

    # Linha 1 — label do ponto atual
    surf.blit(font_lg.render(label, True, YELLOW), (12, fy + 6))

    # Linha 2 — valores raw + estabilidade
    raw_txt = f"raw_x={raw[0]:4d}   raw_y={raw[1]:4d}" if raw else "aguardando leitura..."
    surf.blit(font_md.render(raw_txt, True, RAW_C), (12, fy + 34))

    stab_txt, stab_col = stability_info(recent)
    s = font_sm.render(stab_txt, True, stab_col)
    surf.blit(s, (WIN_W - s.get_width() - 12, fy + 37))

    # Linha 3 — barra de progresso ou dica
    if collecting or stabilizing:
        bar_x, bar_y, bar_w, bar_h = 12, fy + 62, WIN_W - 24, 10
        pygame.draw.rect(surf, (40, 40, 55), (bar_x, bar_y, bar_w, bar_h))
        if progress > 0:
            pygame.draw.rect(surf, DONE_C, (bar_x, bar_y, int(bar_w * progress), bar_h))
        pygame.draw.rect(surf, BORDER, (bar_x, bar_y, bar_w, bar_h), 1)
        msg = "Estabilizando..." if stabilizing else f"Coletando {int(progress * 100)}%"
        surf.blit(font_sm.render(msg, True, TEXT_C), (12, fy + 78))
    else:
        surf.blit(font_sm.render(
            "Posicione a bola no alvo X e pressione ESPACO", True, HINT_C), (12, fy + 66))

    # Linha 4 — contador de pontos
    s = font_sm.render(f"ponto {step + 1} / {total}", True, TEXT_C)
    surf.blit(s, (WIN_W - s.get_width() - 12, fy + 100))

# ---------------------------------------------------------------------------
# Atualiza touch_screen.h
# ---------------------------------------------------------------------------

def update_header(x_min, x_max, y_min, y_max, x_inv, y_inv, swap):
    with open(HEADER_PATH, "r", encoding="utf-8") as f:
        src = f.read()
    for define, value in [
        ("TOUCH_X_RAW_MIN", str(x_min)),
        ("TOUCH_X_RAW_MAX", str(x_max)),
        ("TOUCH_Y_RAW_MIN", str(y_min)),
        ("TOUCH_Y_RAW_MAX", str(y_max)),
        ("TOUCH_X_INVERT",  str(x_inv)),
        ("TOUCH_Y_INVERT",  str(y_inv)),
        ("TOUCH_SWAP_XY",   str(swap)),
    ]:
        src = re.sub(
            rf"(#define\s+{define}\s+)\S+",
            lambda m, v=value: m.group(1) + v,
            src,
        )
    with open(HEADER_PATH, "w", encoding="utf-8") as f:
        f.write(src)

# ---------------------------------------------------------------------------
# Build + flash
# ---------------------------------------------------------------------------

def build_and_flash(port, screen, font_md, font_sm):
    for msg, args in [
        ("Compilando firmware...", ["build"]),
        (f"Gravando em {port}...",  ["-p", port, "flash"]),
    ]:
        screen.fill(BG)
        s = font_md.render(msg, True, YELLOW)
        screen.blit(s, (WIN_W // 2 - s.get_width() // 2, (WIN_H + FOOTER_H) // 2 - 20))
        s2 = font_sm.render("Aguarde...", True, HINT_C)
        screen.blit(s2, (WIN_W // 2 - s2.get_width() // 2, (WIN_H + FOOTER_H) // 2 + 16))
        pygame.display.flip()
        pygame.event.pump()

        cmd = (f'. "{IDF_EXPORT}"; '
               f'Set-Location "{PROJECT_DIR}"; '
               f'idf.py {" ".join(args)}')
        result = subprocess.run(
            ["powershell", "-NoProfile", "-Command", cmd],
            capture_output=True, text=True
        )
        if result.returncode != 0:
            return False, result.stdout + result.stderr
    return True, ""

# ---------------------------------------------------------------------------
# Logica principal de calibracao
# ---------------------------------------------------------------------------

def calibrate(ser, port):
    global _running

    threading.Thread(target=serial_reader, args=(ser,), daemon=True).start()

    pygame.init()
    screen  = pygame.display.set_mode((WIN_W, WIN_H + FOOTER_H))
    pygame.display.set_caption("Calibracao — Ball Balancer")
    font_lg = pygame.font.SysFont("consolas", 20, bold=True)
    font_md = pygame.font.SysFont("consolas", 16)
    font_sm = pygame.font.SysFont("consolas", 13)
    clock   = pygame.time.Clock()

    # --- Intro ---
    intro = True
    while intro:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                _running = False; pygame.quit(); return
            if ev.type == pygame.KEYDOWN and ev.key in (pygame.K_SPACE, pygame.K_RETURN):
                intro = False
        screen.fill(BG)
        lines = [
            ("CALIBRACAO DA TELA RESISTIVA", YELLOW),
            ("", None),
            ("ORIENTACAO DA TELA:", (100, 220, 100)),
            ("  Segure a tela com o cabo FPC para a ESQUERDA.", TEXT_C),
            ("  O marcador verde na borda esquerda indica o cabo.", TEXT_C),
            ("", None),
            ("Coloque a bola sobre o alvo  X  em cada ponto.", TEXT_C),
            ("Pressione ESPACO para registrar.", TEXT_C),
            ("", None),
            ("O indicador ESTAVEL/RUIDOSO mostra a qualidade", HINT_C),
            ("da leitura. Espere ficar ESTAVEL antes de registrar.", HINT_C),
            ("", None),
            ("Apos os 5 pontos: firmware atualizado e gravado.", DONE_C),
            ("", None),
            ("Pressione ESPACO para comecar...", (100, 220, 100)),
        ]
        total_h = len(lines) * 28
        y0 = (WIN_H + FOOTER_H) // 2 - total_h // 2
        for i, (txt, col) in enumerate(lines):
            if col:
                s = font_md.render(txt, True, col)
                screen.blit(s, (WIN_W // 2 - s.get_width() // 2, y0 + i * 28))
        pygame.display.flip()
        clock.tick(30)

    # --- Coleta ---
    confirmed = {}
    recent    = collections.deque(maxlen=30)

    for step, (key, label, xn, yn) in enumerate(POINTS):
        px, py      = mm_to_px(xn, yn)
        phase       = "wait"   # wait | stabilize | collect | done
        samples     = []
        phase_start = 0.0

        while phase != "done":
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    _running = False; pygame.quit(); return
                if ev.type == pygame.KEYDOWN:
                    if ev.key == pygame.K_ESCAPE:
                        _running = False; pygame.quit(); return
                    if ev.key in (pygame.K_SPACE, pygame.K_RETURN) and phase == "wait":
                        phase       = "stabilize"
                        phase_start = time.time()
                        samples     = []

            r = get_raw()
            if r:
                recent.append(r)

            now = time.time()

            if phase == "stabilize":
                if now - phase_start >= STABILIZE_S:
                    phase       = "collect"
                    phase_start = now
                    samples     = []

            elif phase == "collect":
                if r:
                    samples.append(r)
                elapsed  = now - phase_start
                progress = min(1.0, elapsed / COLLECT_S)
                if elapsed >= COLLECT_S:
                    if len(samples) >= 5:
                        # Mediana separada por eixo — robusta a outliers
                        rxs = sorted(s[0] for s in samples)
                        rys = sorted(s[1] for s in samples)
                        confirmed[key] = (rxs[len(rxs) // 2], rys[len(rys) // 2])
                        phase = "done"
                    else:
                        phase = "wait"  # sem amostras, tenta de novo

            # Progresso para barra
            if phase == "stabilize":
                progress  = 0.0
                stab_flag = True
                coll_flag = False
            elif phase == "collect":
                elapsed   = now - phase_start
                progress  = min(1.0, elapsed / COLLECT_S)
                stab_flag = False
                coll_flag = True
            else:
                progress  = 0.0
                stab_flag = False
                coll_flag = False

            screen.fill(BG)
            draw_screen_area(screen, font_sm)
            draw_confirmed(screen, confirmed)
            draw_target(screen, px, py)
            draw_footer(screen, label, step, len(POINTS), get_raw(), recent,
                        coll_flag, stab_flag, progress, font_lg, font_md, font_sm)
            pygame.display.flip()
            clock.tick(30)

    _running = False

    # --- Calculos ---
    tl = confirmed["tl"]; tr = confirmed["tr"]
    bl = confirmed["bl"]; br = confirmed["br"]
    cx = confirmed["cx"]

    x_all = [tl[0], tr[0], bl[0], br[0]]
    y_all = [tl[1], tr[1], bl[1], br[1]]
    x_min, x_max = min(x_all), max(x_all)
    y_min, y_max = min(y_all), max(y_all)

    x_left_avg   = (tl[0] + bl[0]) / 2
    x_right_avg  = (tr[0] + br[0]) / 2
    x_invert     = 1 if x_right_avg < x_left_avg else 0

    y_top_avg    = (tl[1] + tr[1]) / 2
    y_bottom_avg = (bl[1] + br[1]) / 2
    y_invert     = 1 if y_bottom_avg < y_top_avg else 0

    dx_horiz = abs(tr[0] - tl[0])
    dy_horiz = abs(tr[1] - tl[1])
    swap_xy  = 1 if dy_horiz > dx_horiz else 0

    margin_x  = max(10, int((x_max - x_min) * 0.05))
    margin_y  = max(10, int((y_max - y_min) * 0.05))
    x_min_cal = max(0,    x_min - margin_x)
    x_max_cal = min(4095, x_max + margin_x)
    y_min_cal = max(0,    y_min - margin_y)
    y_max_cal = min(4095, y_max + margin_y)

    # Salva calibration_result.txt
    out_path = os.path.join(PROJECT_DIR, "calibration_result.txt")
    with open(out_path, "w") as f:
        f.write(f"#define TOUCH_X_RAW_MIN    {x_min_cal}\n")
        f.write(f"#define TOUCH_X_RAW_MAX    {x_max_cal}\n")
        f.write(f"#define TOUCH_Y_RAW_MIN    {y_min_cal}\n")
        f.write(f"#define TOUCH_Y_RAW_MAX    {y_max_cal}\n\n")
        f.write(f"#define TOUCH_X_INVERT     {x_invert}\n")
        f.write(f"#define TOUCH_Y_INVERT     {y_invert}\n")
        f.write(f"#define TOUCH_SWAP_XY      {swap_xy}\n")

    # Atualiza touch_screen.h
    try:
        update_header(x_min_cal, x_max_cal, y_min_cal, y_max_cal, x_invert, y_invert, swap_xy)
        header_ok = True
    except Exception as e:
        header_ok = False
        header_err = str(e)

    # Fecha serial antes do flash
    ser.close()

    # Build + flash
    flash_ok, flash_err = build_and_flash(port, screen, font_md, font_sm)

    # --- Tela de resultado ---
    result_lines = [
        ("CALIBRACAO CONCLUIDA!", YELLOW),
        ("", None),
        (f"  Top-Left     raw_x={tl[0]:4d}  raw_y={tl[1]:4d}", TEXT_C),
        (f"  Top-Right    raw_x={tr[0]:4d}  raw_y={tr[1]:4d}", TEXT_C),
        (f"  Bottom-Left  raw_x={bl[0]:4d}  raw_y={bl[1]:4d}", TEXT_C),
        (f"  Bottom-Right raw_x={br[0]:4d}  raw_y={br[1]:4d}", TEXT_C),
        ("", None),
        (f"  X_RAW: {x_min_cal} ~ {x_max_cal}   Y_RAW: {y_min_cal} ~ {y_max_cal}", RAW_C),
        (f"  X_INVERT={x_invert}   Y_INVERT={y_invert}   SWAP_XY={swap_xy}", RAW_C),
        ("", None),
        ("  touch_screen.h  " + ("atualizado" if header_ok else "ERRO: " + header_err),
         DONE_C if header_ok else RED_C),
        ("  Firmware  " + ("gravado!" if flash_ok else "ERRO — veja o terminal"),
         DONE_C if flash_ok else RED_C),
        ("", None),
        ("  Abrindo visualizador..." if flash_ok else "  Pressione ESPACO ou feche para sair.", DONE_C if flash_ok else HINT_C),
    ]

    if not flash_ok:
        print("\n--- ERRO BUILD/FLASH ---\n", flash_err)

    # Mostra resultado por 2 segundos e abre o visualizador
    deadline = time.time() + 2.0
    while time.time() < deadline:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                pygame.quit(); return
            if ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE:
                pygame.quit(); return
        screen.fill(BG)
        y0 = 24
        for txt, col in result_lines:
            if col:
                s = font_md.render(txt, True, col)
                screen.blit(s, (30, y0))
            y0 += 26
        pygame.display.flip()
        clock.tick(30)

    pygame.quit()

    if flash_ok:
        time.sleep(2.5)  # aguarda ESP32 reiniciar apos o flash
        visualizer = os.path.join(TOOLS_DIR, "visualizer.py")
        subprocess.Popen([sys.executable, visualizer, port],
                         creationflags=subprocess.CREATE_NEW_CONSOLE)


# ---------------------------------------------------------------------------
# Ponto de entrada
# ---------------------------------------------------------------------------

def list_ports():
    return [p.device for p in serial.tools.list_ports.comports()]

if __name__ == "__main__":
    if len(sys.argv) < 2:
        ports = list_ports()
        if "COM11" in ports:
            port = "COM11"
        elif ports:
            port = ports[0]
        else:
            print("Nenhuma porta serial encontrada.")
            sys.exit(1)
        print(f"Porta nao informada — usando: {port}")
    else:
        port = sys.argv[1]

    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    try:
        ser = serial.Serial(port, baud, timeout=1)
        print(f"Conectado: {port} @ {baud} baud")
        time.sleep(1.5)
        calibrate(ser, port)
        if ser.is_open:
            ser.close()
    except serial.SerialException as e:
        print(f"Erro serial: {e}")
        sys.exit(1)
