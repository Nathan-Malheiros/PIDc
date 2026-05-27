"""
Ball Balancer - Visualizador em tempo real
==========================================
Recebe dados de posicao do ESP32-S3 via porta serial e exibe
vista superior + vista lateral isometrica em tempo real.

Uso:
    python visualizer.py COM11
    python visualizer.py COM11 115200

Teclas:
    C   - Limpa rastro
    ESC - Sai
"""

import sys
import math
import threading
from collections import deque

import pygame
import serial
import serial.tools.list_ports

# ---------------------------------------------------------------------------
# Dimensoes fisicas da tela (mm)
# ---------------------------------------------------------------------------
W_MM  = 187.0
H_MM  = 141.0
CX    = W_MM / 2
CY    = H_MM / 2
BALL_R_MM = 9.5

# ---------------------------------------------------------------------------
# Layout
# ---------------------------------------------------------------------------
SCALE    = 2
PAD      = 18
DIVIDER  = 22
FOOTER_H = 110

PF_W  = int(W_MM * SCALE)
PF_H  = int(H_MM * SCALE)
TOP_X = PAD
TOP_Y = PAD

LAT_X = PAD + PF_W + DIVIDER
LAT_W = 420
LAT_Y = PAD

VIEW_H = PAD + PF_H + PAD
WIN_W  = LAT_X + LAT_W + PAD
WIN_H  = VIEW_H + FOOTER_H

BALL_R_PX = max(4, int(BALL_R_MM * SCALE))
TRAIL_LEN = 120

# ---------------------------------------------------------------------------
# Isometrica
# ---------------------------------------------------------------------------
ISO_S  = 0.90
Z_S    = 8.0
ISO_CX = LAT_X + LAT_W // 2
ISO_CY = LAT_Y + 35 + 150

HZ      = 4.25
MAX_OUT = 0.25
PHI_DEG = [90.0, 210.0, 330.0]

def out_to_tilt_deg(out):
    return math.degrees(math.atan(out / HZ))

def z_surface(x_mm, y_mm, out_x, out_y):
    tx = math.radians(out_to_tilt_deg(out_x))
    ty = math.radians(out_to_tilt_deg(out_y))
    return -(x_mm - CX) * math.sin(tx) - (y_mm - CY) * math.sin(ty)

def motor_rel_height(i, out_x, out_y):
    phi = math.radians(PHI_DEG[i])
    mx = CX + 110 * math.cos(phi)
    my = CY + 110 * math.sin(phi)
    return z_surface(mx, my, out_x, out_y)

def iso(x_mm, y_mm, z_mm):
    x  = (x_mm - CX) * ISO_S
    y  = (y_mm - CY) * ISO_S
    sx = (x - y) * 0.866
    sy = (x + y) * 0.500 - z_mm * Z_S
    return (int(ISO_CX + sx), int(ISO_CY + sy))

# ---------------------------------------------------------------------------
# Cores
# ---------------------------------------------------------------------------
C_BG        = ( 14,  14,  22)
C_PF        = (232, 234, 246)
C_GRID_MIN  = (200, 202, 216)
C_GRID_MAJ  = (152, 154, 178)
C_GRID_LBL  = (108, 110, 138)
C_BORDER    = ( 34,  34,  52)
C_BALL      = (220,  36,  36)
C_BALL_SH   = (255, 192, 175)
C_BALL_SD   = ( 78,   8,   8)
C_TRAIL_A   = ( 22,  42, 195)
C_TRAIL_B   = ( 70, 132, 255)
C_WHITE     = (255, 255, 255)
C_GRAY      = (128, 130, 152)
C_DARK      = ( 44,  44,  64)
C_GREEN     = ( 46, 200,  88)
C_RED       = (220,  60,  60)
C_YELLOW    = (236, 196,  48)
C_CYAN      = ( 62, 210, 218)
C_ORANGE    = (240, 138,  34)
C_LBLUE     = (154, 184, 255)
C_INFO_BG   = ( 18,  18,  30)
C_INFO_BD   = ( 42,  42,  62)
C_ISO_PF    = (224, 228, 244)
C_ISO_GRID  = (180, 184, 208)
C_ISO_EDGE  = ( 70,  74, 118)
C_ISO_PIL   = ( 90, 130, 214)
C_ISO_GND   = ( 34,  38,  56)
C_ISO_SHAD  = ( 24,  24,  40)
C_LAT_BG    = ( 16,  18,  28)
C_LAT_BD    = ( 42,  46,  64)
C_NO_BALL   = (200,  80,  80)
C_GHOST     = (100, 100, 120)   # esfera "fantasma" (ultima posicao conhecida)
MOTOR_C     = [(255, 118, 55), (55, 178, 255), (178, 98, 255)]

# ---------------------------------------------------------------------------
# Utilitarios de desenho
# ---------------------------------------------------------------------------

def lerp_c(a, b, t):
    t = max(0.0, min(1.0, t))
    return tuple(int(a[k] + (b[k] - a[k]) * t) for k in range(3))

def top_px(x_mm, y_mm):
    return (TOP_X + int(x_mm * SCALE), TOP_Y + int(y_mm * SCALE))

def draw_glow(surf, cx, cy, r, color, alpha=80, steps=3):
    for i in range(steps, 0, -1):
        gr = r + i * 5
        a  = alpha // (i + 1)
        s  = pygame.Surface((gr * 2 + 2, gr * 2 + 2), pygame.SRCALPHA)
        pygame.draw.circle(s, (*color, a), (gr + 1, gr + 1), gr)
        surf.blit(s, (cx - gr - 1, cy - gr - 1))

def draw_sphere(surf, cx, cy, r, base_c, shadow_c, hi_c):
    pygame.draw.circle(surf, shadow_c, (cx + 2, cy + 3), r)
    pygame.draw.circle(surf, base_c,   (cx,     cy),     r)
    rim_c = lerp_c(base_c, (8, 0, 0), 0.45)
    pygame.draw.circle(surf, rim_c, (cx, cy), r, max(1, r // 4))
    hx = cx - r // 3
    hy = cy - r // 3
    pygame.draw.circle(surf, hi_c,           (hx,     hy),     max(2, r // 3))
    pygame.draw.circle(surf, (255, 252, 250), (hx - 2, hy - 2), max(1, r // 6))

# ---------------------------------------------------------------------------
# Vista superior
# ---------------------------------------------------------------------------

def draw_top_view(surf, ball_xy, last_ball, trail_snap, font_sm):
    pygame.draw.rect(surf, C_PF, (TOP_X, TOP_Y, PF_W, PF_H))

    for x in range(0, int(W_MM) + 1, 10):
        px  = TOP_X + int(x * SCALE)
        maj = (x % 50 == 0)
        pygame.draw.line(surf, C_GRID_MAJ if maj else C_GRID_MIN,
                         (px, TOP_Y), (px, TOP_Y + PF_H))
        if maj and 0 < x < W_MM:
            l = font_sm.render(str(x), True, C_GRID_LBL)
            surf.blit(l, (px - l.get_width() // 2, TOP_Y + PF_H + 3))

    for y in range(0, int(H_MM) + 1, 10):
        py  = TOP_Y + int(y * SCALE)
        maj = (y % 50 == 0)
        pygame.draw.line(surf, C_GRID_MAJ if maj else C_GRID_MIN,
                         (TOP_X, py), (TOP_X + PF_W, py))
        if maj and 0 < y < H_MM:
            l = font_sm.render(str(y), True, C_GRID_LBL)
            surf.blit(l, (TOP_X - l.get_width() - 3, py - l.get_height() // 2))

    pygame.draw.rect(surf, C_BORDER, (TOP_X, TOP_Y, PF_W, PF_H), 2)

    for i in range(1, len(trail_snap)):
        t = i / max(1, len(trail_snap))
        pygame.draw.line(surf, lerp_c(C_TRAIL_A, C_TRAIL_B, t),
                         top_px(*trail_snap[i - 1]), top_px(*trail_snap[i]),
                         max(1, int(t * 3)))

    if ball_xy:
        bx, by = top_px(*ball_xy)
        draw_sphere(surf, bx, by, BALL_R_PX, C_BALL, C_BALL_SD, C_BALL_SH)
        r = BALL_R_PX + 6
        pygame.draw.line(surf, (255, 60, 60), (bx - r, by), (bx + r, by), 1)
        pygame.draw.line(surf, (255, 60, 60), (bx, by - r), (bx, by + r), 1)
    elif last_ball:
        # Esfera "fantasma" — ultima posicao conhecida
        bx, by = top_px(*last_ball)
        pygame.draw.circle(surf, C_GHOST, (bx, by), BALL_R_PX, 2)
        r = BALL_R_PX + 6
        pygame.draw.line(surf, C_GHOST, (bx - r, by), (bx + r, by), 1)
        pygame.draw.line(surf, C_GHOST, (bx, by - r), (bx, by + r), 1)

    surf.blit(font_sm.render("VISTA SUPERIOR", True, C_LBLUE), (TOP_X + 4, TOP_Y + 4))

# ---------------------------------------------------------------------------
# Vista lateral isometrica
# ---------------------------------------------------------------------------

def draw_lateral_view(surf, ball_xy, last_ball, out_x, out_y, font_sm, font_md):
    panel = pygame.Rect(LAT_X, LAT_Y, LAT_W, PF_H)
    pygame.draw.rect(surf, C_LAT_BG, panel, border_radius=6)
    pygame.draw.rect(surf, C_LAT_BD, panel, 1, border_radius=6)

    corners_mm = [(0, 0), (W_MM, 0), (W_MM, H_MM), (0, H_MM)]
    corners_z  = [z_surface(x, y, out_x, out_y) for x, y in corners_mm]
    corners_2d = [iso(x, y, z) for (x, y), z in zip(corners_mm, corners_z)]

    z_min  = min(corners_z) - BALL_R_MM
    gnd_pts = [iso(x, y, z_min) for x, y in corners_mm]

    pygame.draw.polygon(surf, C_ISO_SHAD, gnd_pts)
    pygame.draw.polygon(surf, C_ISO_GND,  gnd_pts, 1)

    for (x, y), s_pt, g_pt in zip(corners_mm, corners_2d, gnd_pts):
        pygame.draw.line(surf, C_ISO_PIL, g_pt, s_pt, 1)
        pygame.draw.circle(surf, C_ISO_PIL, g_pt, 3)

    pygame.draw.polygon(surf, C_ISO_PF, corners_2d)

    for x_g in range(0, int(W_MM) + 1, 40):
        p1 = iso(x_g, 0,    z_surface(x_g, 0,    out_x, out_y))
        p2 = iso(x_g, H_MM, z_surface(x_g, H_MM, out_x, out_y))
        pygame.draw.line(surf, C_ISO_GRID, p1, p2, 1)
    for y_g in range(0, int(H_MM) + 1, 40):
        p1 = iso(0,    y_g, z_surface(0,    y_g, out_x, out_y))
        p2 = iso(W_MM, y_g, z_surface(W_MM, y_g, out_x, out_y))
        pygame.draw.line(surf, C_ISO_GRID, p1, p2, 1)

    n = len(corners_2d)
    for i in range(n):
        pygame.draw.line(surf, C_ISO_EDGE, corners_2d[i], corners_2d[(i + 1) % n], 2)

    R_mot = 108.0
    for i, phi_d in enumerate(PHI_DEG):
        phi = math.radians(phi_d)
        mx  = CX + R_mot * math.cos(phi)
        my  = CY + R_mot * math.sin(phi)
        ex  = max(0.0, min(W_MM, mx))
        ey  = max(0.0, min(H_MM, my))
        ez  = z_surface(ex, ey, out_x, out_y)
        mp  = iso(mx, my, z_min)
        ep  = iso(ex, ey, ez)
        pygame.draw.line(surf, lerp_c(MOTOR_C[i], C_ISO_GND, 0.5), mp, ep, 2)
        pygame.draw.circle(surf, MOTOR_C[i], mp, 6)
        pygame.draw.circle(surf, C_WHITE,    mp, 2)
        lbl = font_sm.render(["A", "B", "C"][i], True, MOTOR_C[i])
        surf.blit(lbl, (mp[0] + 7, mp[1] - 7))

    draw_xy = ball_xy if ball_xy else last_ball
    if draw_xy:
        bx_mm, by_mm = draw_xy
        bz_surf = z_surface(bx_mm, by_mm, out_x, out_y)
        shad_pt = iso(bx_mm, by_mm, bz_surf)
        sr      = max(4, int(BALL_R_MM * ISO_S))
        shad_s  = pygame.Surface((sr * 2 + 2, sr + 2), pygame.SRCALPHA)
        pygame.draw.ellipse(shad_s, (0, 0, 0, 110), (0, 0, sr * 2, sr))
        surf.blit(shad_s, (shad_pt[0] - sr, shad_pt[1] - sr // 2))
        bz_ball = bz_surf + BALL_R_MM
        biso    = iso(bx_mm, by_mm, bz_ball)
        br = max(6, int(BALL_R_MM * ISO_S))
        if ball_xy:
            pygame.draw.line(surf, (180, 60, 60), shad_pt, biso, 1)
            draw_sphere(surf, biso[0], biso[1], br, C_BALL, C_BALL_SD, C_BALL_SH)
        else:
            pygame.draw.circle(surf, C_GHOST, biso, br, 2)

    surf.blit(font_md.render("VISTA LATERAL  (isometrica)", True, C_LBLUE),
              (LAT_X + 8, LAT_Y + 6))
    tx_deg = out_to_tilt_deg(out_x)
    ty_deg = out_to_tilt_deg(out_y)
    surf.blit(font_sm.render(f"Tilt X={tx_deg:+.2f}   Tilt Y={ty_deg:+.2f}", True, C_YELLOW),
              (LAT_X + 8, LAT_Y + PF_H - 18))

# ---------------------------------------------------------------------------
# Rodape
# ---------------------------------------------------------------------------

def draw_footer(surf, ball_xy, last_ball, status, font_sm, font_md):
    iy   = VIEW_H + 6
    rect = pygame.Rect(PAD, iy, WIN_W - 2 * PAD, FOOTER_H - 12)
    pygame.draw.rect(surf, C_INFO_BG, rect, border_radius=8)
    pygame.draw.rect(surf, C_INFO_BD, rect, 1, border_radius=8)

    title = font_md.render("Ball Balancer  |  Rastreador de Posicao", True, C_WHITE)
    surf.blit(title, (PAD + 10, iy + 8))

    if ball_xy:
        pos_t = font_sm.render(
            f"[LIVE]  X = {ball_xy[0]:6.1f} mm    Y = {ball_xy[1]:6.1f} mm",
            True, C_GREEN)
    elif last_ball:
        pos_t = font_sm.render(
            f"[LAST]  X = {last_ball[0]:6.1f} mm    Y = {last_ball[1]:6.1f} mm",
            True, C_GHOST)
    else:
        pos_t = font_sm.render("Sem contato", True, C_NO_BALL)
    surf.blit(pos_t, (PAD + 10, iy + 32))

    st_t = font_sm.render(f"Serial: {status}", True, C_LBLUE)
    surf.blit(st_t, (PAD + 10, iy + 52))

    dim_t = font_sm.render(f"Tela: {int(W_MM)} x {int(H_MM)} mm", True, C_GRAY)
    surf.blit(dim_t, (PAD + 10, iy + 72))

    help_t = font_sm.render("[C] Limpar rastro    [ESC] Sair", True, C_GRAY)
    surf.blit(help_t, (WIN_W - help_t.get_width() - PAD - 10, iy + 72))

# ---------------------------------------------------------------------------
# Classe principal
# ---------------------------------------------------------------------------

class BallVisualizer:
    def __init__(self, port, baud=115200):
        self.port      = port
        self.baud      = baud
        self.ball      = None
        self.last_ball = None   # ultima posicao valida recebida
        self.trail     = deque(maxlen=TRAIL_LEN)
        self.status    = "Conectando..."
        self.running   = True
        self._lock     = threading.Lock()

    def _serial_reader(self):
        try:
            ser = serial.Serial(self.port, self.baud, timeout=1)
            with self._lock:
                self.status = f"Conectado: {self.port} @ {self.baud} baud"
            while self.running:
                try:
                    raw  = ser.readline()
                    if not raw:
                        continue
                    line = raw.decode("utf-8", errors="ignore").strip()
                    if line == "NONE":
                        with self._lock:
                            self.ball = None
                            # last_ball mantido intencionalmente
                    elif line.startswith("X:"):
                        parts = line.split(",")
                        x = float(parts[0].split(":")[1])
                        y = float(parts[1].split(":")[1])
                        with self._lock:
                            self.ball      = (x, y)
                            self.last_ball = (x, y)
                            self.trail.append((x, y))
                except (ValueError, IndexError, UnicodeDecodeError):
                    pass
            ser.close()
        except serial.SerialException as e:
            with self._lock:
                self.status = f"Erro serial: {e}"

    def run(self):
        pygame.init()
        screen  = pygame.display.set_mode((WIN_W, WIN_H))
        pygame.display.set_caption("Ball Balancer - Visualizador em Tempo Real")
        clock   = pygame.time.Clock()
        font_sm = pygame.font.SysFont("monospace", 13)
        font_md = pygame.font.SysFont("monospace", 15, bold=True)

        t = threading.Thread(target=self._serial_reader, daemon=True)
        t.start()

        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        self.running = False
                    elif event.key == pygame.K_c:
                        with self._lock:
                            self.trail.clear()
                            self.last_ball = None

            with self._lock:
                ball       = self.ball
                last_ball  = self.last_ball
                trail_snap = list(self.trail)
                status     = self.status

            screen.fill(C_BG)
            draw_top_view(screen, ball, last_ball, trail_snap, font_sm)
            draw_lateral_view(screen, ball, last_ball, 0.0, 0.0, font_sm, font_md)
            draw_footer(screen, ball, last_ball, status, font_sm, font_md)
            pygame.display.flip()
            clock.tick(60)

        pygame.quit()


# ---------------------------------------------------------------------------
# Ponto de entrada
# ---------------------------------------------------------------------------


def list_ports():
    return [p.device for p in serial.tools.list_ports.comports()]

DEFAULT_PORT = "COM11"

if __name__ == "__main__":
    if len(sys.argv) < 2:
        ports = list_ports()
        if DEFAULT_PORT in ports:
            port = DEFAULT_PORT
            print(f"Porta nao informada — usando padrao: {port}")
        elif ports:
            port = ports[0]
            print(f"Porta nao informada — usando primeira disponivel: {port}")
        else:
            print("Nenhuma porta serial encontrada.")
            sys.exit(1)
    else:
        port = sys.argv[1]

    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    BallVisualizer(port, baud).run()
