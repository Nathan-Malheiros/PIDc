"""
Ball Balancer 3RPS - Simulacao com Vista Superior e Vista Lateral
=================================================================
PID (identico ao Ball_Balancing.ino, escalado para mm):
  integr  += error + errorPrev   (soma trapezio cumulativa, sem dt)
  deriv    = error - errorPrev   (diferenca atrasada, sem dt)
  out      = kp*e + ki*integr + kd*deriv  =>  clamp(-0.25, +0.25)

Controles:
  P           Alterna Manual <-> PID
  0           Setpoint no centro
  1-5         Padroes: circulo / quadrado / figura-8 / linha / senoidal
  Clique      Define setpoint (modo PID)
  Setas       Inclina manualmente
  R           Reseta esfera
  Espaco      Impulso aleatorio
  T           Rastro on/off
  Q/A  W/S  E/D   Ajusta Kp / Ki / Kd
  ESC         Sair
"""

import pygame
import math
import random
from collections import deque

# ---------------------------------------------------------------------------
# Dimensoes fisicas (mm)
# ---------------------------------------------------------------------------
W_MM      = 187.0
H_MM      = 141.0
CX        = W_MM / 2
CY        = H_MM / 2
BALL_R_MM = 9.5

FALL_DURATION = 2.2   # segundos ate auto-reset
WARN_DIST_MM  = 20.0  # mm da borda para acionar aviso

# ---------------------------------------------------------------------------
# Parametros da maquina
# ---------------------------------------------------------------------------
HZ      = 4.25
MAX_OUT = 0.25
PHI_DEG = [90.0, 210.0, 330.0]

def out_to_tilt_deg(out: float) -> float:
    return math.degrees(math.atan(out / HZ))

def tilt_to_accel(tilt_deg: float) -> float:
    return (5.0 / 7.0) * 9800.0 * math.sin(math.radians(tilt_deg))

def z_surface(x_mm: float, y_mm: float, out_x: float, out_y: float) -> float:
    tx = math.radians(out_to_tilt_deg(out_x))
    ty = math.radians(out_to_tilt_deg(out_y))
    return -(x_mm - CX) * math.sin(tx) - (y_mm - CY) * math.sin(ty)

def motor_rel_height(i: int, out_x: float, out_y: float) -> float:
    phi = math.radians(PHI_DEG[i])
    mx = CX + 110 * math.cos(phi)
    my = CY + 110 * math.sin(phi)
    return z_surface(mx, my, out_x, out_y)

# ---------------------------------------------------------------------------
# Fisica
# ---------------------------------------------------------------------------
FRICTION  = 0.986
GRAVITY_Z = 9800.0
FPS       = 60
DT        = 1.0 / FPS

# ---------------------------------------------------------------------------
# Layout
# ---------------------------------------------------------------------------
SCALE   = 2
PAD     = 18
DIVIDER = 22
INFO_H  = 158

PF_W = int(W_MM * SCALE)
PF_H = int(H_MM * SCALE)
TOP_X = PAD
TOP_Y = PAD

LAT_X = PAD + PF_W + DIVIDER
LAT_W = 420
LAT_Y = PAD

VIEW_H = PAD + PF_H + PAD
WIN_W  = LAT_X + LAT_W + PAD
WIN_H  = VIEW_H + INFO_H

BALL_R_PX = max(4, int(BALL_R_MM * SCALE))

# ---------------------------------------------------------------------------
# Projecao isometrica
# ---------------------------------------------------------------------------
ISO_S  = 0.90
Z_S    = 8.0
ISO_CX = LAT_X + LAT_W // 2
ISO_CY = LAT_Y + 35 + 150

def iso(x_mm: float, y_mm: float, z_mm: float):
    x  = (x_mm - CX) * ISO_S
    y  = (y_mm - CY) * ISO_S
    sx = (x - y) * 0.866
    sy = (x + y) * 0.500 - z_mm * Z_S
    return (int(ISO_CX + sx), int(ISO_CY + sy))

# ---------------------------------------------------------------------------
# Cores
# ---------------------------------------------------------------------------
C_BG       = ( 14,  14,  22)
C_PF       = (232, 234, 246)
C_GRID_MIN = (200, 202, 216)
C_GRID_MAJ = (152, 154, 178)
C_GRID_LBL = (108, 110, 138)
C_BORDER   = ( 34,  34,  52)
C_BALL     = (220,  36,  36)
C_BALL_SH  = (255, 192, 175)
C_BALL_SD  = ( 78,   8,   8)
C_SETPT    = ( 44, 215,  90)
C_TRAIL_A  = ( 22,  42, 195)
C_TRAIL_B  = ( 70, 132, 255)
C_INFO_BG  = ( 18,  18,  30)
C_INFO_BD  = ( 42,  42,  62)
C_WHITE    = (255, 255, 255)
C_GRAY     = (128, 130, 152)
C_DARK     = ( 44,  44,  64)
C_GREEN    = ( 46, 200,  88)
C_RED      = (220,  60,  60)
C_YELLOW   = (236, 196,  48)
C_CYAN     = ( 62, 210, 218)
C_ORANGE   = (240, 138,  34)
C_LBLUE    = (154, 184, 255)
C_WARN     = (240,  98,  18)
C_DANGER   = (220,  28,  28)
C_PID_MODE = ( 46, 200,  88)
C_MAN_MODE = (236, 196,  48)
MOTOR_C    = [(255, 118, 55), (55, 178, 255), (178, 98, 255)]

C_ISO_PF     = (224, 228, 244)
C_ISO_GRID   = (180, 184, 208)
C_ISO_EDGE   = ( 70,  74, 118)
C_ISO_PILLAR = ( 90, 130, 214)
C_ISO_GND    = ( 34,  38,  56)
C_ISO_SHADOW = ( 24,  24,  40)
C_LAT_BG     = ( 16,  18,  28)
C_LAT_BD     = ( 42,  46,  64)

# ---------------------------------------------------------------------------
# PID
# ---------------------------------------------------------------------------
class PID:
    KP_DEF = 2.0e-3
    KI_DEF = 8.0e-6
    KD_DEF = 3.5e-2

    def __init__(self):
        self.kp = self.KP_DEF
        self.ki = self.KI_DEF
        self.kd = self.KD_DEF
        self.reset()

    def reset(self):
        self.integr     = 0.0
        self.error      = 0.0
        self.error_prev = 0.0
        self.deriv      = 0.0
        self.out        = 0.0

    def step(self, sp: float, measured: float) -> float:
        self.error_prev  = self.error
        self.error       = sp - measured
        self.integr     += self.error + self.error_prev
        self.integr      = max(-5e4, min(5e4, self.integr))
        self.deriv       = self.error - self.error_prev
        if math.isnan(self.deriv) or math.isinf(self.deriv):
            self.deriv = 0.0
        self.out = (self.kp * self.error
                    + self.ki * self.integr
                    + self.kd * self.deriv)
        self.out = max(-MAX_OUT, min(MAX_OUT, self.out))
        return self.out

# ---------------------------------------------------------------------------
# Esfera
# ---------------------------------------------------------------------------
class Ball:
    def __init__(self):
        self.reset()

    def reset(self):
        self.x  = CX;  self.y  = CY
        self.vx = 0.0; self.vy = 0.0
        self.fallen    = False
        self.fall_z    = 0.0
        self.fall_vz   = 0.0
        self.fall_time = 0.0

    def impulse(self):
        self.vx += random.uniform(-260, 260)
        self.vy += random.uniform(-260, 260)

    def edge_dist(self) -> float:
        """Distancia do centro da bola ate a borda mais proxima, menos o raio."""
        return min(self.x - BALL_R_MM,
                   W_MM - self.x - BALL_R_MM,
                   self.y - BALL_R_MM,
                   H_MM - self.y - BALL_R_MM)

    def update(self, out_x: float, out_y: float):
        if self.fallen:
            self.fall_vz  -= GRAVITY_Z * DT
            self.fall_z   += self.fall_vz * DT
            self.fall_time += DT
            return

        ax = tilt_to_accel(out_to_tilt_deg(out_x))
        ay = tilt_to_accel(out_to_tilt_deg(out_y))
        self.vx = (self.vx + ax * DT) * FRICTION
        self.vy = (self.vy + ay * DT) * FRICTION
        self.x  += self.vx * DT
        self.y  += self.vy * DT

        if self.x < 0 or self.x > W_MM or self.y < 0 or self.y > H_MM:
            self.fallen  = True
            bx_c = max(0.0, min(W_MM, self.x))
            by_c = max(0.0, min(H_MM, self.y))
            self.fall_z  = z_surface(bx_c, by_c, out_x, out_y) + BALL_R_MM
            self.fall_vz = 0.0
            self.fall_time = 0.0

# ---------------------------------------------------------------------------
# Padroes de movimento
# ---------------------------------------------------------------------------
class Pattern:
    HOLD = 0; CIRCLE = 1; SQUARE = 2; FIGURE8 = 3; LINE = 4; SINUS = 5
    NAMES = {0:"Hold", 1:"Circulo", 2:"Quadrado", 3:"Figura-8", 4:"Linha", 5:"Senoidal"}

    def __init__(self):
        self.mode = self.HOLD
        self.t    = 0.0

    def reset(self): self.t = 0.0

    def update(self): self.t += DT

    def setpoint(self):
        t = self.t
        if   self.mode == self.HOLD:    return CX, CY
        elif self.mode == self.CIRCLE:
            r = 52.0
            return CX + r * math.cos(0.04 * t), CY + r * math.sin(0.04 * t)
        elif self.mode == self.SQUARE:
            s = 48.0; p = 6.0; f = (t % p) / p
            if   f < 0.25: return CX + s, CY + s * (f / 0.25 * 2 - 1)
            elif f < 0.50: return CX + s * (1 - (f - 0.25) / 0.25 * 2), CY + s
            elif f < 0.75: return CX - s, CY + s * (1 - (f - 0.50) / 0.25 * 2)
            else:          return CX - s * (1 - (f - 0.75) / 0.25 * 2), CY - s
        elif self.mode == self.FIGURE8:
            r = 48.0; th = 0.04 * t
            sc = r * (2.0 / (3.0 - math.cos(2 * th)))
            return CX + sc * math.cos(th), CY + sc * math.sin(2 * th) / 1.5
        elif self.mode == self.LINE:
            return CX + 62.0 * math.sin(2 * math.pi * 0.5 * t), CY
        elif self.mode == self.SINUS:
            x = CX + 62.0 * math.sin(2 * math.pi * 0.1 * t)
            return x, CY + 48.0 * math.sin((x - CX) / 28.0)
        return CX, CY

# ---------------------------------------------------------------------------
# Utilitarios
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

def draw_sphere(surf, cx, cy, r, base_c, shadow_c, hi_c, speed=0.0):
    # Glow de velocidade
    if speed > 80:
        g_alpha = min(110, int((speed - 80) / 350 * 110))
        g_r = r + min(7, int(speed / 130))
        gs = pygame.Surface((g_r * 2 + 4, g_r * 2 + 4), pygame.SRCALPHA)
        pygame.draw.circle(gs, (*base_c, g_alpha), (g_r + 2, g_r + 2), g_r)
        surf.blit(gs, (cx - g_r - 2, cy - g_r - 2))
    # Sombra
    pygame.draw.circle(surf, shadow_c, (cx + 2, cy + 3), r)
    # Corpo
    pygame.draw.circle(surf, base_c, (cx, cy), r)
    # Borda escura (profundidade)
    rim_c = lerp_c(base_c, (8, 0, 0), 0.45)
    pygame.draw.circle(surf, rim_c, (cx, cy), r, max(1, r // 4))
    # Highlight principal
    hx = cx - r // 3
    hy = cy - r // 3
    pygame.draw.circle(surf, hi_c,         (hx,     hy),     max(2, r // 3))
    # Especular pequeno
    pygame.draw.circle(surf, (255, 252, 250), (hx - 2, hy - 2), max(1, r // 6))

# ---------------------------------------------------------------------------
# VISTA SUPERIOR
# ---------------------------------------------------------------------------
def draw_top_view(surf, ball, trail, sp_x, sp_y, pulse, font_sm):
    ed     = ball.edge_dist() if not ball.fallen else 999.0
    warn_t = max(0.0, min(1.0, 1.0 - ed / WARN_DIST_MM))

    # Glow de perigo na borda
    if warn_t > 0.25:
        flash  = 0.5 + 0.5 * math.sin(pulse * 13)
        gw     = int(warn_t * 14)
        g_col  = lerp_c(C_WARN, C_DANGER, warn_t * flash)
        g_a    = int(warn_t * 70)
        glow_s = pygame.Surface((PF_W + gw * 2, PF_H + gw * 2), pygame.SRCALPHA)
        pygame.draw.rect(glow_s, (*g_col, g_a),
                         (0, 0, PF_W + gw * 2, PF_H + gw * 2), border_radius=gw)
        surf.blit(glow_s, (TOP_X - gw, TOP_Y - gw))

    # Plataforma
    pygame.draw.rect(surf, C_PF, (TOP_X, TOP_Y, PF_W, PF_H))

    # Grade
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

    # Borda (cor dinamica com aviso)
    if warn_t > 0:
        flash  = 0.5 + 0.5 * math.sin(pulse * 13)
        bord_c = lerp_c(C_BORDER, lerp_c(C_WARN, C_DANGER, warn_t * flash), warn_t)
        bord_w = 2 + int(warn_t * 2)
    else:
        bord_c = C_BORDER
        bord_w = 2
    pygame.draw.rect(surf, bord_c, (TOP_X, TOP_Y, PF_W, PF_H), bord_w)

    # Setpoint com glow
    spx, spy = top_px(sp_x, sp_y)
    if TOP_X <= spx <= TOP_X + PF_W and TOP_Y <= spy <= TOP_Y + PF_H:
        r_sp = 8 + int(2 * math.sin(pulse * 5))
        draw_glow(surf, spx, spy, 4, C_SETPT, alpha=55, steps=2)
        pygame.draw.line(surf, C_SETPT, (spx - r_sp, spy), (spx + r_sp, spy), 1)
        pygame.draw.line(surf, C_SETPT, (spx, spy - r_sp), (spx, spy + r_sp), 1)
        pygame.draw.circle(surf, C_SETPT, (spx, spy), 4, 1)

    # Rastro
    pts = list(trail)
    for i in range(1, len(pts)):
        t = i / max(1, len(pts))
        pygame.draw.line(surf, lerp_c(C_TRAIL_A, C_TRAIL_B, t),
                         top_px(*pts[i - 1]), top_px(*pts[i]),
                         max(1, int(t * 3)))

    # Esfera (visivel ate cair)
    if not ball.fallen:
        bx, by = top_px(ball.x, ball.y)
        speed  = math.hypot(ball.vx, ball.vy)
        draw_sphere(surf, bx, by, BALL_R_PX, C_BALL, C_BALL_SD, C_BALL_SH, speed)

    # Titulo
    surf.blit(font_sm.render("VISTA SUPERIOR", True, C_LBLUE), (TOP_X + 4, TOP_Y + 4))

# ---------------------------------------------------------------------------
# VISTA LATERAL (isometrica)
# ---------------------------------------------------------------------------
def draw_lateral_view(surf, ball, out_x, out_y, sp_x, sp_y, font_sm, font_md):
    panel = pygame.Rect(LAT_X, LAT_Y, LAT_W, PF_H)
    pygame.draw.rect(surf, C_LAT_BG, panel, border_radius=6)
    pygame.draw.rect(surf, C_LAT_BD, panel, 1, border_radius=6)

    corners_mm = [(0, 0), (W_MM, 0), (W_MM, H_MM), (0, H_MM)]
    corners_z  = [z_surface(x, y, out_x, out_y) for x, y in corners_mm]
    corners_2d = [iso(x, y, z) for (x, y), z in zip(corners_mm, corners_z)]

    z_min   = min(corners_z) - BALL_R_MM
    gnd_pts = [iso(x, y, z_min) for x, y in corners_mm]

    pygame.draw.polygon(surf, C_ISO_SHADOW, gnd_pts)
    pygame.draw.polygon(surf, C_ISO_GND,    gnd_pts, 1)

    for (x, y), z_c, g_pt, s_pt in zip(corners_mm, corners_z, gnd_pts, corners_2d):
        pygame.draw.line(surf, C_ISO_PILLAR, g_pt, s_pt, 1)
        pygame.draw.circle(surf, C_ISO_PILLAR, g_pt, 3)

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

    # Motores A, B, C
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
        lbl = font_sm.render(["A","B","C"][i], True, MOTOR_C[i])
        surf.blit(lbl, (mp[0] + 7, mp[1] - 7))

    # Setpoint
    spx_iso, spy_iso = iso(sp_x, sp_y, z_min)
    draw_glow(surf, spx_iso, spy_iso, 4, C_SETPT, alpha=35, steps=1)
    pygame.draw.line(surf, C_SETPT, (spx_iso - 8, spy_iso), (spx_iso + 8, spy_iso), 1)
    pygame.draw.line(surf, C_SETPT, (spx_iso, spy_iso - 8), (spx_iso, spy_iso + 8), 1)

    # Esfera
    br = max(6, int(BALL_R_MM * ISO_S))
    if ball.fallen:
        bx_c   = max(0.0, min(W_MM, ball.x))
        by_c   = max(0.0, min(H_MM, ball.y))
        bx_iso, by_iso = iso(bx_c, by_c, ball.fall_z)
        fade   = max(20, int(255 * (1.0 - min(1.0, ball.fall_time / FALL_DURATION) * 0.9)))
        for col, off in [(C_BALL_SD, (2, 3)), (C_BALL, (0, 0))]:
            s = pygame.Surface((br * 2 + 6, br * 2 + 6), pygame.SRCALPHA)
            pygame.draw.circle(s, (*col, fade), (br + 3, br + 3), br)
            surf.blit(s, (bx_iso + off[0] - br - 3, by_iso + off[1] - br - 3))
    else:
        bz_surf = z_surface(ball.x, ball.y, out_x, out_y)
        shad_pt = iso(ball.x, ball.y, bz_surf)
        sr      = max(4, int(BALL_R_MM * ISO_S))
        shad_s  = pygame.Surface((sr * 2 + 2, sr * 2 // 2 + 2), pygame.SRCALPHA)
        pygame.draw.ellipse(shad_s, (0, 0, 0, 110), (0, 0, sr * 2, sr))
        surf.blit(shad_s, (shad_pt[0] - sr, shad_pt[1] - sr // 2))

        bz_ball = bz_surf + BALL_R_MM
        bx_iso, by_iso = iso(ball.x, ball.y, bz_ball)
        pygame.draw.line(surf, (180, 60, 60), shad_pt, (bx_iso, by_iso), 1)
        speed = math.hypot(ball.vx, ball.vy)
        draw_sphere(surf, bx_iso, by_iso, br, C_BALL, C_BALL_SD, C_BALL_SH, speed)

    # Titulo e angulos
    surf.blit(font_md.render("VISTA LATERAL  (isometrica)", True, C_LBLUE),
              (LAT_X + 8, LAT_Y + 6))
    tx_deg = out_to_tilt_deg(out_x)
    ty_deg = out_to_tilt_deg(out_y)
    surf.blit(font_sm.render(f"Tilt X={tx_deg:+.2f}°   Tilt Y={ty_deg:+.2f}°",
                             True, C_YELLOW),
              (LAT_X + 8, LAT_Y + PF_H - 18))

# ---------------------------------------------------------------------------
# OVERLAY: BOLA CAIU
# ---------------------------------------------------------------------------
def draw_fallen_overlay(surf, ball, font_sm, font_lg):
    prog = min(1.0, ball.fall_time / FALL_DURATION)

    ov = pygame.Surface((PF_W, PF_H), pygame.SRCALPHA)
    ov.fill((8, 2, 2, int(prog * 200)))
    surf.blit(ov, (TOP_X, TOP_Y))

    msg    = font_lg.render("BOLA CAIU!", True, C_DANGER)
    msg_sh = font_lg.render("BOLA CAIU!", True, (35, 0, 0))
    mx = TOP_X + PF_W // 2 - msg.get_width() // 2
    my = TOP_Y + PF_H // 2 - 20
    surf.blit(msg_sh, (mx + 2, my + 2))
    surf.blit(msg,    (mx,     my))

    remaining = max(0.0, FALL_DURATION - ball.fall_time)
    cd = font_sm.render(f"Resetando em {remaining:.1f}s...  [R] para resetar agora",
                        True, C_GRAY)
    surf.blit(cd, (TOP_X + PF_W // 2 - cd.get_width() // 2, my + 34))

# ---------------------------------------------------------------------------
# RODAPE DE INFORMACAO
# ---------------------------------------------------------------------------
def draw_info(surf, ball, sp_x, sp_y, pid_x, pid_y,
              out_x, out_y, pid_mode, pattern, font_sm, font_md):
    iy   = VIEW_H + 6
    rect = pygame.Rect(PAD, iy, WIN_W - 2 * PAD, INFO_H - 12)
    pygame.draw.rect(surf, C_INFO_BG, rect, border_radius=8)
    pygame.draw.rect(surf, C_INFO_BD, rect, 1, border_radius=8)

    speed  = math.hypot(ball.vx, ball.vy)
    err    = math.hypot(sp_x - ball.x, sp_y - ball.y)
    mode_s = "PID" if pid_mode else "MANUAL"
    mode_c = C_PID_MODE if pid_mode else C_MAN_MODE
    pat_s  = Pattern.NAMES.get(pattern.mode, "")
    err_c  = C_GREEN if err < 8 else (C_YELLOW if err < 25 else C_RED)

    c1 = PAD + 10;  c2 = c1 + 248;  c3 = c2 + 248

    # Linha de cabecalho
    row0 = iy + 8
    pill = pygame.Rect(c1, row0, 74, 18)
    pygame.draw.rect(surf, lerp_c(mode_c, C_INFO_BG, 0.75), pill, border_radius=9)
    pygame.draw.rect(surf, mode_c, pill, 1, border_radius=9)
    ml = font_sm.render(mode_s, True, mode_c)
    surf.blit(ml, (c1 + (74 - ml.get_width()) // 2, row0 + 2))
    surf.blit(font_md.render(f"Padrao: {pat_s}", True, C_WHITE), (c1 + 84, row0 + 1))

    # Dados
    rows = [
        [(f"Pos X   : {ball.x:6.1f} mm",   C_WHITE, c1, iy + 32),
         (f"Pos Y   : {ball.y:6.1f} mm",   C_WHITE, c1, iy + 48),
         (f"Veloc   : {speed:6.1f} mm/s",  C_CYAN,  c1, iy + 64),
         (f"Erro    : {err:6.1f} mm",      err_c,   c1, iy + 80)],
        [(f"Out X   : {out_x:+.4f} un",   C_LBLUE, c2, iy + 32),
         (f"Out Y   : {out_y:+.4f} un",   C_LBLUE, c2, iy + 48),
         (f"Int X   : {pid_x.integr:.0f}", C_GRAY,  c2, iy + 64),
         (f"Int Y   : {pid_y.integr:.0f}", C_GRAY,  c2, iy + 80)],
        [(f"Kp={pid_x.kp:.5f}  [Q/A]",    C_YELLOW, c3, iy + 32),
         (f"Ki={pid_x.ki:.6f}  [W/S]",    C_YELLOW, c3, iy + 48),
         (f"Kd={pid_x.kd:.4f}  [E/D]",    C_YELLOW, c3, iy + 64)],
    ]
    for col in rows:
        for text, col_c, tx, ty in col:
            surf.blit(font_sm.render(text, True, col_c), (tx, ty))

    # Rodape de teclas
    help_t = font_sm.render(
        "R:reset  Espaco:impulso  T:rastro  Clique:setpoint  Setas:manual  P:modo  0-5:padrao  ESC:sair",
        True, C_GRAY)
    surf.blit(help_t, (c1, iy + INFO_H - 28))

    # Barras dos motores
    bar_x = WIN_W - PAD - 160
    bar_y = iy + 22
    surf.blit(font_sm.render("Pernas:", True, C_GRAY), (bar_x, bar_y - 2))
    for i in range(3):
        hr   = motor_rel_height(i, out_x, out_y)
        denom = CX * math.sin(math.radians(out_to_tilt_deg(MAX_OUT))) + 0.01
        norm = max(-1.0, min(1.0, hr / denom))
        bx   = bar_x + i * 44
        by_m = bar_y + 14
        BH   = 52
        pygame.draw.rect(surf, C_DARK, (bx, by_m, 30, BH), border_radius=3)
        mid  = by_m + BH // 2
        hpx  = int(norm * BH // 2)
        if hpx > 0:
            pygame.draw.rect(surf, MOTOR_C[i], (bx, mid - hpx, 30, hpx), border_radius=2)
        elif hpx < 0:
            pygame.draw.rect(surf, MOTOR_C[i], (bx, mid, 30, -hpx), border_radius=2)
        pygame.draw.line(surf, C_GRAY, (bx, mid), (bx + 30, mid), 1)
        lbl = font_sm.render(["A","B","C"][i], True, MOTOR_C[i])
        surf.blit(lbl, (bx + 15 - lbl.get_width() // 2, by_m + BH + 2))

# ---------------------------------------------------------------------------
# LOOP PRINCIPAL
# ---------------------------------------------------------------------------
def main():
    pygame.init()
    screen  = pygame.display.set_mode((WIN_W, WIN_H))
    pygame.display.set_caption("Ball Balancer 3RPS — Simulacao")
    clock   = pygame.time.Clock()
    font_sm = pygame.font.SysFont("monospace", 13)
    font_md = pygame.font.SysFont("monospace", 15, bold=True)
    font_lg = pygame.font.SysFont("monospace", 30, bold=True)

    ball    = Ball()
    pid_x   = PID()
    pid_y   = PID()
    pattern = Pattern()
    trail   = deque(maxlen=180)

    out_x      = 0.0
    out_y      = 0.0
    sp_x       = CX
    sp_y       = CY
    pid_mode   = True
    show_trail = True
    pulse      = 0.0

    KP_S, KI_S, KD_S = 2.0e-4, 2.0e-6, 5.0e-3

    running = True
    while running:
        dt    = min(clock.tick(FPS) / 1000.0, 0.05)
        pulse += dt

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                k = event.key
                if   k == pygame.K_ESCAPE: running = False
                elif k == pygame.K_p:
                    pid_mode = not pid_mode
                    pid_x.reset(); pid_y.reset()
                    if pid_mode: out_x = out_y = 0.0
                elif k == pygame.K_r:
                    ball.reset(); trail.clear()
                    out_x = out_y = 0.0
                    pid_x.reset(); pid_y.reset()
                elif k == pygame.K_SPACE:
                    if not ball.fallen: ball.impulse()
                elif k == pygame.K_t:
                    show_trail = not show_trail
                    if not show_trail: trail.clear()
                elif k == pygame.K_0:
                    pattern.mode = Pattern.HOLD;    pattern.reset(); pid_x.reset(); pid_y.reset()
                elif k == pygame.K_1:
                    pattern.mode = Pattern.CIRCLE;  pattern.reset(); pid_x.reset(); pid_y.reset()
                elif k == pygame.K_2:
                    pattern.mode = Pattern.SQUARE;  pattern.reset(); pid_x.reset(); pid_y.reset()
                elif k == pygame.K_3:
                    pattern.mode = Pattern.FIGURE8; pattern.reset(); pid_x.reset(); pid_y.reset()
                elif k == pygame.K_4:
                    pattern.mode = Pattern.LINE;    pattern.reset(); pid_x.reset(); pid_y.reset()
                elif k == pygame.K_5:
                    pattern.mode = Pattern.SINUS;   pattern.reset(); pid_x.reset(); pid_y.reset()
                elif k == pygame.K_q: pid_x.kp = pid_y.kp = round(pid_x.kp + KP_S, 6)
                elif k == pygame.K_a: pid_x.kp = pid_y.kp = round(max(0.0, pid_x.kp - KP_S), 6)
                elif k == pygame.K_w: pid_x.ki = pid_y.ki = round(pid_x.ki + KI_S, 7)
                elif k == pygame.K_s: pid_x.ki = pid_y.ki = round(max(0.0, pid_x.ki - KI_S), 7)
                elif k == pygame.K_e: pid_x.kd = pid_y.kd = round(pid_x.kd + KD_S, 5)
                elif k == pygame.K_d: pid_x.kd = pid_y.kd = round(max(0.0, pid_x.kd - KD_S), 5)

            elif event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = pygame.mouse.get_pos()
                rx, ry = mx - TOP_X, my - TOP_Y
                if 0 <= rx <= PF_W and 0 <= ry <= PF_H:
                    sp_x = max(0.0, min(W_MM, rx / SCALE))
                    sp_y = max(0.0, min(H_MM, ry / SCALE))
                    pattern.mode = Pattern.HOLD
                    pid_x.reset(); pid_y.reset()

        # Auto-reset apos a bola cair
        if ball.fallen and ball.fall_time >= FALL_DURATION:
            ball.reset(); trail.clear()
            pid_x.reset(); pid_y.reset()

        # Controle suspenso enquanto bola cai
        if not ball.fallen:
            if pid_mode:
                sp_x, sp_y = pattern.setpoint()
                pattern.update()
                out_x = pid_x.step(sp_x, ball.x)
                out_y = pid_y.step(sp_y, ball.y)
            else:
                keys = pygame.key.get_pressed()
                step = MAX_OUT * 0.55 * dt / 0.017
                if   keys[pygame.K_LEFT]:  out_x = max(-MAX_OUT, out_x - step)
                elif keys[pygame.K_RIGHT]: out_x = min( MAX_OUT, out_x + step)
                else: out_x *= 0.87
                if   keys[pygame.K_UP]:    out_y = max(-MAX_OUT, out_y - step)
                elif keys[pygame.K_DOWN]:  out_y = min( MAX_OUT, out_y + step)
                else: out_y *= 0.87

        ball.update(out_x, out_y)
        if show_trail and not ball.fallen:
            trail.append((ball.x, ball.y))

        # Renderizacao
        screen.fill(C_BG)
        draw_top_view(screen, ball, trail, sp_x, sp_y, pulse, font_sm)
        draw_lateral_view(screen, ball, out_x, out_y, sp_x, sp_y, font_sm, font_md)
        draw_info(screen, ball, sp_x, sp_y, pid_x, pid_y,
                  out_x, out_y, pid_mode, pattern, font_sm, font_md)
        if ball.fallen:
            draw_fallen_overlay(screen, ball, font_sm, font_lg)
        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    main()
