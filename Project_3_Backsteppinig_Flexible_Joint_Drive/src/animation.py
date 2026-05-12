"""
animation.py
------------
Pygame-based interactive animation of the nonlinear flexible-joint drive.

Physical causality visualised
------------------------------
  motor torque  →  motor motion  →  shaft twisting
  →  elastic energy storage  →  load acceleration
  →  oscillation damping by backstepping

Window layout
-------------
  ┌─────────────────────────────────────────────────────┐
  │  HEADER:  title + time + playback controls hint      │
  ├──────────────┬──────────────────────┬───────────────┤
  │  TELEMETRY   │   MECHANICAL SCENE   │  ENERGY BAR   │
  │  panel       │   motor─shaft─load   │  panel        │
  ├──────────────┴──────────────────────┴───────────────┤
  │  MINI-PLOTS  (4 scrolling traces)                    │
  └─────────────────────────────────────────────────────┘

Keyboard controls
-----------------
  SPACE       pause / resume
  LEFT / RIGHT  step one frame
  UP / DOWN   speed ×2 / ÷2
  R           restart
  ESC / Q     quit
"""

from __future__ import annotations
import sys
import math
import time
from pathlib import Path
from collections import deque
from typing import Sequence

import numpy as np

# ── pygame import (hard dependency for this module) ──────────────────
try:
    import pygame
    import pygame.gfxdraw
except ImportError:
    raise ImportError(
        "pygame is required for the animation module.\n"
        "Install with:  pip install pygame"
    )

from simulation import SimResult

# ─────────────────────────────────────────────────────────────────────
# Colour palette  (matches matplotlib dark theme)
# ─────────────────────────────────────────────────────────────────────
C = {
    "bg":         (13,  15,  20),
    "panel":      (19,  22,  30),
    "panel_b":    (22,  26,  36),
    "border":     (31,  36,  51),
    "grid":       (31,  36,  51),
    "text":       (224, 224, 224),
    "subtext":    (144, 164, 174),
    "accent0":    ( 79, 195, 247),   # load / cyan
    "accent1":    (240,  98, 146),   # motor / pink
    "accent2":    (174, 213, 129),   # reference / green
    "accent3":    (255, 183,  77),   # torque / amber
    "accent4":    (206, 147, 216),   # energy / purple
    "error":      (239,  83,  80),
    "white":      (255, 255, 255),
    "black":      (  0,   0,   0),
    # shaft energy colour stops (green→yellow→red)
    "shaft_lo":   ( 76, 175,  80),
    "shaft_mid":  (255, 193,   7),
    "shaft_hi":   (244,  67,  54),
}

# ─────────────────────────────────────────────────────────────────────
# Geometry constants  (all in pixels, computed from WIN_W × WIN_H)
# ─────────────────────────────────────────────────────────────────────
WIN_W = 1400
WIN_H = 820
FPS_TARGET = 60

HEADER_H   = 44
SCENE_H    = 390
TELEM_W    = 230
ENERGY_W   = 170
PLOT_H     = WIN_H - HEADER_H - SCENE_H   # bottom strip

SCENE_X    = TELEM_W
SCENE_W    = WIN_W - TELEM_W - ENERGY_W
SCENE_Y    = HEADER_H

DISK_R     = 68          # inertia disk radius
MOTOR_CX   = SCENE_X + SCENE_W // 4
LOAD_CX    = SCENE_X + 3 * SCENE_W // 4
DISK_CY    = SCENE_Y + SCENE_H // 2 + 10

PLOT_AREA_X = 0
PLOT_AREA_Y = HEADER_H + SCENE_H
PLOT_AREA_W = WIN_W
N_MINI      = 4
MINI_W      = WIN_W // N_MINI
MINI_H      = PLOT_H

TRAIL_LEN   = 120        # frames of angle-marker trail


# ─────────────────────────────────────────────────────────────────────
# Helper utilities
# ─────────────────────────────────────────────────────────────────────

def lerp_color(c0: tuple, c1: tuple, t: float) -> tuple:
    t = max(0.0, min(1.0, t))
    return tuple(int(a + (b - a) * t) for a, b in zip(c0, c1))


def torque_color(tau_norm: float) -> tuple:
    """Map normalised torque ∈ [0,1] to green→yellow→red."""
    if tau_norm < 0.5:
        return lerp_color(C["shaft_lo"], C["shaft_mid"], tau_norm * 2)
    return lerp_color(C["shaft_mid"], C["shaft_hi"], (tau_norm - 0.5) * 2)


def draw_glow(surf: pygame.Surface, cx: int, cy: int,
              radius: int, color: tuple, alpha: int = 35) -> None:
    """Draw a soft circular glow using concentric transparent circles."""
    glow = pygame.Surface((radius * 4, radius * 4), pygame.SRCALPHA)
    for r in range(radius * 2, 0, -4):
        a = int(alpha * (1 - r / (radius * 2)))
        pygame.draw.circle(glow, (*color[:3], a),
                           (radius * 2, radius * 2), r)
    surf.blit(glow, (cx - radius * 2, cy - radius * 2))


def draw_rounded_rect(surf: pygame.Surface, rect: pygame.Rect,
                      color: tuple, radius: int = 8,
                      border: tuple | None = None,
                      border_w: int = 1) -> None:
    pygame.draw.rect(surf, color, rect, border_radius=radius)
    if border:
        pygame.draw.rect(surf, border, rect, border_w, border_radius=radius)


def draw_text(surf: pygame.Surface, font: pygame.font.Font,
              text: str, x: int, y: int, color: tuple,
              align: str = "left") -> None:
    img = font.render(text, True, color)
    if align == "center":
        x -= img.get_width() // 2
    elif align == "right":
        x -= img.get_width()
    surf.blit(img, (x, y))


# ─────────────────────────────────────────────────────────────────────
# Inertia disk renderer
# ─────────────────────────────────────────────────────────────────────

def draw_disk(surf: pygame.Surface,
              cx: int, cy: int, radius: int,
              theta: float,
              fill_color: tuple,
              border_color: tuple,
              label: str,
              font_big: pygame.font.Font,
              font_sm: pygame.font.Font,
              torque_mag: float = 0.0,
              trail: deque | None = None) -> None:
    """
    Draw a rotating inertia disk with:
      - filled circle with rim
      - radial angle marker (white spoke)
      - angle-marker trail (fading)
      - torque glow when |u| is large
      - label below
    """
    # Glow proportional to torque
    if torque_mag > 0.05:
        glow_r = int(radius * (1.0 + 0.6 * min(torque_mag, 1.0)))
        draw_glow(surf, cx, cy, glow_r, fill_color,
                  alpha=int(50 * min(torque_mag, 1.0)))

    # Trail of previous marker angles
    if trail:
        for k, ang in enumerate(trail):
            alpha_f = int(120 * k / len(trail))
            ex = cx + int((radius - 6) * math.sin(ang))
            ey = cy - int((radius - 6) * math.cos(ang))
            col = (*fill_color[:3], alpha_f)
            # draw as a small dot on a SRCALPHA surface
            dot = pygame.Surface((6, 6), pygame.SRCALPHA)
            pygame.draw.circle(dot, col, (3, 3), 2)
            surf.blit(dot, (ex - 3, ey - 3))

    # Body
    pygame.draw.circle(surf, fill_color, (cx, cy), radius)
    pygame.draw.circle(surf, border_color, (cx, cy), radius, 3)

    # Inner ring for aesthetics
    pygame.draw.circle(surf, lerp_color(fill_color, C["bg"], 0.45),
                       (cx, cy), radius - 14, 2)

    # Radial spoke (angle marker)
    mx = cx + int((radius - 6) * math.sin(theta))
    my = cy - int((radius - 6) * math.cos(theta))
    pygame.draw.line(surf, C["white"], (cx, cy), (mx, my), 3)
    pygame.draw.circle(surf, C["white"], (mx, my), 5)

    # Centre dot
    pygame.draw.circle(surf, C["bg"], (cx, cy), 8)
    pygame.draw.circle(surf, border_color, (cx, cy), 8, 2)

    # Label
    draw_text(surf, font_big, label, cx, cy + radius + 10,
              C["text"], align="center")


# ─────────────────────────────────────────────────────────────────────
# Flexible shaft renderer  –  physically correct torsional geometry
# ─────────────────────────────────────────────────────────────────────

def _shaft_attachment(cx: int, cy: int, radius: int, theta: float
                      ) -> tuple[int, int]:
    """
    Return the pixel coordinates of the shaft attachment point on a disk.

    Uses exactly the same formula as the spoke tip drawn by draw_disk:

        x = cx + r * sin(theta)
        y = cy - r * cos(theta)

    where r = radius - 6 (the same inset used for the white dot).
    Convention: theta = 0  →  12-o'clock,  increases clockwise on screen
    (screen y points down, so clockwise rotation = increasing theta).
    This matches the spoke drawn in draw_disk, so the shaft end lands
    precisely on the angle-indicator dot and moves with it.
    """
    r  = radius - 6          # same inset as the spoke tip in draw_disk
    ax = cx + int(r * math.sin(theta))
    ay = cy - int(r * math.cos(theta))
    return ax, ay


def draw_shaft(surf: pygame.Surface,
               motor_cx: int, motor_cy: int,
               load_cx:  int, load_cy:  int,
               disk_r:   int,
               theta_m:  float,
               theta_l:  float,
               tau_norm: float) -> None:
    """
    Draw the torsional flexible shaft between the two rotating disks.

    Physical model
    --------------
    The shaft is a compliant torsional coupling.  Its two ends are
    *rigidly attached* to the disk flanges:

      • LEFT  end  rotates with the motor  → attachment angle = θ_m
      • RIGHT end  rotates with the load   → attachment angle = θ_l

    The shaft *internal shape* deforms in proportion to

        Δθ = θ_m − θ_l   (shaft twist / torsional strain)

    When Δθ ≈ 0 the shaft is relaxed (nearly straight).
    When |Δθ| is large the shaft appears wound-up / elastically stressed.

    Rendering approach
    ------------------
    We draw N parallel "fibres" of the shaft.  Each fibre is a cubic
    Hermite spline whose:
      • start point  = attachment on motor rim  (rotates with θ_m)
      • end   point  = attachment on load  rim  (rotates with θ_l)
      • start tangent = tangential to the motor rim at the attachment
      • end   tangent = tangential to the load  rim at the attachment

    The cubic Hermite spline guarantees C¹ continuity at both disk
    attachments (the curve is tangent to the rim, not poking through it).

    Multiple fibres are offset ±perp around the centreline to give the
    shaft visual *width*, and their colours grade from centre-bright to
    edge-dark so it looks tubular.

    The midpoint bulge of the spline grows with |Δθ|, making the
    torsional strain immediately visible.
    """
    delta = theta_m - theta_l

    # ── Attachment points on each rim ────────────────────────────────
    ax_m, ay_m = _shaft_attachment(motor_cx, motor_cy, disk_r, theta_m)
    ax_l, ay_l = _shaft_attachment(load_cx,  load_cy,  disk_r, theta_l)

    # ── Tangent vectors at attachment points ─────────────────────────
    # The radius at attachment angle theta is (sin θ, -cos θ) in screen
    # coords.  The tangent (90° CCW from radius) is (cos θ, sin θ).
    # We always want the tangent component pointing FROM motor TOWARD load
    # (positive x-direction), so we flip if necessary.
    def rim_tangent(theta: float, point_right: bool) -> tuple[float, float]:
        tx =  math.cos(theta)
        ty =  math.sin(theta)
        # flip so the tangent points in the required x-direction
        if point_right and tx < 0:
            tx, ty = -tx, -ty
        if not point_right and tx > 0:
            tx, ty = -tx, -ty
        return tx, ty

    # Motor tangent points rightward (toward load)
    tmx, tmy = rim_tangent(theta_m, point_right=True)
    # Load tangent points leftward (back toward motor)
    tlx, tly = rim_tangent(theta_l, point_right=False)

    # Tangent scale: controls how "stiff" the spline looks.
    # Grows with shaft length so the curve doesn't fold back.
    shaft_span = math.hypot(ax_l - ax_m, ay_l - ay_m)
    tangent_scale = shaft_span * 0.55

    # ── Cubic Hermite spline evaluator ───────────────────────────────
    def hermite(p0, p1, m0, m1, t):
        """
        Cubic Hermite interpolation.
        p0, p1: endpoints (x,y)
        m0, m1: tangents (scaled) at endpoints
        t ∈ [0, 1]
        """
        h00 =  2*t**3 - 3*t**2 + 1
        h10 =    t**3 - 2*t**2 + t
        h01 = -2*t**3 + 3*t**2
        h11 =    t**3 -   t**2
        x = h00*p0[0] + h10*m0[0] + h01*p1[0] + h11*m1[0]
        y = h00*p0[1] + h10*m0[1] + h01*p1[1] + h11*m1[1]
        return x, y

    p0 = (ax_m, ay_m)
    p1 = (ax_l, ay_l)
    m0 = (tmx * tangent_scale, tmy * tangent_scale)
    m1 = (tlx * tangent_scale, tly * tangent_scale)

    # ── Torsional strain deformation ──────────────────────────────────
    # The midpoint of the spline is pushed perpendicularly by Δθ.
    # This "bows" the shaft when twisted — exactly the physical behaviour
    # of a torsionally loaded compliant coupling seen from the side.
    # We add a perpendicular offset to the midpoint control tangents.
    perp_x = -(ay_l - ay_m) / max(shaft_span, 1)   # unit perp to shaft axis
    perp_y =  (ax_l - ax_m) / max(shaft_span, 1)

    # Exaggeration factor: visible even for small Δθ
    strain_amp = min(abs(delta) * 60.0, 52.0) * math.copysign(1.0, delta
                                                               ) if delta else 0.0
    # Add strain bow to both tangent midpoints
    m0_bow = (m0[0] + perp_x * strain_amp, m0[1] + perp_y * strain_amp)
    m1_bow = (m1[0] + perp_x * strain_amp, m1[1] + perp_y * strain_amp)

    # ── Build centreline polyline ─────────────────────────────────────
    N_PTS  = 80
    centre: list[tuple[float, float]] = []
    for k in range(N_PTS + 1):
        t = k / N_PTS
        cx2, cy2 = hermite(p0, p1, m0_bow, m1_bow, t)
        centre.append((cx2, cy2))

    # ── Draw shaft as a "tube" (multiple parallel fibres) ────────────
    # Perpendicular direction at each point (rotated tangent)
    base_color = torque_color(tau_norm)
    thickness_px = max(3, int(3 + 4 * tau_norm))

    # Tube width in pixels
    tube_half = max(3, int(4 + 3 * tau_norm))

    # Draw from outermost (darkest) to innermost (brightest) fibre
    n_fibres = 5
    for fi in range(n_fibres, -1, -1):
        offset_frac = fi / n_fibres           # 1 = outer edge, 0 = centre
        brightness  = 1.0 - 0.55 * offset_frac
        fibre_color = tuple(min(255, int(c * brightness)) for c in base_color)
        offset_px   = offset_frac * tube_half

        # Build two offset polylines (±perp) for this fibre
        for side in (+1, -1):
            pts: list[tuple[int, int]] = []
            for k in range(len(centre)):
                cx2, cy2 = centre[k]
                # Compute local perpendicular from adjacent points
                if k < len(centre) - 1:
                    dx = centre[k+1][0] - cx2
                    dy = centre[k+1][1] - cy2
                else:
                    dx = cx2 - centre[k-1][0]
                    dy = cy2 - centre[k-1][1]
                ln = max(math.hypot(dx, dy), 1e-9)
                nx, ny = -dy / ln, dx / ln       # unit normal
                px2 = int(cx2 + side * nx * offset_px)
                py2 = int(cy2 + side * ny * offset_px)
                pts.append((px2, py2))

            if len(pts) >= 2:
                lw = max(1, thickness_px - fi)
                pygame.draw.lines(surf, fibre_color, False, pts, lw)

    # ── Centreline highlight ──────────────────────────────────────────
    if len(centre) >= 2:
        highlight = lerp_color(base_color, C["white"], 0.35)
        centre_px = [(int(x), int(y)) for x, y in centre]
        pygame.draw.lines(surf, highlight, False, centre_px, 1)

    # ── Shimmer when torque is high ───────────────────────────────────
    if tau_norm > 0.4:
        shimmer_alpha = int(60 * (tau_norm - 0.4) / 0.6)
        glow_surf = pygame.Surface((WIN_W, WIN_H), pygame.SRCALPHA)
        shimmer_c = (*lerp_color(base_color, C["white"], 0.5)[:3],
                     shimmer_alpha)
        mid = centre[len(centre) // 2]
        pygame.draw.circle(glow_surf, shimmer_c,
                           (int(mid[0]), int(mid[1])),
                           int(tube_half * 3))
        surf.blit(glow_surf, (0, 0))

    # ── Attachment pin markers (bolted flange visualisation) ──────────
    pin_color = lerp_color(base_color, C["white"], 0.4)
    pygame.draw.circle(surf, pin_color, (int(ax_m), int(ay_m)), 6)
    pygame.draw.circle(surf, C["bg"],   (int(ax_m), int(ay_m)), 3)
    pygame.draw.circle(surf, pin_color, (int(ax_l), int(ay_l)), 6)
    pygame.draw.circle(surf, C["bg"],   (int(ax_l), int(ay_l)), 3)


# ─────────────────────────────────────────────────────────────────────
# Torque arrow renderer
# ─────────────────────────────────────────────────────────────────────

def draw_torque_arrow(surf: pygame.Surface,
                      cx: int, cy: int, radius: int,
                      u: float, u_max: float,
                      font: pygame.font.Font) -> None:
    """
    Draw a curved arc-arrow around the motor disk indicating torque
    direction and magnitude.
    """
    if abs(u) < 0.5:
        return

    norm    = min(abs(u) / max(u_max, 1.0), 1.0)
    arc_r   = radius + 18
    arc_span = math.pi * 0.6 * norm        # grows with |u|
    color   = lerp_color(C["accent3"], C["shaft_hi"], norm)

    # Draw arc
    arc_pts = []
    start_a = math.pi / 2
    n_arc   = 30
    direction = 1 if u > 0 else -1
    for i in range(n_arc + 1):
        a = start_a + direction * (arc_span * i / n_arc)
        ax = int(cx + arc_r * math.cos(a))
        ay = int(cy - arc_r * math.sin(a))
        arc_pts.append((ax, ay))

    if len(arc_pts) >= 2:
        pygame.draw.lines(surf, color, False, arc_pts, 3)

    # Arrowhead at end
    if len(arc_pts) >= 2:
        tip = arc_pts[-1]
        before = arc_pts[-2]
        dx = tip[0] - before[0]
        dy = tip[1] - before[1]
        ln = max(math.hypot(dx, dy), 1)
        dx, dy = dx / ln * 10, dy / ln * 10
        perp_x, perp_y = -dy, dx
        pygame.draw.polygon(surf, color, [
            tip,
            (int(tip[0] - dx + perp_x), int(tip[1] - dy + perp_y)),
            (int(tip[0] - dx - perp_x), int(tip[1] - dy - perp_y)),
        ])

    # Torque value label
    label = f"u={u:+.1f}Nm"
    draw_text(surf, font, label,
              cx, cy - arc_r - 22,
              color, align="center")


# ─────────────────────────────────────────────────────────────────────
# Telemetry panel
# ─────────────────────────────────────────────────────────────────────

def draw_telemetry(surf: pygame.Surface,
                   rect: pygame.Rect,
                   data: dict,
                   font_title: pygame.font.Font,
                   font_mono: pygame.font.Font) -> None:
    draw_rounded_rect(surf, rect, C["panel"], radius=6, border=C["border"])

    x = rect.x + 14
    y = rect.y + 12

    draw_text(surf, font_title, "TELEMETRY", x, y, C["subtext"])
    y += 28

    rows = [
        ("θ_l",  data.get("theta_l", 0.0),  "rad",   C["accent0"]),
        ("θ_m",  data.get("theta_m", 0.0),  "rad",   C["accent1"]),
        ("θ_d",  data.get("theta_d", 0.0),  "rad",   C["accent2"]),
        ("Δθ",   data.get("delta",   0.0),  "rad",   C["accent3"]),
        ("Δω",   data.get("nu",      0.0),  "rad/s", C["accent3"]),
        ("τ_c",  data.get("tau_c",   0.0),  "N·m",   C["accent3"]),
        ("u",    data.get("u",       0.0),  "N·m",   C["accent4"]),
        ("E",    data.get("energy",  0.0),  "J",     C["accent4"]),
        ("V",    data.get("lyapunov",0.0),  "",      C["accent4"]),
        ("e₁",   data.get("e1",      0.0),  "rad",   C["error"]),
        ("e₂",   data.get("e2",      0.0),  "r/s",   C["error"]),
        ("e₃",   data.get("e3",      0.0),  "N·m",   C["error"]),
    ]

    for name, val, unit, color in rows:
        # Label
        lbl_surf = font_mono.render(f"{name:<4}", True, C["subtext"])
        surf.blit(lbl_surf, (x, y))
        # Value
        val_str  = f"{val:+9.4f}" if abs(val) < 1e4 else f"{val:+9.2e}"
        val_surf = font_mono.render(val_str, True, color)
        surf.blit(val_surf, (x + 46, y))
        # Unit
        unit_surf = font_mono.render(unit, True, C["subtext"])
        surf.blit(unit_surf, (x + 46 + val_surf.get_width() + 4, y))
        y += 22

        # Separator after Δω
        if name == "Δω":
            pygame.draw.line(surf, C["border"],
                             (x, y - 4), (rect.right - 14, y - 4), 1)


# ─────────────────────────────────────────────────────────────────────
# Energy bar panel
# ─────────────────────────────────────────────────────────────────────

def draw_energy_bars(surf: pygame.Surface,
                     rect: pygame.Rect,
                     E_load: float, E_motor: float,
                     E_elastic: float, E_total: float,
                     E_max: float,
                     font_title: pygame.font.Font,
                     font_sm: pygame.font.Font) -> None:
    draw_rounded_rect(surf, rect, C["panel"], radius=6, border=C["border"])

    x  = rect.x + 14
    y  = rect.y + 12
    bw = rect.width - 28   # bar width area
    bh = 16

    draw_text(surf, font_title, "ENERGY", x, y, C["subtext"])
    y += 28

    bars = [
        ("KE load",   E_load,    C["accent0"]),
        ("KE motor",  E_motor,   C["accent1"]),
        ("Elastic",   E_elastic, C["accent3"]),
        ("Total",     E_total,   C["accent4"]),
    ]

    E_ref = max(E_max, 0.01)
    for label, val, color in bars:
        draw_text(surf, font_sm, label, x, y, C["subtext"])
        y += 16
        frac = max(0.0, min(1.0, val / E_ref))
        bg_rect = pygame.Rect(x, y, bw, bh)
        fill_rect = pygame.Rect(x, y, int(bw * frac), bh)
        draw_rounded_rect(surf, bg_rect,  C["panel_b"], radius=4)
        if frac > 0.01:
            draw_rounded_rect(surf, fill_rect, color, radius=4)
        val_str = f"{val:.3f} J"
        draw_text(surf, font_sm, val_str,
                  x + bw + 6, y + 1, color)
        y += bh + 8

    # Fraction elastic / total — key insight visualised
    y += 8
    frac_e = E_elastic / max(E_total, 0.001)
    draw_text(surf, font_sm,
              f"Elastic fraction: {frac_e*100:.1f}%",
              x, y, C["accent3"])


# ─────────────────────────────────────────────────────────────────────
# Mini scrolling plot
# ─────────────────────────────────────────────────────────────────────

class MiniPlot:
    """A scrolling single-trace strip rendered with pygame."""

    def __init__(self, rect: pygame.Rect,
                 label: str, unit: str,
                 color: tuple,
                 history_len: int = 400) -> None:
        self.rect = rect
        self.label = label
        self.unit  = unit
        self.color = color
        self.history: deque[float] = deque(maxlen=history_len)
        self.ref_history: deque[float] | None = None

    def push(self, val: float, ref: float | None = None) -> None:
        self.history.append(val)
        if ref is not None:
            if self.ref_history is None:
                self.ref_history = deque(maxlen=self.history.maxlen)
            self.ref_history.append(ref)

    def draw(self, surf: pygame.Surface,
             font_sm: pygame.font.Font) -> None:
        r = self.rect
        draw_rounded_rect(surf, r, C["panel_b"], radius=4, border=C["border"])

        if len(self.history) < 2:
            return

        vals = list(self.history)
        lo   = min(vals)
        hi   = max(vals)
        span = max(hi - lo, 0.01)
        lo  -= span * 0.08
        hi  += span * 0.08
        span = hi - lo

        def to_px(v):
            frac = (v - lo) / span
            return int(r.bottom - 4 - frac * (r.height - 20))

        n = len(vals)
        pts = []
        for i, v in enumerate(vals):
            px = r.x + int(i / (self.history.maxlen - 1) * (r.width - 2))
            py = to_px(v)
            pts.append((px, py))

        if len(pts) >= 2:
            pygame.draw.lines(surf, self.color, False, pts, 2)

        # Reference trace
        if self.ref_history and len(self.ref_history) >= 2:
            ref_vals = list(self.ref_history)
            rpts = []
            for i, v in enumerate(ref_vals):
                px = r.x + int(i / (self.history.maxlen - 1) * (r.width - 2))
                py = to_px(v)
                rpts.append((px, py))
            pygame.draw.lines(surf, C["accent2"], False, rpts, 1)

        # Zero line
        if lo < 0 < hi:
            zy = to_px(0.0)
            pygame.draw.line(surf, C["border"],
                             (r.x, zy), (r.right, zy), 1)

        # Current value indicator (right edge)
        cur_y = to_px(vals[-1])
        pygame.draw.circle(surf, self.color, (r.right - 3, cur_y), 4)

        # Labels
        draw_text(surf, font_sm, self.label, r.x + 6, r.y + 4, C["subtext"])
        val_str = f"{vals[-1]:+.3f} {self.unit}"
        draw_text(surf, font_sm, val_str,
                  r.right - 6, r.y + 4, self.color, align="right")


# ─────────────────────────────────────────────────────────────────────
# Header renderer
# ─────────────────────────────────────────────────────────────────────

def draw_header(surf: pygame.Surface,
                t_sim: float, t_total: float,
                frame: int, n_frames: int,
                paused: bool, speed: float,
                font_title: pygame.font.Font,
                font_sm: pygame.font.Font) -> None:
    rect = pygame.Rect(0, 0, WIN_W, HEADER_H)
    pygame.draw.rect(surf, C["panel_b"], rect)
    pygame.draw.line(surf, C["border"], (0, HEADER_H - 1), (WIN_W, HEADER_H - 1), 1)

    draw_text(surf, font_title,
              "Flexible-Joint Drive — Backstepping Controller",
              WIN_W // 2, 8, C["text"], align="center")

    # Progress bar
    bar_x, bar_y = 20, 28
    bar_w = 320
    prog = frame / max(n_frames - 1, 1)
    pygame.draw.rect(surf, C["border"],  (bar_x, bar_y, bar_w, 8), border_radius=4)
    pygame.draw.rect(surf, C["accent4"],
                     (bar_x, bar_y, int(bar_w * prog), 8), border_radius=4)
    draw_text(surf, font_sm, f"t = {t_sim:.2f} / {t_total:.1f} s",
              bar_x + bar_w + 10, bar_y - 2, C["subtext"])

    # Status
    status = "⏸ PAUSED" if paused else f"▶ ×{speed:.2g}"
    col = C["accent3"] if paused else C["accent2"]
    draw_text(surf, font_sm, status, WIN_W - 120, bar_y - 2, col)

    draw_text(surf, font_sm,
              "SPACE:pause  ←→:step  ↑↓:speed  R:restart  ESC:quit",
              WIN_W - 560, 8, C["subtext"])


# ─────────────────────────────────────────────────────────────────────
# GIF export helper (requires Pillow)
# ─────────────────────────────────────────────────────────────────────

def _try_save_gif(frames: list[pygame.Surface],
                  path: Path,
                  fps: int = 20) -> bool:
    try:
        from PIL import Image
        pil_frames = []
        for surf in frames:
            raw = pygame.image.tobytes(surf, "RGB")
            img = Image.frombytes("RGB", surf.get_size(), raw)
            pil_frames.append(img.convert("P", dither=Image.Dither.NONE))
        pil_frames[0].save(
            path, save_all=True,
            append_images=pil_frames[1:],
            loop=0,
            duration=int(1000 / fps),
            optimize=False,
        )
        print(f"  [gif saved] {path}")
        return True
    except ImportError:
        print("  Pillow not installed – GIF export skipped.  pip install Pillow")
        return False
    except Exception as e:
        print(f"  GIF export failed: {e}")
        return False


# ─────────────────────────────────────────────────────────────────────
# Main animation class
# ─────────────────────────────────────────────────────────────────────

class FlexibleJointAnimator:
    """
    Drives the full pygame window.  Call run() to enter the event loop.
    """

    def __init__(self,
                 result: SimResult,
                 save_gif: bool = False,
                 gif_path: Path | None = None,
                 gif_fps: int = 20,
                 gif_max_frames: int = 400) -> None:
        self.r          = result
        self.save_gif   = save_gif
        self.gif_path   = gif_path
        self.gif_fps    = gif_fps
        self.gif_max_frames = gif_max_frames

        # Playback state
        self.frame      = 0
        self.paused     = False
        self.speed      = 1.0          # playback multiplier
        self._frac_acc  = 0.0          # sub-frame accumulator

        # Precompute derived arrays
        n = len(result.t)
        self.delta     = result.x[:, 2] - result.x[:, 0]   # Δθ
        self.nu        = result.x[:, 3] - result.x[:, 1]   # Δω
        p = 50.0; k3 = 20.0   # default params (cosmetic only)
        self.E_elastic = (0.5 * p * self.delta**2
                          + 0.25 * k3 * self.delta**4)
        self.E_load    = 0.5 * 0.5 * result.x[:, 1]**2
        self.E_motor   = 0.5 * 0.1 * result.x[:, 3]**2

        self._tau_max  = max(np.abs(result.tau_c).max(), 0.1)
        self._u_max    = max(np.abs(result.u).max(), 1.0)
        self._E_max    = result.energy.max() * 1.1

        # Angle-marker trails
        self._motor_trail: deque = deque(maxlen=TRAIL_LEN)
        self._load_trail:  deque = deque(maxlen=TRAIL_LEN)

        # GIF frame buffer
        self._gif_frames: list[pygame.Surface] = []
        self._gif_stride = max(1, n // gif_max_frames)

        # Pygame init
        pygame.init()
        pygame.display.set_caption("Flexible-Joint Drive — Backstepping")
        self.screen = pygame.display.set_mode((WIN_W, WIN_H))
        self.clock  = pygame.time.Clock()

        # Fonts
        mono_candidates = ["DejaVuSansMono", "Courier New", "monospace"]
        self.font_title = pygame.font.SysFont("sans-serif", 14, bold=True)
        self.font_big   = pygame.font.SysFont("sans-serif", 15, bold=True)
        self.font_sm    = pygame.font.SysFont("monospace",  11)
        self.font_mono  = pygame.font.SysFont("monospace",  12)

        # Mini-plots
        self._build_mini_plots()

    # ── mini-plot layout ─────────────────────────────────────────────

    def _build_mini_plots(self) -> None:
        configs = [
            ("θ_l vs θ_d",  "rad",   C["accent0"]),
            ("τ_c",         "N·m",   C["accent3"]),
            ("Energy E",    "J",     C["accent4"]),
            ("Δθ",          "rad",   C["accent3"]),
        ]
        self.mini_plots: list[MiniPlot] = []
        for i, (label, unit, color) in enumerate(configs):
            r = pygame.Rect(
                PLOT_AREA_X + i * MINI_W + 2,
                PLOT_AREA_Y + 2,
                MINI_W - 4,
                MINI_H - 4,
            )
            mp = MiniPlot(r, label, unit, color)
            if label == "θ_l vs θ_d":
                mp.ref_history = deque(maxlen=mp.history.maxlen)
            self.mini_plots.append(mp)

    # ── push current frame data into mini-plots ───────────────────────

    def _push_frame_data(self, i: int) -> None:
        r = self.r
        self.mini_plots[0].push(r.x[i, 0], ref=r.theta_d[i])
        self.mini_plots[1].push(r.tau_c[i])
        self.mini_plots[2].push(r.energy[i])
        self.mini_plots[3].push(self.delta[i])

    # ── draw one frame ────────────────────────────────────────────────

    def _draw(self) -> None:
        i  = self.frame
        r  = self.r
        surf = self.screen

        theta_m = float(r.x[i, 2])
        theta_l = float(r.x[i, 0])
        delta_v = float(self.delta[i])
        tau_c   = float(r.tau_c[i])
        u       = float(r.u[i])
        energy  = float(r.energy[i])
        E_el    = float(self.E_elastic[i])
        E_lo    = float(self.E_load[i])
        E_mo    = float(self.E_motor[i])
        tau_norm = min(abs(tau_c) / self._tau_max, 1.0)
        u_norm   = min(abs(u) / self._u_max, 1.0)

        surf.fill(C["bg"])

        # ── Header ────────────────────────────────────────────────────
        draw_header(surf, float(r.t[i]), float(r.t[-1]),
                    i, len(r.t),
                    self.paused, self.speed,
                    self.font_title, self.font_sm)

        # ── Scene background ──────────────────────────────────────────
        scene_rect = pygame.Rect(SCENE_X, SCENE_Y, SCENE_W, SCENE_H)
        draw_rounded_rect(surf, scene_rect, C["panel"], radius=0,
                          border=C["border"])

        # Axis label
        draw_text(surf, self.font_sm,
                  "MOTOR ─── flexible shaft ─── LOAD",
                  SCENE_X + SCENE_W // 2, SCENE_Y + 8,
                  C["subtext"], align="center")

        # Ground rails (cosmetic horizontal lines)
        rail_y = DISK_CY + DISK_R + 18
        pygame.draw.line(surf, C["border"],
                         (SCENE_X + 20, rail_y),
                         (SCENE_X + SCENE_W - 20, rail_y), 2)
        # Hatching
        for hx in range(SCENE_X + 24, SCENE_X + SCENE_W - 20, 14):
            pygame.draw.line(surf, C["border"],
                             (hx, rail_y), (hx - 8, rail_y + 10), 1)

        # ── Shaft ─────────────────────────────────────────────────────
        draw_shaft(surf,
                   MOTOR_CX, DISK_CY,
                   LOAD_CX,  DISK_CY,
                   DISK_R,
                   theta_m, theta_l,
                   tau_norm)

        # ── Motor disk ────────────────────────────────────────────────
        self._motor_trail.append(theta_m)
        draw_disk(surf, MOTOR_CX, DISK_CY, DISK_R,
                  theta_m,
                  fill_color=lerp_color(C["panel"], C["accent1"], 0.35),
                  border_color=C["accent1"],
                  label="MOTOR  Jₘ",
                  font_big=self.font_big,
                  font_sm=self.font_sm,
                  torque_mag=u_norm,
                  trail=self._motor_trail)

        # ── Load disk ─────────────────────────────────────────────────
        self._load_trail.append(theta_l)
        draw_disk(surf, LOAD_CX, DISK_CY, DISK_R,
                  theta_l,
                  fill_color=lerp_color(C["panel"], C["accent0"], 0.35),
                  border_color=C["accent0"],
                  label="LOAD  Jₗ",
                  font_big=self.font_big,
                  font_sm=self.font_sm,
                  torque_mag=0.0,
                  trail=self._load_trail)

        # ── Torque arrow on motor ─────────────────────────────────────
        draw_torque_arrow(surf, MOTOR_CX, DISK_CY, DISK_R,
                          u, self._u_max, self.font_sm)

        # ── Δθ annotation between disks ───────────────────────────────
        mid_x = (MOTOR_CX + LOAD_CX) // 2
        ann_y = DISK_CY - DISK_R - 44
        col_ann = torque_color(tau_norm)
        draw_text(surf, self.font_big,
                  f"Δθ = {delta_v:+.3f} rad",
                  mid_x, ann_y, col_ann, align="center")
        draw_text(surf, self.font_sm,
                  f"τ_c = {tau_c:+.2f} N·m",
                  mid_x, ann_y + 20, col_ann, align="center")

        # ── Telemetry panel ───────────────────────────────────────────
        telem_rect = pygame.Rect(0, SCENE_Y, TELEM_W, SCENE_H)
        draw_telemetry(surf, telem_rect, {
            "theta_l":  r.x[i, 0],
            "theta_m":  r.x[i, 2],
            "theta_d":  r.theta_d[i],
            "delta":    delta_v,
            "nu":       self.nu[i],
            "tau_c":    tau_c,
            "u":        u,
            "energy":   energy,
            "lyapunov": r.lyapunov[i],
            "e1":       r.e1[i],
            "e2":       r.e2[i],
            "e3":       r.e3[i],
        }, self.font_title, self.font_mono)

        # ── Energy bar panel ──────────────────────────────────────────
        ebar_rect = pygame.Rect(SCENE_X + SCENE_W, SCENE_Y,
                                ENERGY_W, SCENE_H)
        draw_energy_bars(surf, ebar_rect,
                         E_lo, E_mo, E_el, energy,
                         self._E_max,
                         self.font_title, self.font_sm)

        # ── Mini-plots ────────────────────────────────────────────────
        self._push_frame_data(i)
        for mp in self.mini_plots:
            mp.draw(surf, self.font_sm)

        # ── Divider between scene and mini-plots ──────────────────────
        pygame.draw.line(surf, C["border"],
                         (0, PLOT_AREA_Y),
                         (WIN_W, PLOT_AREA_Y), 1)

        pygame.display.flip()

    # ── advance playback ─────────────────────────────────────────────

    def _advance(self, dt_real: float) -> None:
        if self.paused:
            return
        n = len(self.r.t)
        if n < 2:
            return
        sim_dt = float(self.r.t[1] - self.r.t[0])
        # How many simulation frames correspond to real dt?
        frames_per_real = self.speed / (sim_dt * FPS_TARGET)
        self._frac_acc += frames_per_real
        steps = int(self._frac_acc)
        self._frac_acc -= steps
        self.frame = min(self.frame + steps, n - 1)

    # ── event handling ───────────────────────────────────────────────

    def _handle_events(self) -> bool:
        """Return False to quit."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            if event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_ESCAPE, pygame.K_q):
                    return False
                elif event.key == pygame.K_SPACE:
                    self.paused = not self.paused
                elif event.key == pygame.K_RIGHT:
                    self.frame = min(self.frame + 1, len(self.r.t) - 1)
                elif event.key == pygame.K_LEFT:
                    self.frame = max(self.frame - 1, 0)
                elif event.key == pygame.K_UP:
                    self.speed = min(self.speed * 2.0, 32.0)
                elif event.key == pygame.K_DOWN:
                    self.speed = max(self.speed / 2.0, 0.125)
                elif event.key == pygame.K_r:
                    self.frame = 0
                    self._frac_acc = 0.0
                    self._motor_trail.clear()
                    self._load_trail.clear()
                    for mp in self.mini_plots:
                        mp.history.clear()
                        if mp.ref_history is not None:
                            mp.ref_history.clear()
        return True

    # ── public entry point ───────────────────────────────────────────

    def run(self) -> None:
        """Enter the pygame event loop."""
        running = True
        capture_gif = self.save_gif
        gif_written = False

        while running:
            dt_real = self.clock.tick(FPS_TARGET) / 1000.0
            running = self._handle_events()
            self._advance(dt_real)
            self._draw()

            # Capture frame for GIF export
            if capture_gif and not gif_written:
                i = self.frame
                if i % self._gif_stride == 0:
                    frame_copy = self.screen.copy()
                    # Downscale 2× for reasonable GIF size
                    small = pygame.transform.scale(
                        frame_copy,
                        (WIN_W // 2, WIN_H // 2)
                    )
                    self._gif_frames.append(small)
                if i >= len(self.r.t) - 1:
                    # Reached end – write GIF and stop capturing
                    if self.gif_path and self._gif_frames:
                        _try_save_gif(self._gif_frames, self.gif_path,
                                      fps=self.gif_fps)
                    gif_written = True

        pygame.quit()


# ─────────────────────────────────────────────────────────────────────
# Public API  (called from visualization.py / main.py)
# ─────────────────────────────────────────────────────────────────────

def create_animation(result: SimResult,
                     save_gif: bool = False,
                     gif_path: Path | None = None,
                     gif_fps: int = 20,
                     gif_max_frames: int = 400) -> None:
    """
    Launch the pygame animation window.

    Parameters
    ----------
    result          : SimResult from simulation.py
    save_gif        : if True, write frames to a GIF (requires Pillow)
    gif_path        : output path for the GIF
    gif_fps         : GIF playback speed
    gif_max_frames  : max frames captured (downsampled evenly)
    """
    animator = FlexibleJointAnimator(
        result,
        save_gif=save_gif,
        gif_path=gif_path,
        gif_fps=gif_fps,
        gif_max_frames=gif_max_frames,
    )
    animator.run()