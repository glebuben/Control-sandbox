"""
visualization_aircraft.py
-------------------------
Pygame visualisation: side-view aircraft pitching in real time, with a live
Lyapunov function plot strip at the bottom of the window.

Layout (1280 × 780)
-------------------
  ┌──────────────────────────────────────────┐
  │  HUD (left)   │  Sky/aircraft scene      │  ← SCENE_H = 580 px
  ├──────────────────────────────────────────┤
  │  Lyapunov V(t) strip  (full width)       │  ← PLOT_H  = 160 px
  │  [V_total | ½r² | est term]  scrolling   │
  ├──────────────────────────────────────────┤
  │  Footer: time bar + key hints            │  ← FOOTER_H = 40 px
  └──────────────────────────────────────────┘

Controls
--------
  SPACE / →  step forward      ←   step backward
  R          restart            A   toggle auto-play
  + / -      speed up/down      S   screenshot
  G          export GIF now     Q   quit

Usage (standalone)
------------------
  python visualization_aircraft.py                        # adaptive, interactive
  python visualization_aircraft.py --controller baseline  # baseline controller
  python visualization_aircraft.py --gif-only             # GIF, no window

Usage (from main.py)
--------------------
  from visualization_aircraft import run_aircraft_view, export_gif
  run_aircraft_view(results, label="Adaptive")
  export_gif(results, "results/adaptive.gif", label="Adaptive")
"""

from __future__ import annotations

import os, sys, math, argparse
import numpy as np

# ---------------------------------------------------------------------------
# Window layout constants
# ---------------------------------------------------------------------------
W        = 1280
SCENE_H  = 560     # sky + aircraft area
PLOT_H   = 140     # Lyapunov strip
FOOTER_H =  40
H        = SCENE_H + PLOT_H + FOOTER_H   # 740 total

GAMMA_C  = 300.0   # must match controller default

# ---------------------------------------------------------------------------
# Colour palette
# ---------------------------------------------------------------------------
SKY_TOP     = ( 10,  18,  50)
SKY_BOT     = ( 35,  80, 160)
GROUND_TOP  = ( 45,  85,  35)
GROUND_BOT  = ( 28,  52,  22)
HORIZON_COL = (170, 195, 220)

HUD_BG      = ( 10,  12,  22)
HUD_TEXT    = (210, 220, 235)
HUD_WARN    = (255, 200,  55)
HUD_DANGER  = (240,  60,  60)
HUD_OK      = ( 80, 210, 120)
HUD_DIM     = (110, 120, 140)
HUD_GRID    = ( 45,  55,  72)

FUSE_COL    = (215, 222, 238)
WING_COL    = (185, 192, 208)
ELEV_NOM    = (175, 182, 198)
ELEV_ACT    = (255, 175,  50)
ENG_COL     = ( 95,  98, 115)
COCKPIT_COL = (110, 175, 220)
ICE_COL     = (155, 210, 248)

PLOT_BG     = ( 12,  15,  25)
PLOT_BORDER = ( 45,  55,  72)
PLOT_GRID   = ( 22,  28,  42)
COL_TOTAL   = ( 56, 158, 255)   # V total — blue
COL_TRACK   = ( 80, 210, 120)   # ½r²     — green
COL_EST     = (220,  80,  80)   # est term — red
COL_CURSOR  = (200, 200, 200)
COL_ICE_MRK = (220,  60,  60)
COL_ADP_MRK = (255, 160,  40)

import pygame
import pygame.gfxdraw


# ---------------------------------------------------------------------------
# Lyapunov helpers
# ---------------------------------------------------------------------------
def _compute_lyapunov_history(results: dict) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return (V_total, V_track, V_est) arrays over the full simulation."""
    r         = results["r"]
    hat       = results["delta_CL_alpha_hat"]
    true_d    = results["C_La_iced"] - results["C_La_clean"]
    V_track   = 0.5 * r ** 2
    tilde     = hat - true_d
    V_est     = 0.5 / GAMMA_C * tilde ** 2
    V_total   = V_track + V_est
    return V_total, V_track, V_est


# ---------------------------------------------------------------------------
# Drawing primitives
# ---------------------------------------------------------------------------
def _lerp_col(c1, c2, t):
    return tuple(int(c1[i] + (c2[i] - c1[i]) * t) for i in range(3))


def _rot_pt(px, py, cx, cy, a):
    s, c = math.sin(a), math.cos(a)
    dx, dy = px - cx, py - cy
    return (cx + dx * c - dy * s, cy + dx * s + dy * c)


def _rot_pts(pts, cx, cy, a):
    return [_rot_pt(px, py, cx, cy, a) for px, py in pts]


def _solid_poly(surf, pts, color):
    ipts = [(int(x), int(y)) for x, y in pts]
    pygame.draw.polygon(surf, color, ipts)
    pygame.gfxdraw.aapolygon(surf, ipts, color)


def _alpha_poly(surf, overlay, pts, color, alpha):
    overlay.fill((0, 0, 0, 0))
    ipts = [(int(x), int(y)) for x, y in pts]
    pygame.draw.polygon(overlay, (*color, alpha), ipts)
    pygame.gfxdraw.aapolygon(overlay, ipts, (*color, alpha))
    surf.blit(overlay, (0, 0))


# ---------------------------------------------------------------------------
# Aircraft geometry
# ---------------------------------------------------------------------------
def _build_geom(s=1.0):
    g = {}
    g["fuselage"] = [(88*s*math.cos(math.radians(a)),
                      13*s*math.sin(math.radians(a)))
                     for a in range(0, 361, 9)]
    g["wing_l"]  = [(-8*s,-5*s),(-8*s,5*s),(-82*s,18*s),(-82*s,-13*s)]
    g["wing_r"]  = [(-8*s,-5*s),(-8*s,5*s),( 82*s,18*s),( 82*s,-13*s)]
    g["stab_l"]  = [(-72*s,-3*s),(-72*s,3*s),(-108*s, 9*s),(-108*s,-7*s)]
    g["stab_r"]  = [(-72*s,-3*s),(-72*s,3*s),(-108*s, 9*s),(-108*s,-7*s)]
    g["fin"]     = [(-72*s,0),(-62*s,0),(-90*s,-32*s)]
    g["elev_l"]  = [(-107*s,-6*s),(-107*s,8*s),(-128*s,11*s),(-128*s,-9*s)]
    g["elev_r"]  = [(-107*s,-6*s),(-107*s,8*s),(-128*s,11*s),(-128*s,-9*s)]
    g["eng_l"]   = [(-18*s,10*s),(-52*s,15*s),(-56*s,23*s),(-14*s,21*s)]
    g["eng_r"]   = [(-18*s,10*s),(-52*s,15*s),(-56*s,23*s),(-14*s,21*s)]
    g["cockpit"] = [(72*s,7*s),(86*s,2*s),(80*s,-4*s),(63*s,-2*s)]
    return g


def _draw_aircraft(surf, overlay, cx, cy, theta_rad, de_rad, ice_frac, s=1.0):
    g     = _build_geom(s)
    pitch = -theta_rad
    hinge = -107 * s

    def to_scr(pts):
        rot = _rot_pts(pts, 0, 0, pitch)
        return [(cx + px, cy - py) for px, py in rot]

    def elev_pts(raw, sign):
        rot = [_rot_pt(px, py, hinge, 0.0, de_rad * sign) for px, py in raw]
        return to_scr(rot)

    _solid_poly(surf, to_scr(g["stab_l"]), WING_COL)
    _solid_poly(surf, to_scr(g["stab_r"]), WING_COL)
    de_col = ELEV_ACT if abs(de_rad) > 0.02 else ELEV_NOM
    _solid_poly(surf, elev_pts(g["elev_l"],  1), de_col)
    _solid_poly(surf, elev_pts(g["elev_r"], -1), de_col)
    _solid_poly(surf, to_scr(g["fin"]),    WING_COL)
    _solid_poly(surf, to_scr(g["wing_l"]), WING_COL)
    _solid_poly(surf, to_scr(g["wing_r"]), WING_COL)
    if ice_frac > 0.01:
        av = int(ice_frac * 160)
        _alpha_poly(surf, overlay, to_scr(g["wing_l"]), ICE_COL, av)
        _alpha_poly(surf, overlay, to_scr(g["wing_r"]), ICE_COL, av)
    _solid_poly(surf, to_scr(g["eng_l"]),    ENG_COL)
    _solid_poly(surf, to_scr(g["eng_r"]),    ENG_COL)
    _solid_poly(surf, to_scr(g["fuselage"]), FUSE_COL)
    _solid_poly(surf, to_scr(g["cockpit"]),  COCKPIT_COL)


# ---------------------------------------------------------------------------
# Background (pre-built gradient surfaces)
# ---------------------------------------------------------------------------
_SKY_SURF = None
_GND_SURF = None


def _get_bg():
    global _SKY_SURF, _GND_SURF
    if _SKY_SURF is None:
        _SKY_SURF = pygame.Surface((W, SCENE_H))
        for row in range(SCENE_H):
            _SKY_SURF.fill(_lerp_col(SKY_TOP, SKY_BOT, row / SCENE_H),
                           rect=(0, row, W, 1))
        _GND_SURF = pygame.Surface((W, SCENE_H))
        for row in range(SCENE_H):
            _GND_SURF.fill(_lerp_col(GROUND_TOP, GROUND_BOT, row / SCENE_H),
                           rect=(0, row, W, 1))
    return _SKY_SURF, _GND_SURF


def _draw_background(surf, theta_rad):
    sky, gnd = _get_bg()
    offset    = int(math.sin(theta_rad) * SCENE_H * 0.45)
    horizon_y = max(60, min(SCENE_H - 60, SCENE_H // 2 + offset))
    surf.blit(sky, (0, 0),         area=(0, 0,            W, horizon_y))
    surf.blit(gnd, (0, horizon_y), area=(0, horizon_y,    W, SCENE_H - horizon_y))
    pygame.draw.line(surf, HORIZON_COL, (0, horizon_y), (W, horizon_y), 2)


# ---------------------------------------------------------------------------
# HUD panel (left side of scene area)
# ---------------------------------------------------------------------------
def _draw_hud(surf, results, frame, fonts, label: str):
    t       = results["t"]
    t_ice   = results["t_ice"]
    t_now   = t[frame]
    iced    = t_now >= t_ice
    adp     = bool(results["adaptive_mode"][frame])

    alpha_deg = math.degrees(results["alpha"][frame])
    q_deg     = math.degrees(results["q"][frame])
    theta_deg = math.degrees(results["theta"][frame])
    de_deg    = math.degrees(results["delta_e"][frame])
    V_val     = results["V"][frame]
    r_val     = results["r"][frame]
    hat       = results["delta_CL_alpha_hat"][frame]
    true_d    = results["C_La_iced"] - results["C_La_clean"]
    ref_deg   = math.degrees(results["alpha_ref"][frame])

    lya_track = 0.5 * r_val ** 2
    tilde     = hat - true_d
    lya_est   = 0.5 / GAMMA_C * tilde ** 2
    lya_total = lya_track + lya_est

    fmd = fonts["md"]
    fsm = fonts["sm"]
    fxs = fonts["xs"]

    panel_w, panel_h = 318, 370
    panel = pygame.Surface((panel_w, panel_h))
    panel.fill(HUD_BG)
    panel.set_alpha(210)
    surf.blit(panel, (10, 10))
    pygame.draw.rect(surf, HUD_GRID, (10, 10, panel_w, panel_h), 1)

    def row(lbl, val, col, y):
        surf.blit(fsm.render(lbl, True, HUD_DIM),  (18, y))
        surf.blit(fmd.render(val, True, col),        (170, y))

    y = 18
    # Title shows which controller is active
    ctrl_col = HUD_WARN if "Adaptive" in label else (100, 180, 255)
    surf.blit(fmd.render(f"CONTROLLER: {label.upper()}", True, ctrl_col), (18, y)); y += 22
    pygame.draw.line(surf, HUD_GRID, (14, y), (14+panel_w-8, y), 1); y += 7

    row("Time",      f"{t_now:7.2f} s",          HUD_TEXT, y); y += 20
    row("α  (ref)",  f"{alpha_deg:+7.2f}°  ({ref_deg:.1f}°)",
        HUD_WARN if abs(alpha_deg - ref_deg) > 1.0 else HUD_OK, y); y += 20
    row("q",         f"{q_deg:+7.2f} °/s",        HUD_TEXT, y); y += 20
    row("θ",         f"{theta_deg:+7.2f}°",        HUD_TEXT, y); y += 20
    row("V",         f"{V_val:7.2f} m/s",          HUD_TEXT, y); y += 20
    row("δe",        f"{de_deg:+7.2f}°",
        HUD_WARN if abs(de_deg) > 15 else HUD_TEXT, y); y += 20
    pygame.draw.line(surf, HUD_GRID, (14, y), (14+panel_w-8, y), 1); y += 7

    row("Lyap V",    f"{lya_total:.4e}",           HUD_TEXT, y); y += 18
    row("  ½r²",     f"{lya_track:.4e}",            HUD_DIM,  y); y += 18
    row("  est",     f"{lya_est:.4e}",              HUD_DIM,  y); y += 20
    pygame.draw.line(surf, HUD_GRID, (14, y), (14+panel_w-8, y), 1); y += 7

    row("ΔĈ_Lα",     f"{hat:+.4f}",
        HUD_WARN if abs(hat) > 0.05 else HUD_DIM, y); y += 20
    row("ΔC_Lα*",    f"{true_d:+.4f}",             HUD_DANGER, y); y += 20
    pct = 100*hat/true_d if abs(true_d) > 1e-6 else 0.0
    row("Est. acc.", f"{pct:.1f} %",
        HUD_OK if abs(pct) > 80 else HUD_WARN,    y); y += 20
    pygame.draw.line(surf, HUD_GRID, (14, y), (14+panel_w-8, y), 1); y += 8

    ice_col = HUD_DANGER if iced  else HUD_DIM
    adp_col = HUD_WARN   if adp   else HUD_DIM
    surf.blit(fmd.render("● ICING ACTIVE"  if iced else "○ CLEAN",        True, ice_col), (18, y)); y += 22
    surf.blit(fmd.render("● ADAPTIVE MODE" if adp  else "○ NOMINAL MODE", True, adp_col), (18, y))


# ---------------------------------------------------------------------------
# Lyapunov plot strip
# ---------------------------------------------------------------------------
def _draw_lyapunov_strip(surf, lya_total, lya_track, lya_est,
                          results, frame, fonts):
    """
    Draw the Lyapunov strip below the scene area.
    Shows V_total, ½r², and estimation term as scrolling time-series lines,
    with a vertical cursor at the current frame and event markers.
    """
    t     = results["t"]
    t_ice = results["t_ice"]
    N     = len(t)

    # Strip pixel rectangle (inside the window)
    px, py = 0, SCENE_H
    pw, ph = W, PLOT_H

    # Inner plot margins
    ml, mr, mt, mb = 58, 12, 18, 24   # left, right, top, bottom margin

    plot_x = px + ml
    plot_y = py + mt
    plot_w = pw - ml - mr
    plot_h = ph - mt - mb

    # Background
    pygame.draw.rect(surf, PLOT_BG, (px, py, pw, ph))
    pygame.draw.line(surf, PLOT_BORDER, (px, py), (px+pw, py), 1)  # top border

    fxs = fonts["xs"]

    # Y range: use symlog-like scale — find max of total up to current frame
    V_window = lya_total[:frame+1]
    V_max    = float(np.max(V_window)) if len(V_window) > 0 else 1.0
    V_max    = max(V_max, 1e-8)
    # Add 20% headroom
    y_hi  = V_max * 1.25
    y_lo  = 0.0

    def to_px(t_val, v_val):
        """Map (time, value) to pixel coords inside the plot area."""
        fx = (t_val - t[0]) / max(t[-1] - t[0], 1e-9)
        fy = (v_val - y_lo)  / max(y_hi - y_lo,  1e-12)
        fy = max(0.0, min(1.0, fy))
        x  = int(plot_x + fx * plot_w)
        y  = int(plot_y + plot_h - fy * plot_h)
        return x, y

    # Grid lines — 4 horizontal
    for gi in range(5):
        gv   = y_lo + gi / 4 * (y_hi - y_lo)
        _, gy = to_px(t[0], gv)
        pygame.draw.line(surf, PLOT_GRID, (plot_x, gy), (plot_x+plot_w, gy), 1)
        lbl = fxs.render(f"{gv:.2e}", True, HUD_DIM)
        surf.blit(lbl, (px + 2, gy - 5))

    # Icing onset vertical marker
    gx_ice, _ = to_px(t_ice, 0)
    pygame.draw.line(surf, COL_ICE_MRK,
                     (gx_ice, plot_y), (gx_ice, plot_y+plot_h), 1)
    surf.blit(fxs.render("ice", True, COL_ICE_MRK), (gx_ice+2, plot_y))

    # Adaptive mode onset marker
    adp = results["adaptive_mode"]
    if np.any(adp):
        idx_adp = int(np.argmax(adp))
        gx_adp, _ = to_px(t[idx_adp], 0)
        pygame.draw.line(surf, COL_ADP_MRK,
                         (gx_adp, plot_y), (gx_adp, plot_y+plot_h), 1)
        surf.blit(fxs.render("adp", True, COL_ADP_MRK), (gx_adp+2, plot_y))

    # Draw the three series up to current frame
    def draw_series(arr, color, lw=1):
        pts = []
        for i in range(frame + 1):
            pts.append(to_px(t[i], arr[i]))
        if len(pts) >= 2:
            pygame.draw.lines(surf, color, False, pts, lw)

    draw_series(lya_total, COL_TOTAL, lw=2)
    draw_series(lya_track, COL_TRACK, lw=1)
    draw_series(lya_est,   COL_EST,   lw=1)

    # Current-frame cursor
    cx_cur, _ = to_px(t[frame], 0)
    pygame.draw.line(surf, COL_CURSOR,
                     (cx_cur, plot_y), (cx_cur, plot_y+plot_h), 1)

    # Plot border
    pygame.draw.rect(surf, PLOT_BORDER, (plot_x, plot_y, plot_w, plot_h), 1)

    # Y-axis label (rotated)
    lbl = fxs.render("V(t)", True, HUD_DIM)
    lbl_r = pygame.transform.rotate(lbl, 90)
    surf.blit(lbl_r, (px + 2, plot_y + plot_h//2 - lbl_r.get_height()//2))

    # Legend (top-right)
    leg_items = [
        ("V total", COL_TOTAL),
        ("½r²",     COL_TRACK),
        ("est",     COL_EST),
    ]
    lx = plot_x + plot_w - 4
    ly = plot_y + 3
    for leg_lbl, leg_col in reversed(leg_items):
        ls = fxs.render(leg_lbl, True, leg_col)
        lx -= ls.get_width() + 24
        surf.blit(ls, (lx, ly))
        pygame.draw.line(surf, leg_col, (lx-20, ly+5), (lx-4, ly+5), 2)


# ---------------------------------------------------------------------------
# Footer bar
# ---------------------------------------------------------------------------
def _draw_footer(surf, results, frame, fonts, speed, playing):
    fxs = fonts["xs"]
    fmd = fonts["md"]
    t     = results["t"]
    t_now = t[frame]
    N     = len(t)

    fy = SCENE_H + PLOT_H
    pygame.draw.rect(surf, HUD_BG, (0, fy, W, FOOTER_H))
    pygame.draw.line(surf, PLOT_BORDER, (0, fy), (W, fy), 1)

    # Scrubber bar
    sb_x, sb_w = 160, W - 320
    sb_y, sb_h = fy + 13, 12
    pygame.draw.rect(surf, PLOT_GRID, (sb_x, sb_y, sb_w, sb_h), border_radius=3)
    fill_w = int(sb_w * frame / max(N-1, 1))
    if fill_w > 0:
        pygame.draw.rect(surf, (56, 140, 230), (sb_x, sb_y, fill_w, sb_h), border_radius=3)
    pygame.draw.circle(surf, HUD_TEXT, (sb_x + fill_w, sb_y + sb_h//2), 7)

    # Icing marker on scrubber
    t_ice = results["t_ice"]
    t_end = t[-1]
    ix = sb_x + int(sb_w * t_ice / t_end)
    pygame.draw.line(surf, COL_ICE_MRK, (ix, sb_y-2), (ix, sb_y+sb_h+2), 2)

    # Time label
    surf.blit(fmd.render(f"t={t_now:.1f}/{t_end:.0f}s", True, HUD_TEXT), (10, fy+10))

    # Play indicator + speed
    play_str = f"{'▶' if playing else '⏸'}  ×{speed}"
    surf.blit(fxs.render(play_str, True, HUD_OK if playing else HUD_DIM),
              (W - 160, fy + 8))

    # Key hints
    hints = "[SPACE/→] step  [←] back  [R] restart  [A] play  [+/-] speed  [G] GIF  [S] shot  [Q] quit"
    surf.blit(fxs.render(hints, True, HUD_DIM), (sb_x, fy + FOOTER_H - 13))


# ---------------------------------------------------------------------------
# Full frame render
# ---------------------------------------------------------------------------
def render_frame(surf, overlay, results, frame, fonts,
                 lya_total, lya_track, lya_est,
                 label: str = "Adaptive", scale: float = 1.0):
    t_ice    = results["t_ice"]
    t_now    = results["t"][frame]
    theta    = float(results["theta"][frame])
    de       = float(results["delta_e"][frame])
    ice_frac = min(1.0, max(0.0, (t_now - t_ice) / 2.0)) if t_now >= t_ice else 0.0

    # Scene (top SCENE_H rows only)
    scene_clip = pygame.Rect(0, 0, W, SCENE_H)
    surf.set_clip(scene_clip)
    _draw_background(surf, theta)
    _draw_aircraft(surf, overlay,
                   W // 2, SCENE_H // 2,
                   theta, de, ice_frac, s=scale)
    _draw_hud(surf, results, frame, fonts, label)
    surf.set_clip(None)

    # Lyapunov strip
    _draw_lyapunov_strip(surf, lya_total, lya_track, lya_est,
                          results, frame, fonts)

    # Footer
    from visualization_aircraft import _playing_ref, _speed_ref
    _draw_footer(surf, results, frame, fonts,
                 _speed_ref[0], _playing_ref[0])


# Mutable refs so render_frame can read playback state without globals
_playing_ref = [False]
_speed_ref   = [4]


# ---------------------------------------------------------------------------
# Fonts
# ---------------------------------------------------------------------------
def _make_fonts():
    return {
        "lg": pygame.font.SysFont("monospace", 18, bold=True),
        "md": pygame.font.SysFont("monospace", 13, bold=True),
        "sm": pygame.font.SysFont("monospace", 11),
        "xs": pygame.font.SysFont("monospace",  9),
    }


# ---------------------------------------------------------------------------
# GIF export  (headless-safe)
# ---------------------------------------------------------------------------
def export_gif(results: dict,
               path:    str,
               step:    int   = 15,
               fps:     int   = 25,
               label:   str   = "Adaptive",
               scale:   float = 1.0) -> None:
    """
    Render the full simulation to an animated GIF.

    Parameters
    ----------
    results : simulation results dict
    path    : output .gif file path
    step    : take every N-th frame (reduces file size)
    fps     : output frames per second
    label   : controller label shown in HUD ("Adaptive" or "Baseline")
    scale   : aircraft scale factor
    """
    try:
        from PIL import Image
    except ImportError:
        print("[gif] Pillow not installed — run: pip install pillow")
        return

    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)

    was_init = pygame.get_init()
    if not was_init:
        pygame.init()
    if not pygame.font.get_init():
        pygame.font.init()

    surf    = pygame.Surface((W, H))
    overlay = pygame.Surface((W, H), pygame.SRCALPHA)
    fonts   = _make_fonts()

    lya_total, lya_track, lya_est = _compute_lyapunov_history(results)

    frame_indices = list(range(0, len(results["t"]), step))
    print(f"[gif] Rendering {len(frame_indices)} frames (step={step}, label={label}) ...")

    _playing_ref[0] = True
    _speed_ref[0]   = step

    pil_frames = []
    for i, fi in enumerate(frame_indices):
        render_frame(surf, overlay, results, fi, fonts,
                     lya_total, lya_track, lya_est,
                     label=label, scale=scale)
        raw = pygame.surfarray.array3d(surf)
        img = Image.fromarray(raw.swapaxes(0, 1))
        img = img.resize((W // 2, H // 2), Image.LANCZOS)
        pil_frames.append(img)
        if i % 100 == 0:
            print(f"  {i}/{len(frame_indices)}")

    duration_ms = max(20, int(1000 / fps))
    pil_frames[0].save(
        path,
        save_all=True,
        append_images=pil_frames[1:],
        loop=0,
        duration=duration_ms,
    )
    sz_kb = os.path.getsize(path) // 1024
    print(f"[gif] Saved -> {path}  ({len(pil_frames)} frames, {fps} fps, {sz_kb} KB)")

    if not was_init:
        pygame.quit()


# ---------------------------------------------------------------------------
# Interactive viewer
# ---------------------------------------------------------------------------
class AircraftViewer:
    def __init__(self, results: dict,
                 label:    str = "Adaptive",
                 gif_path: str = "results/aircraft_sim.gif"):
        self.results  = results
        self.label    = label
        self.N        = len(results["t"])
        self.frame    = 0
        self.playing  = False
        self.speed    = 4
        self.gif_path = gif_path

        self.lya_total, self.lya_track, self.lya_est = \
            _compute_lyapunov_history(results)

        pygame.init()
        if not pygame.font.get_init():
            pygame.font.init()

        self.screen  = pygame.display.set_mode((W, H))
        pygame.display.set_caption(
            f"Aircraft Pitch Stabilisation — {label} Controller")
        self.clock   = pygame.time.Clock()
        self.fonts   = _make_fonts()
        self.overlay = pygame.Surface((W, H), pygame.SRCALPHA)

    def run(self):
        running = True
        while running:
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    running = False
                elif ev.type == pygame.KEYDOWN:
                    k = ev.key
                    if k in (pygame.K_q, pygame.K_ESCAPE):
                        running = False
                    elif k in (pygame.K_SPACE, pygame.K_RIGHT):
                        self.frame = min(self.frame + 1, self.N - 1)
                    elif k == pygame.K_LEFT:
                        self.frame = max(self.frame - 1, 0)
                    elif k == pygame.K_r:
                        self.frame = 0; self.playing = False
                    elif k == pygame.K_a:
                        self.playing = not self.playing
                    elif k in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                        self.speed = min(self.speed + 1, 30)
                    elif k in (pygame.K_MINUS, pygame.K_KP_MINUS):
                        self.speed = max(self.speed - 1, 1)
                    elif k == pygame.K_s:
                        self._screenshot()
                    elif k == pygame.K_g:
                        export_gif(self.results, self.gif_path,
                                   label=self.label)
                elif ev.type == pygame.MOUSEBUTTONDOWN:
                    self._handle_scrubber_click(ev.pos)

            if self.playing:
                self.frame = min(self.frame + self.speed, self.N - 1)
                if self.frame >= self.N - 1:
                    self.playing = False

            _playing_ref[0] = self.playing
            _speed_ref[0]   = self.speed

            render_frame(self.screen, self.overlay, self.results,
                         self.frame, self.fonts,
                         self.lya_total, self.lya_track, self.lya_est,
                         label=self.label)
            pygame.display.flip()
            self.clock.tick(60)

        pygame.quit()

    def _handle_scrubber_click(self, pos):
        mx, my = pos
        sb_x, sb_w = 160, W - 320
        sb_y = SCENE_H + PLOT_H + 13
        if sb_x <= mx <= sb_x + sb_w and sb_y - 8 <= my <= sb_y + 20:
            frac = (mx - sb_x) / sb_w
            self.frame = max(0, min(self.N-1, int(frac * (self.N-1))))

    def _screenshot(self):
        os.makedirs("results", exist_ok=True)
        tag = self.label.lower().replace(" ", "_")
        p   = f"results/aircraft_{tag}_frame{self.frame:05d}.png"
        pygame.image.save(self.screen, p)
        print(f"[aircraft] Screenshot -> {p}")


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------
def run_aircraft_view(results: dict,
                      label:    str = "Adaptive",
                      gif_path: str = "results/aircraft_sim.gif") -> None:
    """
    Open the interactive aircraft viewer window.

    Parameters
    ----------
    results  : simulation results dict
    label    : "Adaptive" or "Baseline" — shown in HUD and window title
    gif_path : path used when pressing G inside the viewer
    """
    viewer = AircraftViewer(results, label=label, gif_path=gif_path)
    viewer.run()


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    sys.path.insert(0, os.path.dirname(__file__))
    from simulation import SimConfig, run_simulation

    ap = argparse.ArgumentParser()
    ap.add_argument("--t-end",        type=float, default=200.0)
    ap.add_argument("--t-ice",        type=float, default=10.0)
    ap.add_argument("--severity",     type=float, default=0.30)
    ap.add_argument("--controller",   type=str,   default="adaptive",
                    choices=["adaptive", "baseline"],
                    help="Which controller to animate (default: adaptive)")
    ap.add_argument("--gif-only",     action="store_true",
                    help="Export GIF only, do not open interactive window")
    ap.add_argument("--gif-step",     type=int,   default=15)
    ap.add_argument("--gif-fps",      type=int,   default=25)
    ap.add_argument("--out-dir",      type=str,   default="results")
    args = ap.parse_args()

    use_adp = (args.controller == "adaptive")
    label   = "Adaptive" if use_adp else "Baseline"

    delta_CL_alpha_ice = -args.severity * 3.50
    cfg = SimConfig(t_end=args.t_end, t_ice=args.t_ice,
                    delta_CL_alpha_ice=delta_CL_alpha_ice)

    print(f"Running {label} simulation ...")
    res = run_simulation(config=cfg, use_adaptation=use_adp)

    gif_path = os.path.join(args.out_dir, f"aircraft_{args.controller}.gif")

    if args.gif_only:
        pygame.init()
        export_gif(res, gif_path, step=args.gif_step,
                   fps=args.gif_fps, label=label)
        pygame.quit()
    else:
        pygame.init()
        export_gif(res, gif_path, step=args.gif_step,
                   fps=args.gif_fps, label=label)
        run_aircraft_view(res, label=label, gif_path=gif_path)