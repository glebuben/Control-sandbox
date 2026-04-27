"""
visualization_pygame.py
-----------------------
Real-time animated dashboard for the adaptive icing simulation using pygame.

Layout (1200 × 800 window):
┌──────────────────────────────────────────────────────┐
│  HEADER: title + mode badge + icing indicator        │
├────────────────────────┬─────────────────────────────┤
│  Plot 1: α & α_ref     │  Plot 2: pitch rate q       │
├────────────────────────┼─────────────────────────────┤
│  Plot 3: elevator δe   │  Plot 4: ΔĈ_Lα estimate     │
├────────────────────────┴─────────────────────────────┤
│  Plot 5: airspeed V  (full width)                    │
├──────────────────────────────────────────────────────┤
│  FOOTER: time scrubber + key hints                   │
└──────────────────────────────────────────────────────┘

Controls
--------
  SPACE / →   : step forward one frame
  ←           : step backward one frame
  R           : restart from beginning
  A           : toggle auto-play
  +/-         : speed up / slow down auto-play
  S           : save screenshot  (PNG)
  Q / Esc     : quit
"""

from __future__ import annotations

import os
import math
import numpy as np
import pygame
import pygame.gfxdraw


# ---------------------------------------------------------------------------
# Colour palette  (dark cockpit aesthetic)
# ---------------------------------------------------------------------------
C = {
    "bg":           (13,  17,  23),
    "panel":        (22,  27,  34),
    "border":       (48,  54,  61),
    "header":       (17,  21,  28),

    "text_bright":  (220, 225, 235),
    "text_dim":     (110, 118, 130),
    "text_warn":    (255, 200,  60),
    "text_danger":  (255,  75,  75),
    "text_ok":      ( 80, 200, 120),

    "alpha":        ( 56, 158, 255),   # angle of attack
    "alpha_ref":    (180, 180, 180),   # reference
    "q":            (100, 210, 160),   # pitch rate
    "de_total":     (220, 220, 220),   # total elevator
    "de_nom":       ( 80, 130, 200),   # nominal elevator
    "de_adapt":     (220,  80,  80),   # adaptive elevator
    "delta_hat":    (220,  80,  80),   # estimate
    "delta_true":   (120, 120, 120),   # true value
    "V":            ( 56, 158, 255),   # airspeed

    "icing_bg":     ( 80,  20,  20),   # panel tint when iced
    "icing_line":   (200,  60,  60),   # vertical icing marker
    "adapt_line":   (255, 160,  40),   # adaptive-mode marker

    "grid":         ( 35,  42,  52),
    "zero":         ( 60,  70,  85),
    "scrubber":     ( 80, 150, 255),
    "scrubber_bg":  ( 35,  42,  52),
}


# ---------------------------------------------------------------------------
# Utility helpers
# ---------------------------------------------------------------------------
def _deg(r): return float(np.rad2deg(r))


def lerp(a, b, t): return a + (b - a) * t


def map_val(v, v_min, v_max, px_min, px_max):
    """Map a value linearly to pixel space, clamped."""
    if v_max == v_min:
        return (px_min + px_max) // 2
    t = (v - v_min) / (v_max - v_min)
    t = max(0.0, min(1.0, t))
    return int(px_min + t * (px_max - px_min))


def nice_range(arr, pad=0.15):
    """Return (lo, hi) with symmetric padding around data, ensuring nonzero span."""
    lo, hi = float(np.min(arr)), float(np.max(arr))
    span = hi - lo
    if span < 1e-6:
        lo -= 0.5; hi += 0.5; span = 1.0
    lo -= pad * span
    hi += pad * span
    return lo, hi


def aa_line(surf, color, p1, p2, width=2):
    """Anti-aliased line using gfxdraw (falls back to plain draw for width>1)."""
    if width == 1:
        pygame.gfxdraw.line(surf, int(p1[0]), int(p1[1]),
                            int(p2[0]), int(p2[1]), color)
    else:
        pygame.draw.line(surf, color, p1, p2, width)


def rounded_rect(surf, color, rect, radius=8, alpha=255):
    """Draw a filled rounded rectangle."""
    if alpha < 255:
        s = pygame.Surface((rect[2], rect[3]), pygame.SRCALPHA)
        pygame.draw.rect(s, (*color, alpha), (0, 0, rect[2], rect[3]), border_radius=radius)
        surf.blit(s, (rect[0], rect[1]))
    else:
        pygame.draw.rect(surf, color, rect, border_radius=radius)


# ---------------------------------------------------------------------------
# Plot panel class
# ---------------------------------------------------------------------------
class PlotPanel:
    """
    A self-contained scrolling time-series panel.

    Parameters
    ----------
    surf      : target surface
    rect      : (x, y, w, h) pixel rectangle
    title     : panel title
    y_label   : y-axis label string
    series    : list of dicts  {key, color, label, linewidth, dash}
    """

    TICK_COLOR  = C["text_dim"]
    LABEL_COLOR = C["text_dim"]
    N_YTICKS    = 5

    def __init__(self, surf, rect, title, y_label, series,
                 font_sm, font_xs, y_unit=""):
        self.surf     = surf
        self.rect     = rect
        self.title    = title
        self.y_label  = y_label
        self.y_unit   = y_unit
        self.series   = series   # [{"key", "color", "label", "lw", "dash"}]
        self.font_sm  = font_sm
        self.font_xs  = font_xs

        x, y, w, h = rect
        self.pad_l = 58
        self.pad_r = 14
        self.pad_t = 28
        self.pad_b = 30
        self.plot_rect = (
            x + self.pad_l,
            y + self.pad_t,
            w - self.pad_l - self.pad_r,
            h - self.pad_t - self.pad_b,
        )

    def _px(self, t_val, y_val, t_range, y_range):
        pr = self.plot_rect
        px = map_val(t_val, t_range[0], t_range[1], pr[0], pr[0] + pr[2])
        py = map_val(y_val, y_range[0], y_range[1], pr[1] + pr[3], pr[1])
        return px, py

    def draw(self, results, frame_idx, iced, adaptive_on, t_adapt):
        surf = self.surf
        x, y, w, h = self.rect
        pr = self.plot_rect

        # ---- Panel background ------------------------------------------
        bg = C["icing_bg"] if iced else C["panel"]
        rounded_rect(surf, bg, self.rect, radius=6)
        pygame.draw.rect(surf, C["border"], self.rect, width=1, border_radius=6)

        # ---- Title ------------------------------------------------------
        title_surf = self.font_sm.render(self.title, True, C["text_bright"])
        surf.blit(title_surf, (x + self.pad_l, y + 7))

        # ---- Gather data up to frame_idx --------------------------------
        t   = results["t"][:frame_idx + 1]
        if len(t) < 2:
            return

        t_range = (results["t"][0], results["t"][-1])

        # Collect all values for auto y-range
        all_vals = []
        for s in self.series:
            arr = results[s["key"]][:frame_idx + 1]
            if s.get("deg", False):
                arr = np.rad2deg(arr)
            all_vals.append(arr)

        all_flat = np.concatenate(all_vals)
        y_range  = nice_range(all_flat, pad=0.18)

        # ---- Grid & ticks -----------------------------------------------
        # Horizontal grid lines
        for i in range(self.N_YTICKS + 1):
            frac = i / self.N_YTICKS
            yv   = y_range[0] + frac * (y_range[1] - y_range[0])
            _, gy = self._px(0, yv, t_range, y_range)
            pygame.draw.line(surf, C["grid"],
                             (pr[0], gy), (pr[0] + pr[2], gy), 1)
            # y-axis tick label
            lbl = f"{yv:.1f}"
            ts  = self.font_xs.render(lbl, True, self.LABEL_COLOR)
            surf.blit(ts, (x + 4, gy - 7))

        # Zero line
        if y_range[0] < 0 < y_range[1]:
            _, zy = self._px(0, 0.0, t_range, y_range)
            pygame.draw.line(surf, C["zero"], (pr[0], zy), (pr[0] + pr[2], zy), 1)

        # x-axis ticks (every 5 s)
        t_total = results["t"][-1]
        tick_step = 5.0
        tv = 0.0
        while tv <= t_total + 0.01:
            gx, _ = self._px(tv, 0, t_range, y_range)
            pygame.draw.line(surf, C["grid"], (gx, pr[1]), (gx, pr[1] + pr[3]), 1)
            lbl = f"{tv:.0f}s"
            ts  = self.font_xs.render(lbl, True, self.LABEL_COLOR)
            surf.blit(ts, (gx - ts.get_width() // 2, pr[1] + pr[3] + 4))
            tv += tick_step

        # ---- Icing onset vertical line ----------------------------------
        t_ice = results["t_ice"]
        gx_ice, _ = self._px(t_ice, 0, t_range, y_range)
        pygame.draw.line(surf, C["icing_line"],
                         (gx_ice, pr[1]), (gx_ice, pr[1] + pr[3]), 2)

        # ---- Adaptive-mode onset vertical line --------------------------
        if t_adapt is not None:
            gx_ad, _ = self._px(t_adapt, 0, t_range, y_range)
            pygame.draw.line(surf, C["adapt_line"],
                             (gx_ad, pr[1]), (gx_ad, pr[1] + pr[3]), 2)

        # ---- Clip to plot area ------------------------------------------
        clip_rect = pygame.Rect(pr[0], pr[1], pr[2], pr[3])
        surf.set_clip(clip_rect)

        # ---- Draw series ------------------------------------------------
        for s in self.series:
            arr = results[s["key"]][:frame_idx + 1]
            if s.get("deg", False):
                arr = np.rad2deg(arr)
            lw  = s.get("lw", 2)

            pts = []
            for i, (tv, yv) in enumerate(zip(t, arr)):
                px, py = self._px(tv, yv, t_range, y_range)
                pts.append((px, py))

            if len(pts) >= 2:
                # Dash: draw every other segment
                dash = s.get("dash", False)
                step = 2 if dash else 1
                for i in range(0, len(pts) - 1, step if dash else 1):
                    if dash and (i // 2) % 2 == 1:
                        continue
                    aa_line(surf, s["color"], pts[i], pts[i + 1], lw)

        surf.set_clip(None)

        # ---- Plot border ------------------------------------------------
        pygame.draw.rect(surf, C["border"], pr, width=1)

        # ---- Y-axis label ----------------------------------------------
        lbl_surf = self.font_xs.render(self.y_label, True, self.LABEL_COLOR)
        lbl_rot  = pygame.transform.rotate(lbl_surf, 90)
        surf.blit(lbl_rot, (x + 2, y + h // 2 - lbl_rot.get_height() // 2))

        # ---- Legend (top-right inside panel) ----------------------------
        leg_x = pr[0] + pr[2] - 4
        leg_y = pr[1] + 4
        for s in self.series:
            label = s.get("label", s["key"])
            ls = self.font_xs.render(label, True, s["color"])
            leg_x -= ls.get_width() + 22
            # swatch line
            sw_y = leg_y + ls.get_height() // 2
            pygame.draw.line(surf, s["color"],
                             (leg_x - 18, sw_y), (leg_x - 4, sw_y), 2)
            surf.blit(ls, (leg_x, leg_y))


# ---------------------------------------------------------------------------
# Main dashboard
# ---------------------------------------------------------------------------
class SimDashboard:
    W, H = 1280, 820

    def __init__(self, results: dict):
        self.results = results
        self.N       = len(results["t"])
        self.frame   = 0
        self.playing = False
        self.speed   = 3      # frames per tick (1 = slow, 10 = fast)

        # Pre-compute adaptive-mode switch time
        adp = results["adaptive_mode"]
        idx = int(np.argmax(adp))
        self.t_adapt = float(results["t"][idx]) if adp[idx] else None

        pygame.init()
        self.screen = pygame.display.set_mode((self.W, self.H))
        pygame.display.set_caption("Lyapunov Adaptive Icing Controller — Simulation")
        self.clock  = pygame.time.Clock()

        # Fonts
        self.font_lg = pygame.font.SysFont("monospace", 18, bold=True)
        self.font_md = pygame.font.SysFont("monospace", 14, bold=True)
        self.font_sm = pygame.font.SysFont("monospace", 12)
        self.font_xs = pygame.font.SysFont("monospace", 10)

        self._build_layout()

    def _build_layout(self):
        W, H = self.W, self.H
        HEADER_H = 52
        FOOTER_H = 44
        MARGIN    = 8
        GAP       = 6

        content_y = HEADER_H + MARGIN
        content_h = H - HEADER_H - FOOTER_H - 2 * MARGIN

        # Two rows of two panels + one full-width panel
        row1_h = int(content_h * 0.34)
        row2_h = int(content_h * 0.34)
        row3_h = content_h - row1_h - row2_h - 2 * GAP

        half_w = (W - 2 * MARGIN - GAP) // 2

        def panel_rect(col, row_y, row_h, full=False):
            if full:
                return (MARGIN, row_y, W - 2 * MARGIN, row_h)
            x = MARGIN + col * (half_w + GAP)
            return (x, row_y, half_w, row_h)

        r1y = content_y
        r2y = r1y + row1_h + GAP
        r3y = r2y + row2_h + GAP

        results = self.results

        common_kw = dict(font_sm=self.font_sm, font_xs=self.font_xs)

        self.panels = [
            PlotPanel(
                self.screen,
                panel_rect(0, r1y, row1_h),
                "Angle of Attack", "α [deg]",
                [
                    {"key": "alpha_ref", "color": C["alpha_ref"], "label": "α_ref",
                     "lw": 1, "dash": True, "deg": True},
                    {"key": "alpha",     "color": C["alpha"],     "label": "α",
                     "lw": 2, "deg": True},
                ],
                **common_kw,
            ),
            PlotPanel(
                self.screen,
                panel_rect(1, r1y, row1_h),
                "Pitch Rate", "q [deg/s]",
                [
                    {"key": "q", "color": C["q"], "label": "q", "lw": 2, "deg": True},
                ],
                **common_kw,
            ),
            PlotPanel(
                self.screen,
                panel_rect(0, r2y, row2_h),
                "Elevator Deflection", "δe [deg]",
                [
                    {"key": "delta_e_nom",   "color": C["de_nom"],   "label": "nom",
                     "lw": 1, "dash": True, "deg": True},
                    {"key": "delta_e_adapt", "color": C["de_adapt"], "label": "adapt",
                     "lw": 1, "dash": False, "deg": True},
                    {"key": "delta_e",       "color": C["de_total"], "label": "total",
                     "lw": 2, "deg": True},
                ],
                **common_kw,
            ),
            PlotPanel(
                self.screen,
                panel_rect(1, r2y, row2_h),
                "Adaptive Estimate  ΔĈ_Lα", "ΔC_Lα [–]",
                [
                    {"key": "delta_true",        "color": C["delta_true"], "label": "true",
                     "lw": 1, "dash": True},
                    {"key": "delta_CL_alpha_hat", "color": C["delta_hat"],  "label": "est.",
                     "lw": 2},
                ],
                **common_kw,
            ),
            PlotPanel(
                self.screen,
                panel_rect(0, r3y, row3_h, full=True),
                "Airspeed", "V [m/s]",
                [
                    {"key": "V", "color": C["V"], "label": "V", "lw": 2},
                ],
                **common_kw,
            ),
        ]

        self.header_rect = (0, 0, W, HEADER_H)
        self.footer_rect = (0, H - FOOTER_H, W, FOOTER_H)

    # ------------------------------------------------------------------
    # Draw helpers
    # ------------------------------------------------------------------
    def _draw_header(self):
        surf = self.screen
        W    = self.W
        pygame.draw.rect(surf, C["header"], self.header_rect)
        pygame.draw.line(surf, C["border"], (0, self.header_rect[3]),
                         (W, self.header_rect[3]), 1)

        # Title
        t_surf = self.font_lg.render(
            "LYAPUNOV ADAPTIVE ICING CONTROLLER", True, C["text_bright"])
        surf.blit(t_surf, (14, 10))

        # Sub-info
        t_ice = self.results["t_ice"]
        sub   = (f"C_Lα  {self.results['C_La_clean']:.3f} → {self.results['C_La_iced']:.3f}"
                 f"   |   icing at t = {t_ice:.1f} s"
                 f"   |   t_end = {self.results['t'][-1]:.0f} s")
        s_surf = self.font_sm.render(sub, True, C["text_dim"])
        surf.blit(s_surf, (14, 32))

        # Mode badge (right side)
        t_now = self.results["t"][self.frame]
        iced  = t_now >= t_ice
        adp   = bool(self.results["adaptive_mode"][self.frame])

        bx = W - 310
        # Icing badge
        ice_col   = C["text_danger"] if iced  else C["text_dim"]
        ice_label = "● ICING ACTIVE" if iced else "○ CLEAN"
        ice_surf  = self.font_md.render(ice_label, True, ice_col)
        surf.blit(ice_surf, (bx, 10))

        # Adaptive badge
        adp_col   = C["text_warn"]   if adp   else C["text_dim"]
        adp_label = "● ADAPTIVE MODE" if adp else "○ NOMINAL MODE"
        adp_surf  = self.font_md.render(adp_label, True, adp_col)
        surf.blit(adp_surf, (bx, 32))

    def _draw_footer(self):
        surf = self.screen
        W, H = self.W, self.H
        fy   = self.footer_rect[0]
        fh   = self.footer_rect[3]
        fx   = self.footer_rect[1]   # actually y
        fh_  = fh

        pygame.draw.rect(surf, C["header"], self.footer_rect)
        pygame.draw.line(surf, C["border"], (0, H - fh_), (W, H - fh_), 1)

        # Time display
        t_now = self.results["t"][self.frame]
        t_end = self.results["t"][-1]
        t_str = f"t = {t_now:6.2f} s / {t_end:.0f} s"
        ts    = self.font_md.render(t_str, True, C["text_bright"])
        surf.blit(ts, (14, H - fh_ + 8))

        # Scrubber bar
        sb_x  = 200
        sb_w  = W - sb_x - 160
        sb_y  = H - fh_ + 14
        sb_h  = 14
        pygame.draw.rect(surf, C["scrubber_bg"], (sb_x, sb_y, sb_w, sb_h), border_radius=4)
        fill_w = int(sb_w * self.frame / max(self.N - 1, 1))
        if fill_w > 0:
            pygame.draw.rect(surf, C["scrubber"], (sb_x, sb_y, fill_w, sb_h), border_radius=4)
        # Thumb
        thumb_x = sb_x + fill_w
        pygame.draw.circle(surf, C["text_bright"], (thumb_x, sb_y + sb_h // 2), 8)

        # Icing marker on scrubber
        t_ice = self.results["t_ice"]
        ice_x = sb_x + int(sb_w * t_ice / t_end)
        pygame.draw.line(surf, C["icing_line"], (ice_x, sb_y - 2), (ice_x, sb_y + sb_h + 2), 2)

        # Adaptive marker on scrubber
        if self.t_adapt is not None:
            ad_x = sb_x + int(sb_w * self.t_adapt / t_end)
            pygame.draw.line(surf, C["adapt_line"], (ad_x, sb_y - 2), (ad_x, sb_y + sb_h + 2), 2)

        # Key hints
        speed_label = f"speed ×{self.speed}"
        hints = f"[SPACE/→] step   [←] back   [R] restart   [A] auto   [+/-] {speed_label}   [S] save   [Q] quit"
        hs = self.font_xs.render(hints, True, C["text_dim"])
        surf.blit(hs, (W - hs.get_width() - 10, H - fh_ + 28))

        # Playing indicator
        play_str = "▶ PLAYING" if self.playing else "⏸ PAUSED"
        play_col  = C["text_ok"] if self.playing else C["text_dim"]
        ps = self.font_sm.render(play_str, True, play_col)
        surf.blit(ps, (W - ps.get_width() - 10, H - fh_ + 8))

        # Current estimates readout
        hat   = self.results["delta_CL_alpha_hat"][self.frame]
        true_ = self.results["delta_true"][self.frame]
        pct   = 100 * hat / true_ if abs(true_) > 1e-6 else 0.0
        val_str = f"ΔĈ_Lα = {hat:+.4f}  |  true = {true_:+.4f}  |  {pct:.1f}%"
        vs = self.font_xs.render(val_str, True, C["text_warn"] if abs(hat) > 0.01 else C["text_dim"])
        surf.blit(vs, (14, H - fh_ + 28))

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    def run(self):
        running = True
        tick_count = 0

        while running:
            # ---- Events -----------------------------------------------
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

                elif event.type == pygame.KEYDOWN:
                    if event.key in (pygame.K_q, pygame.K_ESCAPE):
                        running = False

                    elif event.key in (pygame.K_SPACE, pygame.K_RIGHT):
                        self.frame = min(self.frame + 1, self.N - 1)

                    elif event.key == pygame.K_LEFT:
                        self.frame = max(self.frame - 1, 0)

                    elif event.key == pygame.K_r:
                        self.frame   = 0
                        self.playing = False

                    elif event.key == pygame.K_a:
                        self.playing = not self.playing

                    elif event.key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                        self.speed = min(self.speed + 1, 20)

                    elif event.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                        self.speed = max(self.speed - 1, 1)

                    elif event.key == pygame.K_s:
                        self._save_screenshot()

                elif event.type == pygame.MOUSEBUTTONDOWN:
                    # Click on scrubber to seek
                    mx, my = event.pos
                    sb_x  = 200
                    sb_w  = self.W - sb_x - 160
                    sb_y  = self.H - self.footer_rect[3] + 14
                    sb_h  = 14
                    if sb_x <= mx <= sb_x + sb_w and sb_y - 8 <= my <= sb_y + sb_h + 8:
                        frac = (mx - sb_x) / sb_w
                        self.frame = int(frac * (self.N - 1))
                        self.frame = max(0, min(self.N - 1, self.frame))

            # ---- Auto-play advance ------------------------------------
            if self.playing:
                tick_count += 1
                if tick_count >= 1:
                    tick_count = 0
                    self.frame = min(self.frame + self.speed, self.N - 1)
                    if self.frame >= self.N - 1:
                        self.playing = False

            # ---- Draw -------------------------------------------------
            t_now = self.results["t"][self.frame]
            t_ice = self.results["t_ice"]
            iced  = t_now >= t_ice
            adp   = bool(self.results["adaptive_mode"][self.frame])

            self.screen.fill(C["bg"])
            self._draw_header()
            self._draw_footer()

            for panel in self.panels:
                panel.draw(
                    self.results,
                    self.frame,
                    iced,
                    adp,
                    self.t_adapt,
                )

            pygame.display.flip()
            self.clock.tick(60)

        pygame.quit()

    def _save_screenshot(self):
        os.makedirs("results", exist_ok=True)
        path = f"results/screenshot_frame{self.frame:04d}.png"
        pygame.image.save(self.screen, path)
        print(f"[pygame] Screenshot saved → {path}")


# ---------------------------------------------------------------------------
# Entry point (called from main.py)
# ---------------------------------------------------------------------------
def run_pygame_dashboard(results: dict) -> None:
    """
    Inject a 'delta_true' series (constant true ΔC_La) into results,
    then launch the interactive dashboard.

    Parameters
    ----------
    results : dict
        Output of simulation.run_simulation().
    """
    N = len(results["t"])
    true_delta = results["C_La_iced"] - results["C_La_clean"]
    results["delta_true"] = np.full(N, true_delta)

    dash = SimDashboard(results)
    dash.run()


# ---------------------------------------------------------------------------
# Standalone test
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import sys
    sys.path.insert(0, os.path.dirname(__file__))
    from simulation import SimConfig, run_simulation

    cfg     = SimConfig()
    results = run_simulation(config=cfg)
    run_pygame_dashboard(results)