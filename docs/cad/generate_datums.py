#!/usr/bin/env python3
"""Generate the datum drawing for the 9751 CAD prompt pack.

Reads robot-parameters.json and writes robot-datums.svg + robot-datums.png.
Solid geometry is derived from robot code / vendor data. Dashed geometry is
either a rule limit or a dimension that still has to be measured.

Run:  python3 docs/cad/generate_datums.py     (needs matplotlib; the SVG/PNG are
committed, so this only has to be re-run when robot-parameters.json changes)
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrowPatch, Polygon, Rectangle

HERE = Path(__file__).resolve().parent
P = json.loads((HERE / "robot-parameters.json").read_text())

INK = "#111827"
MID = "#6b7280"
FRAME_FILL = "#f6f7f9"
MODULE = "#7c8798"
WHEEL = "#2b3140"
CAM = "#c2410c"
PLANNER = "#b91c1c"
INTAKE = "#0f766e"
BUMPER = "#15803d"
DIMC = "#374151"

# ------------------------------------------------------------------ values --
DT = P["drivetrain"]
X_MOD = DT["wheelbase_x"]["value"] / 2.0            # 266.7
Y_MOD = DT["track_width_y"]["value"] / 2.0          # 336.55
BODY = DT["module_body_footprint"]["value"]         # 100.5
TABS = DT["module_footprint_with_tabs"]["value"]    # 133.1
MOD_H = DT["module_height_with_neo"]["value"]       # 171.5
WHEEL_D = DT["wheel_diameter"]["value"]             # 76.2
WHEEL_W = DT["wheel_width"]["value"]                # 22.2

FRAME_X = X_MOD * 2 + BODY                          # 633.9
FRAME_Y = Y_MOD * 2 + BODY                          # 773.6
CHAMFER = 12.0

CAM_A = P["vision"]["camera_a"]
CAM_B = P["vision"]["camera_b"]

PAD_MIN = P["rules_2026"]["R402_min_padding_depth"]["value"]      # 57.15
BUMP_EXT = P["rules_2026"]["R403_max_bumper_extension"]["value"]  # 101.6
BUMP_LO = P["rules_2026"]["R405_bumper_vertical_band"]["min"]     # 69.85
BUMP_HI = P["rules_2026"]["R405_bumper_vertical_band"]["max"]     # 139.7
START_H = P["rules_2026"]["R104_max_starting_height"]["value"]    # 762
FLY_D = P["shooter"]["flywheel_diameter"]["value"]                # 101.6

fig = plt.figure(figsize=(16.0, 9.6), dpi=150)
fig.patch.set_facecolor("white")
ax = fig.add_axes([0.008, 0.185, 0.600, 0.720])
ax2 = fig.add_axes([0.615, 0.235, 0.375, 0.645])

# ============================================================ PLAN VIEW ====
ax.set_xlim(-790, 790)
ax.set_ylim(-620, 640)
ax.set_aspect("equal")
ax.axis("off")


def chamfer_poly(x0, y0, x1, y1, c):
    return [(x0 + c, y0), (x1 - c, y0), (x1, y0 + c), (x1, y1 - c),
            (x1 - c, y1), (x0 + c, y1), (x0, y1 - c), (x0, y0 + c)]


# -- bumper rule envelopes ----------------------------------------------------
ax.add_patch(Rectangle((-(FRAME_X / 2 + BUMP_EXT), -(FRAME_Y / 2 + BUMP_EXT)),
                       2 * (FRAME_X / 2 + BUMP_EXT), 2 * (FRAME_Y / 2 + BUMP_EXT),
                       fill=False, ec=BUMPER, lw=0.8, ls=(0, (1, 2.4)), zorder=1))
ax.add_patch(Rectangle((-(FRAME_X / 2 + PAD_MIN), -(FRAME_Y / 2 + PAD_MIN)),
                       2 * (FRAME_X / 2 + PAD_MIN), 2 * (FRAME_Y / 2 + PAD_MIN),
                       fill=False, ec=BUMPER, lw=1.0, ls=(0, (6, 3)), zorder=1))


# -- frame --------------------------------------------------------------------
ax.add_patch(Polygon(chamfer_poly(-FRAME_X / 2, -FRAME_Y / 2, FRAME_X / 2, FRAME_Y / 2, CHAMFER),
                     closed=True, fc=FRAME_FILL, ec=INK, lw=2.0, zorder=2))

# -- modules ------------------------------------------------------------------
for sx, sy, tag in ((1, 1, "FL"), (1, -1, "FR"), (-1, 1, "RL"), (-1, -1, "RR")):
    cx, cy = sx * X_MOD, sy * Y_MOD
    ax.add_patch(Rectangle((cx - TABS / 2, cy - TABS / 2), TABS, TABS, fill=False,
                           ec=MID, lw=0.7, ls=(0, (4, 3)), zorder=3))
    ax.add_patch(Rectangle((cx - BODY / 2, cy - BODY / 2), BODY, BODY, fc=MODULE,
                           ec=INK, lw=0.7, zorder=3))
    ax.add_patch(Rectangle((cx - WHEEL_D / 2, cy - WHEEL_W / 2), WHEEL_D, WHEEL_W,
                           fc=WHEEL, ec=INK, lw=0.5, zorder=4))
    ax.text(cx - TABS / 2 - 8, cy, tag, fontsize=7.2, color=MID, weight="bold",
            va="center", ha="right")

# -- mechanisms (indicative only) --------------------------------------------
ax.add_patch(Rectangle((FRAME_X / 2 - 1, -150), 240, 300, fill=False, ec=INTAKE,
                       lw=1.0, ls=(0, (5, 3)), zorder=2))
ax.text(FRAME_X / 2 + 120, 0, "INTAKE", fontsize=6.6, color=INTAKE, ha="center", va="center",
        rotation=90, weight="bold")
ax.text(FRAME_X / 2 + 120, -162, "arm 300 long\npivot inside frame", fontsize=5.8, color=INTAKE,
        ha="center", va="top", linespacing=1.5)

ax.add_patch(Rectangle((-FRAME_X / 2 - 150, -140), 150, 280, fill=False, ec=PLANNER,
                       lw=1.0, ls=(0, (5, 3)), zorder=2))
ax.add_patch(Circle((-FRAME_X / 2 + 34, 0), FLY_D / 2, fill=False, ec=PLANNER,
                    lw=0.8, ls=(0, (2, 2)), zorder=3))
ax.plot([-FRAME_X / 2 + 34] * 2, [-86, 86], color=PLANNER, lw=0.8, ls=":", zorder=3)
ax.text(-FRAME_X / 2 - 118, 0, "SHOOTER  (−X face)" , fontsize=6.4, color=PLANNER,
        ha="center", va="center", rotation=90, weight="bold")

# -- cameras ------------------------------------------------------------------
for cam, tag, note in ((CAM_A, "A", "A  FrontCamera"), (CAM_B, "B", "B  OldLeft")):
    x, y = cam["x"]["value"], cam["y"]["value"]
    ax.add_patch(Rectangle((x - 10, y - 10), 20, 20, fc=CAM, ec="white", lw=0.6, zorder=6))
    ax.text(x, y, tag, color="white", fontsize=6.0, ha="center", va="center",
            weight="bold", zorder=7)
    yaw = math.radians(cam["yaw_deg"])
    ax.add_patch(FancyArrowPatch((x, y), (x + 66 * math.cos(yaw), y + 66 * math.sin(yaw)),
                                 arrowstyle="-|>", mutation_scale=8, lw=1.1, color=CAM, zorder=6))
ax.text(CAM_A["x"]["value"] - 16, CAM_A["y"]["value"] + 16, "A", fontsize=8.0, color=CAM,
        va="bottom", ha="right", weight="bold", zorder=7)
ax.text(CAM_B["x"]["value"] - 16, CAM_B["y"]["value"] - 16, "B", fontsize=8.0, color=CAM,
        va="top", ha="right", weight="bold", zorder=7)

# -- origin & axes ------------------------------------------------------------
ax.plot([-26, 26], [0, 0], color=INK, lw=0.8, zorder=5)
ax.plot([0, 0], [-26, 26], color=INK, lw=0.8, zorder=5)
ax.add_patch(Circle((0, 0), 10, fill=False, ec=INK, lw=0.8, zorder=5))
ax.text(0, -36, "ORIGIN\ncentre of rotation, z = 0", fontsize=6.0, color=INK,
        ha="center", va="top", weight="bold")
ax.add_patch(FancyArrowPatch((-150, -300), (-40, -300), arrowstyle="-|>", mutation_scale=8,
                             lw=1.0, color=DIMC, zorder=6))
ax.text(-32, -300, "+X  forward / intake", fontsize=6.0, color=DIMC, va="center")
ax.add_patch(FancyArrowPatch((-150, -300), (-150, -200), arrowstyle="-|>", mutation_scale=8,
                             lw=1.0, color=DIMC, zorder=6))
ax.text(-150, -192, "+Y  left", fontsize=6.0, color=DIMC, ha="center", va="bottom")


# -- dimensions ---------------------------------------------------------------
def hdim(x1, x2, y, text, color=DIMC, fs=6.8, tick=10, off=7):
    ax.add_patch(FancyArrowPatch((x1, y), (x2, y), arrowstyle="<|-|>", mutation_scale=7,
                                 lw=0.8, color=color, shrinkA=0, shrinkB=0, zorder=8))
    for x in (x1, x2):
        ax.plot([x, x], [y - tick, y + tick], color=color, lw=0.5, zorder=8)
    ax.text((x1 + x2) / 2, y + off, text, ha="center", va="bottom", fontsize=fs,
            color=color, zorder=9, bbox=dict(fc="white", ec="none", pad=1.0, alpha=0.9))


def vdim(y1, y2, x, text, color=DIMC, fs=6.8, tick=10, off=7):
    ax.add_patch(FancyArrowPatch((x, y1), (x, y2), arrowstyle="<|-|>", mutation_scale=7,
                                 lw=0.8, color=color, shrinkA=0, shrinkB=0, zorder=8))
    for y in (y1, y2):
        ax.plot([x - tick, x + tick], [y, y], color=color, lw=0.5, zorder=8)
    ax.text(x - off, (y1 + y2) / 2, text, ha="center", va="center", fontsize=fs,
            color=color, rotation=90, zorder=9, bbox=dict(fc="white", ec="none", pad=1.0, alpha=0.9))


hdim(-Y_MOD, Y_MOD, -520, "TRACK  673.1   (26.5 in between wheel centres)")
hdim(-FRAME_Y / 2, FRAME_Y / 2, 505, "frame width  773.6")
vdim(-X_MOD, X_MOD, -600, "WHEELBASE  533.4   (21 in)")
vdim(-FRAME_X / 2, FRAME_X / 2, 700, "frame length  633.9")

ax.text(-750, 630, "PLAN VIEW", fontsize=9.6, color=INK, weight="bold", va="top")
ax.text(-750, 600, "robot frame: +X forward, +Y left, z up.  Every dimension in mm.",
        fontsize=6.2, color=MID, va="top")

# ============================================================= ELEVATION ====
ax2.set_xlim(-600, 620)
ax2.set_ylim(-70, 900)
ax2.set_aspect("equal")
ax2.axis("off")
ax2.text(-570, 890, "FRONT ELEVATION", fontsize=9.6, color=INK, weight="bold", va="top")
ax2.text(-570, 862, "viewer looks toward −X, so screen right = +Y = the robot's left.  A/B = cameras.",
         fontsize=6.2, color=MID, va="top")

# floor
ax2.add_patch(Rectangle((-560, -18), 1120, 18, fc="#e5e7eb", ec="none", zorder=1))
ax2.plot([-556, 556], [0, 0], color=INK, lw=1.4, zorder=2)
for i in range(-540, 560, 36):
    ax2.plot([i, i - 13], [0, -13], color=MID, lw=0.5, zorder=2)

# bumper band (fill full width, boundaries outside the modules only)
ax2.add_patch(Rectangle((-(FRAME_Y / 2 + PAD_MIN), BUMP_LO), 2 * (FRAME_Y / 2 + PAD_MIN),
                        BUMP_HI - BUMP_LO, fc=BUMPER, alpha=0.13, ec="none", zorder=1))
seg_lo = -(FRAME_Y / 2 + PAD_MIN)
seg_hi = FRAME_Y / 2 + PAD_MIN
for z in (BUMP_LO, BUMP_HI):
    ax2.plot([seg_lo, -Y_MOD - BODY / 2 - 4], [z, z], color=BUMPER, lw=0.7, ls=(0, (3, 3)), zorder=2)
    ax2.plot([Y_MOD + BODY / 2 + 4, seg_hi], [z, z], color=BUMPER, lw=0.7, ls=(0, (3, 3)), zorder=2)
ax2.annotate("", (386.8 + 10, BUMP_LO), (386.8 + 10, BUMP_HI),
             arrowprops=dict(arrowstyle="<|-|>", lw=0.7, color=BUMPER, mutation_scale=6))
ax2.text(386.8 + 16, BUMP_HI + 10, "139.7", fontsize=6.0, color=BUMPER, ha="left", va="bottom")
ax2.text(386.8 + 16, BUMP_LO - 10, "69.9", fontsize=6.0, color=BUMPER, ha="left", va="top")
ax2.text(386.8 + 16, (BUMP_LO + BUMP_HI) / 2, "bumper\nband (R405)", fontsize=5.8,
         color=BUMPER, ha="left", va="center", linespacing=1.5)

# frame (height not in code)
ax2.add_patch(Rectangle((-FRAME_Y / 2, 32), FRAME_Y, 368, fill=False, ec=INK, lw=1.6,
                        ls=(0, (7, 4)), zorder=3))
ax2.text(0, 300, "frame height: MEASURE", fontsize=6.6, color=INK, ha="center", va="center",
         bbox=dict(fc="white", ec="none", pad=1.2), zorder=6)

# modules
for sy in (1, -1):
    y = sy * Y_MOD
    ax2.add_patch(Rectangle((y - BODY / 2, 0), BODY, MOD_H, fc=MODULE, ec=INK, lw=0.7, zorder=3))
    ax2.add_patch(Rectangle((y - WHEEL_W / 2, 0), WHEEL_W, WHEEL_D, fc=WHEEL, ec=INK,
                            lw=0.5, zorder=4))
    if y < 0:
        ax2.text(y, MOD_H + 10, "MAXSwerve + NEO", fontsize=5.9, color=MID, ha="center", va="bottom")
ax2.annotate("", (-Y_MOD - BODY / 2 - 16, 0), (-Y_MOD - BODY / 2 - 16, MOD_H),
             arrowprops=dict(arrowstyle="<|-|>", lw=0.6, color=MID, mutation_scale=6))
ax2.text(-Y_MOD - BODY / 2 - 22, MOD_H / 2, "171.5", rotation=90, fontsize=6.0, color=MID,
         ha="right", va="center")

# FUEL ball for scale
ax2.add_patch(Circle((0, 152.5), 75.05, fill=False, ec="#a16207", lw=1.0, ls=(0, (4, 3)), zorder=3))
ax2.plot([0, 0], [152.5 - 75.05, 152.5 + 75.05], color="#a16207", lw=0.6, ls=":", zorder=3)
ax2.text(0, 152.5, "FUEL\nØ150.1", fontsize=6.2, color="#a16207", ha="center", va="center",
         linespacing=1.5, zorder=6, bbox=dict(fc="white", ec="none", pad=1.0))

# cameras
for cam in (CAM_A, CAM_B):
    y, z = cam["y"]["value"], cam["z"]["value"]
    ax2.add_patch(Rectangle((y - 11, z - 11), 22, 22, fc=CAM, ec="white", lw=0.6, zorder=6))
    ax2.text(y, z, "A" if cam is CAM_A else "B", color="white", fontsize=6.2, ha="center",
             va="center", weight="bold", zorder=7)

# R104 starting height + frame width
ax2.plot([-556, 556], [START_H, START_H], color=PLANNER, lw=0.9, ls=(0, (6, 3)), zorder=2)
ax2.text(552, START_H + 12, "R104 starting height 762 (30 in)", fontsize=6.0, color=PLANNER,
         va="bottom", ha="right")
ax2.add_patch(FancyArrowPatch((-FRAME_Y / 2, 470), (FRAME_Y / 2, 470), arrowstyle="<|-|>",
                              mutation_scale=7, lw=0.8, color=INK, zorder=7))
ax2.text(0, 478, "frame width 773.6", fontsize=6.6, ha="center", va="bottom", color=INK,
         bbox=dict(fc="white", ec="none", pad=1.0), zorder=8)

# ================================================================ NOTES ====
fig.text(0.008, 0.975, "9751 GEMINI   |   2026 REBUILT   |   CAD DATUM DRAWING",
         fontsize=13.0, color=INK, weight="bold")
fig.text(0.008, 0.952, "Generated from docs/cad/robot-parameters.json - use together with "
                       "docs/ROBOT_CAD_PROMPT.md.  Dashed items must be confirmed by measurement "
                       "before any part is cut.", fontsize=7.2, color=MID)

left = (
    "SOLID (trust it)     4 x REV 3in MAXSwerve at ±266.7 / ±336.55 mm. Module body 100.5 sq,\n"
    "                     mounting tabs 133.1 sq, 171.5 tall with NEO. Wheel Ø76.2 x 22.2 wide.\n"
    "FRAME (minimum)      Drawn as the smallest rectangle that hugs the module bodies: 633.9 x 773.6.\n"
    "                     Plain perimeter 2815 mm = 110.8 in. The 110 in limit (R104) is measured\n"
    "                     around the outermost non-bumper parts, so the corners are chamfered here\n"
    "                     (12 mm legs remove 28 mm, leaving 109.7 in). Verify the real outermost\n"
    "                     perimeter with a tape and string - this drawing only shows a lower bound."
)
right = (
    "CAMERA A             Rear-facing unit on the top-right of the shooter, looking out over the\n"
    "                     muzzle: the only camera whose line of sight is parallel to the shot.\n"
    "CAMERA B             Faces the robot's RIGHT (−Y). The old name/comments in Constants.java say\n"
    "                     'left'; the transform is what counts, and it is −90°.\n"
    "INTAKE / SHOOTER     Envelopes only. Shape, width, pivot location, stack height and the flywheel\n"
    "                     axle pitch are NOT in the code. Section 6 of ROBOT_CAD_PROMPT.md lists the\n"
    "                     ~15 tape measurements that turn this drawing into a real CAD model.\n"
    "GREEN                Bumper envelopes: dashed = 57.15 min padding, dotted = 101.6 max\n"
    "                     extension from the frame perimeter (R402, R403). Vertical band 69.9-139.7\n"
    "                     above the floor (R405) is shown in the elevation only.\n"
    "MASS                 Sim assumes 65 kg = 143.3 lb. 2026 limits: 115 lb (R103), 135 lb with\n"
    "                     bumpers (R408). Weigh the robot."
)
fig.text(0.008, 0.030, left, fontsize=6.1, color=MID, va="bottom", linespacing=1.55)
fig.text(0.508, 0.030, right, fontsize=6.1, color=MID, va="bottom", linespacing=1.55)

out_svg = HERE / "robot-datums.svg"
out_png = HERE / "robot-datums.png"
fig.savefig(out_svg, format="svg", facecolor="white")
fig.savefig(out_png, dpi=150, facecolor="white")
print(f"wrote {out_svg.name} and {out_png.name}")
