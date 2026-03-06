import time
import math
import browserbotics as bb

# ================================================================
# TWO-ROOM MEDICAL SIMULATION  v7
# ================================================================
# NEW in v7:
#
#  GOTO_BED1/2/3 now triggers the FULL autonomous delivery mission:
#    1. Pick the assigned object from the source table in Room A
#    2. Carry it through the corridor into Room B
#    3. Navigate to the bed foot aisle
#    4. Place the object on the bedside table next to the patient
#    5. Retract arm to carry pose
#    6. Return through corridor back to starting position in Room A
#    7. Arm returns to home pose — robot is ready for next mission
#
#  Each bed has a dedicated bedside-table place position:
#    Bed1 (-9, by) → place at (-9 + BED_W + 0.55, by + BED_L*0.1, place_z)
#    Bed2 ( 0, by) → place at ( 0 + BED_W + 0.55, by + BED_L*0.1, place_z)
#    Bed3 ( 9, by) → place at ( 9 + BED_W + 0.55, by + BED_L*0.1, place_z)
#
#  If the robot is NOT carrying the correct object when GOTO_BEDx is pressed,
#  it automatically picks it first (full auto-pick sequence).
#  If the assigned object is already delivered, a message is printed and
#  the mission is skipped.
# ================================================================

bb.addGroundPlane()
bb.setGravity(0, 0, 0)

# ── World / Room dimensions ───────────────────────────────────
RW       = 30.0
RDA      = 20.0
RDB      = 20.0
WH       = 5.0
CORR_H   = 3.0
CORR_GAP = 0.5

A_Y0 =  CORR_H + CORR_GAP       #  3.5
A_Y1 =  A_Y0 + RDA               # 23.5
A_CY =  (A_Y0 + A_Y1) / 2       # 13.5

B_Y0 = -(CORR_H + CORR_GAP)     # -3.5
B_Y1 = -(CORR_H + CORR_GAP + RDB) # -23.5
B_CY =  (B_Y0 + B_Y1) / 2       # -13.5

# ── Scale reference ───────────────────────────────────────────
MRI_REF   = 4.6
TBL_SCALE = MRI_REF * 0.50

_BASE_ROB = MRI_REF * 0.30
ROB_SCALE = _BASE_ROB * 1.80

ARM_SCALE = MRI_REF * 1.425 * 1.9

# ── Derived table geometry ────────────────────────────────────
T_HX  = TBL_SCALE / 2
T_HY  = TBL_SCALE * 0.45
T_H   = TBL_SCALE * 0.38
T_LEG = T_H / 2
T_TOP = 0.055

OBJ_S = TBL_SCALE * 0.21 * 0.30
OBJ_Z = T_H + T_TOP + OBJ_S

# ================================================================
# HELPERS
# ================================================================
def box(pos, he, col, mass=0):
    return bb.createBody('box', halfExtent=he, position=pos, color=col, mass=mass)

def sphere(pos, r, col, mass=0):
    return bb.createBody('sphere', radius=r, position=pos, color=col, mass=mass)

# ================================================================
# FLOORS
# ================================================================
box([0, A_CY, 0.02], [RW/2, RDA/2, 0.05], '#E8F5E8')
box([0, 0,    0.02], [RW/2, CORR_H, 0.05], '#F5F2EC')
box([0, B_CY, 0.02], [RW/2, RDB/2, 0.05], '#E8EEF8')

for tx in range(-14, 15, 3):
    box([tx, A_CY, 0.07], [0.03, RDA/2, 0.004], '#D0E8D0')
for ty_i in range(int(A_Y0), int(A_Y1)+1, 3):
    box([0, ty_i, 0.07], [RW/2, 0.03, 0.004], '#D0E8D0')

for tx in range(-14, 15, 3):
    box([tx, 0, 0.07], [0.03, CORR_H, 0.004], '#E8E2D8')
for ay in [-1.5, 0.0, 1.5]:
    box([0, ay, 0.09], [0.15, 0.55, 0.005], '#FFDD00')
    box([ 0.22, ay-0.35, 0.09], [0.10, 0.20, 0.005], '#FFDD00')
    box([-0.22, ay-0.35, 0.09], [0.10, 0.20, 0.005], '#FFDD00')

for tx in range(-14, 15, 3):
    box([tx, B_CY, 0.07], [0.03, RDB/2, 0.004], '#D0DCF0')
for ty_i in range(int(B_Y1), int(B_Y0)+1, 3):
    box([0, ty_i, 0.07], [RW/2, 0.03, 0.004], '#D0DCF0')

box([ 0,      B_CY, 0.08], [0.40, RDB/2,        0.005], '#FFEE88')
box([-RW/4,   B_CY, 0.08], [0.12, RDB/2 * 0.85, 0.005], '#88BBFF')
box([ RW/4,   B_CY, 0.08], [0.12, RDB/2 * 0.85, 0.005], '#88BBFF')

# ================================================================
# OUTER WALLS
# ================================================================
full_mid_y = (A_Y1 + B_Y1) / 2
full_hy    = (A_Y1 - B_Y1) / 2

box([-RW/2, full_mid_y, WH/2], [0.22, full_hy, WH/2], '#EFEFED')
box([ RW/2, full_mid_y, WH/2], [0.22, full_hy, WH/2], '#EFEFED')
box([0, A_Y1, WH/2], [RW/2, 0.22, WH/2], '#EFEFED')
box([0, B_Y1, WH/2], [RW/2, 0.22, WH/2], '#EFEFED')

DOOR_W      = 2.5
wall_seg_hw = RW/4 - DOOR_W/2

# Room A south wall with doorway
box([-RW/4 - DOOR_W/2, A_Y0, WH/2], [wall_seg_hw, 0.22, WH/2], '#D8D0C8')
box([ RW/4 + DOOR_W/2, A_Y0, WH/2], [wall_seg_hw, 0.22, WH/2], '#D8D0C8')
box([-DOOR_W-0.1, A_Y0, 1.5], [0.15, 0.25, 1.5], '#888888')
box([ DOOR_W+0.1, A_Y0, 1.5], [0.15, 0.25, 1.5], '#888888')
box([0, A_Y0, 3.05], [DOOR_W+0.25, 0.25, 0.12], '#888888')

# Room B north wall with doorway
box([-RW/4 - DOOR_W/2, B_Y0, WH/2], [wall_seg_hw, 0.22, WH/2], '#D8D0C8')
box([ RW/4 + DOOR_W/2, B_Y0, WH/2], [wall_seg_hw, 0.22, WH/2], '#D8D0C8')
box([-DOOR_W-0.1, B_Y0, 1.5], [0.15, 0.25, 1.5], '#888888')
box([ DOOR_W+0.1, B_Y0, 1.5], [0.15, 0.25, 1.5], '#888888')
box([0, B_Y0, 3.05], [DOOR_W+0.25, 0.25, 0.12], '#888888')

box([-DOOR_W+0.5, A_Y0, 3.4], [0.8, 0.08, 0.22], '#22AA44')
box([ DOOR_W-0.5, A_Y0, 3.4], [0.8, 0.08, 0.22], '#2244CC')
box([-DOOR_W+0.5, B_Y0, 3.4], [0.8, 0.08, 0.22], '#2244CC')
box([ DOOR_W-0.5, B_Y0, 3.4], [0.8, 0.08, 0.22], '#22AA44')

CORR_WALL_H = WH * 0.6
box([-RW/2, 0, CORR_WALL_H/2], [0.22, CORR_H, CORR_WALL_H/2], '#DDDDD5')
box([ RW/2, 0, CORR_WALL_H/2], [0.22, CORR_H, CORR_WALL_H/2], '#DDDDD5')

# ================================================================
# CEILING LIGHTS
# ================================================================
def ceiling_light(x, y):
    box([x, y, WH-0.08], [0.40, 0.40, 0.06], '#FFFFEE')
    box([x, y, WH-0.18], [0.32, 0.32, 0.04], '#FFFFE8')

for lx in [-9, 0, 9]:
    for ly_f in [A_Y0+RDA*0.25, A_Y0+RDA*0.75]:
        ceiling_light(lx, ly_f)
ceiling_light(0, 0)

# ================================================================
# WALL PANELS
# ================================================================
for px in [-10, -4, 4, 10]:
    box([px, A_Y1-0.12, 2.8], [0.80, 0.05, 0.55], '#FFFFFF')
    box([px, A_Y1-0.10, 2.8], [0.72, 0.04, 0.48], '#DDEEFF')

for px in [-9, 0, 9]:
    box([px, B_Y1+0.12, 2.4], [1.1, 0.06, 0.90], '#EEEEFF')
    box([px, B_Y1+0.10, 2.4], [1.0, 0.05, 0.80], '#CCE0FF')

# ================================================================
# ROOM A — PHARMACY / PREPARATION
# ================================================================
SHELF_Z = [0.55, 1.20, 1.85, 2.50]
shelf_cols = ['#FF6666','#66AAFF','#66CC66','#FFAA44','#CC66FF',
              '#FF88AA','#44DDCC','#FFEE44','#FF4444','#8888FF']
for unit in range(4):
    sx = -12.0 + unit * 7.0
    sy = A_Y1 - 1.2
    box([sx-3.0, sy, 1.80], [0.06, 0.50, 1.80], '#C8C4BE')
    box([sx+3.0, sy, 1.80], [0.06, 0.50, 1.80], '#C8C4BE')
    box([sx, sy+0.50, 1.80], [3.0, 0.04, 1.80], '#D8D4CE')
    for sz in SHELF_Z:
        box([sx, sy, sz], [2.9, 0.48, 0.04], '#E0DCD6')
    for shelf_z in SHELF_Z[:-1]:
        for bxi in range(6):
            col = shelf_cols[(unit*6+bxi) % len(shelf_cols)]
            box([sx-2.2+bxi*0.88, sy, shelf_z+0.22], [0.18, 0.14, 0.18], col)
            box([sx-2.2+bxi*0.88, sy-0.15, shelf_z+0.16], [0.17, 0.02, 0.06], '#FFFFFF')

PTX, PTY = 8.0, A_CY
box([PTX, PTY, T_H], [T_HX, T_HY, T_TOP], '#F0EEE8')
for plx in [-T_HX*0.82, T_HX*0.82]:
    for ply in [-T_HY*0.82, T_HY*0.82]:
        box([PTX+plx, PTY+ply, T_LEG], [0.07, 0.07, T_LEG], '#888888')
box([PTX-0.8, PTY, T_H+0.12], [0.35, 0.25, 0.08], '#FFFFFF')
box([PTX+0.5, PTY, T_H+0.28], [0.18, 0.14, 0.20], '#DDDDDD')
box([PTX+0.5, PTY, T_H+0.50], [0.14, 0.10, 0.04], '#0055AA')

LBX, LBY = -8.0, A_CY
box([LBX, LBY, T_H], [T_HX*0.85, T_HY, T_TOP], '#F0EEE8')
for llx in [-T_HX*0.75, T_HX*0.75]:
    for lly in [-T_HY*0.82, T_HY*0.82]:
        box([LBX+llx, LBY+lly, T_LEG], [0.07, 0.07, T_LEG], '#888888')
box([LBX-0.8, LBY, T_H+0.24], [0.32, 0.26, 0.20], '#333344')
box([LBX+0.5, LBY, T_H+0.18], [0.24, 0.18, 0.16], '#555555')

# ── SOURCE TABLE (Room A centre) ──────────────────────────────
ITX = 0.0
ITY = A_Y0 + RDA * 0.5   # Y = 13.5

box([ITX, ITY, T_H], [T_HX, T_HY, T_TOP], '#D8F0E8')
for tlx in [-T_HX*0.82, T_HX*0.82]:
    for tly in [-T_HY*0.82, T_HY*0.82]:
        box([ITX+tlx, ITY+tly, T_LEG], [0.09, 0.09, T_LEG], '#909090')
box([ITX, ITY, T_H*0.45], [T_HX*0.85, T_HY*0.85, T_TOP*0.6], '#C8E8D8')
box([ITX, ITY-T_HY-0.02, T_H*0.55], [T_HX, 0.06, 0.12], '#2277EE')
box([ITX, ITY+T_HY+0.02, T_H*0.55], [T_HX, 0.06, 0.12], '#2277EE')

obj_spacing = T_HX * 0.55
obj_A = box([ITX-obj_spacing, ITY, OBJ_Z], [OBJ_S, OBJ_S, OBJ_S], '#EE2222')  # RED
obj_B = box([ITX,             ITY, OBJ_Z], [OBJ_S, OBJ_S, OBJ_S], '#22AA22')  # GREEN
obj_C = box([ITX+obj_spacing, ITY, OBJ_Z], [OBJ_S, OBJ_S, OBJ_S], '#2244EE')  # BLUE

for ox, mc in [(ITX-obj_spacing,'#FF0000'),(ITX,'#00CC00'),(ITX+obj_spacing,'#0022FF')]:
    sphere([ox, ITY, OBJ_Z+OBJ_S*2.0], OBJ_S*0.45, mc)

CSX, CSY = -RW/2+1.2, A_Y0+RDA/3
box([CSX, CSY, 1.14], [0.80, 0.60, 1.10], '#333344')
box([CSX, CSY-0.60, 1.14], [0.76, 0.04, 1.05], '#222233')
sphere([CSX+0.50, CSY-0.62, 1.60], 0.07, '#FF2200')
box([CSX, CSY-0.62, 1.80], [0.28, 0.04, 0.18], '#FF4400')

HWX, HWY = -RW/2+1.0, A_Y0+RDA*0.25
box([HWX, HWY, 2.40], [0.52, 0.40, 0.05], '#FFFFFF')
box([HWX, HWY, 2.12], [0.45, 0.34, 0.28], '#E8F4FF')
box([HWX, HWY-0.10, 2.62], [0.04, 0.04, 0.20], '#C0C0C0')
box([HWX, HWY-0.10, 2.84], [0.14, 0.04, 0.04], '#AAAAAA')
box([HWX+0.35, HWY, 2.62], [0.08, 0.10, 0.20], '#FF8844')
box([HWX-0.28, HWY, 2.62], [0.16, 0.10, 0.26], '#DDDDDD')

for wx2, wc in [(-3.0,'#DD2222'),(-1.8,'#2244AA'),(-0.6,'#EE8800')]:
    box([PTX+wx2, PTY+1.8, 0.48], [0.28, 0.28, 0.44], wc)
    box([PTX+wx2, PTY+1.8, 0.97], [0.24, 0.24, 0.05], '#111111')

box([8.0, A_Y0, 1.5], [1.2, 0.12, 1.5], '#4A8A55')
sphere([9.15, A_Y0, 1.5], 0.12, '#C8A820')

for cx4, cy4 in [(-RW/2+1, A_Y1-1),(RW/2-1, A_Y1-1),
                  (-RW/2+1, B_Y1+1),(RW/2-1, B_Y1+1)]:
    box([cx4, cy4, WH-0.22], [0.10, 0.10, 0.10], '#222222')
    sphere([cx4, cy4-0.22, WH-0.30], 0.05, '#880000')

# ================================================================
# ROOM B — PATIENT WARD
# ================================================================
BED_W  = 1.2
BED_L  = 3.0
BED_H  = 0.55
RAIL_H = 1.0

def make_bed(bx, by, col='#E8E8FF'):
    box([bx, by, 0.28], [BED_W, BED_L, 0.28], '#BBBBBB')
    box([bx, by, BED_H], [BED_W*0.92, BED_L*0.94, 0.18], col)
    box([bx, by+BED_L*0.72, BED_H+0.12], [BED_W*0.70, BED_L*0.18, 0.10], '#FFFFFF')
    box([bx, by+BED_L*0.55, BED_H+0.14], [BED_W*0.88, BED_L*0.30, 0.06], '#D8D8F0')
    box([bx, by+BED_L+0.10, BED_H+0.40], [BED_W, 0.12, 0.40], '#8899BB')
    box([bx, by+BED_L+0.10, BED_H+0.90], [BED_W*0.80, 0.10, 0.08], '#8899BB')
    box([bx-BED_W-0.06, by, BED_H+RAIL_H/2], [0.04, BED_L*0.85, RAIL_H/2], '#CCCCCC')
    box([bx+BED_W+0.06, by, BED_H+RAIL_H/2], [0.04, BED_L*0.85, RAIL_H/2], '#CCCCCC')
    for wox in [-BED_W*0.70, BED_W*0.70]:
        for woy in [-BED_L*0.70, BED_L*0.70]:
            sphere([bx+wox, by+woy, 0.08], 0.08, '#333333')
    box([bx+BED_W+0.25, by+BED_L+0.25, 1.4],  [0.04, 0.04, 1.4],  '#CCCCCC')
    box([bx+BED_W+0.25, by+BED_L+0.25, 2.84], [0.50, 0.04, 0.04], '#BBBBBB')
    sphere([bx+BED_W+0.25, by+BED_L+0.25, 2.90], 0.12, '#AADDFF')
    box([bx+BED_W+0.25, by+BED_L+0.28, 2.65], [0.18, 0.04, 0.22], '#CCF0FF')
    box([bx+BED_W+0.25, by+BED_L+0.30, 2.40], [0.025, 0.025, 0.25], '#DDDDDD')
    box([bx-BED_W-0.40, by+BED_L*0.2, 0.65], [0.10, 0.10, 0.65], '#55AA55')
    box([bx-BED_W-0.40, by+BED_L*0.2, 1.32], [0.07, 0.07, 0.06], '#444444')
    sphere([bx-BED_W-0.40, by+BED_L*0.2, 1.42], 0.08, '#666666')
    box([bx+BED_W+0.55, by+BED_L*0.1,      1.50], [0.30, 0.24, 1.50], '#DDDDCC')
    box([bx+BED_W+0.55, by+BED_L*0.1-0.20, 2.55], [0.26, 0.04, 0.22], '#111111')
    box([bx+BED_W+0.55, by+BED_L*0.1-0.18, 2.55], [0.22, 0.03, 0.18], '#22CC44')
    box([bx-BED_W-0.60, by+BED_L*0.5, 0.80], [0.30, 0.30, 0.80], '#DDD8C8')
    box([bx-BED_W-0.60, by+BED_L*0.5, 1.62], [0.32, 0.32, 0.04], '#EEE8D8')
    ceiling_light(bx, by)

# ================================================================
# PATIENT IN BED — detailed lying humanoid figure
# skin_col: skin tone, gown_col: hospital gown colour
# ================================================================
def make_patient(bx, by, skin_col='#F5C5A3', gown_col='#AACCFF',
                 blanket_col='#8899CC', hair_col='#2A1A0A'):
    """
    Place a patient lying in a bed centred at (bx, by).
    The head is at the +Y end (headboard side).
    """
    PAT_Z = BED_H + 0.18 + 0.08   # just above mattress surface

    # ── Torso (hospital gown) ─────────────────────────────────
    box([bx, by + BED_L*0.18, PAT_Z + 0.10],
        [BED_W*0.36, BED_L*0.38, 0.10], gown_col)

    # ── Gown pattern stripes ──────────────────────────────────
    for stripe_y in [-0.25, 0.0, 0.25]:
        box([bx, by + BED_L*0.18 + stripe_y, PAT_Z + 0.21],
            [BED_W*0.34, 0.04, 0.008], '#7799DD')

    # ── Blanket over lower body ───────────────────────────────
    box([bx, by - BED_L*0.20, PAT_Z + 0.14],
        [BED_W*0.82, BED_L*0.50, 0.08], blanket_col)
    # Blanket fold top edge
    box([bx, by + BED_L*0.05, PAT_Z + 0.20],
        [BED_W*0.82, 0.08, 0.04], '#99AACC')

    # ── Legs under blanket (subtle bumps) ─────────────────────
    box([bx - BED_W*0.18, by - BED_L*0.28, PAT_Z + 0.10],
        [0.10, BED_L*0.35, 0.10], '#7788BB')
    box([bx + BED_W*0.18, by - BED_L*0.28, PAT_Z + 0.10],
        [0.10, BED_L*0.35, 0.10], '#7788BB')

    # ── Neck ─────────────────────────────────────────────────
    box([bx, by + BED_L*0.50, PAT_Z + 0.13],
        [0.09, 0.10, 0.09], skin_col)

    # ── Head ─────────────────────────────────────────────────
    sphere([bx, by + BED_L*0.60, PAT_Z + 0.22], 0.18, skin_col)

    # ── Hair ─────────────────────────────────────────────────
    box([bx, by + BED_L*0.60, PAT_Z + 0.36],
        [0.17, 0.16, 0.06], hair_col)
    # Hair sides
    box([bx - 0.14, by + BED_L*0.60, PAT_Z + 0.26],
        [0.06, 0.14, 0.10], hair_col)
    box([bx + 0.14, by + BED_L*0.60, PAT_Z + 0.26],
        [0.06, 0.14, 0.10], hair_col)

    # ── Eyes ─────────────────────────────────────────────────
    sphere([bx - 0.07, by + BED_L*0.63, PAT_Z + 0.24], 0.025, '#222222')
    sphere([bx + 0.07, by + BED_L*0.63, PAT_Z + 0.24], 0.025, '#222222')

    # ── Nose (tiny box) ──────────────────────────────────────
    box([bx, by + BED_L*0.64, PAT_Z + 0.20],
        [0.02, 0.01, 0.02], '#D4A882')

    # ── Mouth (thin dark strip) ───────────────────────────────
    box([bx, by + BED_L*0.61, PAT_Z + 0.18],
        [0.05, 0.01, 0.015], '#AA7766')

    # ── Left arm (resting along side, slightly bent) ──────────
    # Upper arm
    box([bx - BED_W*0.52, by + BED_L*0.22, PAT_Z + 0.10],
        [0.07, 0.25, 0.07], skin_col)
    # Forearm — angled toward body
    box([bx - BED_W*0.48, by + BED_L*0.02, PAT_Z + 0.12],
        [0.06, 0.18, 0.06], skin_col)
    # Hand
    sphere([bx - BED_W*0.45, by - BED_L*0.10, PAT_Z + 0.12], 0.07, skin_col)

    # ── Right arm (slightly raised, IV in forearm) ────────────
    # Upper arm
    box([bx + BED_W*0.52, by + BED_L*0.22, PAT_Z + 0.10],
        [0.07, 0.25, 0.07], skin_col)
    # Forearm resting outward
    box([bx + BED_W*0.55, by + BED_L*0.04, PAT_Z + 0.10],
        [0.06, 0.18, 0.06], skin_col)
    # Hand
    sphere([bx + BED_W*0.56, by - BED_L*0.08, PAT_Z + 0.10], 0.07, skin_col)

    # ── IV / medical wristband on right wrist ─────────────────
    box([bx + BED_W*0.56, by - BED_L*0.08, PAT_Z + 0.16],
        [0.075, 0.03, 0.02], '#FFFFFF')
    box([bx + BED_W*0.56, by - BED_L*0.08, PAT_Z + 0.16],
        [0.075, 0.03, 0.022], '#FF6688')   # pink wristband stripe

    # ── Oxygen / nasal tube detail (thin line on face) ────────
    box([bx, by + BED_L*0.63, PAT_Z + 0.20],
        [0.14, 0.008, 0.008], '#DDDDDD')

    # ── Pillow under head ─────────────────────────────────────
    box([bx, by + BED_L*0.60, PAT_Z + 0.06],
        [BED_W*0.70, 0.28, 0.07], '#F8F8F8')


# BED_POSITIONS[0] = Bed1 (-9, -14.5) ← RED
# BED_POSITIONS[1] = Bed2 ( 0, -14.5) ← BLUE
# BED_POSITIONS[2] = Bed3 ( 9, -14.5) ← GREEN
BED_POSITIONS = [
    (-9.0, B_Y1 + RDB*0.45),
    ( 0.0, B_Y1 + RDB*0.45),
    ( 9.0, B_Y1 + RDB*0.45),
]

# Bed-to-object assignment
BED_OBJECT_MAP = {
    0: "red",    # Bed1 ← RED
    1: "blue",   # Bed2 ← BLUE
    2: "green",  # Bed3 ← GREEN
}

# Patient appearance per bed (skin, gown, blanket, hair)
BED_PATIENT_PARAMS = [
    # Bed1 — elderly male, pale skin, blue gown
    {'skin_col': '#F0D5B8', 'gown_col': '#AACCFF', 'blanket_col': '#7788BB', 'hair_col': '#AAAAAA'},
    # Bed2 — middle-aged female, medium skin, green gown
    {'skin_col': '#C8956C', 'gown_col': '#AAFFCC', 'blanket_col': '#6699AA', 'hair_col': '#1A0A00'},
    # Bed3 — younger patient, darker skin, white gown
    {'skin_col': '#8B5E3C', 'gown_col': '#EEEEFF', 'blanket_col': '#99AACC', 'hair_col': '#0A0A0A'},
]

for i, bpos in enumerate(BED_POSITIONS):
    make_bed(bpos[0], bpos[1])
    make_patient(bpos[0], bpos[1], **BED_PATIENT_PARAMS[i])

# Curtain dividers — visual columns only
for cx in [-4.5, 4.5]:
    for cz_s in range(1, 10):
        z_mid = cz_s * 0.30 + 0.15
        box([cx, B_CY, z_mid], [0.04, RDB/2*0.55, 0.12],
            '#EEEEFF' if cz_s % 2 == 0 else '#DDDDEE')
    box([cx, B_CY, 0.0], [0.04, 0.04, WH/2], '#AAAAAA')

# ── DESTINATION TABLE (Room B) ────────────────────────────────
DTX = 0.0
DTY = B_Y1 + RDB * 0.25   # Y = -18.5

box([DTX, DTY, T_H], [T_HX, T_HY, T_TOP], '#F0E8D8')
for tlx in [-T_HX*0.82, T_HX*0.82]:
    for tly in [-T_HY*0.82, T_HY*0.82]:
        box([DTX+tlx, DTY+tly, T_LEG], [0.09, 0.09, T_LEG], '#909090')
for pad_x, pad_col in [(-T_HX*0.5,'#FFAAAA'),(0.0,'#AAFFAA'),(T_HX*0.5,'#AAAAFF')]:
    box([DTX+pad_x, DTY, T_H+0.07], [T_HX*0.28, T_HY*0.38, 0.012], pad_col)
box([DTX, DTY-T_HY-0.02, T_H*0.55], [T_HX, 0.06, 0.12], '#22AA44')
box([DTX, DTY+T_HY+0.02, T_H*0.55], [T_HX, 0.06, 0.12], '#22AA44')

NSX, NSY = RW/2-3.5, B_Y0 - 4.0
box([NSX, NSY, 1.25], [2.5, 1.0, 1.25], '#D8D4CE')
box([NSX, NSY, 2.56], [2.5, 1.0, 0.06], '#E8E4DE')
box([NSX, NSY-0.82, 3.10], [0.65, 0.05, 0.40], '#111111')
box([NSX, NSY-0.80, 3.10], [0.55, 0.03, 0.32], '#0044CC')
box([NSX-1.2, NSY, 2.62], [0.30, 0.22, 0.03], '#FFFFFF')
box([NSX+1.2, NSY-0.65, 2.62], [0.12, 0.12, 0.04], '#FF4444')
box([NSX, NSY+1.5, 1.72], [0.32, 0.32, 0.05], '#4466AA')
box([NSX, NSY+1.5, 0.86], [0.04, 0.04, 0.86], '#555555')

ETX, ETY = -RW/2+2.0, B_Y0 - 3.0
box([ETX, ETY, 0.70], [0.60, 0.35, 0.70], '#CC2200')
box([ETX, ETY, 1.42], [0.62, 0.37, 0.04], '#BB1800')
box([ETX, ETY, 0.90], [0.55, 0.30, 0.04], '#EE4422')
box([ETX, ETY, 0.60], [0.55, 0.30, 0.04], '#EE4422')
sphere([ETX+0.62, ETY, 0.70], 0.06, '#AAAAAA')

box([0, B_Y0+0.8, 0.08], [DOOR_W, 0.8, 0.005], '#FFEE44')
box([0, B_Y0+1.65, 0.08], [DOOR_W, 0.06, 0.005], '#FFEE44')

for ex2, ey2 in [(-RW/2+0.5, B_Y1+3.0), (RW/2-0.5, B_Y1+3.0)]:
    box([ex2, ey2, 0.48], [0.12, 0.12, 0.42], '#CC2200')
    sphere([ex2, ey2, 0.98], 0.10, '#AA1800')

# ================================================================
# ROBOT CHASSIS
# ================================================================
C_HX  = ROB_SCALE * 0.30
C_HY  = ROB_SCALE * 0.18
C_HZ  = ROB_SCALE * 0.07
W_R   = ROB_SCALE * 0.14
W_W   = ROB_SCALE * 0.055
W_Z   = W_R
COL_H = ROB_SCALE * 0.18

ARM_Z = W_R + C_HZ*2 + COL_H + 0.18

WX_F =  C_HX - ROB_SCALE*0.04
WX_B = -C_HX + ROB_SCALE*0.04
WY_L =  C_HY + W_W + 0.01
WY_R = -(C_HY + W_W + 0.01)

ROB_X0 = ITX + T_HX + C_HX + 1.6
ROB_Y0 = ITY

chassis  = box([ROB_X0, ROB_Y0, W_R+C_HZ],
               [C_HX, C_HY, C_HZ], '#1A2A3A', mass=1)
deck     = box([ROB_X0, ROB_Y0, W_R+C_HZ*2+C_HZ*0.25],
               [C_HX*0.90, C_HY*0.85, C_HZ*0.25], '#253545')
col_body = box([ROB_X0, ROB_Y0, W_R+C_HZ*2+C_HZ*0.5+COL_H/2],
               [ROB_SCALE*0.055, ROB_SCALE*0.055, COL_H/2], '#3A4A5A')
arr1 = box([ROB_X0+C_HX*0.36, ROB_Y0, W_R+C_HZ*2.2],
           [C_HX*0.36, C_HY*0.08, C_HZ*0.16], '#FF4400')
arr2 = box([ROB_X0+C_HX*0.82, ROB_Y0, W_R+C_HZ*2.2],
           [C_HX*0.10, C_HY*0.20, C_HZ*0.16], '#FF4400')

wheel_data   = [('FL',WX_F,WY_L),('FR',WX_F,WY_R),
                ('BL',WX_B,WY_L),('BR',WX_B,WY_R)]
wheel_bodies = {}
for label, wx_off, wy_off in wheel_data:
    wb = box([ROB_X0+wx_off, ROB_Y0+wy_off, W_Z],
             [W_R, W_W, W_R], '#222222')
    hub_s = 1 if 'L' in label else -1
    hb = sphere([ROB_X0+wx_off, ROB_Y0+wy_off+hub_s*(W_W+0.015), W_Z],
                W_W*0.70, '#FFCC00')
    wheel_bodies[label] = (wb, hb, wx_off, wy_off)

axle_f = box([ROB_X0+WX_F, ROB_Y0, W_Z],
             [W_W*0.45, C_HY+W_W+0.015, W_W*0.45], '#555555')
axle_b = box([ROB_X0+WX_B, ROB_Y0, W_Z],
             [W_W*0.45, C_HY+W_W+0.015, W_W*0.45], '#555555')

# ================================================================
# ARM URDF
# ================================================================
robot     = bb.loadURDF('panda.urdf', [ROB_X0, ROB_Y0, ARM_Z], fixedBase=True)
ee_link   = 10
home_jpos = [0, -0.5, 0.0, -1.5, 0.1, 1.0, 0.0]
for ji, jp in enumerate(home_jpos):
    bb.setJointMotorControl(robot, ji, targetPosition=jp)

# ================================================================
# GUI
# ================================================================
bb.addDebugButton('FORWARD')
bb.addDebugButton('BACK')
bb.addDebugButton('LEFT')
bb.addDebugButton('RIGHT')

bb.addDebugSlider('arm_x',  0.6,  -1.8, 1.8)
bb.addDebugSlider('arm_y',  0.0,  -1.8, 1.8)
bb.addDebugSlider('arm_z',  0.8,   0.1, 2.8)

bb.addDebugSlider('ee_rx',  180.0,   0.0, 360.0)
bb.addDebugSlider('ee_ry',    0.0, -180.0, 180.0)
bb.addDebugSlider('ee_rz',    0.0, -180.0, 180.0)

bb.addDebugButton('PICK')
bb.addDebugButton('PLACE')
bb.addDebugButton('HOME')
bb.addDebugButton('RESET')

bb.addDebugButton('GOTO_SOURCE')
bb.addDebugButton('GOTO_DEST')
bb.addDebugButton('GOTO_BED1')
bb.addDebugButton('GOTO_BED2')
bb.addDebugButton('GOTO_BED3')

# ================================================================
# OBJECT STATE SYSTEM
# ================================================================
OBJECTS = {
    "red":   {"handle": obj_A, "origin": [ITX-obj_spacing, ITY, OBJ_Z],
              "picked": False, "delivered": False},
    "green": {"handle": obj_B, "origin": [ITX,             ITY, OBJ_Z],
              "picked": False, "delivered": False},
    "blue":  {"handle": obj_C, "origin": [ITX+obj_spacing, ITY, OBJ_Z],
              "picked": False, "delivered": False},
}
PICK_ORDER = ["red", "green", "blue"]

def next_available_object():
    for key in PICK_ORDER:
        o = OBJECTS[key]
        if not o["picked"] and not o["delivered"]:
            return key
    return None

def nearest_available_object():
    best_key, best_dist = None, float("inf")
    for key in PICK_ORDER:
        o = OBJECTS[key]
        if o["picked"] or o["delivered"]:
            continue
        ox, oy = o["origin"][0], o["origin"][1]
        d = math.sqrt((rx-ox)**2 + (ry-oy)**2)
        if d < best_dist:
            best_dist, best_key = d, key
    return best_key

# ================================================================
# ROBOT STATE
# ================================================================
rx   = float(ROB_X0)
ry   = float(ROB_Y0)
rang = 0.0

STEP_NORMAL = 0.18
TURN_NORMAL = math.pi / 14
STEP_LOADED = 0.10
TURN_LOADED = math.pi / 22

carried_obj      = None
_active_pick_key = None

btn_keys = ['FORWARD','BACK','LEFT','RIGHT',
            'PICK','PLACE','HOME','RESET',
            'GOTO_SOURCE','GOTO_DEST',
            'GOTO_BED1','GOTO_BED2','GOTO_BED3']
btn_counts = {k: 0 for k in btn_keys}

_init_world  = [ROB_X0 + 0.6, ROB_Y0, ARM_Z + 0.8]
cmd_ax, cmd_ay, cmd_az = _init_world
ik_cur       = list(_init_world)
ik_euler_cur = [math.pi, 0.0, 0.0]

APPROACH_H = 1.0
IK_LERP    = 0.10
ORI_LERP   = 0.12

# ================================================================
# STATIC OBSTACLE REGISTRY
# ================================================================
OBSTACLES = []

def register_obstacle(cx, cy, radius):
    OBSTACLES.append((float(cx), float(cy), float(radius)))

# ── Room divider walls — leave door gap clear
# FIX-2: reduced wall obstacle radius from 1.1 → 0.65
DOOR_CLEAR = DOOR_W + 0.5   # 3.0
for _wx in range(int(-RW/2), int(RW/2)+1, 2):
    if abs(_wx) > DOOR_CLEAR:
        register_obstacle(_wx, A_Y0, 0.65)   # FIX-2: was 1.1
        register_obstacle(_wx, B_Y0, 0.65)   # FIX-2: was 1.1

# ── Outer east / west perimeter walls
for _wy in range(int(B_Y1), int(A_Y1)+1, 3):
    register_obstacle(-RW/2 + 0.5, _wy, 1.0)
    register_obstacle( RW/2 - 0.5, _wy, 1.0)

# ── Beds
for _bx, _by in BED_POSITIONS:
    register_obstacle(_bx, _by, BED_W + 1.8)

# ── Curtain dividers
for _cx in [-4.5, 4.5]:
    register_obstacle(_cx, B_CY + 2.0, 0.7)
    register_obstacle(_cx, B_CY - 2.0, 0.7)

# ── Tables
register_obstacle(ITX, ITY, T_HX + C_HX + 0.4)
register_obstacle(DTX, DTY, T_HX + C_HX + 0.4)
register_obstacle(PTX, PTY, T_HX + 0.6)
register_obstacle(LBX, LBY, T_HX + 0.6)

# ── Nurse station and trolley
register_obstacle(NSX, NSY, 1.8)
register_obstacle(ETX, ETY, 1.1)

OBSTACLE_MARGIN = C_HX + 0.28


# ================================================================
# TELEPORT
# ================================================================
def teleport_all(new_rx, new_ry, new_ang):
    bz = W_R + C_HZ
    bb.resetBasePose(chassis,  [new_rx, new_ry, bz])
    bb.resetBasePose(deck,     [new_rx, new_ry, bz+C_HZ+C_HZ*0.25])
    bb.resetBasePose(col_body, [new_rx, new_ry, bz+C_HZ*2+C_HZ*0.5+COL_H/2])

    def rot(ox, oy):
        c, s = math.cos(new_ang), math.sin(new_ang)
        return (new_rx + ox*c - oy*s, new_ry + ox*s + oy*c)

    a1x, a1y = rot(C_HX*0.36, 0);  bb.resetBasePose(arr1, [a1x, a1y, bz+C_HZ*1.3])
    a2x, a2y = rot(C_HX*0.82, 0);  bb.resetBasePose(arr2, [a2x, a2y, bz+C_HZ*1.3])

    for lbl, (wb, hb, wx_off, wy_off) in wheel_bodies.items():
        rwx, rwy = rot(wx_off, wy_off)
        bb.resetBasePose(wb, [rwx, rwy, W_Z])
        hub_s = 1 if 'L' in lbl else -1
        hx2 = rwx + hub_s*(W_W+0.015)*(-math.sin(new_ang))
        hy2 = rwy + hub_s*(W_W+0.015)*( math.cos(new_ang))
        bb.resetBasePose(hb, [hx2, hy2, W_Z])

    af_x, af_y = rot(WX_F, 0);  bb.resetBasePose(axle_f, [af_x, af_y, W_Z])
    ab_x, ab_y = rot(WX_B, 0);  bb.resetBasePose(axle_b, [ab_x, ab_y, W_Z])
    bb.resetBasePose(robot, [new_rx, new_ry, ARM_Z])


# ================================================================
# ARM IK
# ================================================================
def local_to_world(lx, ly, lz):
    c, s = math.cos(rang), math.sin(rang)
    return [rx + lx*c - ly*s,  ry + lx*s + ly*c,  ARM_Z + lz]

def arm_ik(world_pos, world_euler=None):
    q = bb.getQuaternionFromEuler(world_euler if world_euler else [math.pi, 0, rang])
    jpos = bb.calculateInverseKinematics(robot, ee_link, world_pos, q)
    for idx, jp in enumerate(jpos):
        bb.setJointMotorControl(robot, idx, targetPosition=jp)

def _update_carried(obj):
    if obj is not None:
        ep, _ = bb.getLinkPose(robot, ee_link)
        bb.resetBasePose(obj, [ep[0], ep[1], ep[2] - OBJ_S * 1.1])

def resync_ik_cur():
    global ik_cur, ik_euler_cur
    ep, eq = bb.getLinkPose(robot, ee_link)
    ik_cur[:] = list(ep)
    try:
        eu = bb.getEulerFromQuaternion(eq)
        ik_euler_cur[:] = list(eu)
    except Exception:
        ik_euler_cur[:] = [math.pi, 0.0, rang]

def send_arm_world(world_pos, world_euler=None, wait=18):
    global ik_cur, ik_euler_cur
    target = list(world_pos)
    steps  = max(wait, 1)
    for step_i in range(steps):
        t = (step_i + 1) / steps
        interp = [ik_cur[i] + (target[i] - ik_cur[i]) * t for i in range(3)]
        arm_ik(interp, world_euler)
        _update_carried(carried_obj)
        time.sleep(0.05)
    ik_cur[:] = target
    if world_euler:
        ik_euler_cur[:] = list(world_euler)

# carry_jpos: arm tucked safely above chassis while driving with load
carry_jpos  = [0.0, 0.2, 0.0, -1.0, 0.0, 1.2, 0.0]

def set_arm_joints(jpos, wait=20):
    for ji, jp in enumerate(jpos):
        bb.setJointMotorControl(robot, ji, targetPosition=jp)
    for _ in range(wait):
        _update_carried(carried_obj)
        time.sleep(0.05)
    # FIX-4: extra settling time before resync so joints finish moving
    time.sleep(0.6)
    resync_ik_cur()


# ================================================================
# OBSTACLE AVOIDANCE HELPERS
# ================================================================
def _obstacle_repulsion(px, py):
    rep_x = rep_y = 0.0
    influence = OBSTACLE_MARGIN * 2.5
    for ox, oy, orad in OBSTACLES:
        eff_r = orad + OBSTACLE_MARGIN
        dx, dy = px-ox, py-oy
        dist = math.sqrt(dx*dx + dy*dy)
        if 0.001 < dist < influence + eff_r:
            strength = (influence + eff_r - dist) / (dist**2 + 0.1)
            rep_x += dx * strength
            rep_y += dy * strength
    return rep_x, rep_y

def _is_clear(px, py):
    for ox, oy, orad in OBSTACLES:
        if math.sqrt((px-ox)**2 + (py-oy)**2) < orad + OBSTACLE_MARGIN:
            return False
    return True


# ================================================================
# NAVIGATION
# ================================================================
def _turn_to(angle):
    global rang
    turn_step = TURN_LOADED if carried_obj else TURN_NORMAL
    for _ in range(50):
        diff = (angle - rang + math.pi) % (2*math.pi) - math.pi
        if abs(diff) < 0.03:
            break
        rang += math.copysign(min(abs(diff), turn_step), diff)
        teleport_all(rx, ry, rang)
        _update_carried(carried_obj)
        time.sleep(0.03)
    rang = angle
    teleport_all(rx, ry, rang)

def drive_to(target_x, target_y, target_ang=None):
    global rx, ry, rang

    ATTRACT_K = 1.0
    REPULSE_K = 0.55
    MAX_STEPS = 1000
    step = STEP_LOADED if carried_obj else STEP_NORMAL

    dx0, dy0 = target_x-rx, target_y-ry
    if math.sqrt(dx0*dx0 + dy0*dy0) > 0.05:
        _turn_to(math.atan2(dy0, dx0))

    prev_rx, prev_ry = rx, ry
    stuck_count = 0

    for step_i in range(MAX_STEPS):
        dx, dy = target_x-rx, target_y-ry
        dist = math.sqrt(dx*dx + dy*dy)
        if dist < 0.12:
            break

        attr_x = (dx/dist) * ATTRACT_K
        attr_y = (dy/dist) * ATTRACT_K
        rep_x, rep_y = _obstacle_repulsion(rx, ry)
        rep_x *= REPULSE_K;  rep_y *= REPULSE_K

        mx, my = attr_x+rep_x, attr_y+rep_y
        mag = math.sqrt(mx*mx + my*my)
        if mag < 0.001: mag = 0.001
        nx, ny = mx/mag, my/mag

        desired = math.atan2(ny, nx)
        diff = (desired-rang + math.pi) % (2*math.pi) - math.pi
        ts = TURN_LOADED if carried_obj else TURN_NORMAL
        rang += math.copysign(min(abs(diff), ts), diff)

        new_rx = rx + step*math.cos(rang)
        new_ry = ry + step*math.sin(rang)

        if _is_clear(new_rx, new_ry):
            rx, ry = new_rx, new_ry
        else:
            moved = False
            for sign in [1, -1]:
                pa = rang + sign*math.pi/2
                srx = rx + step*math.cos(pa)
                sry = ry + step*math.sin(pa)
                if _is_clear(srx, sry):
                    rx, ry = srx, sry
                    moved = True
                    break
            if not moved:
                stuck_count += 1

        teleport_all(rx, ry, rang)
        _update_carried(carried_obj)
        time.sleep(0.03)

        moved_d = math.sqrt((rx-prev_rx)**2 + (ry-prev_ry)**2)
        stuck_count = stuck_count+1 if moved_d < step*0.3 else 0
        prev_rx, prev_ry = rx, ry

        if stuck_count > 12:
            na = rang + (math.pi/2)*(1 if step_i%2==0 else -1)
            for _ in range(8):
                crx = rx + step*math.cos(na)
                cry = ry + step*math.sin(na)
                if _is_clear(crx, cry):
                    rx, ry = crx, cry
                    teleport_all(rx, ry, rang)
                    _update_carried(carried_obj)
                    time.sleep(0.03)
            stuck_count = 0

    rx, ry = target_x, target_y
    _turn_to(target_ang) if target_ang is not None else teleport_all(rx, ry, rang)


# ================================================================
# FIXED WAYPOINT ROUTES
# ================================================================
# FIX-1 & FIX-5: Both corridor entry waypoints pushed further from walls
CORR_ENTRY_A = (0.0,  A_Y0 + 2.5)   # FIX-5: was A_Y0+1.2 → now +2.5 (Y=6.0)
CORR_CENTRE  = (0.0,  0.0)
CORR_ENTRY_B = (0.0,  B_Y0 - 2.8)   # FIX-1: was B_Y0-1.2 → now -2.8 (Y=-6.3)

# FIX-6: larger clearance south of beds
BED_FOOT_Y = BED_POSITIONS[0][1] - BED_L - 3.5   # was -2.5 → now -3.5

def _route_to_room_b():
    drive_to(CORR_ENTRY_A[0], CORR_ENTRY_A[1])
    drive_to(CORR_CENTRE[0],  CORR_CENTRE[1])
    drive_to(CORR_ENTRY_B[0], CORR_ENTRY_B[1])

def _route_to_room_a():
    drive_to(CORR_ENTRY_B[0], CORR_ENTRY_B[1])
    drive_to(CORR_CENTRE[0],  CORR_CENTRE[1])
    drive_to(CORR_ENTRY_A[0], CORR_ENTRY_A[1])

def _auto_pick(obj_key):
    """
    Full pick sequence for the named object key ("red"/"green"/"blue").
    Navigates to source table, picks the object, tucks arm for travel.
    Returns True on success, False if already picked/delivered.
    """
    global carried_obj, _active_pick_key

    sel_obj = OBJECTS[obj_key]
    if sel_obj["delivered"]:
        print(f"[AUTO-PICK] {obj_key.upper()} already delivered — skipping.")
        return False
    if sel_obj["picked"] and carried_obj == sel_obj["handle"]:
        print(f"[AUTO-PICK] Already carrying {obj_key.upper()}.")
        return True

    orig = sel_obj["origin"]
    print(f"[AUTO-PICK] Picking {obj_key.upper()} from source table …")

    # 1. Drive to pick position beside source table
    pick_rx_pos = ITX + T_HX + C_HX + 1.8
    pick_ry_pos = ITY
    drive_to(pick_rx_pos, pick_ry_pos, target_ang=math.pi)

    resync_ik_cur()
    pick_euler = [math.pi, 0.0, rang]

    # 2. Prepare arm
    set_arm_joints(home_jpos, wait=30)

    # 3. Extend above object
    approach_pos = [orig[0], orig[1], orig[2] + APPROACH_H]
    send_arm_world(approach_pos, pick_euler, wait=28)

    # 4. Fine XY align
    ep_a, _ = bb.getLinkPose(robot, ee_link)
    ax = orig[0] + max(-0.04, min(0.04, orig[0] - ep_a[0]))
    ay = orig[1] + max(-0.04, min(0.04, orig[1] - ep_a[1]))
    send_arm_world([ax, ay, orig[2] + APPROACH_H], pick_euler, wait=14)

    # 5. Descend and grasp
    grasp_z = orig[2] + OBJ_S * 0.6
    send_arm_world([ax, ay, grasp_z], pick_euler, wait=24)

    carried_obj       = sel_obj["handle"]
    sel_obj["picked"] = True
    _active_pick_key  = obj_key
    print(f"[AUTO-PICK] Grasped {obj_key.upper()}")

    # 6. Lift and tuck for travel
    send_arm_world([ax, ay, orig[2] + APPROACH_H], pick_euler, wait=22)
    set_arm_joints(carry_jpos, wait=40)
    return True


def _auto_place_at_bed(bed_index):
    """
    Place the currently carried object on the bedside table of bed_index.
    Bedside table is at (bx + BED_W + 0.55, by + BED_L*0.1, place_z).
    """
    global carried_obj, _active_pick_key

    if carried_obj is None:
        print("[AUTO-PLACE] Nothing to place.")
        return

    bx, by = BED_POSITIONS[bed_index]

    # Bedside table top surface Z (matches make_bed construction)
    place_z = 1.50 + 0.06   # monitor stand top ~1.56
    # Place on the flat top of the bedside monitor unit beside the bed
    place_x = bx + BED_W + 0.55
    place_y = by + BED_L * 0.1

    resync_ik_cur()
    place_euler = [math.pi, 0.0, rang]

    app_pos = [place_x, place_y, place_z + APPROACH_H]
    send_arm_world(app_pos, place_euler, wait=22)
    send_arm_world([place_x, place_y, place_z], place_euler, wait=22)

    # Release object
    ep, _ = bb.getLinkPose(robot, ee_link)
    bb.resetBasePose(carried_obj, [ep[0], ep[1], place_z])

    if _active_pick_key:
        OBJECTS[_active_pick_key]["delivered"] = True
        OBJECTS[_active_pick_key]["picked"]    = False
        print(f"[AUTO-PLACE] {_active_pick_key.upper()} placed at Bed{bed_index+1}.")

    carried_obj      = None
    _active_pick_key = None

    # Retract arm up then tuck
    send_arm_world(app_pos, place_euler, wait=16)
    set_arm_joints(home_jpos, wait=25)


def _goto_bed(bed_index):
    """
    Full autonomous delivery mission for bed_index:
      1. Pick assigned object from source table (Room A)
      2. Drive through corridor to Room B
      3. Navigate to bed foot aisle → approach bed
      4. Place object on bedside table
      5. Return through corridor back to starting position in Room A
      6. Return arm to home pose
    """
    global carried_obj

    bx, by = BED_POSITIONS[bed_index]
    obj_key = BED_OBJECT_MAP[bed_index]

    # ── Step 1: ensure we have the correct object ────────────
    if carried_obj is not None and _active_pick_key != obj_key:
        print(f"[MISSION] Carrying wrong object — drop it first or use RESET.")
        return

    if OBJECTS[obj_key]["delivered"]:
        print(f"[MISSION] Bed{bed_index+1} object ({obj_key.upper()}) already delivered.")
        return

    print(f"[MISSION] Starting delivery to Bed{bed_index+1} ({obj_key.upper()}) …")

    # Pick only if not already carrying the right object
    if not (carried_obj is not None and _active_pick_key == obj_key):
        success = _auto_pick(obj_key)
        if not success:
            return

    # ── Step 2: drive to Room B through corridor ─────────────
    print(f"[MISSION] Heading to Room B …")
    _route_to_room_b()

    # ── Step 3: navigate to bed foot aisle ───────────────────
    drive_to(bx, BED_FOOT_Y)

    # ── Step 4: approach bed from south foot ─────────────────
    bed_foot_stop_y = by - BED_L - 1.5
    drive_to(bx, bed_foot_stop_y, target_ang=math.pi/2)
    print(f"[MISSION] At Bed{bed_index+1} foot — placing object …")

    # ── Step 5: place on bedside table ───────────────────────
    _auto_place_at_bed(bed_index)

    # ── Step 6: return to Room A starting position ───────────
    print(f"[MISSION] Returning to Room A …")
    # Move back to bed foot aisle before going north
    drive_to(bx, BED_FOOT_Y)
    _route_to_room_a()
    # Back to robot home position beside source table
    drive_to(ROB_X0, ROB_Y0, target_ang=0.0)

    # Arm back to home
    set_arm_joints(home_jpos, wait=20)
    print(f"[MISSION] Delivery complete. Robot back at start.")


# ================================================================
# MAIN LOOP
# ================================================================
frame_i = 0
while True:
    new_counts = {k: bb.readDebugParameter(k) for k in btn_keys}

    STEP = STEP_LOADED if carried_obj else STEP_NORMAL
    TURN = TURN_LOADED if carried_obj else TURN_NORMAL

    # ── DRIVE ─────────────────────────────────────────────────
    moved = False
    if new_counts['FORWARD'] > btn_counts['FORWARD']:
        rx += STEP*math.cos(rang); ry += STEP*math.sin(rang); moved = True
    if new_counts['BACK']    > btn_counts['BACK']:
        rx -= STEP*math.cos(rang); ry -= STEP*math.sin(rang); moved = True
    if new_counts['LEFT']    > btn_counts['LEFT']:
        rang += TURN; moved = True
    if new_counts['RIGHT']   > btn_counts['RIGHT']:
        rang -= TURN; moved = True
    if moved:
        teleport_all(rx, ry, rang)
        _update_carried(carried_obj)
        resync_ik_cur()

    # ── ARM SLIDERS ────────────────────────────────────────────
    sl_x = bb.readDebugParameter('arm_x')
    sl_y = bb.readDebugParameter('arm_y')
    sl_z = bb.readDebugParameter('arm_z')
    cmd_ax, cmd_ay, cmd_az = local_to_world(sl_x, sl_y, sl_z)

    raw_rx = math.radians(bb.readDebugParameter('ee_rx'))
    raw_ry = math.radians(bb.readDebugParameter('ee_ry'))
    raw_rz = math.radians(bb.readDebugParameter('ee_rz'))
    target_euler = [raw_rx, raw_ry, rang + raw_rz]

    for i in range(3):
        ik_cur[i]       += (([cmd_ax,cmd_ay,cmd_az])[i] - ik_cur[i])       * IK_LERP
        ik_euler_cur[i] += (target_euler[i]              - ik_euler_cur[i]) * ORI_LERP
    world_euler = ik_euler_cur[:]
    arm_ik(ik_cur, world_euler)

    # ── PICK ───────────────────────────────────────────────────
    if new_counts['PICK'] > btn_counts['PICK']:
        if carried_obj is not None:
            print("[PICK] Already carrying -- PLACE first.")
        else:
            sel_key = nearest_available_object() or next_available_object()
            if sel_key is None:
                print("[PICK] All objects delivered.")
            else:
                sel_obj = OBJECTS[sel_key]
                orig    = sel_obj["origin"]
                print(f"[PICK] Auto-navigating to source for {sel_key.upper()}")

                pick_rx_pos = ITX + T_HX + C_HX + 1.8
                pick_ry_pos = ITY
                drive_to(pick_rx_pos, pick_ry_pos, target_ang=math.pi)

                resync_ik_cur()
                pick_euler = [math.pi, 0.0, rang]

                # FIX-3: increased wait from 16 → 30 for arm to fully retract
                set_arm_joints(home_jpos, wait=30)

                approach_pos = [orig[0], orig[1], orig[2] + APPROACH_H]
                send_arm_world(approach_pos, pick_euler, wait=28)

                ep_a, _ = bb.getLinkPose(robot, ee_link)
                ax = orig[0] + max(-0.04, min(0.04, orig[0] - ep_a[0]))
                ay = orig[1] + max(-0.04, min(0.04, orig[1] - ep_a[1]))
                send_arm_world([ax, ay, orig[2] + APPROACH_H], pick_euler, wait=14)

                grasp_z = orig[2] + OBJ_S * 0.6
                send_arm_world([ax, ay, grasp_z], pick_euler, wait=24)

                carried_obj          = sel_obj["handle"]
                sel_obj["picked"]    = True
                _active_pick_key     = sel_key
                print(f"[PICK] Grasped {sel_key.upper()}")

                send_arm_world([ax, ay, orig[2] + APPROACH_H], pick_euler, wait=22)

                # FIX-3: increased carry_jpos wait from 20 → 40
                set_arm_joints(carry_jpos, wait=40)

    # ── PLACE ──────────────────────────────────────────────────
    if new_counts['PLACE'] > btn_counts['PLACE']:
        if carried_obj is None:
            print("[PLACE] Nothing to place.")
        else:
            resync_ik_cur()
            place_euler = [math.pi, 0.0, rang]

            app = [cmd_ax, cmd_ay, cmd_az + APPROACH_H]
            send_arm_world(app, place_euler, wait=20)
            send_arm_world([cmd_ax, cmd_ay, cmd_az], place_euler, wait=20)

            ep, _ = bb.getLinkPose(robot, ee_link)
            bb.resetBasePose(carried_obj, [ep[0], ep[1], cmd_az])

            if _active_pick_key:
                OBJECTS[_active_pick_key]["delivered"] = True
                OBJECTS[_active_pick_key]["picked"]    = False
                print(f"[PLACE] {_active_pick_key.upper()} delivered.")

            carried_obj = None;  _active_pick_key = None

            send_arm_world(app, place_euler, wait=16)
            set_arm_joints(home_jpos, wait=20)

    # ── HOME ───────────────────────────────────────────────────
    if new_counts['HOME'] > btn_counts['HOME']:
        if carried_obj is not None:
            ep, _ = bb.getLinkPose(robot, ee_link)
            bb.resetBasePose(carried_obj, [ep[0], ep[1], OBJ_Z])
            if _active_pick_key:
                OBJECTS[_active_pick_key]["picked"] = False
            carried_obj = None;  _active_pick_key = None
        set_arm_joints(home_jpos, wait=20)

    # ── RESET ──────────────────────────────────────────────────
    if new_counts['RESET'] > btn_counts['RESET']:
        carried_obj = None;  _active_pick_key = None
        rx, ry, rang = float(ROB_X0), float(ROB_Y0), 0.0
        teleport_all(rx, ry, rang)
        for key in PICK_ORDER:
            o = OBJECTS[key]
            o["picked"] = o["delivered"] = False
            bb.resetBasePose(o["handle"], o["origin"])
        set_arm_joints(home_jpos, wait=20)

    # ── GOTO_SOURCE ────────────────────────────────────────────
    if new_counts['GOTO_SOURCE'] > btn_counts['GOTO_SOURCE']:
        drive_to(ITX + T_HX + C_HX + 1.8, ITY, target_ang=math.pi)

    # ── GOTO_DEST ──────────────────────────────────────────────
    if new_counts['GOTO_DEST'] > btn_counts['GOTO_DEST']:
        _route_to_room_b()
        drive_to(DTX + T_HX + C_HX + 1.8, DTY, target_ang=math.pi)

    # ── GOTO_BED1  (Bed1 → RED) ───────────────────────────────
    if new_counts['GOTO_BED1'] > btn_counts['GOTO_BED1']:
        _goto_bed(0)

    # ── GOTO_BED2  (Bed2 → BLUE) ──────────────────────────────
    if new_counts['GOTO_BED2'] > btn_counts['GOTO_BED2']:
        _goto_bed(1)

    # ── GOTO_BED3  (Bed3 → GREEN) ─────────────────────────────
    if new_counts['GOTO_BED3'] > btn_counts['GOTO_BED3']:
        _goto_bed(2)

    btn_counts = new_counts

    if carried_obj is not None:
        ep, _ = bb.getLinkPose(robot, ee_link)
        bb.resetBasePose(carried_obj, [ep[0], ep[1], ep[2]-OBJ_S*1.1])

    time.sleep(0.05)
    frame_i += 1
