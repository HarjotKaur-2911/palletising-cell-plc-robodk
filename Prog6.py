# Palletise 12 boxes (4 x 3) with a UR10e in RoboDK, driven by the PLC over Modbus TCP.
# VERSION 6.2 = your version 5 placement logic + the PLC handshake.
# Set DRY_RUN = True to place all 12 boxes WITHOUT the PLC (for checking box height and look).
#
# Handshake for each box (PLC = OpenPLC Runtime v4, Modbus server on port 1502):
#   1. Wait until Robot_Pick (coil 1) is 1
#   2. Pick and place one box
#   3. Set Robot_Done_In (coil 8) to 1
#   4. Wait until Robot_Pick goes back to 0
#   5. Set Robot_Done_In back to 0
#   6. Repeat until 12 boxes are placed
from robodk.robolink import *
from robodk.robomath import *
import time

DRY_RUN = False          
# ---------------------------------

# ---------- PLC / Modbus settings ----------
PLC_HOST, PLC_PORT = '127.0.0.1', 1502
PICK_COIL = 1            # Robot_Pick    (%QX0.1)  confirmed by test
DONE_COIL = 8            # Robot_Done_In (%QX1.0)
WAIT_PICK_S = 300        # how long to wait for the PLC to ask for a box (press Start in this time)
WAIT_DROP_S = 30         # how long to wait for Robot_Pick to go back to 0
# -------------------------------------------

# ---------- settings you may need to adjust ----------
BOX_W, BOX_L, BOX_H = 178.0, 305.0, 125.0   # box size in mm (X, Y, Z)
GAP = 15.0                                   # gap between boxes
NX, NY = 4, 3                                # 4 across the 800 mm side, 3 along the 1200 mm side
PALLET_W, PALLET_L = 800.0, 1200.0           # pallet size along Pallet frame X and Y
PALLET_H = 0.0                             # height of the pallet deck above the Pallet frame (CHECK THIS)
HOVER = 60.0                                 # height above pick/place used for approach and retreat
MID_UP = 400.0                               # how high the safe middle position is above the placed box
# ------------------------------------------------------

RDK = Robolink()
RDK.ShowMessage('Palletize v6.2 (PLC) running' + (' - DRY RUN' if DRY_RUN else ''), False)

# ---------- Modbus ----------
try:
    from pymodbus.client import ModbusTcpClient
except ImportError:
    try:
        from robodk.robolink import import_install
        import_install('pymodbus')
        from pymodbus.client import ModbusTcpClient
    except Exception:
        raise Exception('pymodbus is not installed in the RoboDK Python. '
                        'Install it for the Python RoboDK uses (RoboDK: Tools > Select Python), then run again.')


def _call(fn, *args, **kw):
    """pymodbus uses 'slave' in some versions and 'device_id' in newer ones. Try both."""
    for key in ('device_id', 'slave'):
        try:
            return fn(*args, **{key: 1}, **kw)
        except TypeError:
            continue
    return fn(*args, **kw)


def read_coil(addr):
    # New connection every time: the PLC closes idle connections.
    c = ModbusTcpClient(PLC_HOST, port=PLC_PORT)
    try:
        if not c.connect():
            raise Exception('Cannot connect to the PLC on port %d. Is the PLC running?' % PLC_PORT)
        rr = _call(c.read_coils, addr, count=1)
        if rr.isError():
            raise Exception('Modbus read failed at coil %d' % addr)
        return bool(rr.bits[0])
    finally:
        c.close()


def write_coil(addr, value):
    if DRY_RUN:
        return
    c = ModbusTcpClient(PLC_HOST, port=PLC_PORT)
    try:
        if not c.connect():
            raise Exception('Cannot connect to the PLC on port %d. Is the PLC running?' % PLC_PORT)
        rr = _call(c.write_coil, addr, bool(value))
        if rr.isError():
            raise Exception('Modbus write failed at coil %d' % addr)
    finally:
        c.close()


def wait_for(addr, state, what, timeout_s):
    if DRY_RUN:
        return
    t0 = time.time()
    while True:
        try:
            if read_coil(addr) == state:
                return
        except Exception as e:
            # a short Modbus hiccup (e.g. PLC restarting) is retried until the timeout
            if time.time() - t0 > timeout_s:
                raise
        if time.time() - t0 > timeout_s:
            raise Exception('Timed out waiting for %s' % what)
        time.sleep(0.2)


# ---------- robot setup (same as v5) ----------
robot = RDK.Item('', ITEM_TYPE_ROBOT)
tool = RDK.ItemList(ITEM_TYPE_TOOL)[0]
pick_target = RDK.Item('Pick_Box', ITEM_TYPE_TARGET)
pallet_frame = RDK.Item('Pallet', ITEM_TYPE_FRAME)
if not (robot.Valid() and tool.Valid() and pick_target.Valid() and pallet_frame.Valid()):
    raise Exception('Could not find robot, tool, Pick_Box target or Pallet frame')

robot.setPoseTool(tool.PoseTool())
robot.setSpeed(800)
robot.setSpeedJoints(120)
robot.setRounding(0)

# Remove boxes from an earlier run, and let go of anything the gripper still holds
tool.DetachAll()
for it in RDK.ItemList(ITEM_TYPE_OBJECT):
    if it.Name().startswith('PalBox_'):
        it.Delete()
for it in RDK.ItemList(ITEM_TYPE_OBJECT):
    if it.Name().startswith('Box 12x7in'):
        it.setVisible(False, False)


def make_box(index, pose_abs):
    """Create a box whose top face centre sits at pose_abs (tool Z points down into the box)."""
    x0, x1 = -BOX_W / 2, BOX_W / 2
    y0, y1 = -BOX_L / 2, BOX_L / 2
    z0, z1 = 0.0, BOX_H
    v = [[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
         [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]]
    faces = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7),
             (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5),
             (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    pts = [v[i] for f in faces for i in f]
    mesh = Mat([[p[0] for p in pts], [p[1] for p in pts], [p[2] for p in pts]])
    shape = RDK.AddShape(mesh)
    shape.setName('PalBox_%02d' % index)
    shape.setColor([0.1, 0.4, 0.9, 1.0])   # bright blue so it stands out from the wooden pallet
    shape.setPoseAbs(pose_abs)
    return shape


def ik_count(pose_abs):
    """How many arm configurations can reach this tool pose (0 = truly out of reach, -1 = check failed)."""
    try:
        sols = robot.SolveIK_All(invH(robot.PoseAbs()) * pose_abs, tool=robot.PoseTool())
        return sols.size(1) if sols.size(0) >= 6 else 0
    except Exception:
        return -1


def try_place(place_abs, mid_abs):
    """One clean try: hover, then straight down. No retries, so the arm never wanders."""
    try:
        robot.MoveJ(place_abs * transl(0, 0, -HOVER))
        robot.MoveL(place_abs)
        return 'hover+place'
    except Exception:
        return None


# Box centres on the pallet (Pallet frame coordinates)
span_x = NX * BOX_W + (NX - 1) * GAP
span_y = NY * BOX_L + (NY - 1) * GAP
x_start = (PALLET_W - span_x) / 2 + BOX_W / 2
y_start = (PALLET_L - span_y) / 2 + BOX_L / 2
positions = [(x_start + i * (BOX_W + GAP), y_start + j * (BOX_L + GAP))
             for j in range(NY) for i in range(NX)]

pick_abs = pick_target.PoseAbs()
pick_hover = pick_abs * transl(0, 0, -HOVER)
pallet_abs = pallet_frame.PoseAbs()
place_z = PALLET_H + BOX_H
box_bottom_z = (pallet_abs * transl(0, 0, PALLET_H)).Pos()[2]
print('HEIGHT CHECK: Pallet frame z=%.1f mm, box bottoms placed at z=%.1f mm (world)' % (pallet_abs.Pos()[2], box_bottom_z))

TURNS = [0]


def place_pose(x, y, turn):
    return pallet_abs * transl(x, y, place_z) * rotx(pi) * rotz(turn * pi / 180.0)


# ---- REACH CHECK: nothing moves here ----
check = []
bad = []
for n, (x, y) in enumerate(positions):
    best = 0
    parts = []
    for turn in TURNS:
        pa = place_pose(x, y, turn)
        h = ik_count(pa * transl(0, 0, -HOVER))
        p = ik_count(pa)
        parts.append('turn%d hover=%d place=%d' % (turn, h, p))
        best = max(best, min(h, p))
    check.append('box %02d (x=%.0f y=%.0f): %s' % (n, x, y, ' | '.join(parts)))
    if best <= 0:
        bad.append(n)
print('REACH CHECK\n' + '\n'.join(check))
if bad:
    RDK.ShowMessage('REACH CHECK: the arm cannot reach boxes %s.\nNothing was moved.\n\n%s' %
                    (bad, '\n'.join(check)), True)
    raise Exception('Out of reach: boxes %s. Move the Pallet frame closer and run again.' % bad)

# ---------- main loop: one box each time the PLC asks ----------
log = []
shapes = []
placed = 0
try:
    write_coil(DONE_COIL, False)                 # start from a clean state

    for n, (x, y) in enumerate(positions):
        RDK.ShowMessage('Waiting for PLC: Robot_Pick (box %d of %d)' % (n + 1, len(positions)), False)
        wait_for(PICK_COIL, True, 'Robot_Pick = 1 (did you press Start and set Box_Sensor?)', WAIT_PICK_S)

        RDK.ShowMessage('Placing box %d of %d' % (n + 1, len(positions)), False)
        shape = make_box(n, pick_abs)            # a fresh box appears at the pick position
        shapes.append(shape)

        robot.MoveJ(pick_hover)
        robot.MoveL(pick_abs)
        shape.setParentStatic(tool)              # vacuum on: the box now travels with the gripper
        robot.MoveL(pick_hover)

        method = None
        used_turn = None
        place_abs = None
        for turn in TURNS:
            place_abs = place_pose(x, y, turn)
            mid_abs = pallet_abs * transl(PALLET_W / 2, PALLET_L / 2, place_z + MID_UP) * rotx(pi)
            method = try_place(place_abs, mid_abs)
            if method:
                used_turn = turn
                break

        hover_abs = place_abs * transl(0, 0, -HOVER)
        log.append('box %02d (x=%.0f y=%.0f): %s turn=%s | configs hover=%d place=%d' %
                   (n, x, y, method, used_turn, ik_count(hover_abs), ik_count(place_abs)))

        if method:
            shape.setParentStatic(pallet_frame)  # vacuum off: the box stays where it is, on the pallet
        else:
            tool.DetachAll()
        if method is None:
            shape.Delete()                       # remove the box that could not be placed
            raise Exception('Could not place box %d (details in the log)' % n)
        placed += 1
        try:
            robot.MoveL(place_abs * transl(0, 0, -HOVER))
        except Exception:
            pass                                 # retreat out of reach: skip it

        # tell the PLC this box is done, then wait for it to drop Robot_Pick
        write_coil(DONE_COIL, True)
        wait_for(PICK_COIL, False, 'Robot_Pick = 0', WAIT_DROP_S)
        write_coil(DONE_COIL, False)             # must clear, or the PLC thinks the next box is already done

    try:
        robot.MoveJ(pick_hover)
    except Exception:
        pass
finally:
    try:
        write_coil(DONE_COIL, False)             # never leave Done stuck at 1
    except Exception:
        pass
    print('\n'.join(log))

if DRY_RUN:
    rows = []
    for sh in shapes:
        try:
            rel = (invH(pallet_abs) * sh.PoseAbs()).Pos()
            rows.append('%s: x=%.0f y=%.0f z=%.0f  parent=%s' % (sh.Name(), rel[0], rel[1], rel[2], sh.Parent().Name()))
        except Exception as e:
            rows.append('%s: could not read position (%s)' % (getattr(sh, 'name', 'box'), e))
    msg = ('DRY RUN done: %d boxes placed (no PLC).\n'
           'Pallet frame z=%.1f mm, box bottoms planned at z=%.1f mm.\n'
           'Box positions relative to the Pallet frame (box TOP should be at z=%.0f):\n%s'
           % (placed, pallet_abs.Pos()[2], box_bottom_z, PALLET_H + BOX_H, '\n'.join(rows)))
    print(msg)
    RDK.ShowMessage(msg, True)
else:
    RDK.ShowMessage('Done: %d boxes placed with the PLC' % placed, False)