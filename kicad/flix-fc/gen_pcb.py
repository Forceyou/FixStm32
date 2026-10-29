# -*- coding: utf-8 -*-
# Generate flix-fc.kicad_pcb from flix-fc.kicad_sch + KiCad 10 footprint library
import re, uuid, os, sys, math, heapq

BASE = r"c:\Users\admin\Desktop\flix-master\flix-stm32\kicad\flix-fc"
SCH = os.path.join(BASE, "flix-fc.kicad_sch")
PCB = os.path.join(BASE, "flix-fc.kicad_pcb")
LIB = r"C:\Program Files\KiCad\10.0\share\kicad\footprints"

# placement: ref -> (x, y, rot)
PLACE = {
    "U1":  (125.0, 122.0, 0),
    "U4":  (107.0, 122.0, 0),
    "U2":  (117.0, 109.0, 0),
    "Q1":  (113.5, 135.5, 0),
    "R1":  (140.0, 130.0, 0),
    "R2":  (140.0, 134.0, 0),
    "R3":  (113.0, 120.0, 0),
    "R4":  (122.5, 111.5, 270),
    "R5":  (121.0, 140.5, 0),
    "R6":  (129.0, 140.5, 0),
    "R7":  (144.0, 114.0, 90),
    "R8":  (110.5, 136.5, 90),
    "R9":  (117.0, 136.5, 0),
    "C1":  (113.5, 103.0, 90),
    "C2":  (124.0, 108.0, 0),
    "C3":  (116.5, 104.0, 0),
    "C4":  (133.0, 109.0, 0),
    "C5":  (125.5, 112.5, 0),
    "C6":  (129.0, 112.0, 0),
    "C7":  (107.5, 117.0, 0),
    "C8":  (136.0, 114.0, 0),
    "C9":  (135.5, 124.0, 0),
    "C10": (103.5, 117.0, 0),
    "C11": (106.0, 127.0, 0),
    "X1":  (139.0, 120.0, 0),
    "SW1": (112.5, 144.0, 0),
    "D1":  (144.0, 110.0, 90),
    "J1":  (109.5, 102.54, 0),
    "J2":  (125.0, 147.0, 0),
    "J3":  (146.0, 118.0, 0),
    "J4":  (140.0, 102.54, 0),
    "J6":  (102.54, 133.0, 0),
}

def new_uuid():
    return str(uuid.uuid4())

# ---------- pad -> net mapping (from schematic design) ----------
NETMAP = {
    "U1": {"3":"IMU_CS","6":"+3V3","7":"LED_STAT","10":"GND","11":"+3V3","12":"OSC_IN","13":"OSC_OUT",
           "14":"nRST","19":"GND","20":"+3V3","21":"+3V3","22":"VBAT_ADC","26":"GND","27":"+3V3",
           "29":"SPI_SCK","30":"SPI_MISO","31":"SPI_MOSI","48":"VCAP1","49":"+3V3","58":"MOT_RL",
           "59":"MOT_RR","60":"MOT_FR","61":"MOT_FL","68":"SBUS_RX","69":"PA11_DM","70":"PA12_DP",
           "71":"SWDIO","72":"VCAP2","73":"GND","74":"+3V3","75":"SWCLK","93":"BOOT0","98":"+3V3","99":"+3V3","100":"GND"},
    "U4": {"23":"SPI_SCK","24":"SPI_MOSI","9":"SPI_MISO","22":"IMU_CS","12":"IMU_INT","13":"+3V3",
           "8":"+3V3","18":"GND","10":"IMU_REGOUT","11":"GND","25":"GND"},
    "U2": {"1":"GND","2":"+3V3","3":"+5V","4":"+3V3"},
    "Q1": {"1":"Q1_B","2":"GND","3":"SBUS_RX"},
    "R1": {"1":"+VBAT","2":"VBAT_ADC"},
    "R2": {"1":"VBAT_ADC","2":"GND"},
    "R3": {"1":"+3V3","2":"nRST"},
    "R4": {"1":"BOOT0","2":"GND"},
    "R5": {"1":"USB_DM","2":"PA11_DM"},
    "R6": {"1":"USB_DP","2":"PA12_DP"},
    "R7": {"1":"LED_A","2":"+3V3"},
    "R8": {"1":"SBUS_IN","2":"Q1_B"},
    "R9": {"1":"+3V3","2":"SBUS_RX"},
    "C1": {"1":"+5V","2":"GND"},
    "C2": {"1":"+3V3","2":"GND"},
    "C3": {"1":"+3V3","2":"GND"},
    "C4": {"1":"+3V3","2":"GND"},
    "C5": {"1":"VCAP1","2":"GND"},
    "C6": {"1":"VCAP2","2":"GND"},
    "C7": {"1":"nRST","2":"GND"},
    "C8": {"1":"OSC_IN","2":"GND"},
    "C9": {"1":"OSC_OUT","2":"GND"},
    "C10": {"1":"IMU_REGOUT","2":"GND"},
    "C11": {"1":"+3V3","2":"GND"},
    "X1": {"1":"OSC_IN","3":"OSC_OUT","2":"GND","4":"GND"},
    "SW1": {"1":"nRST","2":"GND"},
    "D1": {"1":"LED_STAT","2":"LED_A"},
    "J1": {"1":"+VBAT","2":"+5V","3":"GND"},
    "J2": {"1":"+5V","2":"USB_DM","3":"USB_DP","5":"GND","SH":"GND"},
    "J3": {"1":"SWDIO","2":"SWCLK","3":"+3V3","4":"GND"},
    "J4": {"1":"MOT_RL","2":"MOT_RR","3":"MOT_FR","4":"MOT_FL","5":"+VBAT","6":"GND"},
    "J6": {"1":"SBUS_IN","2":"+5V","3":"GND"},
}
NETIDX = {}

def inject_nets(txt, netmap):
    if not netmap:
        return txt
    out = []
    i = 0
    while True:
        m = re.search(r'\(pad "([^"]+)"', txt[i:])
        if not m:
            out.append(txt[i:]); break
        s = i + m.start()
        d = 0; j = s
        while True:
            c = txt[j]
            if c == '(': d += 1
            elif c == ')':
                d -= 1
                if d == 0: break
            j += 1
        padnum = m.group(1)
        block = txt[i:j+1]
        n = netmap.get(padnum, "")
        if n:
            block = block[:-1] + '\n\t\t\t(net %d "%s")\n\t\t)' % (NETIDX[n], n)
        out.append(block)
        i = j + 1
    return ''.join(out)

# ---------- 1. parse schematic for instances ----------
sch = open(SCH, encoding="utf-8").read()

# split top-level symbol instances (they start with "(symbol (lib_id")
insts = []
for m in re.finditer(r'\(symbol \(lib_id "([^"]+)"\)(.*?)\n\t\)\n', sch, re.S):
    body = m.group(2)
    ref = re.search(r'\(property "Reference" "([^"]+)"', body)
    val = re.search(r'\(property "Value" "([^"]+)"', body)
    fp  = re.search(r'\(property "Footprint" "([^"]+)"', body)
    if ref and fp and fp.group(1):
        insts.append({
            "ref": ref.group(1),
            "val": val.group(1) if val else "",
            "fp":  fp.group(1),
        })

print("instances parsed:", len(insts))
if len(insts) < 30:
    print("ERROR: too few instances, abort"); sys.exit(1)

# ---------- 2. nets from global labels ----------
nets = sorted(set(re.findall(r'\(global_label "([^"]+)"', sch)))
for i, n in enumerate(nets, 1):
    NETIDX[n] = i
print("nets:", len(nets))

# ---------- 3. build footprints ----------
def load_footprint(libid):
    libname, fpname = libid.split(":", 1)
    path = os.path.join(LIB, libname + ".pretty", fpname + ".kicad_mod")
    if not os.path.exists(path):
        print("MISSING FOOTPRINT:", path); sys.exit(1)
    return open(path, encoding="utf-8").read()

def make_instance(libid, ref, val, x, y, rot):
    txt = load_footprint(libid)
    # rename footprint to Lib:Name
    fpname = libid.split(":", 1)[1]
    txt = txt.replace('(footprint "%s"' % fpname, '(footprint "%s"' % libid, 1)
    # insert uuid + at right after (layer "F.Cu")
    at = '(uuid "%s") (at %.3f %.3f%s)' % (new_uuid(), x, y, (" %.0f" % rot) if rot else "")
    txt = txt.replace('(layer "F.Cu")', '(layer "F.Cu")\n\t\t%s' % at, 1)
    # set Reference / Value
    txt = re.sub(r'\(property "Reference" "[^"]*"', '(property "Reference" "%s"' % ref, txt, count=1)
    txt = re.sub(r'\(property "Value" "[^"]*"', '(property "Value" "%s"' % val, txt, count=1)
    # regenerate every uuid for uniqueness
    txt = re.sub(r'\(uuid "[0-9a-f-]{36}"\)', lambda m: '(uuid "%s")' % new_uuid(), txt)
    # assign pad nets
    txt = inject_nets(txt, NETMAP.get(ref, {}))
    return txt

fp_blocks = []
for inst in insts:
    ref = inst["ref"]
    if ref not in PLACE:
        print("NO PLACEMENT for", ref); sys.exit(1)
    x, y, rot = PLACE[ref]
    fp_blocks.append(make_instance(inst["fp"], ref, inst["val"], x, y, rot))

# mounting holes (not in schematic)
for i, (hx, hy) in enumerate([(104.0,104.0),(146.0,104.0),(104.0,146.0),(146.0,146.0)], 1):
    fp_blocks.append(make_instance("MountingHole:MountingHole_3.2mm_M3", "MH%d" % i, "M3", hx, hy, 0))

print("footprint blocks:", len(fp_blocks))

# ---------- 4b. autorouter (2-layer grid maze router) ----------
PITCH      = 0.1
NGRID      = 500
BLOCK      = 255
TRACK_W    = 0.2
CLEAR      = 0.2
PAD_MARGIN = CLEAR + TRACK_W / 2.0 + 0.05   # 0.35, rounded-rect expansion
ROUTE_R    = 4
VIA_R      = 6
VIA_COST   = 14.0
VIA_DIA    = 0.5
VIA_DRILL  = 0.3

def cell_of(x, y):
    return int(round((x - 100.0) / PITCH)), int(round((y - 100.0) / PITCH))

def xy_of(ix, iy):
    return 100.0 + ix * PITCH, 100.0 + iy * PITCH

def k_of(ix, iy):
    return iy * NGRID + ix

def span(cx, cy, hw, hh):
    ix0, iy0 = cell_of(cx - hw, cy - hh)
    ix1, iy1 = cell_of(cx + hw, cy + hh)
    return max(0, ix0), max(0, iy0), min(NGRID - 1, ix1), min(NGRID - 1, iy1)

def rect_cells(cx, cy, hw, hh, strict=False):
    """cells whose centre lies inside (or strictly inside) the rect"""
    ix0 = max(0, int(math.floor((cx - hw - 100.0) / PITCH)))
    ix1 = min(NGRID - 1, int(math.ceil((cx + hw - 100.0) / PITCH)))
    iy0 = max(0, int(math.floor((cy - hh - 100.0) / PITCH)))
    iy1 = min(NGRID - 1, int(math.ceil((cy + hh - 100.0) / PITCH)))
    for iy in range(iy0, iy1 + 1):
        y = 100.0 + iy * PITCH
        dy = abs(y - cy)
        if (dy >= hh - 1e-9) if strict else (dy > hh + 1e-9):
            continue
        for ix in range(ix0, ix1 + 1):
            x = 100.0 + ix * PITCH
            dx = abs(x - cx)
            if (dx >= hw - 1e-9) if strict else (dx > hw + 1e-9):
                continue
            yield ix, iy

def pad_zone_cells(cx, cy, hw, hh, r):
    """cells whose centre is closer than r (Euclidean) to the pad rectangle.
    Expands the rectangle with rounded corners, unlike a plain bounding box."""
    ix0 = max(0, int(math.floor((cx - hw - r - 100.0) / PITCH)))
    ix1 = min(NGRID - 1, int(math.ceil((cx + hw + r - 100.0) / PITCH)))
    iy0 = max(0, int(math.floor((cy - hh - r - 100.0) / PITCH)))
    iy1 = min(NGRID - 1, int(math.ceil((cy + hh + r - 100.0) / PITCH)))
    rr = r * r - 1e-9
    for iy in range(iy0, iy1 + 1):
        dy = abs(100.0 + iy * PITCH - cy) - hh
        dy = dy if dy > 0.0 else 0.0
        dy2 = dy * dy
        if dy2 >= rr:
            continue
        for ix in range(ix0, ix1 + 1):
            dx = abs(100.0 + ix * PITCH - cx) - hw
            dx = dx if dx > 0.0 else 0.0
            if dx * dx + dy2 < rr:
                yield ix, iy

def rot_pt(px, py, ang):
    if not ang:
        return px, py
    a = math.radians(ang)
    c, s = math.cos(a), math.sin(a)
    return px * c + py * s, -px * s + py * c

# --- parse pads from generated footprint blocks ---
pad_specs = []
drills = []
for blk in fp_blocks:
    m = re.search(r'\(layer "F\.Cu"\)\s*\(uuid "[^"]+"\) \(at ([-\d.]+) ([-\d.]+)(?: ([-\d.]+))?\)', blk)
    if not m:
        continue
    fx, fy = float(m.group(1)), float(m.group(2))
    frot = float(m.group(3)) if m.group(3) else 0.0
    for pm in re.finditer(r'\(pad "', blk):
        s = pm.start(); d = 0; j = s
        while j < len(blk):
            ch = blk[j]
            if ch == '(':
                d += 1
            elif ch == ')':
                d -= 1
                if d == 0:
                    break
            j += 1
        seg = blk[s:j + 1]
        nat = re.search(r'\(at ([-\d.]+) ([-\d.]+)(?: ([-\d.]+))?\)', seg)
        nsz = re.search(r'\(size ([-\d.]+) ([-\d.]+)\)', seg)
        nnt = re.search(r'\(net (\d+) "([^"]+)"\)', seg)
        nly = re.search(r'\(layers ([^)]*)\)', seg)
        ndr = re.search(r'\(drill ([-\d.]+)', seg)
        if not (nat and nsz):
            continue
        if nly and 'F.Cu' not in nly.group(1) and '*.Cu' not in nly.group(1):
            continue        # paste-only / mask-only graphic, no copper
        px, py = float(nat.group(1)), float(nat.group(2))
        prot = float(nat.group(3)) if nat.group(3) else 0.0
        w, h = float(nsz.group(1)), float(nsz.group(2))
        netid = int(nnt.group(1)) if nnt else 0
        tht = bool(nly and '*.Cu' in nly.group(1))
        ccx = ccy = 0.0
        if '(primitives' in seg:
            # custom pad:真实铜箔由多边形定义,size 只是 anchor
            npw = re.search(r'\(width ([-\d.]+)\)\s*\(fill', seg)
            pw = float(npw.group(1)) / 2.0 if npw else 0.0
            xys = [(float(m.group(1)), float(m.group(2)))
                   for m in re.finditer(r'\(xy ([-\d.]+) ([-\d.]+)\)', seg)]
            if xys:
                xs = [p[0] for p in xys]; ys = [p[1] for p in xys]
                ccx = (min(xs) + max(xs)) / 2.0
                ccy = (min(ys) + max(ys)) / 2.0
                w = (max(xs) - min(xs)) + 2.0 * pw
                h = (max(ys) - min(ys)) + 2.0 * pw
        ang = (frot + prot) % 180.0
        ox, oy = rot_pt(ccx, ccy, ang)
        ax, ay = rot_pt(px, py, frot)
        ax += fx + ox; ay += fy + oy
        if abs(ang - 90.0) < 1e-6:
            w, h = h, w
        pad_specs.append((netid, ax, ay, w, h, tht))
        if ndr:
            drills.append((ax, ay, float(ndr.group(1)) / 2.0))

for hx, hy in ((104.0, 104.0), (146.0, 104.0), (104.0, 146.0), (146.0, 146.0)):
    drills.append((hx, hy, 1.6))


print("pads parsed:", len(pad_specs))

# --- build occupancy grid: 0 free, 1..63 own net, 255 blocked ---
owner = [bytearray(NGRID * NGRID), bytearray(NGRID * NGRID)]
conf  = [bytearray(NGRID * NGRID), bytearray(NGRID * NGRID)]

for netid, ax, ay, w, h, tht in pad_specs:
    if netid <= 0:
        continue
    for L in ((0, 1) if tht else (0,)):
        o = owner[L]
        c = conf[L]
        for ix, iy in pad_zone_cells(ax, ay, w / 2.0, h / 2.0, PAD_MARGIN):
            k = iy * NGRID + ix
            if o[k] == 0:
                o[k] = netid
            elif o[k] != netid:
                c[k] = 1

cu = [bytearray(NGRID * NGRID), bytearray(NGRID * NGRID)]
for L in (0, 1):
    o, c, u = owner[L], conf[L], cu[L]
    for k in range(NGRID * NGRID):
        if o[k]:
            u[k] = BLOCK if c[k] else o[k]

# pads with no net are still physical copper -> block them
for netid, ax, ay, w, h, tht in pad_specs:
    if netid != 0:
        continue
    for L in ((0, 1) if tht else (0,)):
        u = cu[L]
        for ix, iy in pad_zone_cells(ax, ay, w / 2.0, h / 2.0, PAD_MARGIN):
            u[iy * NGRID + ix] = BLOCK

# pad cores are always accessible by their own net
for netid, ax, ay, w, h, tht in pad_specs:
    if netid <= 0:
        continue
    for L in ((0, 1) if tht else (0,)):
        u = cu[L]
        for ix, iy in rect_cells(ax, ay, w / 2.0, h / 2.0):
            u[iy * NGRID + ix] = netid

# board edge keepout (0.6 mm) and mounting holes
EDGE = 100.7
ix_lo = int(math.ceil((EDGE - 100.0) / PITCH))
ix_hi = int(math.floor((149.3 - 100.0) / PITCH))
for L in (0, 1):
    u = cu[L]
    for iy in range(NGRID):
        y = 100.0 + iy * PITCH
        base = iy * NGRID
        if y < EDGE or y > 149.3:
            rng = range(NGRID)
        else:
            rng = list(range(0, ix_lo)) + list(range(ix_hi + 1, NGRID))
        for ix in rng:
            if u[base + ix] == 0:
                u[base + ix] = BLOCK

for hx, hy in ((104.0, 104.0), (146.0, 104.0), (104.0, 146.0), (146.0, 146.0)):
    for L in (0, 1):
        u = cu[L]
        for ix, iy in rect_cells(hx, hy, 1.95, 1.95, strict=True):
            k = iy * NGRID + ix
            if u[k] == 0:
                u[k] = BLOCK

# cells where placing a via would break hole-to-hole clearance
via_hole = bytearray(NGRID * NGRID)
for hx, hy, hr in drills:
    r = hr + VIA_DRILL / 2.0 + 0.25
    for ix, iy in rect_cells(hx, hy, r, r):
        if math.hypot(100.0 + ix * PITCH - hx, 100.0 + iy * PITCH - hy) <= r:
            via_hole[iy * NGRID + ix] = 1

# snapshot of the obstacle-only grid, so routing passes can be replayed cleanly
cu_base = [bytearray(cu[0]), bytearray(cu[1])]


# --- routing jobs: MST per net, longest nets first, GND last ---
net_pads = {}
for netid, ax, ay, w, h, tht in pad_specs:
    if netid <= 0:
        continue
    net_pads.setdefault(netid, []).append((ax, ay, w, h, tht))

def mst_edges(pts):
    n = len(pts); used = [False] * n; used[0] = True; out = []
    while len(out) < n - 1:
        best = None
        for i in range(n):
            if not used[i]:
                continue
            for j in range(n):
                if used[j]:
                    continue
                d = (pts[i][0] - pts[j][0]) ** 2 + (pts[i][1] - pts[j][1]) ** 2
                if best is None or d < best[0]:
                    best = (d, i, j)
        if not best:
            break
        used[best[2]] = True
        out.append((best[1], best[2]))
    return out

jobs = []
for netid, pts in net_pads.items():
    if len(pts) < 2:
        continue
    e = mst_edges(pts)
    tot = sum(math.hypot(pts[i][0] - pts[j][0], pts[i][1] - pts[j][1]) for i, j in e)
    jobs.append((tot, netid, e))
jobs.sort(reverse=True)

gnd_id = NETIDX.get("GND", -1)
# GND 由双面铺铜连接（布线后用 --refill-zones 填充），不占用走线通道
jobs = [j for j in jobs if j[1] != gnd_id]

MOVES = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
         (1, 1, 1.41421356), (1, -1, 1.41421356), (-1, 1, 1.41421356), (-1, -1, 1.41421356)]

def via_ok(net, ix, iy):
    if ix < VIA_R or iy < VIA_R or ix >= NGRID - VIA_R or iy >= NGRID - VIA_R:
        return False
    if via_hole[iy * NGRID + ix]:
        return False
    for L in (0, 1):
        buf = cu[L]
        for yy in range(iy - VIA_R, iy + VIA_R + 1):
            base = yy * NGRID
            for xx in range(ix - VIA_R, ix + VIA_R + 1):
                v = buf[base + xx]
                if v and v != net:
                    return False
    return True

def astar(net, start, goals, gx, gy):
    pq = [(0.0, 0.0, start)]
    best = {start: 0.0}
    par = {start: None}
    n_exp = 0
    while pq:
        f, g, cur = heapq.heappop(pq)
        if g > best.get(cur, 1e18) + 1e-9:
            continue
        if cur in goals:
            path = []
            while cur is not None:
                path.append(cur)
                cur = par[cur]
            return path[::-1]
        n_exp += 1
        if n_exp > 900000:
            return None
        ix, iy, L = cur
        buf = cu[L]
        for dx, dy, cost in MOVES:
            nx, ny = ix + dx, iy + dy
            if nx < 0 or ny < 0 or nx >= NGRID or ny >= NGRID:
                continue
            v = buf[ny * NGRID + nx]
            if v and v != net:
                continue
            if dx and dy:
                if buf[iy * NGRID + nx] not in (0, net) or buf[ny * NGRID + ix] not in (0, net):
                    continue
            st = (nx, ny, L)
            ng = g + cost
            if ng < best.get(st, 1e18) - 1e-9:
                best[st] = ng
                par[st] = cur
                hh = max(abs(nx - gx), abs(ny - gy)) + 0.41421356 * min(abs(nx - gx), abs(ny - gy))
                heapq.heappush(pq, (ng + hh, ng, st))
        if via_ok(net, ix, iy):
            st = (ix, iy, 1 - L)
            ng = g + VIA_COST
            if ng < best.get(st, 1e18) - 1e-9:
                best[st] = ng
                par[st] = cur
                hh = max(abs(ix - gx), abs(iy - gy)) + 0.41421356 * min(abs(ix - gx), abs(iy - gy))
                heapq.heappush(pq, (ng + hh, ng, st))
    return None

def mark_route(net, path):
    via_cells = set()
    for a, b in zip(path, path[1:]):
        if a[2] != b[2]:
            via_cells.add((a[0], a[1]))
    for ix, iy, L in path:
        cu[L][iy * NGRID + ix] = net
    for ix, iy, L in path:
        r = VIA_R if (ix, iy) in via_cells else ROUTE_R
        buf = cu[L]
        for yy in range(max(0, iy - r), min(NGRID - 1, iy + r) + 1):
            base = yy * NGRID
            for xx in range(max(0, ix - r), min(NGRID - 1, ix + r) + 1):
                k = base + xx
                if buf[k] == 0:
                    buf[k] = net

tracks = []

def emit_seg(net, a, b, L):
    x0, y0 = xy_of(a[0], a[1])
    x1, y1 = xy_of(b[0], b[1])
    if abs(x0 - x1) < 1e-9 and abs(y0 - y1) < 1e-9:
        return
    tracks.append('\t(segment (start %.3f %.3f) (end %.3f %.3f) (width %.2f) (layer "%s") (net %d) (uuid "%s"))'
                  % (x0, y0, x1, y1, TRACK_W, "F.Cu" if L == 0 else "B.Cu", net, new_uuid()))

def emit_via(net, ix, iy):
    x, y = xy_of(ix, iy)
    tracks.append('\t(via (at %.3f %.3f) (size %.2f) (drill %.2f) (layers "F.Cu" "B.Cu") (net %d) (uuid "%s"))'
                  % (x, y, VIA_DIA, VIA_DRILL, net, new_uuid()))

def path_to_geo(net, path):
    start = path[0]
    prev = path[0]
    for cur in path[1:]:
        if cur[2] != prev[2]:
            if (start[0], start[1]) != (prev[0], prev[1]):
                emit_seg(net, start, prev, prev[2])
            emit_via(net, prev[0], prev[1])
            start = cur
        else:
            d = ((cur[0] > prev[0]) - (cur[0] < prev[0]),
                 (cur[1] > prev[1]) - (cur[1] < prev[1]))
            ds = ((prev[0] > start[0]) - (prev[0] < start[0]),
                  (prev[1] > start[1]) - (prev[1] < start[1]))
            if (start[0], start[1]) != (prev[0], prev[1]) and d != ds:
                emit_seg(net, start, prev, prev[2])
                start = prev
        prev = cur
    if (start[0], start[1]) != (prev[0], prev[1]):
        emit_seg(net, start, prev, prev[2])

def run_pass(order):
    global cu, tracks
    cu = [bytearray(cu_base[0]), bytearray(cu_base[1])]
    tracks = []
    routed = 0
    failed = []
    for tot, netid, edges in order:
        pts = net_pads[netid]
        for i, j in edges:
            ax, ay, aw, ah, atht = pts[i]
            bx, by, bw, bh, btht = pts[j]
            la = (0, 1) if atht else (0,)
            lb = (0, 1) if btht else (0,)
            goals = set()
            for Lb in lb:
                for ix, iy in rect_cells(bx, by, bw / 2.0, bh / 2.0):
                    goals.add((ix, iy, Lb))
            gx, gy = cell_of(bx, by)
            path = None
            s = cell_of(ax, ay)
            for La in la:
                path = astar(netid, (s[0], s[1], La), goals, gx, gy)
                if path:
                    break
            if path:
                mark_route(netid, path)
                path_to_geo(netid, path)
                routed += 1
            else:
                failed.append((netid, (ax, ay), (bx, by)))
    return routed, failed

routed, failed = run_pass(jobs)
best = (routed, list(failed), list(tracks), [bytearray(cu[0]), bytearray(cu[1])])
if failed:
    badnets = []
    for netid, _a, _b in failed:
        if netid not in badnets:
            badnets.append(netid)
    order2 = [j for j in jobs if j[1] in badnets] + [j for j in jobs if j[1] not in badnets]
    r2, f2 = run_pass(order2)
    print("pass2 (failed nets first): routed", r2, "failed", len(f2))
    if len(f2) < len(failed):
        best = (r2, list(f2), list(tracks), [bytearray(cu[0]), bytearray(cu[1])])
routed, failed, tracks, cu = best

print("routed nets (pairs):", routed, "failed:", len(failed))
for f in failed[:20]:
    print("  FAIL net %d  %s -> %s" % f)

# ---- GND stitching vias: tie the F.Cu and B.Cu pours together ----
n_stitch = 0
if gnd_id > 0:
    for gy in range(102, 149, 6):
        for gx in range(102, 149, 6):
            ix, iy = cell_of(gx, gy)
            if ix < VIA_R or iy < VIA_R or ix >= NGRID - VIA_R or iy >= NGRID - VIA_R:
                continue
            if cu[0][iy * NGRID + ix] not in (0, gnd_id):
                continue
            if not via_ok(gnd_id, ix, iy):
                continue
            for L in (0, 1):
                for yy in range(iy - VIA_R, iy + VIA_R + 1):
                    for xx in range(ix - VIA_R, ix + VIA_R + 1):
                        k = yy * NGRID + xx
                        if cu[L][k] == 0:
                            cu[L][k] = gnd_id
            emit_via(gnd_id, ix, iy)
            n_stitch += 1
print("GND stitching vias:", n_stitch)



# ---------- 4. assemble pcb ----------
header = """(kicad_pcb
	(version 20250513)
	(generator "pcbnew")
	(generator_version "10.0")
	(general
		(thickness 1.6)
		(legacy_teardrops no)
	)
	(paper "A4")
	(title_block
		(title "Flix Flight Controller - STM32F407 + MPU9250 all-in-one")
		(date "2026-10-28")
		(rev "2.0")
	)
	(layers
		(0 "F.Cu" signal)
		(2 "B.Cu" signal)
		(9 "F.Adhes" user "F.Adhesive")
		(11 "B.Adhes" user "B.Adhesive")
		(13 "F.Paste" user)
		(15 "B.Paste" user)
		(5 "F.SilkS" user "F.Silkscreen")
		(7 "B.SilkS" user "B.Silkscreen")
		(1 "F.Mask" user)
		(3 "B.Mask" user)
		(17 "Dwgs.User" user "User.Drawings")
		(19 "Cmts.User" user "User.Comments")
		(21 "Eco1.User" user "User.Eco1")
		(23 "Eco2.User" user "User.Eco2")
		(25 "Edge.Cuts" user)
		(27 "Margin" user)
		(31 "F.CrtYd" user "F.Courtyard")
		(29 "B.CrtYd" user "B.Courtyard")
		(35 "F.Fab" user)
		(33 "B.Fab" user)
		(39 "User.1" user)
	)
	(setup
		(pad_to_mask_clearance 0)
	)
"""

netsec = '\t(net 0 "")\n'
for i, n in enumerate(nets, 1):
    netsec += '\t(net %d "%s")\n' % (i, n)

netclass = '\t(net_class "Default" ""\n\t\t(clearance 0.2)\n\t\t(trace_width 0.25)\n\t\t(via_dia 0.8)\n\t\t(via_drill 0.4)\n'
for n in nets:
    netclass += '\t\t(add_net "%s")\n' % n
netclass += '\t)\n'

outline = """	(gr_rect (start 100 100) (end 150 150)
		(stroke (width 0.1) (type solid)) (fill none) (layer "Edge.Cuts") (uuid "%s"))
	(gr_text "Flix FC rev2.0" (at 125 98 0) (layer "F.SilkS") (uuid "%s")
		(effects (font (size 1.5 1.5) (thickness 0.3))))
""" % (new_uuid(), new_uuid())

# GND copper pours on both layers
gnd = NETIDX["GND"]
zones = ""
for layer in ("F.Cu", "B.Cu"):
    zones += """	(zone (net %d) (net_name "GND") (layer "%s") (uuid "%s")
		(hatch edge 0.508)
		(connect_pads yes (clearance 0.25))
		(min_thickness 0.2)
		(filled_areas_thickness no)
		(fill yes (thermal_gap 0.3) (thermal_bridge_width 0.3))
		(polygon (pts (xy 100.5 100.5) (xy 149.5 100.5) (xy 149.5 149.5) (xy 100.5 149.5)))
	)
""" % (gnd, layer, new_uuid())

with open(PCB, "w", encoding="utf-8", newline="\n") as f:
    f.write(header)
    f.write(netsec)
    f.write(netclass)
    f.write(outline)
    f.write(zones)
    for b in fp_blocks:
        # indent footprint block by one tab
        f.write("\n")
        f.write(re.sub(r"\n", "\n\t", b.rstrip()))
        f.write("\n")
    for t in tracks:
        f.write(t + "\n")
    f.write(")\n")

print("written:", PCB)
