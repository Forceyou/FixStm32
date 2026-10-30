# -*- coding: utf-8 -*-
# Generate flix-fc.kicad_pcb from flix-fc.kicad_sch + KiCad 10 footprint library
import re, uuid, os, sys, math, heapq

BASE = r"c:\Users\admin\Desktop\flix-master\flix-stm32\kicad\flix-fc"
SCH = os.path.join(BASE, "flix-fc.kicad_sch")
PCB = os.environ.get("PCB_OUT") or os.path.join(BASE, "flix-fc.kicad_pcb")
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
PAD_MARGIN = CLEAR + TRACK_W / 2.0          # 0.30: 焊盘边缘到走线中心的最小合法距离
ROUTE_R    = 3
VIA_R      = 6
VIA_COST   = 8.0
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

# 展开成"边"级别的列表, 便于按单条连接重排优先级(整网重排会过度挤占通道)
pairs = []
for _tot, _netid, _edges in jobs:
    for _i, _j in _edges:
        pairs.append((_netid, _i, _j))

# ---------- 手工走线: 显式指定折点, 相邻折点间用 A* 自动找合法路径 ----------
# 每条 = (网络名, [(x, y, layer), ...]); layer: 0=F.Cu, 1=B.Cu
# 第一个/最后一个点必须落在源/目标焊盘上; 中间折点取自由空间格, A* 负责绕障
MANUAL_ROUTES = [
    # --- MISO: 先走线，对角到 (105.5,126) 过孔 B.Cu，右行到 U1 ---
    ("SPI_MISO", [(106.8, 123.5, 0), (105.5, 126.0, 0),
                  (105.5, 126.0, 1), (121.0, 126.0, 1), (121.0, 129.675, 1),
                  (121.0, 129.675, 0)]),
    # --- IMU_REGOUT: F.Cu 下行到 (102.725,125), 过孔 B.Cu, B.Cu 右行+上行, 过孔回 F.Cu 到 pad10 ---
    ("IMU_REGOUT", [(102.725, 117.0, 0), (102.725, 125.0, 0), (102.725, 125.0, 1),
                    (107.2, 125.0, 1), (107.2, 123.5, 1), (107.2, 123.5, 0)]),
    # --- +3V3: F.Cu 下行到 y=124.2 左行到 x=104.5 过孔, B.Cu 到 C11 ---
    ("+3V3",     [(106.4, 123.5, 0), (106.4, 124.2, 0), (104.5, 124.2, 0),
                  (104.5, 124.2, 1), (104.5, 127.0, 1), (105.225, 127.0, 1),
                  (105.225, 127.0, 0)]),
    # --- MOSI: 向左逃逸到 x=105, 过孔 B.Cu 走 y=118 走廊, x=121.5 过孔 F.Cu 下行到 U1 ---
    ("SPI_MOSI", [(106.0, 120.5, 0), (105.0, 120.5, 0), (105.0, 118.0, 0),
                  (105.0, 118.0, 1), (121.5, 118.0, 1), (121.5, 118.0, 0),
                  (121.5, 129.675, 0)]),
    # --- CS: 陡对角到 (107.5,119.2) 过孔 B.Cu, B.Cu 上行到 y=116.5 右行到 U1 pad3 ---
    ("IMU_CS",   [(106.8, 120.5, 0), (107.5, 119.2, 0), (107.5, 119.2, 1),
                  (107.5, 116.5, 1), (117.325, 116.5, 1), (117.325, 117.0, 1),
                  (117.325, 117.0, 0)]),
    # --- SCK: 直上到 y=119.5 就近过孔, B.Cu 左行到 x=103 下行到底部走廊到 U1 ---
    ("SPI_SCK",  [(106.4, 120.5, 0), (106.4, 119.5, 0), (106.4, 119.5, 1),
                  (103.0, 119.5, 1), (103.0, 129.0, 1), (120.5, 129.0, 1),
                  (120.5, 129.0, 0), (120.5, 129.675, 0)]),
    # --- nRST: U1 pad14 -> R3 (走 y=122 避开 SPI) ---
    ("nRST",     [(117.325, 122.5, 0), (113.825, 122.0, 0), (113.825, 120.0, 0)]),
    # --- nRST: R3 -> C7 (下到 y=119 打过孔, B.Cu 下行到 y=117 左行到 C7) ---
    ("nRST",     [(113.825, 120.0, 0), (113.825, 119.0, 0), (113.825, 119.0, 1),
                  (113.825, 117.0, 1), (106.725, 117.0, 1), (106.725, 117.0, 0)]),
    # --- MOT 三条: U1 右边缘焊盘 -> 芯片内部过孔 -> B.Cu -> J4 ---
    ("MOT_RR", [(132.675, 124.0, 0), (127.0, 124.0, 0), (127.0, 124.0, 1),
                (127.0, 105.08, 1), (140.0, 105.08, 1)]),
    ("MOT_FR", [(132.675, 123.5, 0), (131.0, 123.5, 0), (131.0, 123.5, 1),
                (131.0, 107.62, 1), (140.0, 107.62, 1)]),
    ("MOT_FL", [(132.675, 123.0, 0), (135.0, 123.0, 0), (135.0, 123.0, 1),
                (135.0, 110.16, 1), (140.0, 110.16, 1)]),
    # --- SWDIO: U1 pad71 -> J3 pin1 (右侧 y=118 水平直线) ---
    ("SWDIO", [(132.675, 118.0, 0), (146.0, 118.0, 0)]),
    # --- OSC_IN: U1 pad12 -> C8 pad1 (绕 U1 顶部 y=113.5, 左行到 x=115 避开 LED_STAT) ---
    ("OSC_IN", [(117.325, 121.5, 0), (115.0, 121.5, 0), (115.0, 113.5, 0),
                (135.225, 113.5, 0), (135.225, 114.0, 0)]),
    # --- OSC_OUT: U1 pad13 -> C9 pad1 (绕 U1 底部 y=130) ---
    ("OSC_OUT", [(117.325, 122.0, 0), (116.0, 122.0, 0), (116.0, 130.0, 0),
                 (134.725, 130.0, 0), (134.725, 124.0, 0)]),
    # --- VBAT_ADC: U1 pad22 -> R2 pad1 (左行到 x=116 绕 U1 底部) ---
    ("VBAT_ADC", [(117.325, 126.5, 0), (116.0, 126.5, 0), (116.0, 134.0, 0),
                  (139.175, 134.0, 0)]),
    # --- LED_STAT: U1 pad7 -> D1 (走 y=106 绕开 U2 tab 与 R4) ---
    ("LED_STAT", [(117.325, 119.0, 0), (116.0, 119.0, 0), (116.0, 106.0, 0),
                  (144.0, 106.0, 0), (144.0, 110.7875, 0)]),
    # --- VCAP1: U1 pad48 -> C5 (下行到 y=131 绕 U1 底部, 左行到 x=116, 上行到 y=112.5) ---
    ("VCAP1",    [(130.0, 129.675, 0), (130.0, 131.0, 0), (116.0, 131.0, 0),
                  (116.0, 112.5, 0), (124.725, 112.5, 0)]),
]
MANUAL = {}   # pair -> path (填充在 route_pair 定义之后)

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
        if n_exp > 3000000:
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

def route_pair(pair):
    """A* 布线单条连接, 返回路径(网格单元列表)或 None"""
    netid, i, j = pair
    pts = net_pads[netid]
    ax, ay, aw, ah, atht = pts[i]
    bx, by, bw, bh, btht = pts[j]
    la = (0, 1) if atht else (0,)
    lb = (0, 1) if btht else (0,)
    goals = set()
    # 目标只取焊盘铜箔内缩后的区域, 避免走线终点落在圆形/椭圆焊盘的空角上
    gw = max(0.05, bw / 2.0 * 0.55)
    gh = max(0.05, bh / 2.0 * 0.55)
    for Lb in lb:
        for ix, iy in rect_cells(bx, by, gw, gh):
            goals.add((ix, iy, Lb))
    if not goals:
        cgx, cgy = cell_of(bx, by)
        for Lb in lb:
            goals.add((cgx, cgy, Lb))
    gx, gy = cell_of(bx, by)
    s = cell_of(ax, ay)
    for La in la:
        path = astar(netid, (s[0], s[1], La), goals, gx, gy)
        if path:
            return path
    return None

def rebuild(paths, seq):
    """按给定顺序, 用 paths 中的走线重建占用网格与走线几何"""
    global cu, tracks
    cu = [bytearray(cu_base[0]), bytearray(cu_base[1])]
    tracks = []
    # 手工走线优先固定
    for _p, _path in MANUAL.items():
        mark_route(_p[0], _path)
        path_to_geo(_p[0], _path)
    for p in seq:
        path = paths.get(p)
        if path:
            mark_route(p[0], path)
            path_to_geo(p[0], path)

def first_pass(seq):
    global cu, tracks
    cu = [bytearray(cu_base[0]), bytearray(cu_base[1])]
    tracks = []
    paths = dict(MANUAL)
    for _p, _path in MANUAL.items():
        mark_route(_p[0], _path)
        path_to_geo(_p[0], _path)
    failed = []
    for p in seq:
        path = route_pair(p)
        if path:
            paths[p] = path
            mark_route(p[0], path)
            path_to_geo(p[0], path)
        else:
            failed.append(p)
    return paths, failed

# ---- 计算手工走线路径 (此时 cu == cu_base, 在干净网格上逐段 A*) ----
_manual_pairs = set()
for _name, _wps in MANUAL_ROUTES:
    _nid = NETIDX.get(_name)
    if _nid is None:
        print("MANUAL: unknown net", _name); continue
    _sx, _sy = _wps[0][0], _wps[0][1]
    _ex, _ey = _wps[-1][0], _wps[-1][1]
    _pair = None
    for _p in pairs:
        if _p[0] != _nid:
            continue
        _a = net_pads[_nid][_p[1]]; _b = net_pads[_nid][_p[2]]
        if ((abs(_a[0]-_sx)<0.6 and abs(_a[1]-_sy)<0.6 and abs(_b[0]-_ex)<0.6 and abs(_b[1]-_ey)<0.6) or
            (abs(_b[0]-_sx)<0.6 and abs(_b[1]-_sy)<0.6 and abs(_a[0]-_ex)<0.6 and abs(_a[1]-_ey)<0.6)):
            _pair = _p; break
    if _pair is None:
        print("MANUAL: no matching pair for", _name); continue
    _full = []
    _ok = True
    for _k in range(len(_wps) - 1):
        _ax, _ay, _al = _wps[_k]
        _bx, _by, _bl = _wps[_k + 1]
        _s = cell_of(_ax, _ay); _g = cell_of(_bx, _by)
        _leg = astar(_nid, (_s[0], _s[1], _al), {(_g[0], _g[1], _bl)}, _g[0], _g[1])
        if not _leg:
            print("MANUAL leg fail: %s wp%d->%d (%.2f,%.2f,%d)->(%.2f,%.2f,%d)"
                  % (_name, _k, _k+1, _ax,_ay,_al,_bx,_by,_bl))
            _ok = False; break
        _full.extend(_leg if not _full else _leg[1:])
    if _ok:
        MANUAL[_pair] = _full
        _manual_pairs.add(_pair)
        mark_route(_pair[0], _full)          # 立即占用, 后续手工线避让
        print("MANUAL ok:", _name, "len", len(_full))
pairs = [_p for _p in pairs if _p not in _manual_pairs]

paths, failed = first_pass(pairs)
print("first pass: routed", len(paths), "failed", len(failed))
# ---- 全局重排: 把失败的连接提到最前面, 保留连通数最多的一轮 ----
for _it in range(6):
    if not failed:
        break
    badset = set(failed)
    cand = first_pass(failed + [p for p in pairs if p not in badset])
    print("  reorder round: routed", len(cand[0]), "failed", len(cand[1]))
    if len(cand[0]) > len(paths):
        paths, failed = cand

# ---- 局部拆线重布: 只拆掉目标连接附近的既有走线, 给它腾出通道, 再把拆掉的线补回来 ----
RIP_M = 25          # 拆线范围外扩 2.5 mm
def path_bbox(path):
    xs = [c[0] for c in path]; ys = [c[1] for c in path]
    return min(xs), min(ys), max(xs), max(ys)

def pair_bbox(pair):
    pts = net_pads[pair[0]]
    a = cell_of(pts[pair[1]][0], pts[pair[1]][1])
    b = cell_of(pts[pair[2]][0], pts[pair[2]][1])
    return min(a[0], b[0]) - RIP_M, min(a[1], b[1]) - RIP_M, max(a[0], b[0]) + RIP_M, max(a[1], b[1]) + RIP_M

for _round in range(5):
    n_fail = len([p for p in pairs if p not in paths])
    if not n_fail:
        break
    progress = False
    for pair in [p for p in pairs if p not in paths]:
        bx0, by0, bx1, by1 = pair_bbox(pair)
        victims = [q for q in paths
                   if (lambda bb: bb[0] <= bx1 and bb[2] >= bx0 and bb[1] <= by1 and bb[3] >= by0)(path_bbox(paths[q]))]
        if not victims:
            continue
        trial = dict(paths)
        for q in victims:
            del trial[q]
        rebuild(trial, pairs)
        p = route_pair(pair)
        if not p:
            rebuild(paths, pairs)
            continue
        trial[pair] = p
        rebuild(trial, pairs)
        lost = 0
        for q in victims:
            pq = route_pair(q)
            if pq:
                trial[q] = pq
                mark_route(q[0], pq)
                path_to_geo(q[0], pq)
            else:
                lost += 1
        # 只要连通总数严格增加就接受(允许个别被拆的线改走别处失败)
        if len(trial) > len(paths):
            paths = trial
            progress = True
            print("  local reroute OK: net %d (ripped %d, lost %d)" % (pair[0], len(victims), lost))
        else:
            rebuild(paths, pairs)
    if not progress:
        break

rebuild(paths, pairs)
failed = [p for p in pairs if p not in paths]
routed = len(paths)

print("routed nets (pairs):", routed, "failed:", len(failed))
for _netid, _i, _j in failed[:20]:
    _a = net_pads[_netid][_i]; _b = net_pads[_netid][_j]
    print("  FAIL net %d  %s -> %s" % (_netid, (_a[0], _a[1]), (_b[0], _b[1])))

if os.environ.get("DBG"):
    cu_saved = cu
    cu = [bytearray(cu_base[0]), bytearray(cu_base[1])]
    for _netid, _i, _j in failed:
        _ax, _ay = net_pads[_netid][_i][0:2]
        _bx, _by = net_pads[_netid][_j][0:2]
        s = cell_of(_ax, _ay); g = cell_of(_bx, _by)
        p = astar(_netid, (s[0], s[1], 0),
                  {(g[0], g[1], 0), (g[0], g[1], 1)}, g[0], g[1])
        print("  base-grid reachable:", bool(p), "net", _netid, (_ax, _ay), "->", (_bx, _by))
    cu = cu_saved

# ---- GND stitching vias: tie the F.Cu and B.Cu pours together ----
n_stitch = 0
stitch_pts = []
if gnd_id > 0:
    for gy in range(101, 149, 2):
        for gx in range(104, 149, 2):
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
            stitch_pts.append((ix, iy))
            n_stitch += 1
    # explicit corner / edge stitching vias to tie edge GND islands
    for _gx, _gy in [(104.0, 101.4), (148.6, 101.4), (104.0, 148.6), (148.6, 148.6),
                     (125.0, 101.4), (148.6, 125.0), (125.0, 148.6),
                     (112.0, 101.4), (138.0, 101.4), (104.0, 112.0), (104.0, 138.0),
                     (148.6, 112.0), (148.6, 138.0), (112.0, 148.6), (138.0, 148.6)]:
        ix, iy = cell_of(_gx, _gy)
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
        stitch_pts.append((ix, iy))
        n_stitch += 1
    for _x, _y0, _y1 in [(106.0, 101.4, 106.0), (148.6, 101.4, 106.0),
                         (106.0, 144.0, 148.6), (148.6, 144.0, 148.6)]:
        ix0, iy0 = cell_of(_x, _y0)
        ix1, iy1 = cell_of(_x, _y1)
        for _iy in range(min(iy0, iy1), max(iy0, iy1) + 1):
            for _xx in range(ix0 - ROUTE_R, ix0 + ROUTE_R + 1):
                for L in (0, 1):
                    if cu[L][_iy * NGRID + _xx] == 0:
                        cu[L][_iy * NGRID + _xx] = gnd_id
        emit_seg(gnd_id, (ix0, iy0, 0), (ix1, iy1, 0), 0)
    # horizontal GND spine along top edge to tie top strip islands to main pour
    for _y, segs in [(101.3, [(104.0, 108.0), (111.0, 138.5), (141.5, 148.5)]),
                     (148.7, [(104.0, 148.5)])]:
        iy = cell_of(_y, _y)[1]
        for _x0, _x1 in segs:
            ix0 = cell_of(_x0, _y)[0]
            ix1 = cell_of(_x1, _y)[0]
            for _xx in range(min(ix0, ix1), max(ix0, ix1) + 1):
                for _yy in range(iy - ROUTE_R, iy + ROUTE_R + 1):
                    for L in (0, 1):
                        if cu[L][_yy * NGRID + _xx] == 0:
                            cu[L][_yy * NGRID + _xx] = gnd_id
            emit_seg(gnd_id, (ix0, iy, 0), (ix1, iy, 0), 0)
print("GND stitching vias:", n_stitch)

# tie each GND SMD pad to its nearest stitching via
n_stub = 0
if gnd_id > 0 and stitch_pts:
    for netid, ax, ay, w, h, tht in pad_specs:
        if netid != gnd_id or tht:
            continue
        s = cell_of(ax, ay)
        cand = sorted(stitch_pts, key=lambda v: abs(v[0] - s[0]) + abs(v[1] - s[1]))[:30]
        path = None
        for vx, vy in cand:
            if abs(vx - s[0]) + abs(vy - s[1]) > 120:
                break
            path = astar(gnd_id, (s[0], s[1], 0), {(vx, vy, 0)}, vx, vy)
            if path:
                break
        if path:
            mark_route(gnd_id, path)
            path_to_geo(gnd_id, path)
            n_stub += 1
print("GND pad stubs:", n_stub)



if os.environ.get("MAP"):
    def dump_map(x0, y0, x1, y1, L=0, base=False):
        buf = cu_base[L] if base else cu[L]
        ix0, iy0 = cell_of(x0, y0); ix1, iy1 = cell_of(x1, y1)
        print("== map L=%d %s x %.1f..%.1f y %.1f..%.1f ==" % (L, "base" if base else "routed", x0, x1, y0, y1))
        print("      " + "".join(str((i // 10) % 10) for i in range(ix0, ix1 + 1)))
        for iy in range(iy0, iy1 + 1):
            row = []
            for ix in range(ix0, ix1 + 1):
                v = buf[iy * NGRID + ix]
                if v == 0:
                    row.append(".")
                elif v == BLOCK:
                    row.append("#")
                else:
                    row.append("0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz"[v % 62])
            print("%5d %s" % (iy, "".join(row)))
    for _L in (0, 1):
        dump_map(131.0, 103.0, 142.0, 127.0, _L, True)
    dump_map(131.0, 103.0, 142.0, 127.0, 0, False)
    for _L in (0, 1):
        dump_map(100.0, 114.0, 122.0, 130.0, _L, True)
    dump_map(100.0, 114.0, 122.0, 130.0, 0, False)
    _mr = os.environ.get("MAPXY")
    if _mr:
        _x0, _y0, _x1, _y1 = [float(v) for v in _mr.split(",")]
        dump_map(_x0, _y0, _x1, _y1, 0, True)
        dump_map(_x0, _y0, _x1, _y1, 0, False)

# ---------- 3b. 后处理: 为 U1 的两个 GND 引脚让出 +3V3 通道, 并补 GND 过孔 ----------
# 目的: 消除 3 处孤立 GND 铜岛 (U1.pad10 的 F.Cu 岛、U1.pad19 的细边、C10 的大岛)
def _trk_net(s):
    m = re.search(r'\(net (\d+)\)', s)
    return int(m.group(1)) if m else -1

def _trk_at(s):
    m = re.search(r'\(at ([-\d.]+) ([-\d.]+)\)', s)
    return (round(float(m.group(1)), 3), round(float(m.group(2)), 3)) if m else None

def _trk_seg(s):
    m = re.search(r'\(start ([-\d.]+) ([-\d.]+)\) \(end ([-\d.]+) ([-\d.]+)\)', s)
    if not m:
        return None
    a = tuple(round(float(v), 3) for v in m.groups())
    return (a, (a[2], a[3], a[0], a[1]))

def drop_tracks(net_name, segs=(), vias=()):
    """按 网络名 + 端点/坐标 删除 tracks 中的走线或过孔"""
    nid = NETIDX[net_name]
    segset = set()
    for s in segs:
        t = tuple(round(v, 3) for v in s)
        segset.add(t)
        segset.add((t[2], t[3], t[0], t[1]))
    viaset = set(tuple(round(v, 3) for v in p) for p in vias)
    kept = []
    for t in tracks:
        if _trk_net(t) != nid:
            kept.append(t); continue
        if '(via ' in t:
            if _trk_at(t) in viaset:
                continue
        else:
            sg = _trk_seg(t)
            if sg and (sg[0] in segset or sg[1] in segset):
                continue
        kept.append(t)
    tracks[:] = kept

def add_seg(net_name, x0, y0, x1, y1, layer):
    tracks.append('\t(segment (start %.3f %.3f) (end %.3f %.3f) (width %.2f) (layer "%s") (net %d) (uuid "%s"))'
                  % (x0, y0, x1, y1, TRACK_W, layer, NETIDX[net_name], new_uuid()))

def add_via(net_name, x, y):
    tracks.append('\t(via (at %.3f %.3f) (size %.2f) (drill %.2f) (layers "F.Cu" "B.Cu") (net %d) (uuid "%s"))'
                  % (x, y, VIA_DIA, VIA_DRILL, NETIDX[net_name], new_uuid()))

# --- (1) 拆掉压住 U1.pad19(GND, y=125.0) 的 +3V3 走线/过孔 ---
drop_tracks("+3V3",
            segs=[(115.6, 124.5, 116.2, 125.1), (116.2, 125.1, 116.2, 125.4),
                  (116.2, 125.4, 116.3, 125.5), (116.3, 125.5, 116.9, 125.5),
                  (116.6, 126.0, 115.6, 125.0), (117.3, 126.0, 116.6, 126.0),
                  (115.6, 125.0, 119.5, 125.0), (119.1, 121.0, 115.6, 124.5)],
            vias=[(115.6, 124.5), (115.6, 125.0)])

# --- (2) 重新接好 +3V3: 单个西侧过孔(115.6,125.4), 东侧仍经 (119.5,125.0) ---
add_via("+3V3", 115.6, 125.4)
add_seg("+3V3", 117.325, 125.5, 115.6, 125.5, "F.Cu")   # pad20 -> 过孔上方
add_seg("+3V3", 115.6, 125.5, 115.6, 125.4, "F.Cu")
add_seg("+3V3", 117.325, 126.0, 115.6, 126.0, "F.Cu")   # pad21 -> 西侧
add_seg("+3V3", 115.6, 126.0, 115.6, 125.4, "F.Cu")
add_seg("+3V3", 115.6, 125.4, 119.5, 125.0, "B.Cu")     # 连到东侧过孔
add_seg("+3V3", 119.1, 121.0, 119.5, 125.0, "B.Cu")     # 连回上方 +3V3 簇

# --- (2b) +3V3 斜线东移 0.2mm, 给 U1.pad10 的 GND 走线让出通道 ---
drop_tracks("+3V3", segs=[(118.3, 121.0, 119.3, 120.0), (119.3, 120.0, 119.3, 118.2)])
add_seg("+3V3", 118.5, 121.0, 119.3, 120.2, "F.Cu")
add_seg("+3V3", 119.3, 120.2, 119.3, 118.2, "F.Cu")

# --- (3) GND: U1.pad19 -> 过孔(116.0,124.4) ---
add_via("GND", 116.0, 124.4)
add_seg("GND", 117.325, 125.0, 116.0, 125.0, "F.Cu")
add_seg("GND", 116.0, 125.0, 116.0, 124.4, "F.Cu")

# --- (4) GND: U1.pad10 -> 过孔(118.7,119.8) ---
add_via("GND", 118.7, 119.8)
add_seg("GND", 117.325, 120.5, 118.3, 120.5, "F.Cu")
add_seg("GND", 118.3, 120.5, 118.7, 119.8, "F.Cu")

# --- (5) GND: C10 pad2 所在孤岛内补过孔 ---
add_via("GND", 104.45, 121.1)

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
		(connect_pads yes (clearance 0.2))
		(min_thickness 0.1)
		(filled_areas_thickness no)
		(fill yes (thermal_gap 0.3) (thermal_bridge_width 0.3) (island_removal_mode 0))
		(polygon (pts (xy 102.2 101.0) (xy 149.0 101.0) (xy 149.0 149.0) (xy 102.2 149.0)))
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
