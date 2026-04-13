#!/usr/bin/env python3
"""
RPLidar Live Map Visualizer  –  v2
====================================
Two-layer renderer:
  • Bottom layer  – persistent accumulated wall map (stays between scans)
  • Top layer     – current live scan (cleared and redrawn every frame)

ROS2 spins in a background thread so pygame never stalls waiting for data.

Controls:
  C        – Clear the persistent map
  S        – Save PNG screenshot
  + / -    – Zoom in / out
  Arrows / WASD  – Pan
  Q / ESC  – Quit
"""

import sys
import math
import datetime
import threading

# ── ROS2 ─────────────────────────────────────────────────────────────────────
try:
    import rclpy
    from rclpy.node import Node
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    class Node:
        pass

try:
    from sensor_msgs.msg import LaserScan as _LaserScan
except ImportError:
    _LaserScan = None

# ── Pygame ────────────────────────────────────────────────────────────────────
try:
    import pygame
except ImportError:
    print("pygame not found – run:  pip install pygame  (inside your venv)")
    sys.exit(1)

# ─────────────────────────────────────────────────────────────────────────────
# CONFIG
# ─────────────────────────────────────────────────────────────────────────────
SCAN_TOPIC  = "/scan"
WINDOW_W    = 900
WINDOW_H    = 900
MAP_RES     = 0.03      # metres per pixel at zoom=1
MAX_RANGE   = 10.0      # metres – clamp readings beyond this
PAN_SPEED   = 8         # pixels per frame when key held
ZOOM_STEP   = 0.1

# Colours
COL_BG          = (12,  14,  26)    # near-black background
COL_PERSISTENT  = (0,   180, 140)   # walls drawn onto the saved map
COL_LIVE_RAY    = (30,  60,  90)    # free-space ray (current scan)
COL_LIVE_PT     = (0,   255, 200)   # bright live obstacle point
COL_ROBOT       = (255, 70,  70)    # crosshair
COL_GRID        = (22,  26,  48)    # faint grid
COL_HUD_FG      = (180, 255, 210)
COL_HUD_BG      = (0,   0,   0,   170)
# ─────────────────────────────────────────────────────────────────────────────


class LidarNode(Node):
    """ROS2 node – runs in a background thread, thread-safe scan storage."""
    def __init__(self):
        super().__init__("lidar_map_v2")
        self._lock       = threading.Lock()
        self._scan       = None
        self.scan_count  = 0
        self.create_subscription(
            _LaserScan, SCAN_TOPIC, self._cb, 10)
        self.get_logger().info(f"Listening on {SCAN_TOPIC}")

    def _cb(self, msg):
        with self._lock:
            self._scan      = msg
            self.scan_count += 1

    def get_scan(self):
        with self._lock:
            return self._scan


# ─── helpers ──────────────────────────────────────────────────────────────────

def m2p(mx, my, origin_x, origin_y, res, zoom):
    """Metric (x,y) → screen pixel."""
    scale = zoom / res
    px = int(origin_x + mx * scale)
    py = int(origin_y - my * scale)
    return px, py


def draw_grid(surface, ox, oy, res, zoom):
    grid_px = int(1.0 * zoom / res)
    if grid_px < 30:
        return
    w, h = surface.get_size()
    x = ox % grid_px
    while x < w:
        pygame.draw.line(surface, COL_GRID, (int(x), 0), (int(x), h))
        x += grid_px
    y = oy % grid_px
    while y < h:
        pygame.draw.line(surface, COL_GRID, (0, int(y)), (w, int(y)))
        y += grid_px


def parse_scan(scan):
    """Return list of (mx, my) for valid ranges."""
    points = []
    angle  = scan.angle_min
    rmin   = scan.range_min
    rmax   = min(scan.range_max, MAX_RANGE)
    for r in scan.ranges:
        if rmin < r < rmax:
            points.append((r * math.cos(angle), r * math.sin(angle)))
        angle += scan.angle_increment
    return points


def demo_scan(t):
    """Fake animated room for testing without hardware."""
    import random, time
    pts = []
    for i in range(360):
        a = math.radians(i)
        r = 3.0 + 0.4 * math.sin(3 * a + t) + random.gauss(0, 0.015)
        pts.append((r * math.cos(a), r * math.sin(a)))
    return pts


def render_hud(screen, font, scan_count, zoom, ros_ok):
    lines = [
        f"Scans : {scan_count}",
        f"Zoom  : {zoom:.2f}x",
        f"ROS2  : {'LIVE' if ros_ok else 'DEMO'}",
        "",
        "C – Clear map",
        "S – Save PNG",
        "+/- – Zoom",
        "Arrows – Pan",
        "Q – Quit",
    ]
    lh  = font.get_height() + 3
    pad = 8
    w   = 190
    h   = len(lines) * lh + pad * 2
    hud = pygame.Surface((w, h), pygame.SRCALPHA)
    hud.fill(COL_HUD_BG)
    screen.blit(hud, (10, 10))
    y = 10 + pad
    for i, line in enumerate(lines):
        col = (255, 220, 60) if i < 3 else COL_HUD_FG
        screen.blit(font.render(line, True, col), (18, y))
        y += lh


# ─────────────────────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────────────────────

def main():
    # ── ROS2 background thread ────────────────────────────────────────────────
    node = None
    if ROS2_AVAILABLE and _LaserScan is not None:
        rclpy.init(args=sys.argv)
        node = LidarNode()
        spin_thread = threading.Thread(
            target=rclpy.spin, args=(node,), daemon=True)
        spin_thread.start()

    # ── Pygame setup ──────────────────────────────────────────────────────────
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H), pygame.RESIZABLE)
    pygame.display.set_caption("RPLidar Live Map  |  Jetson Orin Nano")
    font  = pygame.font.SysFont("monospace", 13)
    clock = pygame.time.Clock()

    W, H = screen.get_size()

    # View state – origin starts exactly at centre
    zoom  = 1.0
    pan_x = W // 2
    pan_y = H // 2

    # Persistent map layer (accumulated walls survive between scans)
    map_layer = pygame.Surface((W, H))
    map_layer.fill(COL_BG)

    def clear_map():
        nonlocal map_layer
        map_layer.fill(COL_BG)
        draw_grid(map_layer, pan_x, pan_y, MAP_RES, zoom)

    clear_map()

    import time
    frame       = 0
    last_count  = -1
    live_pts    = []          # current scan points in metric space

    running = True
    while running:
        W, H = screen.get_size()

        # ── Events ────────────────────────────────────────────────────────────
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type == pygame.VIDEORESIZE:
                # Resize map layer to match new window
                new_layer = pygame.Surface((ev.w, ev.h))
                new_layer.fill(COL_BG)
                new_layer.blit(map_layer, (0, 0))
                map_layer = new_layer
            elif ev.type == pygame.KEYDOWN:
                if ev.key in (pygame.K_q, pygame.K_ESCAPE):
                    running = False
                elif ev.key == pygame.K_c:
                    clear_map()
                    live_pts = []
                elif ev.key == pygame.K_s:
                    fname = f"lidar_{datetime.datetime.now().strftime('%H%M%S')}.png"
                    pygame.image.save(screen, fname)
                    print(f"Saved → {fname}")
                elif ev.key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                    zoom = min(zoom + ZOOM_STEP, 6.0)
                    clear_map()
                elif ev.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                    zoom = max(zoom - ZOOM_STEP, 0.1)
                    clear_map()

        # Continuous pan keys
        keys = pygame.key.get_pressed()
        moved = False
        if keys[pygame.K_LEFT]  or keys[pygame.K_a]: pan_x += PAN_SPEED; moved = True
        if keys[pygame.K_RIGHT] or keys[pygame.K_d]: pan_x -= PAN_SPEED; moved = True
        if keys[pygame.K_UP]    or keys[pygame.K_w]: pan_y += PAN_SPEED; moved = True
        if keys[pygame.K_DOWN]  or keys[pygame.K_s]: pan_y -= PAN_SPEED; moved = True
        if moved:
            clear_map()   # redraw grid at new pan position

        # ── Fetch new scan ────────────────────────────────────────────────────
        new_pts = None
        if node is not None:
            scan = node.get_scan()
            if scan is not None and node.scan_count != last_count:
                last_count = node.scan_count
                new_pts = parse_scan(scan)
        else:
            # Demo mode: update every frame
            new_pts = demo_scan(time.time())

        # ── Paint new points onto persistent map layer ─────────────────────
        if new_pts:
            live_pts = new_pts
            ox, oy = pan_x, pan_y
            for mx, my in new_pts:
                wx, wy = m2p(mx, my, ox, oy, MAP_RES, zoom)
                # Faint free-space ray
                pygame.draw.line(map_layer, COL_LIVE_RAY, (ox, oy), (wx, wy), 1)
                # Persistent wall dot
                if 0 <= wx < map_layer.get_width() and 0 <= wy < map_layer.get_height():
                    pygame.draw.circle(map_layer, COL_PERSISTENT, (wx, wy), 2)

        # ── Compose frame ─────────────────────────────────────────────────────
        # 1. Persistent map
        screen.blit(map_layer, (0, 0))

        # 2. Live scan overlay (bright, redrawn every frame)
        if live_pts:
            for mx, my in live_pts:
                wx, wy = m2p(mx, my, pan_x, pan_y, MAP_RES, zoom)
                if 0 <= wx < W and 0 <= wy < H:
                    pygame.draw.circle(screen, COL_LIVE_PT, (wx, wy), 3)

        # 3. Robot crosshair – always at pan_x, pan_y (the true centre)
        cx, cy = pan_x, pan_y
        pygame.draw.circle(screen, COL_ROBOT, (cx, cy), 5)
        pygame.draw.line(screen, COL_ROBOT, (cx - 14, cy), (cx + 14, cy), 2)
        pygame.draw.line(screen, COL_ROBOT, (cx, cy - 14), (cx, cy + 14), 2)

        # 4. HUD
        render_hud(screen, font,
                   node.scan_count if node else frame,
                   zoom,
                   node is not None)

        # 5. Live/Demo badge
        badge_col  = (0, 255, 100) if node else (255, 180, 0)
        badge_text = "● LIVE" if node else "● DEMO"
        badge = font.render(badge_text, True, badge_col)
        screen.blit(badge, (W - badge.get_width() - 12, 12))

        pygame.display.flip()
        clock.tick(30)
        frame += 1

    # ── Cleanup ───────────────────────────────────────────────────────────────
    if node is not None:
        node.destroy_node()
        rclpy.shutdown()
    pygame.quit()
    sys.exit(0)


if __name__ == "__main__":
    main()
