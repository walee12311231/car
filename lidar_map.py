#!/usr/bin/env python3
"""
RPLidar Live Map Visualizer
============================
Subscribes to /scan (LaserScan) from ROS2 and renders a persistent
occupancy-style map using pygame.

Controls:
  C     - Clear the map
  S     - Save map as PNG screenshot
  +/-   - Zoom in / out
  Arrow keys / WASD - Pan the view
  Q / ESC - Quit
"""

import sys
import math
import datetime

# ── ROS2 guard ──────────────────────────────────────────────────────────────
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    # Stub so the class definition below doesn't crash
    class Node:
        pass
    LaserScan = None

# ── Pygame ───────────────────────────────────────────────────────────────────
try:
    import pygame
except ImportError:
    print("ERROR: pygame is not installed.  Run:  pip3 install pygame --break-system-packages")
    sys.exit(1)

# ─────────────────────────────────────────────────────────────────────────────
# CONFIGURATION  – tweak these to match your setup
# ─────────────────────────────────────────────────────────────────────────────
SCAN_TOPIC    = "/scan"          # ROS2 topic name
WINDOW_W      = 900              # pixels
WINDOW_H      = 900
MAP_RES       = 0.03             # metres per pixel  (smaller = more detail, more RAM)
MAX_RANGE     = 10.0             # metres  – ignore readings beyond this
PAN_SPEED     = 8                # pixels per keypress
ZOOM_STEP     = 0.1              # fraction per keypress

# Colours (R,G,B)
COL_BG        = (18,  18,  28)   # dark navy background
COL_FREE      = (35,  55,  80)   # free-space rays
COL_WALL      = (0,   220, 180)  # obstacle points  (teal)
COL_ROBOT     = (255, 80,  80)   # robot origin dot
COL_GRID      = (30,  30,  45)   # faint grid lines
COL_HUD_FG    = (200, 255, 220)  # HUD text
COL_HUD_BG    = (0,   0,   0,  160)  # HUD background (with alpha)
# ─────────────────────────────────────────────────────────────────────────────


class LidarMapNode(Node):
    """Thin ROS2 subscriber – stores the most recent LaserScan."""
    def __init__(self):
        super().__init__("lidar_map_visualizer")
        self.latest_scan = None
        self.scan_count   = 0
        self.create_subscription(LaserScan, SCAN_TOPIC, self._cb, 10)
        self.get_logger().info(f"Subscribed to {SCAN_TOPIC}")

    def _cb(self, msg):
        self.latest_scan = msg
        self.scan_count  += 1


def m2p(mx: float, my: float, cx: float, cy: float,
        res: float, zoom: float) -> tuple[int, int]:
    """Convert metric (x,y) → screen pixel with current zoom & pan offset."""
    px = int(cx + mx / (res / zoom))
    py = int(cy - my / (res / zoom))
    return px, py


def draw_grid(surface, cx, cy, res, zoom):
    """Draw faint metric grid lines (1 m spacing)."""
    grid_px = int(1.0 / (res / zoom))          # pixels per 1 m
    if grid_px < 20:                            # too small, skip
        return
    w, h = surface.get_size()
    # vertical lines
    x = cx % grid_px
    while x < w:
        pygame.draw.line(surface, COL_GRID, (int(x), 0), (int(x), h))
        x += grid_px
    # horizontal lines
    y = cy % grid_px
    while y < h:
        pygame.draw.line(surface, COL_GRID, (0, int(y)), (w, int(y)))
        y += grid_px


def render_hud(screen, font, small_font, scan_count, zoom, ros_ok):
    """Overlay transparent HUD panel."""
    lines = [
        f"Scans received : {scan_count}",
        f"Zoom           : {zoom:.2f}x",
        f"ROS2           : {'LIVE' if ros_ok else 'DEMO (no ROS2)'}",
        "",
        "C  - Clear map",
        "S  - Save PNG",
        "+/- - Zoom",
        "Arrows/WASD - Pan",
        "Q / ESC - Quit",
    ]
    pad = 8
    line_h = small_font.get_height() + 2
    panel_w = 220
    panel_h = len(lines) * line_h + pad * 2

    hud = pygame.Surface((panel_w, panel_h), pygame.SRCALPHA)
    hud.fill(COL_HUD_BG)
    screen.blit(hud, (10, 10))

    y = 10 + pad
    for i, line in enumerate(lines):
        colour = (255, 220, 80) if i == 0 else COL_HUD_FG
        txt = small_font.render(line, True, colour)
        screen.blit(txt, (10 + pad, y))
        y += line_h


def demo_scan(t: float):
    """Generate a fake circular room scan so you can test without hardware."""
    import random
    ranges = []
    num_pts = 360
    for i in range(num_pts):
        angle = math.radians(i)
        # simple room: octagon ≈ 3 m + noise
        r = 3.0 + 0.3 * math.sin(4 * angle) + random.gauss(0, 0.02)
        ranges.append(max(0.15, r))
    return ranges, math.radians(-180), math.radians(1.0), 0.15, MAX_RANGE


def main():
    # ── ROS2 init ────────────────────────────────────────────────────────────
    node = None
    if ROS2_AVAILABLE:
        rclpy.init(args=sys.argv)
        node = LidarMapNode()

    # ── Pygame init ───────────────────────────────────────────────────────────
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H), pygame.RESIZABLE)
    pygame.display.set_caption("RPLidar Live Map  |  Jetson Orin Nano")

    font       = pygame.font.SysFont("monospace", 16, bold=True)
    small_font = pygame.font.SysFont("monospace", 13)
    clock      = pygame.time.Clock()

    # Persistent map layer – we draw free-space + obstacles onto this
    map_layer = pygame.Surface((WINDOW_W, WINDOW_H))
    map_layer.fill(COL_BG)

    # View state
    zoom   = 1.0
    pan_x  = WINDOW_W // 2   # screen-space origin (where robot is drawn)
    pan_y  = WINDOW_H // 2
    frame  = 0

    def clear_map():
        map_layer.fill(COL_BG)
        draw_grid(map_layer, pan_x, pan_y, MAP_RES, zoom)

    clear_map()

    running = True
    while running:
        W, H = screen.get_size()

        # ── Events ────────────────────────────────────────────────────────────
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False

            elif ev.type == pygame.VIDEORESIZE:
                map_layer = pygame.transform.scale(map_layer, (ev.w, ev.h))

            elif ev.type == pygame.KEYDOWN:
                if ev.key in (pygame.K_q, pygame.K_ESCAPE):
                    running = False
                elif ev.key == pygame.K_c:
                    clear_map()
                elif ev.key == pygame.K_s:
                    fname = f"lidar_map_{datetime.datetime.now().strftime('%H%M%S')}.png"
                    pygame.image.save(screen, fname)
                    print(f"[Saved] {fname}")
                elif ev.key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                    zoom = min(zoom + ZOOM_STEP, 5.0)
                elif ev.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                    zoom = max(zoom - ZOOM_STEP, 0.1)

        # ── Continuous key pan ────────────────────────────────────────────────
        keys = pygame.key.get_pressed()
        if keys[pygame.K_LEFT]  or keys[pygame.K_a]: pan_x += PAN_SPEED
        if keys[pygame.K_RIGHT] or keys[pygame.K_d]: pan_x -= PAN_SPEED
        if keys[pygame.K_UP]    or keys[pygame.K_w]: pan_y += PAN_SPEED
        if keys[pygame.K_DOWN]  or keys[pygame.K_s]: pan_y -= PAN_SPEED

        # ── Fetch scan data ───────────────────────────────────────────────────
        scan_data   = None
        scan_count  = 0

        if node is not None:
            rclpy.spin_once(node, timeout_sec=0.005)
            if node.latest_scan:
                msg = node.latest_scan
                scan_data  = (
                    msg.ranges,
                    msg.angle_min,
                    msg.angle_increment,
                    msg.range_min,
                    msg.range_max,
                )
                scan_count = node.scan_count
        else:
            # Demo mode – animated fake scan
            import time
            scan_data = demo_scan(time.time())
            scan_count = frame

        # ── Render scan onto map_layer ────────────────────────────────────────
        if scan_data:
            ranges, angle_min, angle_inc, rmin, rmax = scan_data
            rmax = min(rmax, MAX_RANGE)
            angle = angle_min

            for r in ranges:
                if rmin < r < rmax:
                    mx =  r * math.cos(angle)
                    my =  r * math.sin(angle)
                    ox, oy = m2p(0,  0,  pan_x, pan_y, MAP_RES, zoom)
                    wx, wy = m2p(mx, my, pan_x, pan_y, MAP_RES, zoom)

                    # Free-space ray (faint)
                    pygame.draw.line(map_layer, COL_FREE, (ox, oy), (wx, wy), 1)
                    # Obstacle point
                    if 0 <= wx < map_layer.get_width() and 0 <= wy < map_layer.get_height():
                        pygame.draw.circle(map_layer, COL_WALL, (wx, wy), 2)

                angle += angle_inc

        # ── Compose final frame ───────────────────────────────────────────────
        screen.blit(map_layer, (0, 0))

        # Robot origin crosshair
        pygame.draw.circle(screen, COL_ROBOT, (pan_x, pan_y), 6)
        pygame.draw.line(screen, COL_ROBOT, (pan_x - 12, pan_y), (pan_x + 12, pan_y), 2)
        pygame.draw.line(screen, COL_ROBOT, (pan_x, pan_y - 12), (pan_x, pan_y + 12), 2)

        # HUD
        render_hud(screen, font, small_font, scan_count, zoom, node is not None)

        # Mode badge
        mode = "LIVE" if node is not None else "DEMO  (no ROS2)"
        col  = (0, 255, 100) if node is not None else (255, 180, 0)
        badge = font.render(f"● {mode}", True, col)
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
