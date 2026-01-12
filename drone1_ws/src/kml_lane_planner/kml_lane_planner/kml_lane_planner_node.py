#!/usr/bin/env python3
# Copyright 2024 Shaan Shoukath
# SPDX-License-Identifier: Apache-2.0

"""
KML Lane Planner Node

Watches for KML files in drone1_ws/missions folder, converts them to
survey waypoints using the KMLToWaypointV8 algorithm, and publishes
lane segments for navigation.

Features:
- File watching for new KML files in missions folder
- Smart corner detection and optimized waypoint ordering
- Parallel flight lines along longest polygon side
- Buffer distance from boundaries
- Home position optimization

Subscribers:
    /mavros/global_position/global (sensor_msgs/NavSatFix): For home position

Publishers:
    /mission/lane_segments (drone1_msgs/LaneSegmentArray): Survey lanes
    /drone1/next_waypoint (sensor_msgs/NavSatFix): Current target waypoint
"""

import os
import glob
import math
import sys
import xml.etree.ElementTree as ET
import re
import json
from datetime import datetime
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import String, Header
from sensor_msgs.msg import NavSatFix

from drone1_msgs.msg import Waypoint, LaneSegment, LaneSegmentArray


@dataclass
class MissionConfig:
    """Mission configuration parameters."""
    altitude: float = 6.7  # 22 feet
    spacing: float = 5.0
    buffer_distance: float = 2.0
    home_lat: float = 0.0
    home_lon: float = 0.0


class KMLToWaypointConverter:
    """
    KML to ArduPilot Waypoint Converter
    Enhanced with smart corner detection and optimized waypoint ordering
    """
    
    def __init__(self, config: MissionConfig):
        self.altitude = config.altitude
        self.spacing = config.spacing
        self.buffer_distance = config.buffer_distance
        self.home_position = (config.home_lat, config.home_lon) if config.home_lat != 0.0 else None
    
    def calculate_distance(self, point1: Tuple[float, float], point2: Tuple[float, float]) -> float:
        """Calculate distance between two lat/lon points in meters using Haversine."""
        lat1, lon1 = point1
        lat2, lon2 = point2
        
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        
        a = (math.sin(dlat/2)**2 + 
             math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        R = 6371000  # Earth's radius in meters
        return R * c

    def _wgs84_constants(self):
        a = 6378137.0
        f = 1.0 / 298.257223563
        e2 = f * (2 - f)
        return a, f, e2

    def latlon_to_ecef(self, lat: float, lon: float, alt: float = 0.0) -> Tuple[float, float, float]:
        a, f, e2 = self._wgs84_constants()
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        N = a / math.sqrt(1 - e2 * (math.sin(lat_rad) ** 2))
        x = (N + alt) * math.cos(lat_rad) * math.cos(lon_rad)
        y = (N + alt) * math.cos(lat_rad) * math.sin(lon_rad)
        z = (N * (1 - e2) + alt) * math.sin(lat_rad)
        return x, y, z

    def ecef_to_latlon(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        a, f, e2 = self._wgs84_constants()
        lon = math.degrees(math.atan2(y, x))
        p = math.sqrt(x * x + y * y)
        lat = math.atan2(z, p * (1 - e2))
        for _ in range(5):
            N = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)
            alt = p / math.cos(lat) - N
            lat = math.atan2(z, p * (1 - e2 * (N / (N + alt))))
        N = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)
        alt = p / math.cos(lat) - N
        return math.degrees(lat), lon, alt

    def latlon_to_enu(self, lat: float, lon: float, lat0: float, lon0: float, 
                      alt: float = 0.0, alt0: float = 0.0) -> Tuple[float, float, float]:
        x, y, z = self.latlon_to_ecef(lat, lon, alt)
        x0, y0, z0 = self.latlon_to_ecef(lat0, lon0, alt0)
        dx, dy, dz = x - x0, y - y0, z - z0
        
        lat0_r = math.radians(lat0)
        lon0_r = math.radians(lon0)
        sin_lat, cos_lat = math.sin(lat0_r), math.cos(lat0_r)
        sin_lon, cos_lon = math.sin(lon0_r), math.cos(lon0_r)
        
        e = -sin_lon * dx + cos_lon * dy
        n = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
        u = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
        return e, n, u

    def enu_to_latlon(self, e: float, n: float, lat0: float, lon0: float, 
                      u: float = 0.0, alt0: float = 0.0) -> Tuple[float, float]:
        x0, y0, z0 = self.latlon_to_ecef(lat0, lon0, alt0)
        
        lat0_r = math.radians(lat0)
        lon0_r = math.radians(lon0)
        sin_lat, cos_lat = math.sin(lat0_r), math.cos(lat0_r)
        sin_lon, cos_lon = math.sin(lon0_r), math.cos(lon0_r)
        
        dx = -sin_lon * e - sin_lat * cos_lon * n + cos_lat * cos_lon * u
        dy = cos_lon * e - sin_lat * sin_lon * n + cos_lat * sin_lon * u
        dz = cos_lat * n + sin_lat * u
        
        lat, lon, alt = self.ecef_to_latlon(x0 + dx, y0 + dy, z0 + dz)
        return lat, lon

    def parse_kml(self, kml_file: str) -> List[Tuple[float, float]]:
        """Parse KML file and extract polygon coordinates."""
        try:
            tree = ET.parse(kml_file)
            root = tree.getroot()

            namespace = {'kml': 'http://www.opengis.net/kml/2.2'}
            if root.tag.startswith('{'):
                namespace_match = re.match(r'\{([^}]+)\}', root.tag)
                if namespace_match:
                    namespace = {'kml': namespace_match.group(1)}

            coordinates_elem = None
            paths_to_try = [
                './/kml:Polygon/kml:outerBoundaryIs/kml:LinearRing/kml:coordinates',
                './/kml:Polygon/kml:exterior/kml:LinearRing/kml:coordinates',
                './/kml:coordinates'
            ]

            for path in paths_to_try:
                try:
                    coordinates_elem = root.find(path, namespace)
                    if coordinates_elem is not None:
                        break
                except:
                    continue

            if coordinates_elem is None:
                for path in ['.//Polygon//coordinates', './/coordinates']:
                    coordinates_elem = root.find(path)
                    if coordinates_elem is not None:
                        break

            if coordinates_elem is None:
                raise ValueError("Could not find coordinates in KML file")

            coords_text = coordinates_elem.text.strip()
            coordinates = []

            for line in coords_text.split():
                if line.strip():
                    parts = line.strip().split(',')
                    if len(parts) >= 2:
                        lon = float(parts[0])
                        lat = float(parts[1])
                        coordinates.append((lat, lon))

            if len(coordinates) < 3:
                raise ValueError("Need at least 3 coordinates to define a polygon")

            return coordinates

        except Exception as e:
            print(f"Error parsing KML file: {e}")
            return []

    def create_buffer_polygon(self, polygon: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        Create an inward buffer polygon to keep drone away from boundaries.
        
        Uses edge-based offsetting: moves each edge inward by buffer_distance,
        then finds new vertex positions at edge intersections.
        This ensures ALL sides are shrunk uniformly.
        """
        if len(polygon) < 3:
            return polygon
        
        # Convert to ENU for proper distance calculations
        center_lat = sum(p[0] for p in polygon) / len(polygon)
        center_lon = sum(p[1] for p in polygon) / len(polygon)
        
        polygon_enu = []
        for lat, lon in polygon:
            e, n, _ = self.latlon_to_enu(lat, lon, center_lat, center_lon)
            polygon_enu.append((e, n))
        
        n_pts = len(polygon_enu)
        
        # Calculate inward normal for each edge and offset the edge
        offset_edges = []
        for i in range(n_pts):
            p1 = polygon_enu[i]
            p2 = polygon_enu[(i + 1) % n_pts]
            
            # Edge direction
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            length = math.hypot(dx, dy)
            
            if length < 1e-6:
                continue
            
            # Inward normal (perpendicular, pointing inward for CCW polygon)
            # For CCW: inward normal is (-dy, dx) normalized
            # For CW: inward normal is (dy, -dx) normalized
            # We'll check winding and adjust
            nx = -dy / length
            ny = dx / length
            
            # Offset the edge inward by buffer_distance
            offset_p1 = (p1[0] + nx * self.buffer_distance, p1[1] + ny * self.buffer_distance)
            offset_p2 = (p2[0] + nx * self.buffer_distance, p2[1] + ny * self.buffer_distance)
            
            offset_edges.append((offset_p1, offset_p2))
        
        if len(offset_edges) < 3:
            return polygon
        
        # Find new vertices at intersections of consecutive offset edges
        buffered_enu = []
        for i in range(len(offset_edges)):
            edge1 = offset_edges[i]
            edge2 = offset_edges[(i + 1) % len(offset_edges)]
            
            # Find intersection of edge1 and edge2
            intersection = self._line_intersection_point(
                edge1[0], edge1[1], edge2[0], edge2[1]
            )
            
            if intersection:
                buffered_enu.append(intersection)
            else:
                # Edges are parallel, use midpoint of endpoints
                mid = ((edge1[1][0] + edge2[0][0]) / 2, (edge1[1][1] + edge2[0][1]) / 2)
                buffered_enu.append(mid)
        
        # Check if polygon is valid (not self-intersecting due to over-buffering)
        # If buffer is too large, the polygon may collapse
        if len(buffered_enu) < 3:
            return polygon  # Return original if buffer failed
        
        # Convert back to lat/lon
        buffered_polygon = []
        for e, n in buffered_enu:
            lat, lon = self.enu_to_latlon(e, n, center_lat, center_lon)
            buffered_polygon.append((lat, lon))
        
        return buffered_polygon
    
    def _line_intersection_point(self, p1, p2, p3, p4):
        """Find intersection point of two lines (p1-p2) and (p3-p4)."""
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3
        x4, y4 = p4
        
        denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if abs(denom) < 1e-10:
            return None  # Parallel lines
        
        t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
        
        ix = x1 + t * (x2 - x1)
        iy = y1 + t * (y2 - y1)
        
        return (ix, iy)

    def find_polygon_corners(self, polygon: List[Tuple[float, float]]) -> Dict[str, Tuple[float, float]]:
        """Find the four corners of the polygon."""
        if len(polygon) < 3:
            return {}
        
        lats = [p[0] for p in polygon]
        lons = [p[1] for p in polygon]
        min_lat, max_lat = min(lats), max(lats)
        min_lon, max_lon = min(lons), max(lons)
        
        corners = {'top_right': None, 'top_left': None, 'bottom_left': None, 'bottom_right': None}
        
        for point in polygon:
            lat, lon = point
            
            tr_dist = math.sqrt((lat - max_lat)**2 + (lon - max_lon)**2)
            tl_dist = math.sqrt((lat - max_lat)**2 + (lon - min_lon)**2)
            bl_dist = math.sqrt((lat - min_lat)**2 + (lon - min_lon)**2)
            br_dist = math.sqrt((lat - min_lat)**2 + (lon - max_lon)**2)
            
            if corners['top_right'] is None or tr_dist < math.sqrt(
                (corners['top_right'][0] - max_lat)**2 + (corners['top_right'][1] - max_lon)**2):
                corners['top_right'] = point
            if corners['top_left'] is None or tl_dist < math.sqrt(
                (corners['top_left'][0] - max_lat)**2 + (corners['top_left'][1] - min_lon)**2):
                corners['top_left'] = point
            if corners['bottom_left'] is None or bl_dist < math.sqrt(
                (corners['bottom_left'][0] - min_lat)**2 + (corners['bottom_left'][1] - min_lon)**2):
                corners['bottom_left'] = point
            if corners['bottom_right'] is None or br_dist < math.sqrt(
                (corners['bottom_right'][0] - min_lat)**2 + (corners['bottom_right'][1] - max_lon)**2):
                corners['bottom_right'] = point
        
        return corners

    def line_intersection(self, p1: Tuple[float, float], p2: Tuple[float, float],
                         p3: Tuple[float, float], p4: Tuple[float, float]) -> Optional[Tuple[float, float]]:
        """Find intersection point between two line segments."""
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3
        x4, y4 = p4
        
        denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if abs(denom) < 1e-10:
            return None
        
        t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
        u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom
        
        if 0 <= u <= 1:
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            return (x, y)
        
        return None

    def line_polygon_intersections(self, line_start: Tuple[float, float], 
                                   line_end: Tuple[float, float],
                                   polygon: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Find intersection points between a line and polygon edges."""
        intersections = []
        
        for i in range(len(polygon)):
            edge_start = polygon[i]
            edge_end = polygon[(i + 1) % len(polygon)]
            
            intersection = self.line_intersection(line_start, line_end, edge_start, edge_end)
            if intersection:
                is_duplicate = False
                for existing in intersections:
                    if abs(intersection[0] - existing[0]) < 1e-8 and abs(intersection[1] - existing[1]) < 1e-8:
                        is_duplicate = True
                        break
                if not is_duplicate:
                    intersections.append(intersection)
        
        return intersections

    def generate_waypoints(self, polygon: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Generate flight waypoints using parallel lines along longest side."""
        if len(polygon) < 3:
            return []

        buffered_polygon = self.create_buffer_polygon(polygon)
        corners = self.find_polygon_corners(buffered_polygon)

        # Convert to ENU
        center_lat = sum(p[0] for p in buffered_polygon) / len(buffered_polygon)
        center_lon = sum(p[1] for p in buffered_polygon) / len(buffered_polygon)

        polygon_enu = []
        for lat, lon in buffered_polygon:
            e, n, _ = self.latlon_to_enu(lat, lon, center_lat, center_lon)
            polygon_enu.append((e, n))

        # Find longest side in ENU
        max_length = 0.0
        best_angle_deg = 0.0
        for i in range(len(polygon_enu)):
            e1, n1 = polygon_enu[i]
            e2, n2 = polygon_enu[(i + 1) % len(polygon_enu)]
            dx, dy = e2 - e1, n2 - n1
            length = math.hypot(dx, dy)
            if length > max_length:
                max_length = length
                best_angle_deg = math.degrees(math.atan2(dy, dx)) % 360

        # Direction vectors
        angle_rad = math.radians(best_angle_deg)
        dir_ux, dir_uy = math.cos(angle_rad), math.sin(angle_rad)
        perp_ux, perp_uy = -dir_uy, dir_ux

        # Calculate bounds
        es = [p[0] for p in polygon_enu]
        ns = [p[1] for p in polygon_enu]
        diag = math.hypot(max(es) - min(es), max(ns) - min(ns))
        max_offset = diag / 2 + self.spacing * 2

        all_lines = []
        k = 0
        while True:
            offset = k * self.spacing
            if offset > max_offset:
                break

            for sign in [-1, 1] if k != 0 else [1]:
                off = offset * sign
                p0_e, p0_n = perp_ux * off, perp_uy * off

                L = diag * 2 + 1000
                start = (p0_e - dir_ux * L, p0_n - dir_uy * L)
                end = (p0_e + dir_ux * L, p0_n + dir_uy * L)

                intersections_enu = self.line_polygon_intersections(start, end, polygon_enu)
                if len(intersections_enu) >= 2:
                    def proj_dist(p):
                        return (p[0] - p0_e) * dir_ux + (p[1] - p0_n) * dir_uy
                    intersections_enu.sort(key=proj_dist)
                    
                    intersections_latlon = [
                        self.enu_to_latlon(p[0], p[1], center_lat, center_lon) 
                        for p in intersections_enu
                    ]
                    all_lines.append({
                        'offset': off,
                        'start_point': intersections_latlon[0],
                        'end_point': intersections_latlon[-1]
                    })
            k += 1

        all_lines.sort(key=lambda x: x['offset'])

        # v8 Optimized ordering: start from top-right, end near top-left
        waypoints = self.optimize_line_ordering_for_corners(all_lines, corners, best_angle_deg)

        return waypoints
    
    def optimize_line_ordering_for_corners(self, all_lines: List[Dict], corners: Dict, bearing: float) -> List[Tuple[float, float]]:
        """Optimize line ordering to start from top-right and end near top-left for minimal return distance."""
        if not all_lines:
            return []
        
        first_line = all_lines[0]
        last_line = all_lines[-1]
        
        waypoints = []
        
        if self.home_position and corners.get('top_right') and corners.get('top_left'):
            first_line_start = first_line['start_point']
            first_line_end = first_line['end_point']
            last_line_start = last_line['start_point']
            last_line_end = last_line['end_point']
            
            tr_corner = corners['top_right']
            
            pattern_starts = [
                (first_line_start, "first_start"),
                (first_line_end, "first_end"),
                (last_line_start, "last_start"), 
                (last_line_end, "last_end")
            ]
            
            closest_to_tr = min(pattern_starts, 
                               key=lambda x: self.calculate_distance(x[0], tr_corner))
            
            if closest_to_tr[1].startswith("first"):
                line_order = range(len(all_lines))
                start_from_end = (closest_to_tr[1] == "first_end")
            else:
                line_order = range(len(all_lines) - 1, -1, -1)
                start_from_end = (closest_to_tr[1] == "last_end")
        else:
            line_order = range(len(all_lines))
            start_from_end = False
        
        for i, line_idx in enumerate(line_order):
            line = all_lines[line_idx]
            start_point = line['start_point']
            end_point = line['end_point']
            
            if i == 0:
                if start_from_end:
                    waypoints.extend([end_point, start_point])
                else:
                    waypoints.extend([start_point, end_point])
            else:
                prev_end = waypoints[-1]
                start_dist = self.calculate_distance(prev_end, start_point)
                end_dist = self.calculate_distance(prev_end, end_point)
                
                if start_dist < end_dist:
                    waypoints.extend([start_point, end_point])
                else:
                    waypoints.extend([end_point, start_point])
        
        if self.home_position and len(waypoints) > 0:
            first_to_home = self.calculate_distance(waypoints[0], self.home_position)
            last_to_home = self.calculate_distance(waypoints[-1], self.home_position)
            
            if last_to_home < first_to_home * 0.8:
                waypoints = list(reversed(waypoints))
        
        return waypoints


class KmlLanePlannerNode(Node):
    """ROS2 node for KML to waypoint conversion with file watching."""
    
    def __init__(self):
        super().__init__('kml_lane_planner_node')
        
        # Declare parameters
        self.declare_parameter('missions_folder', 'missions')
        self.declare_parameter('kml_filename', '')  # Specific KML file to load
        self.declare_parameter('altitude_m', 6.7)  # 22 feet
        self.declare_parameter('lane_spacing_m', 15.0)  # Distance between parallel lines
        self.declare_parameter('buffer_distance_m', 2.0)
        self.declare_parameter('watch_interval_sec', 2.0)
        self.declare_parameter('enable_republish', False)  # Prefer TRANSIENT_LOCAL over periodic republish
        self.declare_parameter('require_gps_home', False)  # Set to False for SITL
        self.declare_parameter('default_home_lat', 10.0478)  # User location India
        self.declare_parameter('default_home_lon', 76.3303)
        
        # Get parameters
        self.missions_folder = os.path.expanduser(self.get_parameter('missions_folder').value)
        self.kml_filename = self.get_parameter('kml_filename').value
        self.enable_republish = self.get_parameter('enable_republish').value
        self.require_gps_home = self.get_parameter('require_gps_home').value
        default_lat = self.get_parameter('default_home_lat').value
        default_lon = self.get_parameter('default_home_lon').value
        
        self.config = MissionConfig(
            altitude=self.get_parameter('altitude_m').value,
            spacing=self.get_parameter('lane_spacing_m').value,
            buffer_distance=self.get_parameter('buffer_distance_m').value,
            home_lat=default_lat if not self.require_gps_home else 0.0,
            home_lon=default_lon if not self.require_gps_home else 0.0
        )
        watch_interval = self.get_parameter('watch_interval_sec').value
        
        # State - initialize home from default if not requiring GPS
        if not self.require_gps_home:
            self.home_position = (default_lat, default_lon)
        else:
            self.home_position = None
        self.processed_files = set()
        self.current_mission_id = 0
        self.last_mission_msg = None  # Store for republishing
        self.republish_count = 0  # Track republish attempts
        self.max_republish = 15  # Republish for 30 seconds (15 x 2s)
        
        # QoS
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )
        
        # Subscribers
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_callback,
            sensor_qos
        )
        
        self.load_kml_sub = self.create_subscription(
            String,
            '/mission/load_kml',
            self.load_kml_callback,
            10
        )
        
        # Publishers
        self.lane_pub = self.create_publisher(
            LaneSegmentArray,
            '/mission/lane_segments',
            reliable_qos
        )
        
        self.waypoint_pub = self.create_publisher(
            NavSatFix,
            '/drone1/next_waypoint',
            10
        )
        
        # File watch timer
        self.watch_timer = self.create_timer(watch_interval, self.check_for_new_kml)
        
        # Mission republish timer (optional; TRANSIENT_LOCAL already covers late subscribers)
        self.republish_timer = None
        if self.enable_republish:
            self.republish_timer = self.create_timer(2.0, self.republish_mission)
        
        # Ensure missions folder exists
        os.makedirs(self.missions_folder, exist_ok=True)
        
        self.get_logger().info('KML Lane Planner Node initialized')
        self.get_logger().info(f'  Watching folder: {self.missions_folder}')
        self.get_logger().info(f'  KML filename: {self.kml_filename if self.kml_filename else "auto-detect"}')
        self.get_logger().info(f'  Altitude: {self.config.altitude}m')
        self.get_logger().info(f'  Lane spacing: {self.config.spacing}m')
        self.get_logger().info(f'  Require GPS home: {self.require_gps_home}')
        self.get_logger().info(f'  Enable republish: {self.enable_republish}')
        if not self.require_gps_home:
            self.get_logger().info(f'  Using default home: ({default_lat:.6f}, {default_lon:.6f})')
        
        # Process specific KML file if provided
        if self.kml_filename:
            kml_path = os.path.join(self.missions_folder, self.kml_filename)
            if os.path.exists(kml_path):
                self.get_logger().info(f'Loading specified KML: {kml_path}')
                self.process_kml_file(kml_path)
                self.processed_files.add(kml_path)
            else:
                self.get_logger().error(f'Specified KML file not found: {kml_path}')
    
    def gps_callback(self, msg: NavSatFix):
        """Update home position from GPS."""
        if self.home_position is None and msg.latitude != 0.0:
            self.home_position = (msg.latitude, msg.longitude)
            self.config.home_lat = msg.latitude
            self.config.home_lon = msg.longitude
            self.get_logger().info(
                f'Home position set: ({msg.latitude:.6f}, {msg.longitude:.6f})'
            )
    
    def republish_mission(self):
        """Republish mission periodically for late subscribers."""
        if self.last_mission_msg is None:
            return
        
        if self.republish_count >= self.max_republish:
            return  # Stop republishing after max attempts
        
        self.republish_count += 1
        
        # Update timestamp
        self.last_mission_msg.header.stamp = self.get_clock().now().to_msg()
        self.lane_pub.publish(self.last_mission_msg)
        
        if self.republish_count == 1:
            self.get_logger().info('Republishing mission for late subscribers...')
        elif self.republish_count == self.max_republish:
            self.get_logger().info('Stopped mission republishing (30s elapsed)')
    
    def load_kml_callback(self, msg: String):
        """Manually trigger KML loading."""
        kml_path = msg.data
        if os.path.exists(kml_path):
            self.process_kml_file(kml_path)
        else:
            self.get_logger().error(f'KML file not found: {kml_path}')
    
    def check_for_new_kml(self):
        """Check for new KML files in missions folder."""
        kml_files = glob.glob(os.path.join(self.missions_folder, '*.kml'))
        
        for kml_file in kml_files:
            if kml_file not in self.processed_files:
                self.get_logger().info(f'New KML file detected: {kml_file}')
                self.process_kml_file(kml_file)
                self.processed_files.add(kml_file)
    
    def process_kml_file(self, kml_path: str):
        """Process KML file and publish lane segments."""
        self.get_logger().info(f'Processing KML: {kml_path}')
        
        # For testing: use default home if no GPS fix yet
        if self.home_position is None or self.home_position == (0.0, 0.0):
            self.get_logger().warn('No GPS home position - using default from parameters')
            default_lat = self.get_parameter('default_home_lat').value
            default_lon = self.get_parameter('default_home_lon').value
            self.home_position = (default_lat, default_lon)
            self.config.home_lat = default_lat
            self.config.home_lon = default_lon
        
        # Create converter
        converter = KMLToWaypointConverter(self.config)
        
        # Parse KML
        polygon = converter.parse_kml(kml_path)
        if not polygon:
            self.get_logger().error('Failed to parse KML polygon')
            return
        
        self.get_logger().info(f'Parsed {len(polygon)} boundary points')
        
        # Generate waypoints
        waypoints = converter.generate_waypoints(polygon)
        if not waypoints:
            self.get_logger().error('No waypoints generated')
            return
        
        self.get_logger().info(f'Generated {len(waypoints)} waypoints')
        
        # Log waypoints to file
        self.log_waypoints_to_file(waypoints, kml_path)
        
        # Convert to lane segments
        self.current_mission_id += 1
        lane_array = self.create_lane_segments(waypoints)
        
        # Store and publish
        self.last_mission_msg = lane_array
        self.republish_count = 0  # Reset republish counter
        self.lane_pub.publish(lane_array)
        
        self.get_logger().info(
            f'Published mission {self.current_mission_id} with '
            f'{len(lane_array.lanes)} lane segments'
        )
    
    def create_lane_segments(self, waypoints: List[Tuple[float, float]]) -> LaneSegmentArray:
        """Convert waypoints to LaneSegmentArray message."""
        msg = LaneSegmentArray()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.mission_id = str(self.current_mission_id)
        
        # Create segments from waypoint pairs
        for i in range(0, len(waypoints) - 1, 2):
            if i + 1 >= len(waypoints):
                break
            
            segment = LaneSegment()
            
            # Start waypoint
            segment.start_waypoint = Waypoint()
            segment.start_waypoint.latitude = waypoints[i][0]
            segment.start_waypoint.longitude = waypoints[i][1]
            segment.start_waypoint.altitude = self.config.altitude
            segment.start_waypoint.altitude_mode = 1  # ALTITUDE_RELATIVE
            segment.start_waypoint.waypoint_id = i
            
            # End waypoint
            segment.end_waypoint = Waypoint()
            segment.end_waypoint.latitude = waypoints[i + 1][0]
            segment.end_waypoint.longitude = waypoints[i + 1][1]
            segment.end_waypoint.altitude = self.config.altitude
            segment.end_waypoint.altitude_mode = 1  # ALTITUDE_RELATIVE
            segment.end_waypoint.waypoint_id = i + 1
            
            segment.lane_width = self.config.spacing
            segment.overlap_percent = 60.0
            segment.status = 0  # STATUS_PENDING
            
            msg.lanes.append(segment)
        
        msg.total_lanes = len(msg.lanes)
        
        return msg
    
    def log_waypoints_to_file(self, waypoints: List[Tuple[float, float]], kml_path: str):
        """Log generated waypoints to ArduPilot .waypoints format."""
        try:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            kml_basename = os.path.splitext(os.path.basename(kml_path))[0]
            log_filename = f'{kml_basename}_waypoints_{timestamp}.waypoints'
            log_path = os.path.join(self.missions_folder, log_filename)
            
            home_point = (self.config.home_lat, self.config.home_lon)
            
            with open(log_path, 'w') as f:
                # Write header
                f.write("QGC WPL 110\n")
                
                waypoint_index = 0
                
                # Home point (command 16)
                f.write(f"{waypoint_index}\t0\t0\t16\t0.000000\t0.000000\t0.000000\t0.000000\t"
                       f"{home_point[0]:.6f}\t{home_point[1]:.6f}\t0.100000\t1\n")
                waypoint_index += 1
                
                # Takeoff command (command 22)
                f.write(f"{waypoint_index}\t0\t3\t22\t0.000000\t0.000000\t0.000000\t0.000000\t"
                       f"0.000000\t0.000000\t{self.config.altitude:.6f}\t1\n")
                waypoint_index += 1
                
                # Survey waypoints (command 16 - waypoint)
                for lat, lon in waypoints:
                    f.write(f"{waypoint_index}\t0\t3\t16\t0.000000\t0.000000\t0.000000\t0.000000\t"
                           f"{lat:.6f}\t{lon:.6f}\t{self.config.altitude:.6f}\t1\n")
                    waypoint_index += 1
                
                # RTL command (command 20)
                f.write(f"{waypoint_index}\t0\t0\t20\t0.000000\t0.000000\t0.000000\t0.000000\t"
                       f"0.000000\t0.000000\t0.000000\t1\n")
            
            self.get_logger().info(f'Waypoints logged to: {log_filename}')
            self.get_logger().info(f'Total mission items: {waypoint_index + 1} (1 home + 1 takeoff + {len(waypoints)} waypoints + 1 RTL)')
            
        except Exception as e:
            self.get_logger().error(f'Failed to log waypoints: {e}')


def main(args=None):
    """Main entry point."""
    # Auto-load default params file if no params-file argument provided
    if args is None:
        args = sys.argv
    
    # Check if user already specified a params file
    has_params_file = any('--params-file' in arg or '-p ' in arg for arg in args)
    
    if not has_params_file:
        # Load default config from package share directory
        try:
            pkg_share = get_package_share_directory('kml_lane_planner')
            default_params = os.path.join(pkg_share, 'config', 'planner_params.yaml')
            
            if os.path.exists(default_params):
                # Insert params-file argument
                args = list(args) + ['--ros-args', '--params-file', default_params]
        except Exception:
            pass  # Package not installed, use hardcoded defaults
    
    rclpy.init(args=args)
    
    node = KmlLanePlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
