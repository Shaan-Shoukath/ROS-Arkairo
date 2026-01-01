# KML Lane Planner Node - Developer Documentation

## Overview

**File**: `drone1_ws/src/kml_lane_planner/kml_lane_planner/kml_lane_planner_node.py`  
**Package**: `kml_lane_planner`  
**Node Name**: `kml_lane_planner_node`  
**Purpose**: Converts KML polygon files into optimized survey waypoints

## What This Node Does

The KML Lane Planner is the mission planning brain for survey operations. It:

1. **Watches the missions folder** for new KML files
2. **Parses KML polygon coordinates** from Google Earth exports
3. **Generates parallel survey lanes** (lawnmower pattern)
4. **Optimizes waypoint ordering** to minimize flight distance
5. **Publishes lane segments** to the navigation node
6. **Handles corner detection** for complex polygon shapes

## Core Logic & Reasoning

### File Watching System

```python
# Continuously monitors missions folder
for kml_file in new_kml_files:
    parse_and_convert()
    publish_mission()
```

**Design Decision**: Uses file polling instead of OS events for cross-platform compatibility.

### Survey Pattern Generation Algorithm

**Step 1: Find Longest Edge**

```python
longest_edge = find_longest_polygon_edge(boundary_points)
flight_direction = perpendicular_to(longest_edge)
```

**Reasoning**: Flying perpendicular to longest edge minimizes turns and coverage time.

**Step 2: Apply Buffer Distance**

```python
buffered_polygon = shrink_polygon_inward(original_polygon, buffer_distance)
```

**Purpose**: Keeps drone inside field boundaries for safety.

**Step 3: Generate Parallel Lines**

```python
for offset in range(0, total_width, line_spacing):
    line = create_parallel_line(flight_direction, offset)
    intersections = clip_to_polygon(line, buffered_polygon)
    lanes.append(intersections)
```

**Step 4: Optimize Start Point**

```python
# Start from lane endpoint closest to home
closest_lane_end = find_closest_point(home_position, all_lane_endpoints)
reorder_lanes_starting_from(closest_lane_end)
```

**Benefit**: Reduces initial flight distance and battery usage.

### Coordinate Systems

**Input**: WGS84 geodetic (latitude, longitude)  
**Processing**: ECEF Cartesian (X, Y, Z) for geometric operations  
**Output**: WGS84 geodetic for ArduPilot compatibility

**Why ECEF?** Accurate distance/angle calculations without projection distortion.

## Subscribers

### 1. `/mavros/global_position/global` (sensor_msgs/NavSatFix)

- **Source**: MAVROS
- **Purpose**: Captures home position for mission optimization
- **Timing**: Read once when mission loaded
- **Usage**: Determines optimal starting lane

## Publishers

### 1. `/mission/lane_segments` (drone1_msgs/LaneSegmentArray)

- **Trigger**: When new KML processed
- **Purpose**: Sends complete mission to navigation node
- **QoS**: RELIABLE + TRANSIENT_LOCAL (guaranteed delivery)
- **Structure**:
  ```
  LaneSegmentArray
  ├── LaneSegment[0]  (First lane)
  │   ├── Waypoint[0]
  │   ├── Waypoint[1]
  │   └── ...
  ├── LaneSegment[1]  (Second lane)
  └── ...
  ```

### 2. `/drone1/next_waypoint` (sensor_msgs/NavSatFix)

- **Purpose**: Debugging/monitoring current target
- **Rate**: On mission load

## Parameters

| Parameter            | Default       | Description                              |
| -------------------- | ------------- | ---------------------------------------- |
| `missions_folder`    | "../missions" | Path to watch for KML files              |
| `default_altitude`   | 50.0          | Survey altitude (meters AGL)             |
| `default_spacing`    | 5.0           | Distance between survey lines (meters)   |
| `buffer_distance`    | 2.0           | Safety margin from polygon edge (meters) |
| `watch_interval_sec` | 5.0           | How often to check for new files         |

## Key Functions Explained

### `parse_kml_file()` - KML Parser

```python
def parse_kml_file(filepath: str) -> List[Tuple[float, float]]
```

**Purpose**: Extracts polygon coordinates from KML XML  
**Built-in Functions**:

- `xml.etree.ElementTree.parse()` - Parses XML file
- `element.find()` - Searches XML tree
- `element.text.strip()` - Extracts text content
- `re.findall()` - Regex pattern matching

**Logic**:

1. Parse XML structure
2. Find `<coordinates>` tags
3. Split comma-separated coordinate triplets
4. Convert strings to floats
5. Return as list of (lat, lon) tuples

### `latlon_to_ecef()` - Coordinate Conversion

```python
def latlon_to_ecef(lat: float, lon: float, alt: float) -> Tuple[float, float, float]
```

**Purpose**: Converts GPS coordinates to Earth-Centered Earth-Fixed (ECEF) Cartesian  
**Formula**: WGS84 ellipsoid equations  
**Built-in Functions**:

- `math.radians()` - Degree to radian conversion
- `math.sin()`, `math.cos()` - Trigonometry
- `math.sqrt()` - Square root for ellipsoid calculations

**Math Explanation**:

```
N = a / sqrt(1 - e² * sin²(lat))    # Radius of curvature
X = (N + h) * cos(lat) * cos(lon)   # ECEF X
Y = (N + h) * cos(lat) * sin(lon)   # ECEF Y
Z = (N * (1-e²) + h) * sin(lat)     # ECEF Z
```

### `ecef_to_latlon()` - Inverse Conversion

```python
def ecef_to_latlon(x: float, y: float, z: float) -> Tuple[float, float, float]
```

**Purpose**: Converts ECEF back to GPS coordinates  
**Method**: Iterative approximation (converges in 3-5 iterations)  
**Built-in Functions**:

- `math.atan2()` - Two-argument arctangent (handles all quadrants)
- `math.degrees()` - Radian to degree conversion

### `calculate_distance()` - Haversine Distance

```python
def calculate_distance(point1: Tuple[float, float], point2: Tuple[float, float]) -> float
```

**Purpose**: Great-circle distance between GPS points  
**Returns**: Distance in meters  
**Formula**: Haversine formula for spherical Earth  
**Earth Radius**: 6,371,000 meters

### `generate_survey_lanes()` - Mission Generation

```python
def generate_survey_lanes(polygon: List, config: MissionConfig) -> List[LaneSegment]
```

**Purpose**: Core algorithm that creates survey pattern  
**Steps**:

1. Convert polygon to ECEF
2. Find longest edge and flight direction
3. Generate parallel lines at spacing intervals
4. Clip lines to polygon boundary
5. Optimize lane ordering
6. Convert back to GPS coordinates
7. Package into LaneSegment messages

### `find_closest_endpoint()` - Start Point Optimization

```python
def find_closest_endpoint(home: Tuple[float, float], lanes: List) -> int
```

**Purpose**: Determines optimal starting lane  
**Logic**: Compares distance from home to all lane endpoints  
**Benefit**: Minimizes initial transit distance

## Package Dependencies

### ROS2 Packages

- **rclpy**: Python client library
  - `Node` - Base node class
  - `QoSProfile` - Message delivery guarantees

### Message Types

- **std_msgs**: `String`, `Header`
- **sensor_msgs**: `NavSatFix` (GPS data)
- **drone1_msgs**: Custom message types
  - `Waypoint` - Single GPS waypoint
  - `LaneSegment` - Array of waypoints forming one lane
  - `LaneSegmentArray` - Complete mission of all lanes

### Python Standard Library

- **xml.etree.ElementTree** (`ET`): XML parsing for KML files
  - `ET.parse()` - Loads XML file
  - `ET.Element` - XML element manipulation
- **re**: Regular expressions for coordinate extraction
  - `re.findall()` - Pattern matching
- **glob**: File pattern matching
  - `glob.glob()` - Finds files matching pattern
- **os**: File system operations
  - `os.path.exists()` - Check file existence
  - `os.path.join()` - Construct file paths
- **math**: Mathematical operations
  - `math.sin()`, `math.cos()`, `math.atan2()` - Trigonometry
  - `math.radians()`, `math.degrees()` - Angle conversion
  - `math.sqrt()` - Square root
- **typing**: Type hints (`List`, `Tuple`, `Optional`, `Dict`)
- **dataclasses**: `@dataclass` decorator for data structures
- **datetime**: Timestamp generation

## KML File Format Expected

```xml
<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <Placemark>
      <Polygon>
        <outerBoundaryIs>
          <LinearRing>
            <coordinates>
              76.3303,10.0478,0
              76.3313,10.0478,0
              76.3313,10.0488,0
              76.3303,10.0488,0
              76.3303,10.0478,0
            </coordinates>
          </LinearRing>
        </outerBoundaryIs>
      </Polygon>
    </Placemark>
  </Document>
</kml>
```

**Format**: `longitude,latitude,altitude` (Note: KML uses lon,lat order!)

## Mission Optimization Logic

### Why Parallel Lines?

**Efficiency**: Lawnmower pattern provides complete coverage with minimal turns.

### Why Perpendicular to Longest Edge?

**Math Proof**: For rectangle ABCD with length L and width W (L > W):

- Perpendicular: Number of lines = W/spacing, Total turns = W/spacing
- Parallel: Number of lines = L/spacing, Total turns = L/spacing
- Since L > W: Perpendicular requires fewer lines = fewer turns = less time

### Buffer Distance Purpose

1. **Safety**: Prevents drone from leaving field boundaries
2. **Wind compensation**: Allows for drift without boundary violation
3. **Turn radius**: Provides space for banking into turns

## Configuration Files

Node expects a YAML parameter file:

```yaml
kml_lane_planner:
  ros__parameters:
    missions_folder: "../missions"
    default_altitude: 50.0
    default_spacing: 5.0
    buffer_distance: 2.0
    watch_interval_sec: 5.0
```

## Workflow Example

**User Action**: Drops `farm_field.kml` into `drone1_ws/missions/`

**Node Processing**:

1. Detects new file after ≤5 seconds
2. Parses polygon coordinates
3. Converts to ECEF for processing
4. Generates 10 parallel lanes (50m field ÷ 5m spacing)
5. Optimizes starting from closest lane to home
6. Converts back to GPS coordinates
7. Publishes `LaneSegmentArray` with ~200 total waypoints
8. Navigation node receives mission and arms for takeoff

## Error Handling

**Invalid KML**: Logs error and continues watching  
**Empty polygon**: Skips file  
**Parsing failure**: Catches exception and reports details  
**No home position**: Uses first waypoint as reference

## Testing Checklist

- [ ] Node starts and monitors missions folder
- [ ] Detects new KML files within watch interval
- [ ] Successfully parses Google Earth exported KML
- [ ] Generates parallel survey lanes
- [ ] Applies buffer distance correctly
- [ ] Optimizes start point based on home position
- [ ] Publishes complete mission to navigation
- [ ] Handles malformed KML gracefully

## Common Issues & Solutions

**Issue**: Lanes too close together  
**Solution**: Increase `default_spacing` parameter

**Issue**: Drone flies outside field boundary  
**Solution**: Increase `buffer_distance` parameter

**Issue**: Mission not detected  
**Solution**: Check `missions_folder` path is correct

**Issue**: Wrong coordinate order  
**Solution**: KML uses lon,lat format (reversed from typical lat,lon)

## SITL Simulation Testing

### Quick Start

```bash
# Terminal 1: Start ArduPilot SITL
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter --console --map -l 10.0478,76.3303,0,0 -w

# Terminal 2: MAVROS (provides GPS for home position)
ros2 launch mavros apm.launch.py fcu_url:=udp://:14550@127.0.0.1:14555

# Terminal 3: Run planner
cd ~/Documents/ROSArkairo/drone1_ws && source install/setup.zsh
ros2 run kml_lane_planner kml_lane_planner_node
```

### Test KML File

Place a test KML file in the missions folder. Example coordinates for testing near SITL home:

```xml
<coordinates>
  76.3303,10.0478,0
  76.3320,10.0478,0
  76.3320,10.0495,0
  76.3303,10.0495,0
  76.3303,10.0478,0
</coordinates>
```

### Monitor Output

```bash
ros2 topic echo /mission/lane_segments
```

---

**Maintained by**: Shaan Shoukath  
**Last Updated**: December 31, 2025
