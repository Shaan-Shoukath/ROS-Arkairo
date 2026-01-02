# Telemetry TX Node - Developer Documentation

## Overview

**File**: `drone1_ws/src/telem_tx/telem_tx/telem_tx_node.py`  
**Package**: `telem_tx`  
**Node Name**: `telem_tx_node`  
**Purpose**: Transmits disease geotags to Drone-2 via MAVLink telemetry radio

## What This Node Does

Simple but critical telemetry bridge:

1. **Receives disease geotags** from detection node
2. **Encodes GPS coordinates** into MAVLink NAMED_VALUE_FLOAT messages
3. **Transmits over radio** via MAVROS debug_value topic
4. **Tracks transmission statistics**

## Core Logic

```python
Geotag arrives → Split into 3 messages (lat, lon, alt) → Send via MAVLink
```

**Why 3 messages?** MAVLink NAMED_VALUE_FLOAT carries single float + name. GPS needs 3 coordinates.

## MAVLink Protocol

### Message Format: NAMED_VALUE_FLOAT

```
Message ID: 251
Fields:
  - time_boot_ms: System uptime (milliseconds)
  - name: String identifier (max 10 chars)
  - value: Float data
```

**Our Encoding**:

- Message 1: `name='d_lat'`, `value=latitude`
- Message 2: `name='d_lon'`, `value=longitude`
- Message 3: `name='d_alt'`, `value=altitude`

**Prefix `d_`**: Distinguishes disease coordinates from other debug values

## Message Flow

```
Drone-1 Detection
      ↓
/drone1/disease_geotag (GeoPointStamped)
      ↓
telem_tx_node
      ↓
/mavros/debug_value/send (DebugValue × 3)
      ↓
MAVROS
      ↓
MAVLink (NAMED_VALUE_FLOAT × 3)
      ↓
Telemetry Radio (SiK/RFD900)
      ↓
Drone-2 Telemetry Radio
      ↓
MAVROS
      ↓
/mavros/debug_value/debug_vector (DebugValue)
      ↓
telem_rx_node (Drone-2)
```

## Subscribers

### `/drone1/disease_geotag` (geographic_msgs/GeoPointStamped)

- **Source**: Detection & Geotag Node
- **QoS**: RELIABLE + VOLATILE
- **Purpose**: GPS coordinates of disease detections
- **Trigger**: Immediately encodes and transmits

## Publishers

### `/mavros/debug_value/send` (mavros_msgs/DebugValue)

- **Rate**: On-demand (per geotag received)
- **Purpose**: MAVLink transmission interface
- **QoS**: Default (best effort)
- **Message Count**: 3 per geotag (lat, lon, alt)

## Key Functions

### `geotag_callback()` - Main Handler

```python
def geotag_callback(self, msg: GeoPointStamped):
    lat = msg.position.latitude
    lon = msg.position.longitude
    alt = msg.position.altitude

    self.send_mavlink_value('d_lat', lat)
    self.send_mavlink_value('d_lon', lon)
    self.send_mavlink_value('d_alt', alt)

    self.tx_count += 1
```

### `send_mavlink_value()` - Message Construction

```python
def send_mavlink_value(self, name: str, value: float):
    msg = DebugValue()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.header.frame_id = 'map'
    msg.index = -1        # Not using indexed array
    msg.array_id = -1     # Not using array
    msg.name = name       # 'd_lat', 'd_lon', or 'd_alt'
    msg.value_float = float(value)
    msg.type = DebugValue.TYPE_NAMED_VALUE_FLOAT
    self.mavlink_pub.publish(msg)
```

**Built-in Functions**:

- `float()` - Type conversion to ensure float type
- `self.get_clock().now().to_msg()` - ROS timestamp

## Package Dependencies

### ROS2 Packages

- **rclpy**: Node, QoS
- **geographic_msgs**: GeoPointStamped, GeoPoint
- **mavros_msgs**: DebugValue
  - Constants: `TYPE_NAMED_VALUE_FLOAT`

### Python Standard Library

- None required (minimal node)

## Message Types Explained

### geographic_msgs/GeoPointStamped

```python
Header header
  uint32 seq
  time stamp
  string frame_id
GeoPoint position
  float64 latitude   # Degrees
  float64 longitude  # Degrees
  float64 altitude   # Meters
```

### mavros_msgs/DebugValue

```python
Header header
int32 index       # Array index (-1 for none)
int32 array_id    # Array ID (-1 for none)
string name       # Value identifier (max 10 chars)
uint8 type        # Message type constant
float32 value_float   # Float payload
int32 value_int       # Int payload (not used)
uint64 data           # Raw data (not used)
```

**Type Constants**:

- `TYPE_DEBUG = 0`
- `TYPE_INDEX_ONLY = 1`
- `TYPE_NAMED_VALUE_FLOAT = 2` ← **We use this**
- `TYPE_NAMED_VALUE_INT = 3`

## Telemetry Radio Configuration

**Required Setup** (on both Drone-1 and Drone-2):

1. **Radio Parameters**:

   - `SERIALx_PROTOCOL = 2` (MAVLink2)
   - `SERIALx_BAUD = 57` (57600 baud)
   - Where x = radio serial port (e.g., SERIAL2)

2. **MAVLink Streams**:

   - `SR2_EXTRA1 = 2` (Enable debug messages)
   - `SR2_POSITION = 2` (Enable position)

3. **System IDs**:
   - Drone-1: `SYSID_THISMAV = 1`
   - Drone-2: `SYSID_THISMAV = 2`

## Transmission Statistics

```python
self.tx_count = 0  # Incremented per geotag sent
```

**Monitoring**:

```bash
ros2 topic echo /mavros/debug_value/send
```

**Expected Output** (per detection):

```
header:
  stamp: {sec: 123, nanosec: 456789}
name: "d_lat"
value_float: 10.0478
---
header:
  stamp: {sec: 123, nanosec: 456790}
name: "d_lon"
value_float: 76.3303
---
header:
  stamp: {sec: 123, nanosec: 456791}
name: "d_alt"
value_float: 50.0
```

## Error Handling

**Geotag received but not transmitted?**

- Check MAVROS connection: `ros2 topic list | grep mavros`
- Verify radio connection: Check `RADIO.rssi` parameter
- Monitor transmission: `ros2 topic echo /mavros/debug_value/send`

**Transmission delayed?**

- Radio bandwidth limited (~60Hz for full stream)
- Debug values have lower priority than position/attitude
- Normal delay: <100ms per message

## Testing Checklist

- [ ] Node starts without errors
- [ ] Subscribes to `/drone1/disease_geotag`
- [ ] Publishes to `/mavros/debug_value/send`
- [ ] Encodes 3 messages per geotag (lat, lon, alt)
- [ ] Message names correct ('d_lat', 'd_lon', 'd_alt')
- [ ] Values match input geotag coordinates
- [ ] Transmission count increments

## Testing Without Real Hardware

**Simulate Geotag Transmission**:

```bash
# Terminal 1: Start node
ros2 run telem_tx telem_tx_node

# Terminal 2: Send test geotag
ros2 topic pub --once /drone1/disease_geotag geographic_msgs/msg/GeoPointStamped \
  "{header: {frame_id: 'map'}, position: {latitude: 10.0478, longitude: 76.3303, altitude: 50.0}}"

# Terminal 3: Monitor transmission
ros2 topic echo /mavros/debug_value/send
```

**Expected**: 3 messages published with names 'd_lat', 'd_lon', 'd_alt'

## Radio Range Considerations

**Typical Ranges**:

- **SiK Radio (100mW)**: ~500m ground, ~1km air
- **RFD900 (1W)**: ~5km ground, ~15km air

**Link Budget**:

- TX power: 100mW to 1W
- Antenna gain: 2-3 dBi (dipole)
- Sensitivity: -117 dBm typical
- Data rate: 57600 baud (default)

**Message Overhead**:

- Each NAMED_VALUE_FLOAT: 18 bytes
- 3 messages per detection: 54 bytes
- At 57600 baud: ~6ms transmission time

## Common Issues

**Issue**: Geotags not reaching Drone-2  
**Solution**: Check radio RSSI, verify system IDs, ensure MAVLink streams enabled

**Issue**: Messages delayed or out of order  
**Solution**: Normal MAVLink behavior, receiver must handle assembly

**Issue**: Only receiving some values  
**Solution**: Radio packet loss, improve link quality or reduce detection rate

## Integration with Drone-2

**Receiver Side** (telem_rx_node on Drone-2):

```python
# Receives: /mavros/debug_value/debug_vector
# Looks for names: 'd_lat', 'd_lon', 'd_alt'
# Assembles: Complete GPS coordinate
# Publishes: /drone2/target_position
```

**Handshake**: None required, fire-and-forget transmission

## SITL Simulation Testing

### Testing Without Radio Hardware

You can test the node without physical telemetry radios:

```bash
# Terminal 1: Run the node (no MAVROS needed for basic test)
ros2 run telem_tx telem_tx_node

# Terminal 2: Send test geotag
ros2 topic pub --once /drone1/disease_geotag geographic_msgs/msg/GeoPointStamped \
  "{header: {frame_id: 'map'}, position: {latitude: 10.0478, longitude: 76.3303, altitude: 50.0}}"

# Terminal 3: Monitor outgoing messages
ros2 topic echo /mavros/debug_value/send
```

### Full SITL Test (with MAVROS)

```bash
# Terminal 1: ArduPilot SITL
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter --console --map -l 10.0478,76.3303,0,0

# Terminal 2: MAVROS
ros2 launch mavros apm.launch.py fcu_url:=udp://:14550@127.0.0.1:14555

# Terminal 3: Telem TX node
ros2 run telem_tx telem_tx_node

# Terminal 4: Simulate detections
ros2 topic pub /drone1/disease_geotag geographic_msgs/msg/GeoPointStamped \
  "{position: {latitude: 10.048, longitude: 76.331, altitude: 50.0}}" --once
```

---

**Last Updated**: December 31, 2025  
**Maintainer**: Shaan Shoukath
