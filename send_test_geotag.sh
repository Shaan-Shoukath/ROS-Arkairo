#!/bin/bash
# Helper script to send test geotags from SOE.kml

case \$1 in
    1)
        LAT=10.04838542
        LON=76.33096854
        echo \"📍 Sending geotag #1: (\$LAT, \$LON)\"
        ;;
    2)
        LAT=10.04772939
        LON=76.33159876
        echo \"📍 Sending geotag #2: (\$LAT, \$LON)\"
        ;;
    3)
        LAT=10.04857533
        LON=76.33210639
        echo \"📍 Sending geotag #3: (\$LAT, \$LON)\"
        ;;
    4)
        LAT=10.04880792
        LON=76.33119056
        echo \"📍 Sending geotag #4: (\$LAT, \$LON)\"
        ;;
    *)
        echo \"Usage: ./send_test_geotag.sh [1-4]\"
        echo \"  1: Point 1 (10.04838542, 76.33096854)\"
        echo \"  2: Point 2 (10.04772939, 76.33159876)\"
        echo \"  3: Point 3 (10.04857533, 76.33210639)\"
        echo \"  4: Point 4 (10.04880792, 76.33119056)\"
        exit 1
        ;;
esac

ros2 topic pub /drone2/target_geotag geographic_msgs/msg/GeoPointStamped \
  "{header: {stamp: now, frame_id: 'map'}, 
    position: {latitude: \$LAT, longitude: \$LON, altitude: 10.0}}" \
  --once

echo "✅ Geotag sent! Watch /drone2/status for state changes."
