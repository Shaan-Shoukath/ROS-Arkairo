#!/usr/bin/env python3
# Copyright 2024 Shaan Shoukath
# SPDX-License-Identifier: Apache-2.0

"""
GCS Forwarder Node

Event-driven geotag relay from Drone 1 to Drone 2.
Uses pymavlink to listen on one radio and forward to another.

Hardware Setup:
    /dev/ttyUSB0 → Drone 1 Radio (Pair A - Ground)
    /dev/ttyUSB1 → Drone 2 Radio (Pair B - Ground)

Only forwards:
    - STATUSTEXT messages starting with "GEOTAG:"
    - Origin SYSID = 1

Does NOT forward:
    - Monitoring telemetry
    - Drone 2 messages
    - Duplicates
"""

import time
import sys
from typing import Optional
from pymavlink import mavutil


class GCSForwarder:
    """Forward geotag events from Drone 1 to Drone 2."""
    
    GEOTAG_PREFIX = "GEOTAG:"
    
    def __init__(
        self,
        drone1_port: str = "/dev/ttyUSB0",
        drone2_port: str = "/dev/ttyUSB1",
        baud: int = 57600,
        source_sysid: int = 1
    ):
        self.source_sysid = source_sysid
        self.last_geotag = None
        self.forward_count = 0
        
        print("=" * 60)
        print("GCS Geotag Forwarder")
        print("=" * 60)
        print(f"Drone 1 port: {drone1_port}")
        print(f"Drone 2 port: {drone2_port}")
        print(f"Baud rate: {baud}")
        print(f"Expected source SYSID: {source_sysid}")
        print("=" * 60)
        
        # Connect to Drone 1 radio
        print(f"Connecting to Drone 1 radio on {drone1_port}...")
        try:
            self.drone1_conn = mavutil.mavlink_connection(
                drone1_port,
                baud=baud,
                source_system=255,  # GCS system ID
                source_component=0
            )
            print("  ✓ Connected")
        except Exception as e:
            print(f"  ✗ Failed: {e}")
            sys.exit(1)
        
        # Connect to Drone 2 radio
        print(f"Connecting to Drone 2 radio on {drone2_port}...")
        try:
            self.drone2_conn = mavutil.mavlink_connection(
                drone2_port,
                baud=baud,
                source_system=255,
                source_component=0
            )
            print("  ✓ Connected")
        except Exception as e:
            print(f"  ✗ Failed: {e}")
            sys.exit(1)
        
        print()
        print("Forwarder ready. Listening for geotags...")
        print("Press Ctrl+C to stop.")
        print()
    
    def run(self):
        """Main loop - listen and forward."""
        while True:
            try:
                # Non-blocking receive from Drone 1
                msg = self.drone1_conn.recv_match(blocking=True, timeout=1.0)
                
                if msg is None:
                    continue
                
                # Only process STATUSTEXT from SYSID 1
                if msg.get_type() != 'STATUSTEXT':
                    continue
                
                # Check source system ID
                if hasattr(msg, '_header') and hasattr(msg._header, 'srcSystem'):
                    if msg._header.srcSystem != self.source_sysid:
                        continue
                
                # Check for geotag prefix
                text = msg.text if hasattr(msg, 'text') else ""
                if not text.startswith(self.GEOTAG_PREFIX):
                    continue
                
                # Duplicate check
                if text == self.last_geotag:
                    print(f"[SKIP] Duplicate geotag ignored: {text}")
                    continue
                
                self.last_geotag = text
                
                # Forward to Drone 2
                self.forward_geotag(msg)
                
            except KeyboardInterrupt:
                print("\nShutting down...")
                break
            except Exception as e:
                print(f"[ERROR] {e}")
                time.sleep(1)
        
        self.shutdown()
    
    def forward_geotag(self, msg):
        """Forward geotag message to Drone 2."""
        try:
            # Get severity and text
            severity = msg.severity if hasattr(msg, 'severity') else 6
            text = msg.text
            
            # Send via Drone 2 connection
            self.drone2_conn.mav.statustext_send(
                severity,
                text.encode('utf-8')
            )
            
            self.forward_count += 1
            timestamp = time.strftime("%H:%M:%S")
            
            print(f"[{timestamp}] FORWARDED #{self.forward_count}: {text}")
            
            # Parse and log coordinates
            try:
                coords = text[len(self.GEOTAG_PREFIX):].split(',')
                lat = float(coords[0])
                lon = float(coords[1])
                alt = float(coords[2]) if len(coords) > 2 else 0
                print(f"           Lat: {lat:.6f}, Lon: {lon:.6f}, Alt: {alt:.1f}m")
            except:
                pass
            
        except Exception as e:
            print(f"[ERROR] Forward failed: {e}")
    
    def shutdown(self):
        """Clean shutdown."""
        print()
        print("=" * 60)
        print(f"Total geotags forwarded: {self.forward_count}")
        print("=" * 60)
        
        try:
            self.drone1_conn.close()
            self.drone2_conn.close()
        except:
            pass


def main():
    """Entry point."""
    import argparse
    
    parser = argparse.ArgumentParser(description='GCS Geotag Forwarder')
    parser.add_argument('--drone1', default='/dev/ttyUSB0', help='Drone 1 radio port')
    parser.add_argument('--drone2', default='/dev/ttyUSB1', help='Drone 2 radio port')
    parser.add_argument('--baud', type=int, default=57600, help='Baud rate')
    parser.add_argument('--sysid', type=int, default=1, help='Expected source SYSID')
    
    args = parser.parse_args()
    
    forwarder = GCSForwarder(
        drone1_port=args.drone1,
        drone2_port=args.drone2,
        baud=args.baud,
        source_sysid=args.sysid
    )
    
    forwarder.run()


if __name__ == '__main__':
    main()
