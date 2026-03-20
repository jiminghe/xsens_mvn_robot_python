#!/usr/bin/env python3

import sys
import time
import signal
import numpy as np
from collections import deque
from datetime import datetime

# Simple, clean import - only XsensWrapper
from xsens_mvn_robot import XsensWrapper

g_running = True

def signal_handler(signum, frame):
    global g_running
    print(f"\nReceived signal {signum}, shutting down gracefully...")
    g_running = False

class MotionDataCollector:
    def __init__(self, port=8001):
        """Initialize with XsensWrapper only"""
        self.device = XsensWrapper(port)
        print(f"MotionDataCollector initialized with port {port}")
        
        self.sample_history = deque(maxlen=256)
        self.last_sample_counter = -1
        self.prev_counter = -1

        self.total_received = 0
        self.total_lost = 0
        self.interval_received = 0
        self.interval_lost = 0
        self.last_stats_time = time.perf_counter()

        self.joint_names = []
        self.link_names = []

    def initialize(self):
        """Initialize the Xsens device"""
        print("Initializing Xsens MVN Device...")
        if not self.device.init():
            print("❌ Failed to initialize MVN device")
            return False
            
        # Get joint and link names using wrapper
        self.joint_names = self.device.get_joint_names()
        self.link_names = self.device.get_link_names()
        print(f"Channels: {len(self.joint_names)} joints, {len(self.link_names)} links")
        
        # DEBUG: Print all joint names to see what we actually have
        print("\n🔍 DEBUG - All Joint Names:")
        for i, name in enumerate(self.joint_names):
            print(f"  [{i:2d}] {name}")
            if i >= 70:  # Limit output to avoid spam
                print(f"  ... and {len(self.joint_names) - i - 1} more")
                break
        
        # DEBUG: Print all link names to see what we actually have  
        print(f"\n🔍 DEBUG - All Link Names:")
        for i, name in enumerate(self.link_names):
            print(f"  [{i:2d}] {name}")
            if i >= 70:  # Limit output to avoid spam
                print(f"  ... and {len(self.link_names) - i - 1} more")
                break
        
        # DEBUG: Look for finger joints specifically
        finger_joints = [name for name in self.joint_names if 'finger' in name.lower() or 'metacarpal' in name.lower() or 'proximal' in name.lower() or 'distal' in name.lower() or 'carpus' in name.lower()]
        finger_links = [name for name in self.link_names if 'finger' in name.lower() or 'metacarpal' in name.lower() or 'proximal' in name.lower() or 'distal' in name.lower() or 'carpus' in name.lower()]
        
        print(f"\n🔍 DEBUG - Found {len(finger_joints)} finger joints:")
        for name in finger_joints:
            print(f"  - {name}")
        
        print(f"\n🔍 DEBUG - Found {len(finger_links)} finger links:")
        for name in finger_links:
            print(f"  - {name}")
        
        return True

    def start_streaming(self):
        """Start data streaming"""
        print("🚀 Starting streaming...")
        self.device.start()

    def stop_streaming(self):
        """Stop data streaming"""
        print("🛑 Stopping streaming...")
        self.device.stop()

    def collect(self):
        """Collect basic motion data for high-frequency processing"""
        ts_ns = time.perf_counter_ns()
        
        # Get sample counter and frame time using wrapper
        cnt = int(self.device.get_sample_counter())
        ft = int(self.device.get_frame_time())

        # Calculate data loss statistics
        if self.prev_counter >= 0:
            delta = cnt - self.prev_counter
            if delta > 0:
                lost = delta - 1
                self.interval_received += 1
                self.total_received += 1
                self.interval_lost += lost
                self.total_lost += lost
        self.prev_counter = cnt

        # Update sample history for FPS calculation
        if cnt > 0 and cnt != self.last_sample_counter:
            self.sample_history.append((cnt, ts_ns))
            self.last_sample_counter = cnt

        # Collect center of mass only if CoM datagram (0x24) is being streamed
        com = None
        try:
            if 0x24 in self.device.get_received_datagram_types():
                com = self.device.get_center_of_mass_position()
        except Exception:
            pass

        return {'ts': ts_ns, 'cnt': cnt, 'ft': ft, 'com': com}

    def collect_detailed(self):
        """Collect detailed joint and link data (slower, for periodic use)"""
        joints = []
        
        # Collect joint angles for first 3 and last 3 joints - maintain original order
        joint_indices_to_collect = []
        if len(self.joint_names) > 0:
            # First 3 joints
            joint_indices_to_collect.extend(range(min(3, len(self.joint_names))))
            # Last 3 joints (if different from first 3)
            if len(self.joint_names) > 3:
                last_start = max(3, len(self.joint_names) - 3)
                joint_indices_to_collect.extend(range(last_start, len(self.joint_names)))
        
        for idx in joint_indices_to_collect:
            name = self.joint_names[idx]
            try:
                ang = self.device.get_joint_angles(name)
                joints.append((idx, name, ang))
            except Exception as e:
                # Skip failed joints to maintain performance
                pass

        links = []
        
        # Collect link positions and orientations for first 3 and last 3 links - maintain original order
        link_indices_to_collect = []
        if len(self.link_names) > 0:
            # First 3 links
            link_indices_to_collect.extend(range(min(3, len(self.link_names))))
            # Last 3 links (if different from first 3)
            if len(self.link_names) > 3:
                last_start = max(3, len(self.link_names) - 3)
                link_indices_to_collect.extend(range(last_start, len(self.link_names)))
        
        for idx in link_indices_to_collect:
            name = self.link_names[idx]
            try:
                pos = self.device.get_link_position(name)
                ori = self.device.get_link_orientation(name)
                links.append((idx, name, pos, ori))
            except Exception as e:
                # Skip failed links to maintain performance
                pass

        return {'joints': joints, 'links': links}

    def calc_fps(self):
        """Calculate FPS using average interval method for better accuracy"""
        if len(self.sample_history) < 2:
            return 0.0
            
        intervals = [
            (self.sample_history[i][1] - self.sample_history[i - 1][1]) * 1e-9
            for i in range(1, len(self.sample_history))
        ]
        avg_interval = sum(intervals) / len(intervals)
        return 1.0 / avg_interval if avg_interval > 0 else 0.0

    def display(self, data, detailed_data=None):
        """Display real-time motion data"""
        wall = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        fps = self.calc_fps()

        # Live status line (non-blocking, updates in place)
        print(f"\r⏰ {wall} | 🔢 Sample: {data['cnt']} | 📊 FPS: {fps:.1f} Hz | ⚡ Frame: {data['ft']} ms", end="")

        # Show detailed stats once per second
        now = time.perf_counter()
        if now - self.last_stats_time >= 1.0:
            print("\n" + "-"*70)
            
            # Display center of mass
            if data['com'] is not None:
                com = data['com']
                print(f"🎯 Center of Mass: [{com[0]:+7.3f}, {com[1]:+7.3f}, {com[2]:+7.3f}]")

            # Show detailed joint and link data if available
            if detailed_data:
                if detailed_data.get('joints'):
                    print("🦴 Joints (First 3 + Last 3):")
                    
                    # No sorting - just display in the order they were collected
                    joints = detailed_data['joints']
                    first_3_count = 0
                    last_3_start = -1
                    
                    for idx, name, angles in joints:
                        if idx < 3:
                            first_3_count += 1
                        elif last_3_start == -1:
                            last_3_start = len([j for j in joints if j[0] < 3])
                    
                    # Display first 3
                    displayed_first = 0
                    for idx, name, angles in joints:
                        if idx < 3:
                            print(f"  [{idx:2d}] {name:18s}: [{angles[0]:+7.2f}, {angles[1]:+7.2f}, {angles[2]:+7.2f}]")
                            displayed_first += 1
                    
                    # Add separator if we have both first 3 and last 3
                    has_last_3 = any(j[0] >= max(3, len(self.joint_names) - 3) for j in joints)
                    if displayed_first > 0 and has_last_3:
                        print("  " + "."*50)
                    
                    # Display last 3
                    for idx, name, angles in joints:
                        if idx >= max(3, len(self.joint_names) - 3):
                            print(f"  [{idx:2d}] {name:18s}: [{angles[0]:+7.2f}, {angles[1]:+7.2f}, {angles[2]:+7.2f}]")
                        
                if detailed_data.get('links'):
                    print("🔗 Links (First 3 + Last 3):")
                    
                    # No sorting - just display in the order they were collected
                    links = detailed_data['links']
                    
                    # Display first 3
                    displayed_first = 0
                    for idx, name, pos, ori in links:
                        if idx < 3:
                            print(f"  [{idx:2d}] {name:18s}: [{pos[0]:+7.3f}, {pos[1]:+7.3f}, {pos[2]:+7.3f}]")
                            displayed_first += 1
                    
                    # Add separator if we have both first 3 and last 3
                    has_last_3 = any(l[0] >= max(3, len(self.link_names) - 3) for l in links)
                    if displayed_first > 0 and has_last_3:
                        print("  " + "."*50)
                    
                    # Display last 3
                    for idx, name, pos, ori in links:
                        if idx >= max(3, len(self.link_names) - 3):
                            print(f"  [{idx:2d}] {name:18s}: [{pos[0]:+7.3f}, {pos[1]:+7.3f}, {pos[2]:+7.3f}]")

            # Display data loss statistics
            interval_total = self.interval_received + self.interval_lost
            total_total = self.total_received + self.total_lost
            
            if interval_total > 0:
                loss_pct = 100.0 * self.interval_lost / interval_total
                recv_pct = 100.0 * self.interval_received / interval_total
                print(f"\n📦 Data Loss (this period): {self.interval_lost}/{interval_total} frames ({loss_pct:.2f}%)")
                print(f"✅ Received (this period): {self.interval_received}/{interval_total} frames ({recv_pct:.2f}%)")
                
            if total_total > 0:
                total_loss_pct = 100.0 * self.total_lost / total_total
                total_recv_pct = 100.0 * self.total_received / total_total
                print(f"📦 Total Loss: {self.total_lost}/{total_total} frames ({total_loss_pct:.2f}%)")
                print(f"✅ Total Received: {self.total_received}/{total_total} frames ({total_recv_pct:.2f}%)")
                
            print(f"📈 Real FPS: {fps:.2f} Hz")
            print("-"*70)

            # Reset interval counters
            self.interval_received = 0
            self.interval_lost = 0
            self.last_stats_time = now

def main():
    """Main function"""
    global g_running
    
    # Set up signal handlers for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Parse command line arguments for port (optional)
    port = 9763  # Default port
    if len(sys.argv) > 1:
        try:
            port = int(sys.argv[1])
            print(f"Using port {port} from command line")
        except ValueError:
            print(f"Invalid port '{sys.argv[1]}', using default {port}")
    
    # Initialize motion data collector
    collector = MotionDataCollector(port)
    
    # Initialize device
    if not collector.initialize():
        print("❌ Failed to initialize device")
        sys.exit(1)
    
    # Start streaming
    collector.start_streaming()
    time.sleep(1)  # Allow device to start streaming
    print("\n🎬 Starting real-time display...")
    
    iteration_count = 0
    detailed_data = None
    
    try:
        while g_running:
            # Always collect basic data for high FPS
            d = collector.collect()
            
            # Collect detailed joint/link data every 30 iterations (~1Hz if running at 30 FPS)
            # This reduces computational load while still showing joint/link info periodically
            if iteration_count % 30 == 0:
                try:
                    detailed_data = collector.collect_detailed()
                except Exception as e:
                    # Handle errors gracefully to maintain streaming
                    detailed_data = None
            
            # Display the data
            collector.display(d, detailed_data)
            
            iteration_count += 1
            
            # Minimal sleep to yield control but maintain high FPS
            time.sleep(0)
            
    except Exception as e:
        print(f"\n❌ Error during streaming: {e}")
    finally:
        # Ensure streaming is stopped
        collector.stop_streaming()
        print("\n✅ Demo stopped gracefully")

if __name__ == '__main__':
    main()
