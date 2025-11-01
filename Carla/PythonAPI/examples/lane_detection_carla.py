#!/usr/bin/env python3
"""
Lane Detection System for CARLA Simulator
Based on IGVC OBJ.py algorithm but adapted for CARLA PythonAPI

This script:
1. Connects to CARLA simulator
2. Spawns a vehicle with a camera sensor
3. Implements lane detection using color masking and peak detection
4. Visualizes the detected lane and steering angle
5. Can be used for autonomous lane keeping
"""

import sys
import os
import glob
import queue
import cv2
import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.signal import find_peaks
import argparse
import time

# Add CARLA to path - Try multiple methods
try:
    # Method 1: Try to import directly (if wheel is installed)
    import carla
except ImportError:
    try:
        # Method 2: Try egg file from dist folder
        sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
            sys.version_info.major,
            sys.version_info.minor,
            'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
        import carla
    except (IndexError, ImportError):
        print("\n" + "="*70)
        print("ERROR: Cannot import CARLA module!")
        print("="*70)
        print("\nPossible solutions:")
        print("1. Install Python 3.12 (CARLA 0.9.16 requires Python 3.12)")
        print("2. Install the CARLA wheel file with Python 3.12:")
        print("   pip install d:\\Carla\\CARLA_0.9.16\\PythonAPI\\carla\\dist\\carla-0.9.16-cp312-cp312-win_amd64.whl")
        print("\nCurrent Python version:", sys.version)
        print("Required: Python 3.12")
        print("="*70)
        sys.exit(1)


class CameraManager:
    """Manages CARLA camera sensor and image queue"""
    
    def __init__(self, vehicle, world, width=800, height=600):
        """
        Initialize camera sensor attached to vehicle
        
        Args:
            vehicle: CARLA vehicle actor
            world: CARLA world
            width: Camera width
            height: Camera height
        """
        self.vehicle = vehicle
        self.world = world
        self.image_queue = queue.Queue(maxsize=10)
        self.image = None
        
        # Camera sensor blueprint
        camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(width))
        camera_bp.set_attribute('image_size_y', str(height))
        camera_bp.set_attribute('fov', '90')
        
        # Spawn camera on vehicle (hood position)
        camera_transform = carla.Transform(carla.Location(x=1.5, z=1.2))
        self.sensor = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
        
        # Register callback
        self.sensor.listen(self.camera_callback)
        
        print(f"Camera sensor spawned: {width}x{height}")
    
    def camera_callback(self, image):
        """Callback for camera sensor data"""
        # Convert CARLA image to numpy array
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]  # Remove alpha channel
        array = array[:, :, ::-1]  # Convert RGB to BGR for OpenCV
        
        self.image = array
        
        # Add to queue if not full
        if not self.image_queue.full():
            self.image_queue.put(array)
    
    def get_image(self):
        """Get latest image from queue or return last image"""
        try:
            return self.image_queue.get(block=False)
        except queue.Empty:
            return self.image
    
    def destroy(self):
        """Destroy camera sensor"""
        if self.sensor is not None:
            self.sensor.stop()
            self.sensor.destroy()


class VehicleController:
    """
    Vehicle controller based on differential steering
    Similar to IGVC microcontroller logic but adapted for CARLA
    """
    
    def __init__(self, vehicle):
        """
        Initialize vehicle controller
        
        Args:
            vehicle: CARLA vehicle actor
        """
        self.vehicle = vehicle
        
        # Controller parameters (from controller_v2.ino)
        self.base_speed = 0.4  # Base throttle (0-1 scale, similar to pwm=79 in Arduino)
        self.k = 0.009  # Gain factor for steering correction (scaled from k2=0.89)
        self.offset_final = 0.0  # Steering offset from lane detection
        self.max_offset = 50.0  # Maximum offset angle
        
        # Speed limits
        self.min_throttle = 0.2
        self.max_throttle = 0.6
        
        # Differential steering parameters
        self.left_speed = 0.0
        self.right_speed = 0.0
        
    def update(self, steering_angle):
        """
        Update vehicle control based on steering angle from lane detection
        
        Args:
            steering_angle: Angle in degrees (-90 to 90)
        """
        # Update offset (negative because positive angle means steer right)
        self.offset_final = -steering_angle
        
        # Clamp offset to max range
        if self.offset_final > self.max_offset:
            self.offset_final = self.max_offset
        elif self.offset_final < -self.max_offset:
            self.offset_final = -self.max_offset
        
        # Calculate differential speeds (from Arduino logic)
        # lft_pwm = pwm + (k2 * offsetFinal)
        # rht_pwm = pwm - (k2 * offsetFinal)
        self.left_speed = self.base_speed + (self.k * self.offset_final)
        self.right_speed = self.base_speed - (self.k * self.offset_final)
        
        # Clamp speeds to valid range
        self.left_speed = np.clip(self.left_speed, self.min_throttle, self.max_throttle)
        self.right_speed = np.clip(self.right_speed, self.min_throttle, self.max_throttle)
        
        # Calculate average throttle and steering from differential speeds
        avg_throttle = (self.left_speed + self.right_speed) / 2.0
        steer_value = (self.right_speed - self.left_speed) / (self.max_throttle - self.min_throttle)
        steer_value = np.clip(steer_value, -1.0, 1.0)
        
        # Apply control to vehicle
        control = carla.VehicleControl(
            throttle=float(avg_throttle),
            steer=float(steer_value),
            brake=0.0,
            hand_brake=False
        )
        self.vehicle.apply_control(control)
        
        return avg_throttle, steer_value
    
    def stop(self):
        """Stop the vehicle"""
        control = carla.VehicleControl(
            throttle=0.0,
            steer=0.0,
            brake=1.0,
            hand_brake=True
        )
        self.vehicle.apply_control(control)
    
    def get_status(self):
        """Get current controller status as string"""
        return (f"L_Speed: {self.left_speed:.3f} | R_Speed: {self.right_speed:.3f} | "
                f"Offset: {self.offset_final:.2f}°")


class LaneDetectionSystem:
    """Lane Detection System using color-based segmentation and peak detection"""
    
    def __init__(self, camera_manager, display=True, debug=False):
        """
        Initialize lane detection system
        
        Args:
            camera_manager: CameraManager instance
            display: Whether to display visualization windows
            debug: Whether to show debug information
        """
        self.camera_manager = camera_manager
        self.display = display
        self.debug = debug
        self.avg_angle = 0.0
        
        # HSV/HLS thresholds for white lane detection
        # These values work well for CARLA's road markings
        self.white_lower = np.array([0, 200, 0])      # H, L, S for white lanes
        self.white_upper = np.array([255, 255, 255])
        
        # HSV/HLS thresholds for yellow lane detection (optional)
        self.yellow_lower = np.array([10, 50, 100])
        self.yellow_upper = np.array([40, 255, 255])
        
        # Trackbars for tuning (if display enabled)
        if self.display:
            cv2.namedWindow("Trackbars")
            cv2.resizeWindow("Trackbars", 360, 360)
            trackbar_names = ["H_min", "L_min", "S_min", "H_max", "L_max", "S_max", "ROI_top", "ROI_bot", "Height"]
            trackbar_vals = [0, 200, 0, 255, 255, 255, 7, 0, 8]
            
            for i, name in enumerate(trackbar_names):
                max_val = 255 if i < 6 else 10
                cv2.createTrackbar(name, "Trackbars", trackbar_vals[i], max_val, lambda x: None)
        
        # Matplotlib for histogram visualization
        if self.debug:
            plt.ion()
            self.fig = plt.figure(figsize=(10, 3))
            self.ax = self.fig.add_subplot(111)
            self.plot, = self.ax.plot([])
            plt.title("Lane Detection Histogram")
            plt.xlabel("X Position (pixels)")
            plt.ylabel("Intensity Sum")
    
    def get_trackbar_values(self):
        """Get current trackbar values for dynamic tuning"""
        if not self.display:
            return None
        
        vals = []
        for name in ["H_min", "L_min", "S_min", "H_max", "L_max", "S_max"]:
            vals.append(cv2.getTrackbarPos(name, "Trackbars"))
        return vals
    
    def detect_lanes(self):
        """
        Main lane detection algorithm
        Returns steering angle in degrees (-90 to 90, where 0 is straight)
        """
        img = self.camera_manager.get_image()
        
        if img is None:
            return self.avg_angle
        
        h, w = img.shape[0], img.shape[1]
        
        # Convert to HLS color space (better for lane detection)
        hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        
        # Get trackbar values if available, otherwise use defaults
        trackbar_vals = self.get_trackbar_values()
        if trackbar_vals:
            white_lower = np.array(trackbar_vals[:3])
            white_upper = np.array(trackbar_vals[3:6])
        else:
            white_lower = self.white_lower
            white_upper = self.white_upper
        
        # Create mask for white lanes
        mask_white = cv2.inRange(hls, white_lower, white_upper)
        
        # Optional: Create mask for yellow lanes and combine
        mask_yellow = cv2.inRange(hls, self.yellow_lower, self.yellow_upper)
        combined_mask = cv2.bitwise_or(mask_white, mask_yellow)
        
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(combined_mask, (5, 5), 0)
        
        # Define Region of Interest (ROI) - trapezoid focusing on road ahead
        roi_top_ratio = 0.75  # Focus on bottom 25% of image
        polygons = np.array([[(0, int(h * roi_top_ratio)), 
                              (w, int(h * roi_top_ratio)), 
                              (w, h), 
                              (0, h)]])
        
        # Create ROI mask
        roi_mask = np.zeros_like(blurred)
        cv2.fillPoly(roi_mask, polygons, 255)
        masked_image = cv2.bitwise_and(blurred, roi_mask)
        
        # Invert for peak detection (lanes become peaks)
        masked_image = 255 - masked_image
        
        # Sum pixel intensities along vertical axis to get horizontal histogram
        histogram = np.sum(masked_image, axis=0)
        
        # Find peaks in histogram (these represent lane positions)
        peaks, properties = find_peaks(histogram, plateau_size=10)
        
        # Visualization
        display_img = img.copy()
        
        # Draw ROI boundary
        cv2.line(display_img, (0, int(h * roi_top_ratio)), 
                (w, int(h * roi_top_ratio)), (0, 255, 0), 2)
        
        current_angle = self.avg_angle
        
        # Process detected peaks
        if 'plateau_sizes' in properties and len(properties['plateau_sizes']) > 0:
            sizes = properties['plateau_sizes']
            
            # Find the largest plateau (most likely the lane)
            max_idx = sizes.argmax()
            left_edge = properties['left_edges'][max_idx]
            right_edge = properties['right_edges'][max_idx]
            
            # Calculate lane center
            lane_center_x = (left_edge + right_edge) // 2
            lane_center_y = int(h * roi_top_ratio)
            
            # Image center (vehicle reference point)
            img_center_x = w // 2
            img_center_y = h
            
            # Draw detected lane boundaries
            cv2.line(display_img, (left_edge, lane_center_y), 
                    (right_edge, lane_center_y), (255, 0, 0), 5)
            
            # Draw steering direction line
            cv2.line(display_img, (lane_center_x, lane_center_y), 
                    (img_center_x, img_center_y), (0, 0, 255), 5)
            
            # Calculate steering angle
            dx = img_center_x - lane_center_x
            dy = img_center_y - lane_center_y
            angle = math.degrees(math.atan2(dy, dx)) - 90
            
            # Apply smoothing to avoid jittery steering
            current_angle = (angle + self.avg_angle) / 2
            self.avg_angle = current_angle
            
            # Display angle
            cv2.putText(display_img, f"Steering: {current_angle:.2f} deg", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.8, (0, 255, 0), 2, cv2.LINE_AA)
        else:
            # No lane detected, check edge cases
            if histogram[0] > 200000:
                cv2.putText(display_img, "Lane on LEFT edge!", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                           1, (0, 0, 255), 2, cv2.LINE_AA)
            elif histogram[-1] > 200000:
                cv2.putText(display_img, "Lane on RIGHT edge!", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                           1, (0, 0, 255), 2, cv2.LINE_AA)
            else:
                cv2.putText(display_img, f"No lane | Last: {self.avg_angle:.2f} deg", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                           1, (0, 255, 255), 2, cv2.LINE_AA)
        
        # Display visualization
        if self.display:
            cv2.imshow('Lane Detection', display_img)
            cv2.imshow('Masked Image', masked_image)
            cv2.imshow('ROI Mask', roi_mask)
            cv2.waitKey(1)
        
        # Debug histogram plot
        if self.debug:
            self.ax.clear()
            self.ax.plot(histogram)
            if 'plateau_sizes' in properties and len(properties['plateau_sizes']) > 0:
                self.ax.plot(peaks, histogram[peaks], "rx", markersize=10)
            self.ax.set_title("Lane Detection Histogram")
            self.ax.set_xlabel("X Position (pixels)")
            self.ax.set_ylabel("Intensity Sum")
            plt.pause(0.001)
        
        return current_angle
    
    def cleanup(self):
        """Clean up visualization windows"""
        if self.display:
            cv2.destroyAllWindows()
        if self.debug:
            plt.close()


def main(args):
    """Main function to run lane detection system"""
    
    client = None
    vehicle = None
    camera_manager = None
    lane_detector = None
    controller = None
    
    try:
        # Connect to CARLA server
        print(f"Connecting to CARLA server at {args.host}:{args.port}...")
        client = carla.Client(args.host, args.port)
        client.set_timeout(10.0)
        
        # Load world
        world = client.get_world()
        print(f"Connected to world: {world.get_map().name}")
        
        # Set synchronous mode for consistent frame rate
        if args.sync:
            settings = world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05  # 20 FPS
            world.apply_settings(settings)
            print("Synchronous mode enabled")
        
        # Get vehicle blueprint
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter('vehicle.*')[args.vehicle_id]
        
        # Spawn vehicle at a spawn point
        spawn_points = world.get_map().get_spawn_points()
        if len(spawn_points) == 0:
            print("No spawn points available!")
            return
        
        spawn_point = spawn_points[args.spawn_point % len(spawn_points)]
        
        # Rotate vehicle 180 degrees
        spawn_point.rotation.yaw += 180.0
        
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        print(f"Vehicle spawned: {vehicle.type_id}")
        
        # Enable autopilot if requested
        if args.autopilot:
            vehicle.set_autopilot(True)
            print("Autopilot enabled")
        
        # Wait for vehicle to settle
        if args.sync:
            world.tick()
        else:
            time.sleep(0.5)
        
        # Create camera manager
        camera_manager = CameraManager(vehicle, world, 
                                      width=args.width, 
                                      height=args.height)
        
        # Wait for first image
        print("Waiting for camera data...")
        if args.sync:
            for _ in range(5):
                world.tick()
        else:
            time.sleep(1.0)
        
        # Create lane detection system
        lane_detector = LaneDetectionSystem(camera_manager, 
                                           display=args.display,
                                           debug=args.debug)
        
        # Create vehicle controller
        controller = VehicleController(vehicle)
        
        print("\n=== Lane Detection System Started ===")
        print("Controls:")
        print("  Q: Quit")
        print("  P: Toggle autopilot")
        print("  C: Toggle lane-following controller")
        print("  S: Emergency stop")
        if args.display:
            print("  Use trackbars to tune detection parameters")
        print("=====================================\n")
        
        # Main loop
        frame_count = 0
        autopilot_enabled = args.autopilot
        controller_enabled = args.auto_steer
        
        print(f"Starting main loop...")
        print(f"  Autopilot: {'ON' if autopilot_enabled else 'OFF'}")
        print(f"  Controller: {'ON' if controller_enabled else 'OFF'}\n")
        
        while True:
            if args.sync:
                world.tick()
            
            # Detect lanes and get steering angle
            steering_angle = lane_detector.detect_lanes()
            
            # Apply controller if enabled and autopilot is off
            if controller_enabled and not autopilot_enabled:
                throttle, steer = controller.update(steering_angle)
                
                # Display controller status
                if args.display and frame_count % 10 == 0:
                    print(f"\rController: {controller.get_status()}", end='', flush=True)
            
            frame_count += 1
            
            # Handle keyboard input
            if args.display:
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("\nQuitting...")
                    break
                elif key == ord('p'):
                    autopilot_enabled = not autopilot_enabled
                    vehicle.set_autopilot(autopilot_enabled)
                    print(f"\nAutopilot: {'ON' if autopilot_enabled else 'OFF'}")
                    if autopilot_enabled:
                        controller_enabled = False
                        print("Controller: OFF (autopilot takes priority)")
                elif key == ord('c'):
                    if not autopilot_enabled:
                        controller_enabled = not controller_enabled
                        print(f"\nController: {'ON' if controller_enabled else 'OFF'}")
                        if not controller_enabled:
                            controller.stop()
                    else:
                        print("\nCannot enable controller while autopilot is active")
                elif key == ord('s'):
                    print("\n[EMERGENCY STOP]")
                    controller.stop()
                    autopilot_enabled = False
                    controller_enabled = False
                    vehicle.set_autopilot(False)
            
            # Limit frame rate in async mode
            if not args.sync:
                time.sleep(0.03)  # ~30 FPS
    
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    
    except Exception as e:
        print(f"\nError occurred: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Cleanup
        print("\nCleaning up...")
        
        if controller:
            controller.stop()
        
        if lane_detector:
            lane_detector.cleanup()
        
        if camera_manager:
            camera_manager.destroy()
        
        if vehicle is not None:
            vehicle.destroy()
            print("Vehicle destroyed")
        
        if client and args.sync:
            # Restore async mode
            world = client.get_world()
            settings = world.get_settings()
            settings.synchronous_mode = False
            world.apply_settings(settings)
            print("Async mode restored")
        
        print("Cleanup complete")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Lane Detection System for CARLA')
    parser.add_argument('--host', default='127.0.0.1', help='CARLA server host')
    parser.add_argument('--port', type=int, default=2000, help='CARLA server port')
    parser.add_argument('--sync', action='store_true', help='Enable synchronous mode')
    parser.add_argument('--width', type=int, default=800, help='Camera width')
    parser.add_argument('--height', type=int, default=600, help='Camera height')
    parser.add_argument('--vehicle-id', type=int, default=0, help='Vehicle blueprint index')
    parser.add_argument('--spawn-point', type=int, default=0, help='Spawn point index')
    parser.add_argument('--autopilot', action='store_true', help='Enable autopilot initially')
    parser.add_argument('--auto-steer', action='store_true', help='Auto-steer based on lane detection')
    parser.add_argument('--display', action='store_true', default=True, help='Display visualization')
    parser.add_argument('--no-display', dest='display', action='store_false', help='Disable visualization')
    parser.add_argument('--debug', action='store_true', help='Show debug histogram')
    
    args = parser.parse_args()
    
    try:
        main(args)
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
