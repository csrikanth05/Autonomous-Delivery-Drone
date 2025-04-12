import time
import logging
import cv2
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pyzbar.pyzbar import decode
from picamera2 import Picamera2
from gpiozero import Servo
from test import control_servo  # Ensure this module exists

# Setup
LOG_FILE = "flight_log.txt"
QR_FILE = "qr_output.txt"
logging.basicConfig(filename=LOG_FILE, level=logging.INFO, format='%(asctime)s [%(levelname)s] %(message)s')

def log_event(event):
    print(event)
    logging.info(event)

# Connect to the Pixhawk
log_event("Connecting to drone...")
vehicle = connect('udp:127.0.0.1:14551', baud=921600, wait_ready=True)
log_event("Connected to drone.")

# Wait for valid GPS lock and set home
while not vehicle.location.global_frame.lat:
    log_event("Waiting for GPS fix...")
    time.sleep(2)
home_location = vehicle.location.global_frame
log_event(f"Home location set: {home_location.lat}, {home_location.lon}")

def check_failsafe():
    """Handle GPS loss failsafe: Hover for 15s, then land if GPS is not restored."""
    if vehicle.gps_0.fix_type < 3:
        log_event("GPS lost! Hovering for 15 seconds to attempt recovery...")
        vehicle.mode = VehicleMode("LOITER")  # Hover in place

        time.sleep(15)  # Give time for GPS to recover

        if vehicle.gps_0.fix_type < 3:  # Check again
            log_event(" GPS not restored! Landing in place...")
            vehicle.mode = VehicleMode("LAND")  # Land at the current location
            return True

        log_event("GPS signal restored. Resuming mission...")
    return False

def takeoff(altitude):
    log_event("Arming drone...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(1)
    log_event("Drone armed.")

    log_event(f"Taking off to {altitude}m...")
    vehicle.simple_takeoff(altitude)

    while True:
        current_alt = vehicle.location.global_relative_frame.alt
        log_event(f"Altitude: {current_alt:.2f} m")
        if current_alt >= 0.95 * altitude:
            log_event("Reached target altitude.")
            break
        if check_failsafe():
            return
        time.sleep(1)

def fly_to(target_location):
    log_event(f"Flying to {target_location.lat}, {target_location.lon}")
    vehicle.simple_goto(target_location)

    while True:
        current_location = vehicle.location.global_relative_frame
        log_event(f"Current Location: {current_location.lat}, {current_location.lon}, Alt: {current_location.alt:.2f}")
        if check_failsafe():
            return
        distance_to_target = ((current_location.lat - target_location.lat) ** 2 + 
                              (current_location.lon - target_location.lon) ** 2) ** 0.5

        if distance_to_target < 0.00003:  # Reached destination
            log_event("Reached destination.")
            break
        time.sleep(2)

def land_slowly():
    """Ensures a slow and controlled descent."""
    log_event("Starting slow descent...")
    vehicle.mode = VehicleMode("LAND")

    while vehicle.location.global_relative_frame.alt > 0.2:
        log_event(f"Altitude: {vehicle.location.global_relative_frame.alt:.2f}m")
        time.sleep(1)

    log_event("Drone landed.")

def scan_qr():
    """Scans QR code after landing."""
    log_event("Initiating QR scan after landing...")
    picam2 = Picamera2()
    picam2.preview_configuration.main.size = (640, 480)
    picam2.preview_configuration.main.format = "RGB888"
    picam2.configure("preview")
    picam2.start()

    start_time = time.time()
    scanned = False
    result_data = None

    while time.time() - start_time < 30:
        frame = picam2.capture_array()
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        qr_codes = decode(gray)

        for qr in qr_codes:
            result_data = qr.data.decode('utf-8')
            log_event(f"QR Code detected: {result_data}")
            with open(QR_FILE, "w") as file:
                file.write(result_data)
            scanned = True
            break

        if scanned:
            break
        time.sleep(1)

    picam2.stop()
    if not scanned:
        log_event("QR scan failed or timed out.")
    
    return result_data

def drop_payload():
    """Activates the servo mechanism to drop the payload after landing."""
    log_event("Dropping payload...")
    control_servo(1, 2)  # Trigger servo
    log_event("Payload dropped.")

def mission():
    """Main mission sequence."""
    log_event(f"Home location: {home_location.lat}, {home_location.lon}")

    dest_lat = float(input("Enter destination latitude: "))
    dest_lon = float(input("Enter destination longitude: "))

    takeoff(6)

    target_location = LocationGlobalRelative(dest_lat, dest_lon, 6)
    fly_to(target_location)

    log_event("Starting slow landing at destination...")
    land_slowly()

    qr_result = scan_qr()

    if qr_result:
        log_event("QR Code successfully scanned.")
    drop_payload()

    log_event("Waiting 20 seconds before returning to launch...")
    time.sleep(20)

    log_event("Returning to Launch (RTL).")
    vehicle.mode = VehicleMode("RTL")

if __name__ == "__main__":
    try:
        mission()
    except Exception as e:
        log_event(f"Error: {e}")
        vehicle.mode = VehicleMode("LAND")
    finally:
        log_event("Mission complete. Disconnecting...")
        vehicle.close()
