import time
import logging
from dronekit import connect, VehicleMode, LocationGlobalRelative

# Logging setup
LOG_FILE = "test_flight_log.txt"
logging.basicConfig(filename=LOG_FILE, level=logging.INFO, format='%(asctime)s %(message)s')

def log_event(event):
    print(event)
    logging.info(event)

# Connect to the Pixhawk
log_event("Connecting to drone...")
vehicle = connect('udp:127.0.0.1:14551', baud=921600, wait_ready=True)
log_event("Connected.")

def check_failsafe():
    if vehicle.gps_0.fix_type < 3:
        log_event("GPS signal lost! Landing immediately.")
        vehicle.mode = VehicleMode("LAND")
        return True

    if vehicle.battery.voltage and vehicle.battery.voltage < 10.5:
        log_event(f"Low battery ({vehicle.battery.voltage}V)! Landing.")
        vehicle.mode = VehicleMode("LAND")
        return True

    return False

def test_takeoff_and_slow_land():
    log_event("Starting test: takeoff to 2m, then descend using simple_goto.")

    # Set GUIDED mode and arm
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        log_event("Waiting for drone to arm...")
        time.sleep(1)

    log_event("Drone armed. Taking off to 2 meters.")
    vehicle.simple_takeoff(2)

    # Wait until we reach ~1.8m altitude
    while True:
        alt = vehicle.location.global_relative_frame.alt
        log_event(f"Current altitude: {alt:.2f}m")
        if alt >= 1.8:
            log_event("Reached test altitude.")
            break
        if check_failsafe():
            return
        time.sleep(1)

    log_event("Hovering for 3 seconds...")
    time.sleep(3)

    # Slow descent using simple_goto to altitude 0.3m
    target_location = LocationGlobalRelative(
        vehicle.location.global_frame.lat,
        vehicle.location.global_frame.lon,
        0.3  # Not 0 to avoid premature disarm by some FCs
    )
    log_event("Descending slowly using simple_goto to 0.3m...")
    vehicle.simple_goto(target_location, groundspeed=0.5)  # very slow descent

    # Wait for descent to complete
    while True:
        alt = vehicle.location.global_relative_frame.alt
        log_event(f"Descending... Current altitude: {alt:.2f}m")
        if alt <= 0.4:
            log_event("Reached near ground level.")
            break
        if check_failsafe():
            return
        time.sleep(1)

    log_event("Waiting to land completely...")
    while True:
        speed = sum(v ** 2 for v in vehicle.velocity) ** 0.5
        if speed < 0.2:
            log_event("Drone has landed.")
            break
        time.sleep(1)

    log_event("Disarming drone.")
    vehicle.armed = False
    while vehicle.armed:
        time.sleep(1)

    log_event("Test complete. Disconnecting.")
    vehicle.close()

if __name__ == "__main__":
    try:
        test_takeoff_and_slow_land()
    except Exception as e:
        log_event(f"Error: {e}")
        vehicle.mode = VehicleMode("LAND")
        vehicle.close()
