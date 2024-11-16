#python 3
import serial
import time
import math
import simple_pid #python -m pip install simple-pid
from simple_pid import PID
pid = PID(1, 0.1, 0.05, setpoint=1)
pid.sample_time = 1.0  # Update every 1 seconds

# Initialize serial communication for actuator control
#serial_port = "/dev/ttyUSB0"  # Update with your port
serial_port = 'COM15'
baud_rate = 115200  # Adjust baud rate as per your actuator settings
ser = serial.Serial(serial_port, baud_rate, timeout=1)

# GPS waypoint (Latitude, Longitude)
WAYPOINT = (40.748817, -73.985428)  # Example: GPS coordinates of a waypoint

# Function to calculate distance between two GPS coordinates (Haversine formula)
def haversine(coord1, coord2):
    lat1, lon1 = math.radians(coord1[0]), math.radians(coord1[1])
    lat2, lon2 = math.radians(coord2[0]), math.radians(coord2[1])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    radius = 6371000  # Earth radius in meters
    return radius * c


# Function to calculate the target bearing in degrees
def calculate_target_bearing(current_position, waypoint):
    lat1 = math.radians(current_position[0])
    lon1 = math.radians(current_position[1])
    lat2 = math.radians(waypoint[0])
    lon2 = math.radians(waypoint[1])
    
    dlon = lon2 - lon1
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(dlon))
    rad_bearing = math.atan2(x, y)
    
    # Convert bearing from radians to degrees, normalize to 0-360
    deg_bearing = math.degrees(rad_bearing)
    target_bearing = (deg_bearing + 360) % 360
    return target_bearing
        
        
# Function to determine the direction to move based on current and target GPS positions
def calculate_move_command(current_position, waypoint,current_heading,target_bearing):
    # Example basic logic based on latitude and longitude comparison
    lat_diff = waypoint[0] - current_position[0]
    lon_diff = waypoint[1] - current_position[1]
    
    if abs(lat_diff) < 0.0001 and abs(lon_diff) < 0.0001:
        return 'S'  # Stop if near the waypoint
        
    if current_heading==None :
        return 'S'
    if abs(target_bearing-current_heading)> 120:
        return 'U' # u turn left side (right not implemented)
    elif abs(target_bearing-current_heading)> 90:
        if (target_bearing-current_heading)> 90:
            return 'R'
        else: #(target_bearing-current_heading)< -90
            return 'L'
    elif abs(target_bearing-current_heading)> 30:
        if (target_bearing-current_heading)> 30:
            return 'R'
        else: #(target_bearing-current_heading)< -30
            return 'L'
    elif abs(target_bearing-current_heading)> 10:
        if (target_bearing-current_heading)> 10:
            return 'r'
        else: #(target_bearing-current_heading)< -90
            return 'l'
    else:# -10<.<10
        return 'F'

# Function to determine the direction to move based on current and target GPS positions
def calculate_PWM_command(current_position, waypoint,current_heading,target_bearing):
    PWM_A = (target_bearing-current_heading)
    PWM_B = (current_heading-target_bearing)
    
# Function to send enable motor command
def send_calib_command():
    if ser.is_open:
        command = f"POST CALIB" #enable motor for 30s
        ser.write(command.encode())
        time.sleep(1)
        response=""
        while "OK" not in response: 
          response = ser.readline().decode().strip()  # Read response from the actuator
          #print(f"got:{response}")
          time.sleep(0.2)
        print(f"Sent: {command}, Received: {response}")

# Function to send enable motor command
def send_enable_command():
    if ser.is_open:
        command = f"POST WD 30" #enable motor for 30s
        ser.write(command.encode())
        time.sleep(0.1)
        response=""
        while "OK" not in response:  
          response = ser.readline().decode().strip()  # Read response from the actuator
          #print(f"got:{response}")
          time.sleep(0.2)
        print(f"Sent: {command}, Received: {response}")
        
# Function to send command to the actuator
def send_move_command(command):
    if ser.is_open:
        command = f"POST MOVE {command}"
        ser.write(command.encode())
        time.sleep(0.1)
        response=""
        while "OK" not in response: 
          response = ser.readline().decode().strip()  # Read response from the actuator
          #print(f"got:{response}")
          time.sleep(0.2)
        print(f"Sent: {command}, Received: {response}")

def send_nav_command(duration, heading):
    if ser.is_open:
        command = f"POST NAV {duration} {heading}"
        ser.write(command.encode())
        time.sleep(0.1)
        response=""
        while "OK" not in response: 
          response = ser.readline().decode().strip()  # Read response from the actuator
          #print(f"got:{response}")
          time.sleep(0.2)
        print(f"Sent: {command}, Received: {response}")

def send_speed_command(speed):
    if ser.is_open:
        command = f"POST SPEED {speed}"
        ser.write(command.encode())
        time.sleep(0.1)
        response=""
        while "OK" not in response: 
          response = ser.readline().decode().strip()  # Read response from the actuator
          #print(f"got:{response}")
          time.sleep(0.2)
        print(f"Sent: {command}, Received: {response}")


# Function to send command to the actuator
def get_heading_command():
    if ser.is_open:
        command = f"GET HEADING"
        ser.write(command.encode())
        response=""
        while "OK" not in response: 
          response = ser.readline().decode().strip()  # Read response from the actuator
          print(f"got:{response}")
          time.sleep(0.2)
        print(f"Sent: {command}, Received: {response}")
        
        # Parse the heading from the response
        try:
            # Find the start and end of the heading value within the response string
            start = response.index("heading:") + len("heading:")
            end = response.index("}", start)
            heading_value = float(response[start:end].strip())  # Convert to float
            return heading_value
        except (ValueError, IndexError) as e:
            print(f"Error parsing heading: {e}")
            return None

# Main control loop
def control_loop():
    heading_error=0
    while True:
        send_enable_command()
        # Get current GPS coordinates (you should replace this with actual GPS data retrieval)
        current_position = get_current_gps()  # Replace with actual GPS retrieval logic
        current_heading = get_heading_command()
        current_heading = get_heading_command()# dual reading to flush !
        # Calculate distance to waypoint
        distance = haversine(current_position, WAYPOINT)
        print(f"Distance to waypoint: {distance:.2f} meters")
        
        # Calculate the target course (bearing) to the waypoint
        target_bearing = calculate_target_bearing(current_position, WAYPOINT)
        print(f"Target Bearing: {target_bearing:.2f} degrees")


        if current_heading != None:
          temp=(int)(target_bearing-current_heading)
          print(f"temp= {temp} degrees")
        
          heading_error= target_bearing-current_heading
        control=pid(heading_error)
        print(f"pid output : {control}")
        #send_nav_command(30, target_bearing)
        p, i, d = pid.components  # The separate terms are now in p, i, d
        print(f"PID: {p:.2f},{i:.2f},{d:.2f}")

        # Calculate the direction command to send
        move_command = calculate_move_command(current_position, WAYPOINT,current_heading,target_bearing)
        print(f"Calculated Move Command: {move_command}")

        # Send the move command to the actuator
        send_move_command(move_command)

        # Wait for 2 seconds before the next correction
        time.sleep(2)

# Mock function to retrieve current GPS coordinates
def get_current_gps():
    # This function should interface with your GPS module and return (latitude, longitude)
    # Replace this with actual code to read GPS data from your GPS module
    return (40.758900, -73.975500)  # Example mock GPS data, update with live data

if __name__ == "__main__":
    try:
        time.sleep(0.5)
        response = ser.readline().decode().strip()  # Read response from the actuator
        print(f"Received: {response}")
        response = ser.readline().decode().strip()  # Read response from the actuator
        print(f"Received: {response}")
        send_enable_command()
        time.sleep(1)
        send_calib_command()
        send_speed_command(30)
        control_loop()
    except KeyboardInterrupt:
        print("Control loop interrupted. Exiting...")
    finally:
        if ser.is_open:
            ser.close()
