import csv
import serial
import time

# CSV File Path
csv_file = "positions.csv"

# Initialize Serial Communication with Arduino
arduino = serial.Serial('/dev/serial0', 9600, timeout=1)
time.sleep(2)  # Allow Arduino to reset

# Read positions from CSV
def read_positions(csv_file):
    positions = {}
    with open(csv_file, mode='r') as file:
        csv_reader = csv.DictReader(file)
        for row in csv_reader:
            name = row['Name']
            x = int(row['X'])
            y = int(row['Y'])
            z = int(row['Z'])
            positions[name] = (x, y, z)
    return positions
    
# Wait for Arduino acknowledgment
def wait_for_acknowledgment():
    while True:
        if arduino.in_waiting > 0:
            try:
                response = arduino.readline().decode('utf-8', errors='replace').strip()
                print(f"Arduino Response: {response}")
                if response == "Complete":  # Match the expected acknowledgment
                    return True
            except UnicodeDecodeError as e:
                print(f"Decoding error: {e}. Skipping invalid data.")
        time.sleep(0.1)


# Main function to control the flow
def main():
    positions = read_positions(csv_file)
    print("Available positions:")
    for name in positions.keys():
        print(f"- {name}")

    while True:
        # Ask the user for a position name
        position_name = input("\nEnter the name of the position to move to (or 'exit' to quit): ").strip()
        if position_name.lower() == 'exit':
            print("Exiting program.")
            break

        if position_name in positions:
            x, y, z = positions[position_name]
            coordinates = f"{x},{y},{z}\n"
            print(f"Sending coordinates: {coordinates.strip()}")
            
            # Send coordinates to Arduino
            arduino.write(coordinates.encode('utf-8'))
            
            # Wait for Arduino acknowledgment
            print("Waiting   for Arduino acknowledgment...")
            if wait_for_acknowledgment():
                print("Arduino has completed the operation. Ready for next input.")
        else:
            print(f"Position '{position_name}' not found. Please try again.")

# Run the main function
if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nProgram interrupted.")
    finally:
        arduino.close()
