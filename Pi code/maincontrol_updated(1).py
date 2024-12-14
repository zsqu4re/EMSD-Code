import csv
import os
import serial
import time

# --- Constants ---
BOOK_COORDINATES_FILE = "book_coordinates.csv"
LAYERS = 2
HEIGHT_DIFFERENCE = 342 # Height difference between layers in z-axis (vertical)
LENGTH = 1000  # Length of each layer in x-axis (horizontal)
GRIPPER_IN = 200
GRIPPER_OUT = 10
y_in = GRIPPER_IN
y_out = GRIPPER_OUT

drop_x = 20
# drop_y = GRIPPER_IN
drop_z = HEIGHT_DIFFERENCE*2

# --- Arduino Setup ---
arduino = serial.Serial('/dev/ttyACM1', 9600, timeout=1)
time.sleep(2)  # Allow Arduino to reset

# --- Helper Functions ---
def wait_for_acknowledgment(str):
    """Wait for Arduino acknowledgment."""
    while True:
        if arduino.in_waiting > 0:
            response = arduino.readline().decode('utf-8').strip()
            print(f"Arduino Response: {response}")
            if response == str:
                return True
        #time.sleep(0.5)

def send_command(command):
    """Send command to Arduino and wait for acknowledgment."""
    print(f"Sending command to Arduino: {command}")
    arduino.write(f"{command}\n".encode('utf-8'))
    if wait_for_acknowledgment("Complete"):
        return
        # print("Command executed successfully.")
    # else:
        # print("Command failed or no acknowledgment received.")

# --- CSV Management ---
def initialize_csv():
    """Ensure book coordinate CSV exists with correct headers."""
    if not os.path.exists(BOOK_COORDINATES_FILE):
        with open(BOOK_COORDINATES_FILE, mode="w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["BookID", "Layer", "X_Coordinate", "Thickness"])

def find_book_position(book_id):
    """Find the position of a book from the CSV file."""
    with open(BOOK_COORDINATES_FILE, mode="r") as file:
        reader = csv.DictReader(file)
        for row in reader:
            if row['BookID'] == book_id:
                layer = int(row['Layer'])
                x = int(row['X_Coordinate'])
                return layer, x
    return None, None

def remove_book_from_csv(book_id):
    """Remove a book from the CSV file."""
    rows = []
    found = False
    with open(BOOK_COORDINATES_FILE, mode="r") as file:
        reader = csv.reader(file)
        header = next(reader)
        for row in reader:
            if row[0] != book_id:
                rows.append(row)
            else:
                found = True
    if found:
        with open(BOOK_COORDINATES_FILE, mode="w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(header)
            writer.writerows(rows)
        print(f"Book '{book_id}' removed from the CSV.")
    else:
        print(f"Book '{book_id}' not found in the CSV.")

def find_gaps(layer, layer_length):
    """Find gaps in a specific layer based on occupied positions."""
    occupied = []
    if os.path.exists(BOOK_COORDINATES_FILE):
        with open(BOOK_COORDINATES_FILE, mode="r") as file:
            reader = csv.reader(file)
            next(reader)  # Skip the header
            for row in reader:
                if int(row[1]) - 1 == layer:
                    x_start = int(row[2])
                    thickness = int(row[3])
                    x_end = x_start + thickness
                    occupied.append((x_start, x_end))
    occupied.sort()
    gaps = []
    current_position = 10
    for x_start, x_end in occupied:
        if x_start > current_position:
            gaps.append((current_position, x_start - current_position))
        current_position = max(current_position, x_end)
    if current_position < layer_length:
        gaps.append((current_position, layer_length - current_position))
    return gaps

# --- Gripper and Gantry Control ---
def move_gantry(x, y, z):
    """Move gantry to specified X, Z, Y coordinates and wait for acknowledgment."""
    coordinates = f"{x},{y},{z}\n"
    send_command(coordinates)
    time.sleep(1)
    return
    

def grip_action(x, z):
    move_gantry(x, y_in, z)
    print("Closing gripper...")
    send_command(1)
    move_gantry(x, y_out, z)
    return

def release_action(x, z):
    
    move_gantry(x, y_in, z)
    print("Releasing gripper...")
    send_command(0)
    move_gantry(x, y_out, z)
    return

def automate_storage(book_id, thickness):
    """Automate storing a book."""
    initialize_csv()

      # Check if the book is already stored
    with open(BOOK_COORDINATES_FILE, mode="r") as file:
        reader = csv.DictReader(file)
        for row in reader:
            if row['BookID'] == book_id:
                print(f"Error: '{book_id}' is already stored.")
                return

    # Find a suitable position
    for layer_index in range(LAYERS):
        gaps = find_gaps(layer_index, LENGTH)
        for gap_start, gap_thickness in gaps:
            if gap_thickness >= thickness:
                x = gap_start
                z = (layer_index + 1) * HEIGHT_DIFFERENCE
                y_in = GRIPPER_IN
                y_out = GRIPPER_OUT

                # Move gantry to pickup position outside bookshelf
                move_gantry(drop_x, y_out, drop_z)

                # Move gantry inside bookshelf, grip, go out
                grip_action(drop_x,drop_z)

                # Move gantry to storing position outside bookshelf
                move_gantry(x, y_out, z)

                # Move gantry inside bookshelf, release, go out
                release_action(x,z)
                
                # Save the position to the CSV
                with open(BOOK_COORDINATES_FILE, mode="a", newline="") as file:
                    writer = csv.writer(file)
                    writer.writerow([book_id, layer_index + 1, round(x, 2), int(thickness)])
                print(f"Book '{book_id}' stored successfully.")
                return

    print("No suitable storage space available.")

def automate_retrieval(book_id):
    """Automate retrieving a book."""
    initialize_csv()

    # Locate the book position
    layer, x = find_book_position(book_id)
    if layer is None:
        print(f"Book '{book_id}' not found.")
        return

    z = layer * HEIGHT_DIFFERENCE
    y_in = GRIPPER_IN
    y_out = GRIPPER_OUT

    #Move gantry to storing position outside bookshelf
    move_gantry(x, y_out, z)

    # Move gantry inside bookshelf, grip, go out
    grip_action(x,z)

    # Move gantry back outside bookshelf
    move_gantry(drop_x, y_out, drop_z)

    # Move gantry inside bookshelf, release, go out
    release_action(drop_x,drop_z)

    # Remove book details from CSV
    remove_book_from_csv(book_id)

# --- Main Program ---
def main():
    initialize_csv()
    
    start_main = wait_for_acknowledgment("Start Main")
    
    while start_main:
        print("\nMain Menu:")
        print("1. Store a Book")
        print("2. Retrieve a Book")
        print("3. Exit")
        choice = input("Enter your choice: ").strip()

        if choice == "1":
            book_id = input("Enter Book ID: ")
            thickness = int(float(input("Enter Book Thickness: ")))
            automate_storage(book_id, thickness)
        elif choice == "2":
            book_id = input("Enter Book ID to Retrieve: ")
            automate_retrieval(book_id)
        elif choice == "3":
            print("Exiting...")
            break
        else:
            print("Invalid choice. Try again.")

if __name__ == "__main__":
    main()
