import serial
import time
import csv

# CSV File Path
csv_file = "book_coordinates.csv"

# Initialize Serial Communication with Arduino
arduino = serial.Serial('/dev/ttyS0', 9600, timeout=1)
time.sleep(2)  # Allow Arduino to reset

# Function to read books from CSV
def read_books(csv_file):
    try:
        books = {}
        with open(csv_file, mode='r') as file:
            csv_reader = csv.DictReader(file)
        
            for row in csv_reader:
                name = row['Name']
                books[name] = True
        return books
    except Exception as e:
        print(f"Error reading CSV: {e}")
        return {}

# Function to write a new book entry to the CSV
def write_book(csv_file, name):
    try:
        with open(csv_file, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([name])
        print(f"Book '{name}' saved.")
    except Exception as e:
        print(f"Error writing to CSV: {e}")

# Function to delete a book from the CSV
def delete_book(csv_file, name):
    try:
        books = read_books(csv_file)
        if name in books:
            del books[name]
            with open(csv_file, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["Name"])
                for book in books.keys():
                    writer.writerow([book])
            print(f"Book '{name}' deleted from CSV.")
        else:
            print(f"Book '{name}' not found in CSV.")
    except Exception as e:
        print(f"Error deleting book from CSV: {e}")

# Function to send a command to Arduino and wait for acknowledgment
def send_command(command):
    print(f"Sending command to Arduino: {command}")
    arduino.write(f"{command}\n".encode('utf-8'))
    time.sleep(1)
    while True:
        if arduino.in_waiting > 0:
            response = arduino.readline().decode('utf-8').strip()
            print(f"Arduino Response: {response}")
            if response == "OK":
                return response
        time.sleep(0.1)

# Storage function
def storage():
    # Step 1: Send "close" to close the gripper
    print("Closing gripper to store a book.")
    send_command("close")

    # Step 2: Record the name in the CSV
    book_name = input("Enter the name of the book: ").strip()
    write_book(csv_file, book_name)

    # Step 3: Send "open" to release the gripper
    print("Releasing the gripper...")
    send_command("open")

# Retrieve function
def retrieve():
    # Step 1: Ask for the book name
    book_name = input("Enter the name of the book to retrieve: ").strip()
    books = read_books(csv_file)

    if book_name not in books:
        print(f"Book '{book_name}' not found in CSV.")
        return

    # Step 2: Send "close" to close the gripper
    print(f"Retrieving the book '{book_name}'...")
    send_command("close")

    # Step 3: Send "open" to release the gripper
    print("Releasing the gripper...")
    send_command("open")

    # Step 4: Delete the book information from the CSV
    delete_book(csv_file, book_name)

# Main function to control the flow
def main():
    print("Starting Gripper Test System")
    while True:
        print("\nOptions:\n1. Store a book\n2. Retrieve a book\n3. Exit")
        choice = input("Enter your choice: ").strip()

        if choice == '1':
            storage()
        elif choice == '2':
            retrieve()
        elif choice == '3':
            print("Exiting program.")
            break
        else:
            print("Invalid choice. Please try again.")

# Ensure CSV has the correct headers
try:
    with open(csv_file, mode='a', newline='') as file:
        writer = csv.writer(file)
        if file.tell() == 0:  # Write headers if file is empty
            writer.writerow(["Name"])
except Exception as e:
    print(f"Error initializing CSV: {e}")

# Run the main function
if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nProgram interrupted.")
    finally:
        arduino.close()
