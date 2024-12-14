import csv
import os
import matplotlib.pyplot as plt

# Bookshelf properties
LAYERS = 2
HEIGHT_DIFFERENCE = 100.0  # Height difference between layers in z-axis (vertical)
LENGTH = 1000.0  # Length of each layer in x-axis (horizontal)

# File to store book coordinates
BOOK_COORDINATES_FILE = "book_coordinates.csv"


def initialize_csv():
    """Initializes the CSV file for storing book data."""
    if not os.path.exists(BOOK_COORDINATES_FILE):
        with open(BOOK_COORDINATES_FILE, mode="w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["BookID", "Layer", "X_Coordinate", "Thickness"])

def read_occupied_positions():
    """Reads occupied positions from the CSV file."""
    occupied_positions = [[] for _ in range(LAYERS)]

    if os.path.exists(BOOK_COORDINATES_FILE):
        with open(BOOK_COORDINATES_FILE, mode="r") as file:
            reader = csv.reader(file)
            next(reader)  # Skip the header
            for row in reader:
                layer = int(row[1]) - 1  # Convert to zero-based index
                x_start = float(row[2])
                thickness = float(row[3])
                x_end = x_start + thickness
                occupied_positions[layer].append((x_start, x_end))
    return occupied_positions


def find_gaps(layer, layer_length):
    """Finds gaps in a layer by reading occupied positions from the CSV file.

    Args:
        layer (int): The layer index (zero-based).
        layer_length (float): The total length of the layer.

    Returns:
        list of tuples: Each tuple contains the starting point and thickness of a gap.
    """
    occupied = []
    if os.path.exists(BOOK_COORDINATES_FILE):
        with open(BOOK_COORDINATES_FILE, mode="r") as file:
            reader = csv.reader(file)
            next(reader)  # Skip the header
            for row in reader:
                if int(row[1]) - 1 == layer:  # Check if the entry belongs to the requested layer
                    x_start = float(row[2])
                    thickness = float(row[3])
                    x_end = x_start + thickness
                    occupied.append((x_start, x_end))

    # Sort occupied positions by starting point
    occupied.sort()

    gaps = []
    current_position = 0.0

    for x_start, x_end in occupied:
        if x_start > current_position:
            # There is a gap before the next occupied space
            gaps.append((current_position, x_start - current_position))
        # Move current position to the end of the current occupied space
        current_position = max(current_position, x_end)

    # Check for a gap at the end of the layer
    if current_position < layer_length:
        gaps.append((current_position, layer_length - current_position))

    return gaps

def store_book(book_id: str, thickness: float):
    """Stores a book on the shelf."""
    for layer_index in range(LAYERS):
        gaps = find_gaps(layer_index, LENGTH)

        for gap_start, gap_thickness in gaps:
            if gap_thickness >= thickness:
                # Place the book in the first suitable gap
                x_start = gap_start

                # Save the book details in the CSV file
                with open(BOOK_COORDINATES_FILE, mode="a", newline="") as file:
                    writer = csv.writer(file)
                    writer.writerow([book_id, layer_index + 1, round(x_start, 2), round(thickness, 2)])

                print(f"Book '{book_id}' stored at Layer {layer_index + 1}, X: {x_start:.2f}, Thickness: {thickness:.2f}")
                return

    print("No space available to store the book.")

def retrieve_book(book_id: str):
    """Retrieves a book from the shelf."""
    rows = []
    book_found = False

    if not os.path.exists(BOOK_COORDINATES_FILE):
        print("No bookshelf data found.")
        return

    with open(BOOK_COORDINATES_FILE, mode="r") as file:
        reader = csv.reader(file)
        header = next(reader)
        for row in reader:
            if row[0] == book_id:
                book_found = True
            else:
                rows.append(row)

    if not book_found:
        print(f"Book '{book_id}' not found.")
        return

    with open(BOOK_COORDINATES_FILE, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(header)
        writer.writerows(rows)

    print(f"Book '{book_id}' successfully retrieved.")


def visualize_bookshelf():
    """Visualizes the bookshelf, showing stored books and empty spaces."""
    fig, ax = plt.subplots(figsize=(10, 5))

    for layer_index in range(LAYERS):
        layer_y = LAYERS - layer_index  # Plot layer index in reverse for visualization
        occupied = []
        gaps = find_gaps(layer_index, LENGTH)

        # Draw the layer base
        ax.plot([0, LENGTH], [layer_y, layer_y], color='black', linewidth=2)

        # Draw occupied spaces
        if os.path.exists(BOOK_COORDINATES_FILE):
            with open(BOOK_COORDINATES_FILE, mode="r") as file:
                reader = csv.reader(file)
                next(reader)  # Skip the header
                for row in reader:
                    if int(row[1]) - 1 == layer_index:  # Match the layer
                        x_start = float(row[2])
                        thickness = float(row[3])
                        x_end = x_start + thickness
                        ax.fill_betweenx([layer_y - 0.1, layer_y + 0.1], x_start, x_end, color='blue', label='Book' if layer_index == 0 else "")

        # Draw gaps
        for gap_start, gap_thickness in gaps:
            gap_end = gap_start + gap_thickness
            ax.fill_betweenx([layer_y - 0.1, layer_y + 0.1], gap_start, gap_end, color='lightgray', label='Empty Space' if layer_index == 0 else "")

    ax.set_title("Bookshelf Visualization")
    ax.set_xlabel("X Coordinate (Horizontal)")
    ax.set_ylabel("Layer (Height)")
    ax.set_xlim(0, LENGTH)
    ax.set_ylim(0, LAYERS + 1)
    ax.legend()
    plt.show()

if __name__ == "__main__":
    initialize_csv()

    while True:
        print("\nBookshelf Manager")
        print("1. Store a book")
        print("2. Retrieve a book")
        print("3. Visualize bookshelf")
        print("4. Exit")
        choice = input("Enter your choice: ")

        if choice == "1":
            book_id = input("Enter the Book ID: ")
            thickness = float(input("Enter the book thickness: "))
            store_book(book_id, thickness)
            visualize_bookshelf()
        elif choice == "2":
            book_id = input("Enter the Book ID: ")
            retrieve_book(book_id)
            visualize_bookshelf()
        elif choice == "3":
            visualize_bookshelf()
        elif choice == "4":
            print("Exiting...")
            break
        else:
            print("Invalid choice. Please try again.")
