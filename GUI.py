import tkinter as tk
from tkinter import simpledialog
from tkinter import messagebox

class AutonomousRobotApp(tk.Tk):
    def __init__(self):
        super().__init__()

        # Set the title of the main window
        self.title("Autonomous Robot Control")
        
        # Create a label with instructions
        self.label = tk.Label(self, text="Tap anywhere to enter a room number", font=("Helvetica", 16))
        self.label.pack(pady=20)
        
        # Bind the click event to the handle_click function
        self.bind("<Button-1>", self.handle_click)
        
    def handle_click(self, event):
        # Open a dialog to ask the user for a room number
        room_number = simpledialog.askstring("Room Number", "Please enter the room number:")
        
        # Check if the user entered a room number
        if room_number:
            try:
                # Convert the entered room number to an integer
                room_number = int(room_number)
                
                # Check if the room number is within the allowed ranges (301-327 or 351-368)
                if (301 <= room_number <= 327) or (351 <= room_number <= 368):
                    # Display the entered room number in the label
                    self.label.config(text=f"Room Number: {room_number}")
                    
                    # Handle the room number (e.g., send it to the robot's navigation system)
                    self.navigate_to_room(room_number)
                else:
                    # Show an error message if the room number is outside the allowed ranges
                    messagebox.showerror("Error", "Invalid room number. Please enter a room number between 301-327 or 351-368.")
                
            except ValueError:
                # Handle invalid input gracefully
                messagebox.showerror("Error", "Invalid room number. Please enter a numeric value.")
                
        else:
            # If no room number was entered, display a different message
            self.label.config(text="No room number entered")
            
    def navigate_to_room(self, room_number):
        # This function can be used to handle the room number entry
        # For example, you could send the room number to the robot's navigation system
        # In this example, we simply print the room number to the console
        print(f"Navigating to room number: {room_number}")
        # Provide user feedback when navigation starts
        messagebox.showinfo("Navigation", f"Navigating to room number: {room_number}")
        # Implement the robot's navigation logic here

# Create an instance of the AutonomousRobotApp class
app = AutonomousRobotApp()

# Run the application
app.mainloop()