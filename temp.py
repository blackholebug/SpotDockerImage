import os

# The parent directory within which the new folders will be created
parent_dirs = [
    "C:\dev\SpotDockerImage\data\\actions",
    "C:\dev\SpotDockerImage\data\hololens",
    "C:\dev\SpotDockerImage\data\spot_odometry",
]

# List of folder names to be created
conditions = [
    "speech_freewalking",
    "speech_stationary",
    "gestures_freewalking",
    "gestures_stationary",
    "controller_freewalking",
    "controller_stationary",
]

# Create each directory within the parent directory
for parent_dir in parent_dirs:
    for folder in conditions:
        # Path of the new folder
        path = os.path.join(parent_dir, folder)
        
        try:
            # Create the folder
            os.makedirs(path, exist_ok=True)  # exist_ok=True allows the function to not raise an error if the folder already exists
            print(f"Directory '{folder}' created successfully")
        except OSError as error:
            print(f"Creation of the directory {folder} failed: {error}")

    # If you want to check the list of directories after creation, you can do this
    created_folders = [name for name in os.listdir(parent_dir) if os.path.isdir(os.path.join(parent_dir, name))]
    print("Folders in parent directory:", created_folders)