import os

def find_file(root_folder, filename):
    for root, dirs, files in os.walk(root_folder):
        if filename in files:
            return os.path.join(root, filename)
    return None

file_path = find_file('.', 'sensor_data.csv')
if file_path:
    print(f"Found at: {file_path}")
else:
    print("File not found.")
