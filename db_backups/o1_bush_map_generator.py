import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os

def generate_test_data(file_name):
    """
    Generates a test CSV file with different scenarios to ensure the math 
    and projections work for various orientations.
    """
    data = {
        'row_id': [1, 2, 3],
        'num_objects': [5, 6, 4],
        # Row 1: Horizontal line
        'head_x': [10, 10, 20],
        'head_y': [10, 30, 70],
        'tail_x': [60, 50, 40],
        'tail_y': [10, 50, 10], 
        # Start points (intentionally offset to test projection)
        'start_x': [15, 12, 22],
        'start_y': [8, 35, 68],
        # End points (intentionally offset to test projection)
        'end_x': [55, 45, 38],
        'end_y': [12, 45, 12]
    }
    
    df = pd.DataFrame(data)
    df.to_csv(file_name, index=False)
    print(f"Test data file '{file_name}' generated successfully.")

def project_point_onto_line(head, tail, point):
    """
    Projects a point onto a line defined by two points (head and tail).
    """
    head = np.array(head)
    tail = np.array(tail)
    point = np.array(point)
    
    line_vec = tail - head
    point_vec = point - head
    
    # Calculate scalar projection
    scalar = np.dot(point_vec, line_vec) / np.dot(line_vec, line_vec)
    
    # Calculate projected point coordinates
    proj_point = head + scalar * line_vec
    return proj_point

def process_harvesting_data(file_path):
    """
    Reads the survey datatable, calculates object coordinates, and returns the final dataframe.
    """
    df = pd.read_csv(file_path)
    output_records = []
    object_id_counter = 1
    
    for _, row in df.iterrows():
        head = (row['head_x'], row['head_y'])
        tail = (row['tail_x'], row['tail_y'])
        start_pt = (row['start_x'], row['start_y'])
        end_pt = (row['end_x'], row['end_y'])
        n_objects = int(row['num_objects'])
        
        # 1. Calculate orientation degree from head to tail
        dy = tail[1] - head[1]
        dx = tail[0] - head[0]
        orientation_deg = np.degrees(np.arctan2(dy, dx))
        
        # 2. Project start and end points onto the head-tail line
        proj_start = project_point_onto_line(head, tail, start_pt)
        proj_end = project_point_onto_line(head, tail, end_pt)
        
        # 3. Interpolate evenly spaced objects between proj_start and proj_end
        if n_objects == 1:
            x_coords = [proj_start[0]]
            y_coords = [proj_start[1]]
        else:
            x_coords = np.linspace(proj_start[0], proj_end[0], n_objects)
            y_coords = np.linspace(proj_start[1], proj_end[1], n_objects)
            
        # 4. Append to output list
        for x, y in zip(x_coords, y_coords):
            output_records.append({
                'object_id': object_id_counter,
                'row_id': int(row['row_id']),
                'x_coord': round(x, 3),
                'y_coord': round(y, 3),
                'orientation_deg': round(orientation_deg, 2)
            })
            object_id_counter += 1
            
    return pd.DataFrame(output_records), df

def visualize_results(input_df, output_df):
    """
    Plots the original markers, the row lines, and the generated objects with arrows.
    """
    plt.figure(figsize=(10, 8))
    
    # 1. Plot the input data markers and row lines
    for _, row in input_df.iterrows():
        # Line (red dashed)
        plt.plot([row['head_x'], row['tail_x']], [row['head_y'], row['tail_y']], 
                 'r--', alpha=0.5, zorder=1)
        
        # Markers
        plt.scatter(row['head_x'], row['head_y'], c='red', s=80, marker='o', zorder=2, label='Head' if _ == 0 else "")
        plt.scatter(row['tail_x'], row['tail_y'], c='orange', s=80, marker='o', zorder=2, label='Tail' if _ == 0 else "")
        plt.scatter(row['start_x'], row['start_y'], c='blue', s=80, marker='x', zorder=2, label='Start (Raw)' if _ == 0 else "")
        plt.scatter(row['end_x'], row['end_y'], c='green', s=80, marker='x', zorder=2, label='End (Raw)' if _ == 0 else "")

    # 2. Plot the calculated objects as arrows using quiver
    X = output_df['x_coord'].values
    Y = output_df['y_coord'].values
    angles_rad = np.radians(output_df['orientation_deg'].values)
    
    # Calculate U and V components of the direction vector
    U = np.cos(angles_rad)
    V = np.sin(angles_rad)
    
    plt.quiver(X, Y, U, V, color='black', pivot='tail', 
               angles='xy', scale_units='xy', scale=1.0, 
               width=0.005, zorder=3, label='Objects (Projected & Spaced)')
    
    plt.title("Harvesting Area Object Placement")
    plt.xlabel("X Coordinate")
    plt.ylabel("Y Coordinate")
    plt.grid(True, linestyle=':', alpha=0.7)
    
    # Handle legends to avoid duplicates
    handles, labels = plt.gca().get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    plt.legend(by_label.values(), by_label.keys(), loc='center left', bbox_to_anchor=(1, 0.5))
    
    plt.axis('equal')
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    input_file_name = 'harvesting_area_survey.csv'
    output_file_name = 'objects_table.csv'
    
    # 1. Generate the input data file
    # generate_test_data(input_file_name)
    
    # 2. Process the data
    final_output_df, raw_input_df = process_harvesting_data(input_file_name)
    
    # 3. Save the output to a CSV file
    final_output_df.to_csv(output_file_name, index=False)
    print(f"Output table saved successfully to '{output_file_name}'.\n")
    
    # 4. Print the resulting output table to the console
    print("=== Final Calculated Object Data ===")
    print(final_output_df.to_string(index=False))
    
    # 5. Display the visual plot
    visualize_results(raw_input_df, final_output_df)