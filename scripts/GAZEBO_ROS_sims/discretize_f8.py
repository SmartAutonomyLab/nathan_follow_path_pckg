import numpy as np

def generate_figure_eight_points(radius=2, num_points=20, num_laps=1):
    """
    Generates discretized points for a figure-eight curve (lemniscate of Gerono).

    Parameters:
    - num_points: Number of points to generate.
    - period: Desired period (time ofr one figure 8 completion).

    Returns:
    - List of NumPy arrays representing the points on the figure-eight curve.
    """
    # Parameter t
    t = np.linspace(0, 2 * num_laps * np.pi, num_points*num_laps)

    # Parametric equations for the lemniscate of Gerono
    x = radius * np.cos(     (t - np.pi/2) )
    y = radius * np.sin( 2 * (t - np.pi/2) ) 

    # Combine x and y into an array of points
    points = np.vstack((x, y)).T

    # Convert the array of points into a list of NumPy arrays
    points_list = [np.array([point[0], point[1]]) for point in points]

    return points_list

# Generate discretized points for the figure-eight curve
figure_eight_points = generate_figure_eight_points(radius=3, num_laps=1)

# Convert the list of points to a NumPy array for saving
figure_eight_points_array = np.array(figure_eight_points).T

# Save the points as an npy file
np.save('figure_eight_points.npy', figure_eight_points_array)

# Print the first few points to verify
for point in figure_eight_points[:10]:
    print(point)

print("Points saved to 'figure_eight_points.npy'.")

# Load the list of points from the .npy file
figure_eight_points_list = np.load('figure_eight_points.npy', allow_pickle=True)

# Convert the loaded array back to a list of NumPy arrays
figure_eight_points_list = [np.array(point) for point in figure_eight_points_list]