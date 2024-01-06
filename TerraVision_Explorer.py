import math
from PIL import Image, ImageDraw
import numpy as np
import argparse

import queue
import collections





terrain_cost = {
    'Open land': 1.0,             # Open land is easy to traverse
    'Rough meadow': 1.5,         # Rough meadow is slightly harder than open land
    'Easy movement forest': 1.2, # Easier than a rough meadow, but harder than open land
    'Slow run forest': 2.0,      # It's hard to run through a dense forest
    'Walk forest': 2.5,          # Even harder than the slow run forest
    'Impassible vegetation': 10000, # Nearly impossible to traverse, so high penalty
    'Lake/Swamp/Marsh': 1000,    # It's not ideal to traverse water bodies, so high penalty
    'Paved road': 0.5,           # Roads are easier to traverse
    'Footpath': 0.8,             # Not as easy as a paved road, but better than open land
    'Out of bounds': 100000      # Forbidden area, extremely high penalty
}


terrain_type = {
    'Open land': (248, 148, 18),
    'Rough meadow': (255, 192, 0),
    'Easy movement forest': (255, 255, 255),
    'Slow run forest': (2, 208, 60),
    'Walk forest': (2, 136, 40),
    'Impassible vegetation': (5, 73, 24),
    'Lake/Swamp/Marsh': (0, 0, 255),
    'Paved road': (71, 51, 3),
    'Footpath': (0, 0, 0),
    'Out of bounds': (205, 0, 101)
}



def calculate_hauristic(current_node_coordinate,goal_node_coordinate):
    dx = (current_node_coordinate[0] - goal_node_coordinate[0]) * 10.29
    dy = (current_node_coordinate[1] - goal_node_coordinate[1]) * 7.55
    return math.sqrt(dx ** 2 + dy ** 2)


WIDTH = 395
HEIGHT = 500

def get_neighbors_coordinates(x, y):

    neighbors = [
        (x - 1, y),  # Left
        (x + 1, y),  # Right
        (x, y - 1),  # Up
        (x, y + 1)   # Down
    ]
    return [n for n in neighbors if 0 <= n[0] < WIDTH and 0 <= n[1] < HEIGHT]


def generate_terrain_type(x, y):
    coordinate = (x, y)
    pixel_rgb = image.getpixel(coordinate)

    if len(pixel_rgb) == 4:
        r, g, b, _ = pixel_rgb
    else:
        r, g, b = pixel_rgb

    temp = (r, g, b)
    for k, v in terrain_type.items():
        if temp == v:
            return k
    return None


def calculate_g_score(current_node_coordinate, parent_coordinate):
    t_type = generate_terrain_type(current_node_coordinate[0], current_node_coordinate[1])
    terrain_c = terrain_cost[t_type]

    elevation_difference = elevations[int(current_node_coordinate[0])][int(current_node_coordinate[1])] - elevations[int(parent_coordinate[0])][int(parent_coordinate[1])]
    elevation_cost = abs(elevation_difference)

    g_score = terrain_c + elevation_cost  # Just terrain and elevation cost now
    return g_score






import heapq

def a_star(start, goal):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: calculate_hauristic(start, goal)}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path, f_score[goal]

        for neighbor in get_neighbors_coordinates(current[0],current[1]):
            tentative_g_val = calculate_g_score(current, neighbor)
            tentative_g_score = g_score[current] + tentative_g_val

            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + calculate_hauristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None, float('inf')

















# terrain-image, elevation-file, path-file, output-image-filename.
def main():
    global image, elevations  # <-- Add this line
    parser = argparse.ArgumentParser()
    parser.add_argument("terrain_image")
    parser.add_argument("elevation_file")
    parser.add_argument("path_file")
    parser.add_argument("output")
    args = parser.parse_args()
    elevations = np.loadtxt(args.elevation_file, dtype=float)
    elevations = elevations[:, :-5].reshape(395, 500)
    image = Image.open(args.terrain_image)
    with open(args.path_file, 'r') as file:
        goal_points = file.read().splitlines()
        goal_point_tuples = []
        for coordinate in goal_points:
            if coordinate:
                (x, y) = coordinate.split()
                x = int(x)
                y = int(y)
                goal_point_tuples.append((x, y))
    result_path = []
    total_distance=0
    accumulate_distance=0
    for i in range(len(goal_point_tuples) - 1):
        current_optimized_path, segment_distance = a_star(goal_point_tuples[i], goal_point_tuples[i + 1])
        total_distance += segment_distance
        if current_optimized_path:
            result_path.append(current_optimized_path)
    path_color = (200, 100, 230)
    path_width = 1
    draw = ImageDraw.Draw(image)
    for closed_set in result_path:
        for i in range(len(closed_set) - 1):
            start_point = closed_set[i]
            end_point = closed_set[i + 1]
            if isinstance(start_point, tuple) and isinstance(end_point, tuple):
                draw.line([start_point, end_point], fill=path_color, width=path_width)
    # total_distance = 0
    # previous_point = None
    #
    # # Loop through the path
    # for closed_set in result_path:
    #     for point in closed_set:
    #         # Skip the first point since there's no previous point to calculate distance
    #         if previous_point is not None:
    #             # Calculate Euclidean distance
    #             dx = (point[0] - previous_point[0]) * 10.29
    #             dy = (point[1] - previous_point[1]) * 7.55
    #             segment_distance = math.sqrt(dx ** 2 + dy ** 2)
    #             total_distance += segment_distance
    #
    #         # Update previous_point
    #         previous_point = point
    # Initialize variables
    total_distance = 0
    previous_point = None

    # Loop through the path
    for closed_set in result_path:
        for point in closed_set:
            # Skip the first point since there's no previous point to calculate distance
            if previous_point is not None:
                # Calculate Euclidean distance
                dx = (point[0] - previous_point[0]) * 10.29
                dy = (point[1] - previous_point[1]) * 7.55
                segment_distance = math.sqrt(dx ** 2 + dy ** 2)

                # # Calculate elevation difference and add to segment_distance
                # elevation_difference = elevations[point[0]][point[1]] - elevations[previous_point[0]][previous_point[1]]
                # elevation_cost = abs(elevation_difference)
                # segment_distance += elevation_cost

                total_distance += segment_distance

            # Update previous_point
            previous_point = point

    # print("Total Distance Traveled:", total_distance)

    print(total_distance)

    image.save(args.output)
    image.close()


if __name__ == "__main__":
    main()

