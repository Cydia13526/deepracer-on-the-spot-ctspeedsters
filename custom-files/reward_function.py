import math

def reward_function(params):
    """
    Optimized reward function for AWS DeepRacer on re:Invent 2018 track with K1999 racing line.
    """
    # Input parameters
    track_width = params['track_width']
    steering_angle = abs(params['steering_angle'])
    speed = params['speed']
    steps = params['steps']
    progress = params['progress']
    all_wheels_on_track = params['all_wheels_on_track']
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    heading = params['heading']
    is_offtrack = params['is_offtrack']
    x = params['x']
    y = params['y']

    # Constants
    MAX_SPEED = 4.0
    MIN_SPEED = 1.0
    DIRECTION_THRESHOLD = 5.0
    BENCHMARK_STEPS = 170
    BENCHMARK_TIME = 9.0  # Tightened to push for faster laps
    CURVATURE_THRESHOLD = 0.08  # Adjusted based on racing line (0.0665–0.12558)

    # K1999 Racing Line with updated speeds from action space
    RACING_LINE = [
        [-2.29464, -5.88662, 4.0, 0.06628], [-2.03416, -5.93312, 4.0, 0.06615],
        [-1.77324, -5.97635, 4.0, 0.06612], [-1.5115, -6.01641, 4.0, 0.0662],
        [-1.24857, -6.05332, 3.662, 0.06638], [-0.98419, -6.08703, 3.2445, 0.06663],
        [-0.71816, -6.11743, 2.8192, 0.06694], [-0.45051, -6.14452, 4.0, 0.06726],
        [-0.1812, -6.16799, 2.8253, 0.06758], [0.08969, -6.18741, 4.0, 0.0679],
        [0.36205, -6.20218, 2.5, 0.06819], [0.63563, -6.21144, 2.6485, 0.06843],
        [0.91003, -6.21403, 3.4038, 0.0686], [1.18467, -6.2085, 4.0, 0.06973],
        [1.45868, -6.19302, 4.0, 0.07842], [1.73083, -6.16529, 3.1516, 0.08818],
        [1.99945, -6.12237, 3.8807, 0.10147], [2.26224, -6.06074, 3.6984, 0.1155],
        [2.51549, -5.97515, 4.0, 0.11439], [2.75339, -5.86013, 2.5, 0.10111],
        [2.97699, -5.72341, 2.8296, 0.09498], [3.18659, -5.56848, 2.6376, 0.08912],
        [3.38292, -5.39827, 4.0, 0.08433], [3.5667, -5.21494, 4.0, 0.08142],
        [3.73825, -5.01988, 4.0, 0.07938], [3.89775, -4.81407, 3.4748, 0.07781],
        [4.04527, -4.59832, 4.0, 0.08112], [4.18081, -4.37326, 4.0, 0.08787],
        [4.30237, -4.13812, 3.6801, 0.0915], [4.40682, -3.89181, 3.2523, 0.09248],
        [4.4918, -3.63437, 2.6283, 0.07195], [4.56471, -3.37077, 2.5, 0.06838],
        [4.6278, -3.10244, 4.0, 0.06891], [4.68312, -2.83055, 3.7259, 0.06936],
        [4.73224, -2.55594, 4.0, 0.06974], [4.77662, -2.27971, 4.0, 0.06994],
        [4.81686, -2.00759, 2.5, 0.06877], [4.85961, -1.73686, 4.0, 0.06852],
        [4.90695, -1.46797, 2.5, 0.06826], [4.96161, -1.20204, 3.058, 0.07728],
        [5.02675, -0.94041, 3.0607, 0.08795], [5.10579, -0.6846, 2.6775, 0.09979],
        [5.20264, -0.43654, 3.9095, 0.09925], [5.32233, -0.199, 3.4946, 0.08674],
        [5.45937, 0.03006, 3.4342, 0.07802], [5.61037, 0.25193, 3.0316, 0.07244],
        [5.77332, 0.46733, 3.3412, 0.06752], [5.94644, 0.6768, 4.0, 0.06794],
        [6.12822, 0.88062, 4.0, 0.06828], [6.31722, 1.0789, 3.0531, 0.06848],
        [6.51205, 1.27164, 2.5, 0.06851], [6.69662, 1.46477, 2.5, 0.06964],
        [6.87283, 1.6621, 4.0, 0.07992], [7.03791, 1.86487, 4.0, 0.0894],
        [7.18837, 2.07443, 4.0, 0.09922], [7.32053, 2.29185, 4.0, 0.11029],
        [7.42991, 2.51794, 4.0, 0.12558], [7.5106, 2.75306, 4.0, 0.12429],
        [7.55291, 2.9964, 4.0, 0.12339], [7.55651, 3.24318, 4.0, 0.11817],
        [7.52435, 3.48859, 4.0, 0.10703], [7.46312, 3.72954, 4.0, 0.09844],
        [7.3777, 3.96444, 4.0, 0.09396], [7.27157, 4.19234, 4.0, 0.0945],
        [6.99896, 4.61992, 4.0, 0.08408], [6.83829, 4.81894, 4.0, 0.0822],
        [6.66364, 5.0076, 4.0, 0.08041], [6.47625, 5.18554, 4.0, 0.07921],
        [6.27713, 5.35235, 4.0, 0.07966], [6.06685, 5.50722, 4.0, 0.09285],
        [5.84606, 5.64934, 4.0, 0.09465], [5.61253, 5.77295, 4.0, 0.09524],
        [5.36807, 5.87599, 4.0, 0.08774], [5.11616, 5.96078, 4.0, 0.0824],
        [4.8589, 6.02904, 4.0, 0.07893], [4.59771, 6.08185, 4.0, 0.07542],
        [4.33369, 6.12039, 4.0, 0.07265], [4.06762, 6.14557, 4.0, 0.07082],
        [3.80009, 6.15798, 4.0, 0.06976], [3.53159, 6.15794, 4.0, 0.06909],
        [3.26254, 6.14572, 4.0, 0.07079], [2.99334, 6.12115, 4.0, 0.07278],
        [2.72447, 6.08358, 4.0, 0.08116], [2.45647, 6.03224, 4.0, 0.08912],
        [2.19034, 5.96378, 4.0, 0.08975], [1.92776, 5.87471, 4.0, 0.07946],
        [1.66884, 5.76966, 4.0, 0.07024], [1.41315, 5.65217, 4.0, 0.07035],
        [1.15993, 5.52562, 4.0, 0.07077], [0.90855, 5.39249, 4.0, 0.07111],
        [0.65838, 5.25506, 4.0, 0.07136], [0.40881, 5.11551, 4.0, 0.07148],
        [0.15308, 4.97464, 4.0, 0.07299], [-0.1039, 4.83589, 4.0, 0.07301],
        [-0.36233, 4.69975, 4.0, 0.07302], [-0.62241, 4.56675, 4.0, 0.07303],
        [-0.88441, 4.43759, 4.0, 0.07303], [-1.14855, 4.31299, 4.0, 0.07302],
        [-1.41498, 4.19346, 4.0, 0.073], [-1.68372, 4.07935, 4.0, 0.07299],
        [-1.9547, 3.97068, 4.0, 0.07299], [-2.22782, 3.86743, 4.0, 0.073],
        [-2.5029, 3.76934, 4.0, 0.07301], [-2.7796, 3.67572, 4.0, 0.07303],
        [-3.0576, 3.58583, 4.0, 0.07304], [-3.33656, 3.49877, 4.0, 0.07306],
        [-3.61617, 3.41372, 4.0, 0.07307], [-3.89621, 3.32998, 4.0, 0.07307],
        [-4.16772, 3.24968, 4.0, 0.07078], [-4.43809, 3.16784, 4.0, 0.07062],
        [-4.70642, 3.08314, 4.0, 0.07034], [-4.97169, 2.9941, 4.0, 0.06995],
        [-5.23291, 2.89915, 4.0, 0.06949], [-5.48901, 2.79655, 4.0, 0.07903],
        [-5.73834, 2.68383, 4.0, 0.08883], [-5.97875, 2.55813, 4.0, 0.10107],
        [-6.20761, 2.41639, 4.0, 0.11594], [-6.4207, 2.25449, 4.0, 0.11526],
        [-6.61098, 2.06759, 4.0, 0.10413], [-6.7808, 1.86203, 4.0, 0.09593],
        [-6.93208, 1.64195, 4.0, 0.08892], [-7.0666, 1.4105, 4.0, 0.08301],
        [-7.18591, 1.17005, 4.0, 0.08004], [-7.29061, 0.92209, 4.0, 0.07783],
        [-7.3811, 0.66787, 4.0, 0.07734], [-7.45721, 0.40833, 4.0, 0.07523],
        [-7.51945, 0.14463, 4.0, 0.07468], [-7.56779, -0.12234, 4.0, 0.07358],
        [-7.60256, -0.39164, 4.0, 0.07401], [-7.62356, -0.66248, 4.0, 0.07446],
        [-7.63057, -0.93401, 4.0, 0.07463], [-7.62344, -1.20537, 4.0, 0.07459],
        [-7.60213, -1.47571, 4.0, 0.07428], [-7.56687, -1.74424, 4.0, 0.07214],
        [-7.51853, -2.0104, 4.0, 0.07052], [-7.45782, -2.27373, 4.0, 0.06923],
        [-7.3857, -2.53394, 4.0, 0.07223], [-7.30195, -2.79044, 4.0, 0.07721],
        [-7.20571, -3.04236, 4.0, 0.08445], [-7.09548, -3.28832, 4.0, 0.0844],
        [-6.96899, -3.52613, 4.0, 0.08651], [-6.82834, -3.75573, 4.0, 0.09686],
        [-6.67173, -3.97483, 4.0, 0.10239], [-6.49599, -4.17939, 4.0, 0.10252],
        [-6.30032, -4.36587, 4.0, 0.09879], [-6.08776, -4.53391, 4.0, 0.08788],
        [-5.86301, -4.68636, 4.0, 0.08053], [-5.62886, -4.82512, 4.0, 0.07418],
        [-5.38743, -4.95181, 4.0, 0.06853], [-5.14041, -5.06787, 4.0, 0.06823],
        [-4.8891, -5.17452, 4.0, 0.06825], [-4.63451, -5.27273, 4.0, 0.06822],
        [-4.37752, -5.36345, 4.0, 0.06813], [-4.11878, -5.44734, 4.0, 0.068],
        [-3.85885, -5.52499, 4.0, 0.06782], [-3.59822, -5.59691, 4.0, 0.06759],
        [-3.33727, -5.6636, 4.0, 0.06733], [-3.07632, -5.72553, 4.0, 0.06705],
        [-2.81556, -5.78311, 4.0, 0.06676], [-2.55503, -5.83671, 4.0, 0.0665]
    ]

    # Initialize reward
    reward = 1.0

    # Distance to racing line
    def distance_to_racing_line(car_x, car_y, racing_line):
        min_distance = float('inf')
        closest_index = 0
        for i, point in enumerate(racing_line):
            dist = math.sqrt((car_x - point[0])**2 + (car_y - point[1])**2)
            if dist < min_distance:
                min_distance = dist
                closest_index = i
        return min_distance, closest_index

    racing_line_distance, closest_index = distance_to_racing_line(x, y, RACING_LINE)
    max_racing_line_distance = track_width / 2

    # Reward for racing line adherence
    if racing_line_distance <= 0.1 * max_racing_line_distance:
        reward *= 3.0
    elif racing_line_distance <= 0.3 * max_racing_line_distance:
        reward *= 2.0
    elif racing_line_distance <= 0.5 * max_racing_line_distance:
        reward *= 1.0
    else:
        reward *= 0.5  # Softer penalty for slight deviations

    # Speed management
    target_speed = RACING_LINE[closest_index][2]
    curvature = RACING_LINE[closest_index][3]
    speed_diff = abs(speed - target_speed)
    if speed_diff <= 0.5:
        reward *= 1.5
    elif speed_diff <= 1.0:
        reward *= 1.0
    else:
        reward *= 0.7

    if curvature < CURVATURE_THRESHOLD:
        if speed >= 3.8:
            reward *= 2.0  # Strong incentive for max speed on straights
        elif speed < MIN_SPEED:
            reward *= 0.3  # Harsh penalty for being too slow
    else:
        if speed <= min(target_speed, 2.5):  # Adjusted for sharp turns (2.0 m/s)
            reward *= 1.3
        elif speed > target_speed + 0.5:
            reward *= 0.7

    # Track direction alignment
    next_point = waypoints[closest_waypoints[1]]
    prev_point = waypoints[closest_waypoints[0]]
    track_direction = math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0])
    track_direction = math.degrees(track_direction)
    direction_diff = abs(track_direction - heading)
    if direction_diff > DIRECTION_THRESHOLD:
        direction_penalty = 1 - (direction_diff / 30.0)
        direction_penalty = max(0, min(1, direction_penalty))
        reward *= direction_penalty
    else:
        reward *= 1.2

    # Track completion
    if progress == 100:
        lap_time = steps / 15.0
        if lap_time < BENCHMARK_TIME:
            reward += 200 * (BENCHMARK_TIME / lap_time)  # Higher reward for beating 9s
        else:
            reward += 100
    elif is_offtrack:
        reward -= 50 + 10 * speed  # Speed-based off-track penalty

    # Progress-based reward
    if (steps % 10) == 0:
        expected_progress = (steps / BENCHMARK_STEPS) * 100
        progress_diff = progress - expected_progress
        reward += 0.5 * progress_diff  # Smooth progress reward

    # Dynamic steering penalty
    if curvature < CURVATURE_THRESHOLD and steering_angle > 15:
        reward *= 0.8  # Penalize high steering on straights
    elif curvature >= CURVATURE_THRESHOLD and steering_angle > 25:
        reward *= 0.9  # Lighter penalty in turns

    # All wheels on track
    if not all_wheels_on_track:
        reward *= 0.5

    return float(reward)