import math

def reward_function(params):
    # Input parameters
    track_width = params['track_width']
    distance_from_center = params['distance_from_center']
    steering_angle = params['steering_angle']
    speed = params['speed']
    steps = params['steps']
    progress = params['progress']
    all_wheels_on_track = params['all_wheels_on_track']
    is_offtrack = params['is_offtrack']
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    heading = params['heading']
    x = params['x']
    y = params['y']

    # Thresholds and constants
    SPEED_THRESHOLD_STRAIGHT = 3.0  # High speed for straights
    SPEED_THRESHOLD_TURN = 1.5      # Lower speed for turns
    DIRECTION_THRESHOLD = 10.0       # Max acceptable heading difference (degrees)
    BENCHMARK_TIME = 9.5            # Target lap time (seconds)
    BENCHMARK_STEPS = 142           # Approx. steps for 9.5s (15 steps/sec)
    STRAIGHT_WAYPOINTS = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,
                          43,44,45,46,47,48,49,50,56,57,58,59,60,61,62,63,64,
                          71,72,73,74,75,76,77,78,89,90,91,92,93,94,95,96,97,98,99,
                          100,101,102,103,112,113,114,115,116,117]  # From Dante’s function

    # Initialize reward
    reward = 1.0

    # Calculate track direction
    next_point = waypoints[closest_waypoints[1]]
    prev_point = waypoints[closest_waypoints[0]]
    track_direction = math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0])
    track_direction = math.degrees(track_direction)
    direction_diff = abs(track_direction - heading)

    # K1999-inspired racing line reward (simplified)
    # Assume K1999 provides optimal waypoints or distance_from_optimal_line
    marker_1 = 0.1 * track_width
    marker_2 = 0.25 * track_width
    marker_3 = 0.5 * track_width
    if distance_from_center <= marker_1:
        reward *= 2.0
    elif distance_from_center <= marker_2:
        reward *= 1.5
    elif distance_from_center <= marker_3:
        reward *= 1.0
    else:
        reward *= 0.01  # Near crash/off-track

    # Speed optimization based on track section
    if closest_waypoints[1] in STRAIGHT_WAYPOINTS:
        if speed >= SPEED_THRESHOLD_STRAIGHT:
            reward += 2.0 * (speed / SPEED_THRESHOLD_STRAIGHT)
        else:
            reward += 0.5 * (speed / SPEED_THRESHOLD_STRAIGHT)
    else:
        if speed <= SPEED_THRESHOLD_TURN:
            reward += 2.0 * (SPEED_THRESHOLD_TURN / speed)
        else:
            reward += 0.5 * (SPEED_THRESHOLD_TURN / speed)

    # Heading alignment
    if direction_diff <= DIRECTION_THRESHOLD:
        reward += 1.0
    else:
        penalty = 1.0 - (direction_diff / 30.0)
        reward *= max(0.1, penalty)

    # Progress and completion rewards
    if progress == 100:
        if steps / 15.0 <= BENCHMARK_TIME:
            reward += 100.0 * (BENCHMARK_TIME / (steps / 15.0))
        else:
            reward += 50.0
    elif is_offtrack or not all_wheels_on_track:
        reward *= 0.001

    # Milestone rewards every 50 steps
    expected_progress = (steps / BENCHMARK_STEPS) * 100
    if (steps % 50) == 0:
        if progress >= expected_progress:
            reward += 10.0
        else:
            reward -= 5.0

    return float(max(reward, 1e-3))  # Ensure non-negative reward