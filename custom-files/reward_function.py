import math

def reward_function(params):
    '''
    Optimized reward function for Rogue Circuit:
    - Maximizes speed on straights (target: 3.8–4.0)
    - Pushes curve speeds (target: 2.7–3.0)
    - Strong penalties for off-track, edge, and high steering
    - High rewards for efficiency and completion
    '''

    # Input parameters
    track_width = params['track_width']
    distance_from_center = params['distance_from_center']
    steering_angle = abs(params['steering_angle'])
    speed = params['speed']
    progress = params['progress']
    all_wheels_on_track = params['all_wheels_on_track']
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    heading = params['heading']
    steps = params['steps']

    # Constants
    MAX_SPEED = 3.8
    CURVE_SPEED = 2.7
    STEERING_LIMIT = 30.0
    MIN_REWARD = 1e-6

    # Initialize reward
    reward = 1.0

    # --- 1. Severe off-track penalty ---
    if not all_wheels_on_track or distance_from_center >= track_width / 2:
        return MIN_REWARD

    # --- 2. Center-line reward ---
    marker_1 = 0.1 * track_width
    marker_2 = 0.2 * track_width
    marker_3 = 0.35 * track_width

    if distance_from_center <= marker_1:
        reward *= 2.5
    elif distance_from_center <= marker_2:
        reward *= 1.8
    elif distance_from_center <= marker_3:
        reward *= 1.2
    else:
        reward *= 0.2

    # --- 3. Steering penalty ---
    steering_penalty = 1.0 - (steering_angle / STEERING_LIMIT) ** 1.5
    steering_penalty = max(0.4, steering_penalty)
    reward *= steering_penalty

    # --- 4. Simplified curvature detection ---
    next_point = waypoints[closest_waypoints[1]]
    prev_point = waypoints[closest_waypoints[0]]
    track_direction = math.degrees(math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0]))
    direction_diff = abs(track_direction - heading)
    if direction_diff > 180:
        direction_diff = 360 - direction_diff

    # --- 5. Speed optimization ---
    is_curve = direction_diff > 12
    if is_curve:
        if CURVE_SPEED - 0.3 <= speed <= CURVE_SPEED + 0.3:
            reward *= 2.5
        elif speed > CURVE_SPEED + 0.3:
            reward *= 0.5
        else:
            reward *= 0.8
    else:
        if speed >= MAX_SPEED:
            reward *= 3.0
        elif speed >= MAX_SPEED - 0.4:
            reward *= 1.7
        else:
            reward *= 0.7

    # --- 6. Heading alignment ---
    if direction_diff < 5:
        reward *= 1.5
    elif direction_diff > 15:
        reward *= 0.6

    # --- 7. Lap efficiency ---
    if steps > 0:
        efficiency = progress / steps
        reward += efficiency * 20.0

    # --- 8. Lap completion bonus ---
    if progress >= 100:
        reward += 100.0

    reward = max(MIN_REWARD, reward)
    return float(reward)