import math

def reward_function(params):
    '''
    Advanced reward function for Rogue Circuit:
    - Maximizes speed on straights (target: 3.8–4.0)
    - Encourages controlled speeds in curves (target: 2.3–2.9)
    - Strongly penalizes off-track and edge cases
    - Optimizes for center-line driving and smooth steering
    - Rewards lap efficiency and completion
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
    MAX_SPEED = 4.0
    CURVE_SPEED = 2.6
    STEERING_LIMIT = 30.0
    MIN_REWARD = 1e-5

    # Initialize reward
    reward = 1.0

    # --- 1. Hard penalty for off-track ---
    if not all_wheels_on_track or distance_from_center >= track_width / 2:
        return MIN_REWARD

    # --- 2. Center-line reward ---
    marker_1 = 0.1 * track_width
    marker_2 = 0.25 * track_width
    marker_3 = 0.4 * track_width

    if distance_from_center <= marker_1:
        reward *= 2.0
    elif distance_from_center <= marker_2:
        reward *= 1.5
    elif distance_from_center <= marker_3:
        reward *= 1.0
    else:
        reward *= 0.3

    # --- 3. Steering penalty ---
    steering_penalty = 1.0 - (steering_angle / STEERING_LIMIT) ** 2
    steering_penalty = max(0.5, steering_penalty)
    reward *= steering_penalty

    # --- 4. Track curvature detection ---
    next_point = waypoints[closest_waypoints[1]]
    prev_point = waypoints[closest_waypoints[0]]
    track_direction = math.degrees(math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0]))
    direction_diff = abs(track_direction - heading)
    if direction_diff > 180:
        direction_diff = 360 - direction_diff

    if closest_waypoints[1] + 1 < len(waypoints):
        next_next_point = waypoints[closest_waypoints[1] + 1]
        dx1 = next_point[0] - prev_point[0]
        dy1 = next_point[1] - prev_point[1]
        dx2 = next_next_point[0] - next_point[0]
        dy2 = next_next_point[1] - next_point[1]
        angle_diff = abs(math.degrees(math.atan2(dy2, dx2) - math.atan2(dy1, dx1)))
        if angle_diff > 180:
            angle_diff = 360 - angle_diff
    else:
        angle_diff = direction_diff

    # --- 5. Speed optimization based on curvature ---
    is_curve = angle_diff > 15 or direction_diff > 10
    if is_curve:
        if CURVE_SPEED - 0.3 <= speed <= CURVE_SPEED + 0.3:
            reward *= 2.0
        elif speed > CURVE_SPEED + 0.3:
            reward *= 0.7
        else:
            reward *= 0.9
    else:
        if speed >= MAX_SPEED - 0.2:
            reward *= 2.5
        elif speed >= MAX_SPEED - 0.5:
            reward *= 1.5
        else:
            reward *= 0.8

    # --- 6. Heading alignment ---
    if direction_diff < 5:
        reward *= 1.3
    elif direction_diff > 20:
        reward *= 0.7

    # --- 7. Lap efficiency ---
    if steps > 0:
        efficiency = progress / steps
        reward += efficiency * 15.0

    # --- 8. Lap completion bonus ---
    if progress >= 100:
        reward += 100.0

    reward = max(MIN_REWARD, reward)
    return float(reward)