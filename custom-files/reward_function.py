import math

def reward_function(params):
    '''
    Fine-tuned reward function for Rogue Circuit:
    - Encourages center-line driving
    - Penalizes sharp turns and going off-track
    - Adjusts speed incentives based on curvature
    - Rewards lap efficiency
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
    SPEED_THRESHOLD_STRAIGHT = 3.0
    SPEED_THRESHOLD_CURVE = 1.6
    STEERING_LIMIT = 30.0  # For scaled penalty

    reward = 1e-3  # Default minimal reward

    # --- 1. Penalize off-track hard ---
    if not all_wheels_on_track:
        return 1e-6

    # --- 2. Distance from center line ---
    marker_1 = 0.1 * track_width
    marker_2 = 0.25 * track_width
    marker_3 = 0.5 * track_width

    if distance_from_center <= marker_1:
        reward += 1.2
    elif distance_from_center <= marker_2:
        reward += 0.6
    elif distance_from_center <= marker_3:
        reward += 0.2
    else:
        return 1e-6  # Too close to edge

    # --- 3. Scaled steering penalty ---
    steering_penalty = max(0.5, 1.0 - (steering_angle / STEERING_LIMIT))
    reward *= steering_penalty

    # --- 4. Direction vs heading ---
    next_point = waypoints[closest_waypoints[1]]
    prev_point = waypoints[closest_waypoints[0]]
    track_direction = math.degrees(math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0]))
    direction_diff = abs(track_direction - heading)
    if direction_diff > 180:
        direction_diff = 360 - direction_diff

    # --- 5. Speed reward based on track curvature ---
    if direction_diff > 10:
        # Curved section
        if speed > SPEED_THRESHOLD_CURVE:
            reward *= 0.6  # Too fast in curves
        else:
            reward += 0.3  # Good speed on curves
    else:
        # Straight section
        if speed >= SPEED_THRESHOLD_STRAIGHT:
            reward += 0.5
        else:
            reward *= 0.9

    # --- 6. Lap efficiency (progress per step) ---
    if steps > 0:
        reward += (progress / steps) * 10.0

    # --- 7. Bonus for lap completion ---
    if progress == 100:
        reward += 50.0

    return float(reward)
