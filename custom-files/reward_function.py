import math

def reward_function(params):
    '''
    Strictly prevents off-track driving while optimizing speed and steering for Rogue Circuit.
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
    is_offtrack = params['is_offtrack']  # New: Direct off-track flag

    # Constants
    SPEED_THRESHOLD_STRAIGHT = 3.5  # m/s (fast on straights)
    SPEED_THRESHOLD_CURVE = 2.0     # m/s (slow on curves)
    STEERING_THRESHOLD = 20.0        # degrees (penalize sharp turns)
    MAX_DISTANCE_FROM_CENTER = 0.5 * track_width  # Hard limit (50% of track width)

    # --- STRICT OFF-TRACK PENALTY ---
    if is_offtrack or not all_wheels_on_track:
        return 1e-5  # Near-zero reward if off-track

    # --- REWARD STRUCTURE ---
    reward = 1e-3  # Minimal baseline reward

    # 1. **Center-line alignment (strict)**
    if distance_from_center <= 0.1 * track_width:
        reward += 1.0  # Max reward for perfect center
    elif distance_from_center <= 0.25 * track_width:
        reward += 0.5  # Good, but not perfect
    elif distance_from_center <= MAX_DISTANCE_FROM_CENTER:
        reward += 0.1  # Acceptable, but not ideal
    else:
        return 1e-5  # Too close to edge (effectively off-track)

    # 2. **Speed optimization (curve vs. straight)**
    next_point = waypoints[closest_waypoints[1]]
    prev_point = waypoints[closest_waypoints[0]]
    track_direction = math.degrees(math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0]))
    direction_diff = abs(track_direction - heading)

    # Slow down on curves, speed up on straights
    if direction_diff > 15:  # Sharp curve
        if speed > SPEED_THRESHOLD_CURVE:
            reward *= 0.5  # Heavy penalty for speeding in curves
        else:
            reward += 0.5  # Bonus for correct slow speed
    else:  # Straight section
        if speed > SPEED_THRESHOLD_STRAIGHT:
            reward += 1.0  # Max bonus for high speed on straights

    # 3. **Steering penalty (prevents zig-zag)**
    if steering_angle > STEERING_THRESHOLD:
        reward *= 0.5  # 50% penalty for aggressive steering

    # 4. **Lap completion bonus (huge reward)**
    if progress == 100:
        reward += 1000  # Massive reward for finishing

    return float(max(reward, 1e-5))  # Ensure reward is never zero