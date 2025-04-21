import math

def reward_function(params):
    '''
    Optimized for Rogue Circuit: Penalizes sharp turns, rewards center-line alignment, and adjusts speed dynamically.
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

    # Constants
    SPEED_THRESHOLD_STRAIGHT = 3.0  # m/s (fast on straights)
    SPEED_THRESHOLD_CURVE = 1.5     # m/s (slow on curves)
    STEERING_THRESHOLD = 15.0        # degrees (penalize sharp turns)

    # Reward baseline
    reward = 1e-3  # Minimal reward by default

    # 1. Reward for staying on track
    if all_wheels_on_track:
        reward += 1.0

    # 2. Reward for proximity to center line (5-tier system)
    markers = [0.1, 0.25, 0.5]  # Fractions of track width
    if distance_from_center <= markers[0] * track_width:
        reward += 1.0
    elif distance_from_center <= markers[1] * track_width:
        reward += 0.5
    elif distance_from_center <= markers[2] * track_width:
        reward += 0.1
    else:
        reward = 1e-3  # Near edge/crash

    # 3. Penalize excessive steering (reduces zig-zag)
    if steering_angle > STEERING_THRESHOLD:
        reward *= 0.7  # 30% penalty for sharp turns

    # 4. Dynamic speed adjustment based on track curvature
    next_point = waypoints[closest_waypoints[1]]
    prev_point = waypoints[closest_waypoints[0]]
    track_direction = math.degrees(math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0]))
    direction_diff = abs(track_direction - heading)

    # Slow down on curves, speed up on straights
    if direction_diff > 10:  # Curve detected
        if speed >= SPEED_THRESHOLD_CURVE:
            reward *= 0.8  # Penalize high speed in curves
    else:  # Straight section
        if speed >= SPEED_THRESHOLD_STRAIGHT:
            reward += 0.5  # Bonus for high speed on straights

    # 5. Bonus for lap completion
    if progress == 100:
        reward += 100

    return float(reward)