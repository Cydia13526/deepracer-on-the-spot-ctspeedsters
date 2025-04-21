def reward_function(params):
    import math

    all_wheels_on_track = params['all_wheels_on_track']
    speed = params['speed']
    steering = abs(params['steering_angle'])
    track_width = params['track_width']
    distance_from_center = params['distance_from_center']
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    heading = params['heading']
    x = params['x']
    y = params['y']

    if not all_wheels_on_track:
        return 1e-3

    # Direction
    next_wp = waypoints[closest_waypoints[1]]
    prev_wp = waypoints[closest_waypoints[0]]
    track_direction = math.degrees(math.atan2(next_wp[1] - prev_wp[1],
                                              next_wp[0] - prev_wp[0]))
    direction_diff = abs(track_direction - heading)
    if direction_diff > 180:
        direction_diff = 360 - direction_diff
    direction_reward = max(0.0, 1.0 - direction_diff / 50.0)

    # Speed with direction alignment consideration
    if direction_diff < 10:
        speed_reward = speed / 3.0
    else:
        speed_reward = speed / 5.0

    # Smooth steering
    steering_penalty = max(0.5, 1.0 - (steering / 30.0) ** 2)

    # Centering
    marker = 0.5 * track_width
    if distance_from_center <= marker:
        center_penalty = 1.0
    else:
        center_penalty = max(0.0, 1 - (distance_from_center / (track_width / 2)))

    # Final reward
    reward = 1.0 * direction_reward * speed_reward * steering_penalty * center_penalty
    return float(max(reward, 1e-3))
