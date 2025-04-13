import math

class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = None
        self.verbose = verbose
        self.speed_history = []
        self.steering_history = []

    def reward_function(self, params):
        # Import package (needed for heading)
        import math

        ################## HELPER FUNCTIONS ###################
        def dist_2_points(x1, x2, y1, y2):
            return math.sqrt((x1-x2)**2 + (y1-y2)**2)

        def closest_2_racing_points_index(racing_coords, car_coords):
            distances = []
            for i in range(len(racing_coords)):
                distance = dist_2_points(x1=racing_coords[i][0], x2=car_coords[0],
                                         y1=racing_coords[i][1], y2=car_coords[1])
                distances.append(distance)

            closest_index = distances.index(min(distances))
            distances_no_closest = distances.copy()
            distances_no_closest[closest_index] = 999
            second_closest_index = distances_no_closest.index(min(distances_no_closest))

            return [closest_index, second_closest_index]

        def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):
            a = dist_2_points(x1=closest_coords[0], x2=second_closest_coords[0],
                              y1=closest_coords[1], y2=second_closest_coords[1])
            b = dist_2_points(x1=car_coords[0], x2=closest_coords[0],
                              y1=car_coords[1], y2=closest_coords[1])
            c = dist_2_points(x1=car_coords[0], x2=second_closest_coords[0],
                              y1=car_coords[1], y2=second_closest_coords[1])

            try:
                distance = abs(-(a**4) + 2*(a**2)*(b**2) + 2*(a**2)*(c**2) -
                               (b**4) + 2*(b**2)*(c**2) - (c**4))**0.5 / (2*a)
            except:
                distance = b
            return distance

        def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):
            heading_vector = [math.cos(math.radians(heading)), math.sin(math.radians(heading))]
            new_car_coords = [car_coords[0]+heading_vector[0], car_coords[1]+heading_vector[1]]

            distance_closest_coords_new = dist_2_points(new_car_coords[0], closest_coords[0],
                                                        new_car_coords[1], closest_coords[1])
            distance_second_closest_coords_new = dist_2_points(new_car_coords[0], second_closest_coords[0],
                                                               new_car_coords[1], second_closest_coords[1])

            if distance_closest_coords_new <= distance_second_closest_coords_new:
                next_point_coords = closest_coords
                prev_point_coords = second_closest_coords
            else:
                next_point_coords = second_closest_coords
                prev_point_coords = closest_coords

            return [next_point_coords, prev_point_coords]

        def racing_direction_diff(closest_coords, second_closest_coords, car_coords, heading):
            next_point, prev_point = next_prev_racing_point(closest_coords,
                                                            second_closest_coords,
                                                            car_coords,
                                                            heading)
            track_direction = math.degrees(math.atan2(next_point[1] - prev_point[1],
                                                      next_point[0] - prev_point[0]))
            direction_diff = abs(track_direction - heading)
            if direction_diff > 180:
                direction_diff = 360 - direction_diff
            return direction_diff

        ################## INPUT PARAMETERS ###################
        all_wheels_on_track = params['all_wheels_on_track']
        x = params['x']
        y = params['y']
        distance_from_center = params['distance_from_center']
        is_left_of_center = params['is_left_of_center']
        heading = params['heading']
        progress = params['progress']
        steps = params['steps']
        speed = params['speed']
        steering_angle = params['steering_angle']
        track_width = params['track_width']
        waypoints = params['waypoints']
        closest_waypoints = params['closest_waypoints']
        is_offtrack = params['is_offtrack']

        ################## RACING LINE ######################
        # Optimal racing line (simplified for Rogue Circuit)
        racing_track = [
            [x, y, 4.0, 0.0] for x, y in waypoints  # Using waypoints as base with max speed
        ]

        ################### REWARD CALCULATION ###############
        reward = 1.0  # Base reward

        # 1. Stay on track penalty
        if not all_wheels_on_track or is_offtrack:
            return 1e-3  # Minimal reward if off track

        # 2. Distance from center reward (5-tier system)
        markers = [0.1, 0.25, 0.5]  # Fractions of track width
        if distance_from_center <= markers[0] * track_width:
            reward += 1.0
        elif distance_from_center <= markers[1] * track_width:
            reward += 0.5
        elif distance_from_center <= markers[2] * track_width:
            reward += 0.1
        else:
            reward = 1e-3  # Near edge

        # 3. Racing line alignment
        closest_index, second_closest_index = closest_2_racing_points_index(racing_track, [x, y])
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist/(track_width*0.5)))
        reward += distance_reward * 1.5  # Increased weight for line following

        # 4. Speed optimization (dynamic based on turns)
        direction_diff = racing_direction_diff(optimals[0:2], optimals_second[0:2], [x, y], heading)

        # Speed thresholds based on turn severity
        if direction_diff < 10:  # Straight
            speed_threshold = 3.5
            speed_reward = min(speed / speed_threshold, 1.0) * 2.0
        elif direction_diff < 30:  # Mild turn
            speed_threshold = 2.5
            speed_reward = min(speed / speed_threshold, 1.0) * 1.5
        else:  # Sharp turn
            speed_threshold = 1.5
            speed_reward = min(speed / speed_threshold, 1.0)

        reward += speed_reward

        # 5. Steering penalty (smooth driving)
        STEERING_THRESHOLD = 15.0
        if abs(steering_angle) > STEERING_THRESHOLD:
            reward *= 0.8  # Penalize sharp turns

        # 6. Progress bonus (encourage completion)
        if progress == 100:
            reward += 100
        else:
            reward += progress / 100  # Small bonus for each percentage

        # 7. Consistency bonus (smooth speed changes)
        self.speed_history.append(speed)
        self.steering_history.append(steering_angle)
        if len(self.speed_history) > 5:
            speed_std = np.std(self.speed_history[-5:])
            steering_std = np.std(self.steering_history[-5:])
            if speed_std < 0.2 and steering_std < 5.0:
                reward += 0.5  # Bonus for consistent driving

        return float(reward)

reward_object = Reward()  # Initialize reward object

def reward_function(params):
    return reward_object.reward_function(params)