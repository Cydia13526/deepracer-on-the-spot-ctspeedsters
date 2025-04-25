import math

class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = None
        self.verbose = verbose

    def reward_function(self, params):
        ################## HELPER FUNCTIONS ###################

        def dist_2_points(x1, x2, y1, y2):
            return abs(abs(x1 - x2) ** 2 + abs(y1 - y2) ** 2) ** 0.5

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
            a = abs(dist_2_points(x1=closest_coords[0], x2=second_closest_coords[0],
                                  y1=closest_coords[1], y2=second_closest_coords[1]))
            b = abs(dist_2_points(x1=car_coords[0], x2=closest_coords[0],
                                  y1=car_coords[1], y2=closest_coords[1]))
            c = abs(dist_2_points(x1=car_coords[0], x2=second_closest_coords[0],
                                  y1=car_coords[1], y2=second_closest_coords[1]))
            # Fix: Add epsilon to avoid division by zero
            epsilon = 1e-6
            try:
                distance = abs(-(a ** 4) + 2 * (a ** 2) * (b ** 2) + 2 * (a ** 2) * (c ** 2) -
                               (b ** 4) + 2 * (b ** 2) * (c ** 2) - (c ** 4)) ** 0.5 / (2 * (a + epsilon))
            except:
                distance = min(b, c)  # Fix: Use minimum of b and c for more accurate fallback
            return distance

        def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):
            heading_vector = [math.cos(math.radians(heading)), math.sin(math.radians(heading))]
            new_car_coords = [car_coords[0] + heading_vector[0], car_coords[1] + heading_vector[1]]
            distance_closest_coords_new = dist_2_points(x1=new_car_coords[0], x2=closest_coords[0],
                                                        y1=new_car_coords[1], y2=closest_coords[1])
            distance_second_closest_coords_new = dist_2_points(x1=new_car_coords[0], x2=second_closest_coords[0],
                                                               y1=new_car_coords[1], y2=second_closest_coords[1])
            if distance_closest_coords_new <= distance_second_closest_coords_new:
                next_point_coords = closest_coords
                prev_point_coords = second_closest_coords
            else:
                next_point_coords = second_closest_coords
                prev_point_coords = closest_coords
            return [next_point_coords, prev_point_coords]

        def racing_direction_diff(closest_coords, second_closest_coords, car_coords, heading):
            next_point, prev_point = next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading)
            track_direction = math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0])
            track_direction = math.degrees(track_direction)
            direction_diff = abs(track_direction - heading)
            if direction_diff > 180:
                direction_diff = 360 - direction_diff
            return direction_diff

        def indexes_cyclical(start, end, array_len):
            if end < start:
                end += array_len
            return [index % array_len for index in range(start, end)]

        def projected_time(first_index, closest_index, step_count, times_list):
            current_actual_time = (step_count - 1) / 15
            indexes_traveled = indexes_cyclical(first_index, closest_index, len(times_list))
            current_expected_time = sum([times_list[i] for i in indexes_traveled])
            total_expected_time = sum(times_list)
            # Fix: Avoid division by zero
            if current_expected_time < 1e-6:
                return total_expected_time  # Default to total lap time
            try:
                projected_time = (current_actual_time / current_expected_time) * total_expected_time
            except:
                projected_time = total_expected_time
            return projected_time

        #################### RACING LINE ######################

        # Placeholder: Update for Rogue Circuit (requires track-specific data)
        racing_track = [...]  # Your existing racing_track list; replace with Rogue Circuit waypoints
        # Note: Update speeds (e.g., 4.5 m/s in straights) after analyzing Rogue Circuit

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

        ############### OPTIMAL X,Y,SPEED,TIME ################

        closest_index, second_closest_index = closest_2_racing_points_index(racing_track, [x, y])
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        # Fix: Always set first_racingpoint_index at step 1
        if steps == 1:
            self.first_racingpoint_index = closest_index

        ################ REWARD AND PUNISHMENT ################

        reward = 1.0

        ## Reward for staying close to racing line
        DISTANCE_MULTIPLE = 2
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist / (track_width * 0.4)))
        reward += distance_reward * DISTANCE_MULTIPLE

        ## Reward for matching optimal speed
        SPEED_DIFF_NO_REWARD = 0.5
        SPEED_MULTIPLE = 3
        speed_diff = abs(optimals[2] - speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            speed_reward = (1 - (speed_diff / SPEED_DIFF_NO_REWARD) ** 2) ** 2
        else:
            speed_reward = 0
        reward += speed_reward * SPEED_MULTIPLE

        ## Bonus for exceeding optimal speed in straights
        if speed > optimals[2] and optimals[2] >= 4.0:
            speed_bonus = 0.5 * (speed - optimals[2])
            reward += speed_bonus

        ## Reward for fewer steps
        REWARD_PER_STEP_FOR_FASTEST_TIME = 2
        STANDARD_TIME = 37  # Update for Rogue Circuit
        FASTEST_TIME = 15  # Update based on leaderboard
        times_list = [row[3] for row in racing_track]
        projected_time = projected_time(self.first_racingpoint_index, closest_index, steps, times_list)
        try:
            steps_prediction = projected_time * 15 + 1
            reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME * FASTEST_TIME /
                                           (STANDARD_TIME - FASTEST_TIME)) * (steps_prediction - (STANDARD_TIME * 15 + 1)))
            steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction)
        except:
            steps_reward = 0
        reward += steps_reward

        ## Penalty for excessive steering
        MAX_STEERING_ANGLE = 30  # Fix: Normalize based on action space
        STEERING_THRESHOLD = 15
        if abs(steering_angle) > STEERING_THRESHOLD:
            steering_penalty = 0.5 * (abs(steering_angle) - STEERING_THRESHOLD) / (MAX_STEERING_ANGLE - STEERING_THRESHOLD)
            reward *= (1 - steering_penalty)

        ## Penalty for wrong direction
        direction_diff = racing_direction_diff(optimals[0:2], optimals_second[0:2], [x, y], heading)
        if direction_diff > 30:
            reward = 1e-3

        ## Penalty for being too slow
        speed_diff_zero = optimals[2] - speed
        if speed_diff_zero > 0.5:
            reward = 1e-3

        ## Reward for finishing
        REWARD_FOR_FASTEST_TIME = 1000
        if progress == 100:
            finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                                       (15 * (STANDARD_TIME - FASTEST_TIME))) * (steps - STANDARD_TIME * 15))
        else:
            finish_reward = 0
        reward += finish_reward

        ## Penalty for off-track (multiplicative instead of overwrite)
        if not all_wheels_on_track:
            reward *= 0.01  # Fix: Reduce reward but preserve partial learning

        ####################### VERBOSE #######################

        if self.verbose:
            print(f"Closest index: {closest_index}")
            print(f"Distance to racing line: {dist:.3f}")
            print(f"Distance reward: {distance_reward:.3f}")
            print(f"Optimal speed: {optimals[2]:.3f}")
            print(f"Speed difference: {speed_diff:.3f}")
            print(f"Speed reward: {speed_reward:.3f}")
            print(f"Direction difference: {direction_diff:.3f}")
            print(f"Projected time: {projected_time:.3f}")
            print(f"Steps reward: {steps_reward:.3f}")
            print(f"Finish reward: {finish_reward:.3f}")

        #################### RETURN REWARD ####################

        return float(reward)

reward_object = Reward(verbose=False)  # Set to True for debugging

def reward_function(params):
    return reward_object.reward_function(params)