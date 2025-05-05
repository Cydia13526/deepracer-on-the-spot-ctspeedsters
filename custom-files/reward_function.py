import math

class Reward:
    def __init__(self, verbose=False, debug=False):
        self.first_racingpoint_index = 0
        self.verbose = verbose
        self.debug = debug

    def reward_function(self, params):
        def dist_2_points(x1, x2, y1, y2):
            return abs(abs(x1-x2)**2 + abs(y1-y2)**2)**0.5

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
            try:
                distance = abs(-(a**4) + 2*(a**2)*(b**2) + 2*(a**2)*(c**2) -
                               (b**4) + 2*(b**2)*(c**2) - (c**4))**0.5 / (2*a)
            except:
                distance = b
            return distance

        def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):
            heading_vector = [math.cos(math.radians(heading)), math.sin(math.radians(heading))]
            new_car_coords = [car_coords[0]+heading_vector[0], car_coords[1]+heading_vector[1]]
            distance_closest_coords_new = dist_2_points(x1=new_car_coords[0], x2=closest_coords[0],
                                                        y1=new_car_coords[1], y2=closest_coords[1])
            distance_second_closest_coords_new = dist_2_points(x1=new_car_coords[0], x2=second_closest_coords[0],
                                                               y1=new_car_coords[1], y2=second_closest_coords[1])
            if distance_closest_coords_new <= distance_second_closest_coords_new:
                next_point_coords = closest_coords
                prev_point_coords = second_closest_coordinates = second_closest_coords
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

        def get_projected_time(first_index, closest_index, step_count):
            times_list = [row[3] for row in racing_track]
            current_actual_time = (step_count-1) / 15
            if self.verbose:
                print("first_index: %i" % first_index)
            indexes_traveled = indexes_cyclical(first_index, closest_index, len(times_list))
            current_expected_time = sum([times_list[i] for i in indexes_traveled])
            total_expected_time = sum(times_list)
            try:
                projected_time = (current_actual_time / current_expected_time) * total_expected_time
            except:
                projected_time = 9999
            return projected_time

        def get_distance_reward():
            DISTANCE_MULTIPLE = 1
            dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
            distance_reward = max(1e-3, 1 - (dist/(track_width*0.5)))
            final_distance_reward = distance_reward * DISTANCE_MULTIPLE
            return dist, final_distance_reward

        def get_direction_reward():
            direction_diff = racing_direction_diff(optimals[0:2], optimals_second[0:2], [x, y], heading)
            STEERING_ANGLE_THRESHOLD = 20
            if direction_diff > STEERING_ANGLE_THRESHOLD:
                reward = 1e-3
            else:
                reward = 1.0 - direction_diff / STEERING_ANGLE_THRESHOLD
            return direction_diff, reward

        def get_speed_reward():
            SPEED_MULTIPLE = 1.5
            SPEED_DIFF_NO_REWARD = 0.5
            speed_diff = abs(optimals[2]-speed)
            if speed_diff <= SPEED_DIFF_NO_REWARD:
                speed_reward = (1 - (speed_diff/(SPEED_DIFF_NO_REWARD))**2)**2
            else:
                speed_reward = 1e-3
            final_speed_reward = speed_reward * SPEED_MULTIPLE
            return speed_diff, final_speed_reward

        def get_step_reward():
            REWARD_PER_STEP_FOR_FASTEST_TIME = 2.0
            FASTEST_TIME = 15.0
            STANDARD_TIME = 17.0
            projected_time = get_projected_time(self.first_racingpoint_index, closest_index, steps)
            try:
                steps_prediction = projected_time * 15 + 1
                reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME*(FASTEST_TIME) /
                                               (STANDARD_TIME-FASTEST_TIME))*(steps_prediction-(STANDARD_TIME*15+1)))
                steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction)
                if self.verbose:
                    print(f'projected_time {projected_time}, steps_prediction: {steps_prediction}, reward_prediction: {reward_prediction}, steps_reward: {steps_reward}')
            except:
                steps_reward = 0
            return projected_time, steps_reward

        def get_finish_reward():
            STANDARD_TIME = 17.0
            if progress == 100:
                finish_reward = max(1e-3, (STANDARD_TIME*15 - steps)**2)
                if self.verbose:
                    print(f'progress is 100, steps: {steps}, finish_reward: {finish_reward}')
            else:
                finish_reward = 0
            return finish_reward

        racing_track = [[-2.29044, -5.90499, 4.0, 0.06577], ...]  # Your track data

        if self.verbose:
            new_params = {key: [value] for key, value in params.items() if key != 'waypoints'}
            print("params: ", new_params)

        is_offtrack = params['is_offtrack']
        x = params['x']
        y = params['y']
        heading = params['heading']
        progress = params['progress']
        steps = params['steps']
        speed = params['speed']
        track_width = params['track_width']

        closest_index, second_closest_index = closest_2_racing_points_index(racing_track, [x, y])
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        if self.debug:
            self.first_racingpoint_index = 0

        if steps == 2.0:
            print('set first_racingpoint_index to %i' % closest_index)
            self.first_racingpoint_index = closest_index

        if is_offtrack:
            if self.verbose:
                print(f"Off-track at step {steps}, position ({x}, {y}), closest_index {closest_index}")
            reward = -10.0
            return float(reward)

        reward = 1.0
        dist, distance_reward = get_distance_reward()
        reward *= distance_reward * 1.5
        speed_diff, speed_reward = get_speed_reward()
        reward *= speed_reward
        direction_diff, direction_reward = get_direction_reward()
        reward *= direction_reward
        projected_time, steps_reward = get_step_reward()
        reward += steps_reward * 0.5
        finish_reward = get_finish_reward()
        reward += finish_reward

        if self.verbose:
            print(f"REWARD: {reward:.4f}, DIS_REW: {distance_reward:.4f}, SPD_REW: {speed_reward:.4f}, DIR_REW: {direction_reward:.4f}, STEP_REW: {steps_reward:.4f}, FIN_REW: {finish_reward:.4f}, "
                  f"ACT_SPD: {speed:.4f}, EXP_SPD: {optimals[2]:.4f}, SPD_DIFF: {speed_diff:.4f}, "
                  f"CLOSET_INDEX: {closest_index}, DIST: {dist:.4f}, DIR_DIFF: {direction_diff:.4f}, STEPS: {steps}, PROGRESS: {progress}")

        return float(reward)

reward_object = Reward(verbose=True, debug=False)

def reward_function(params):
    return reward_object.reward_function(params)