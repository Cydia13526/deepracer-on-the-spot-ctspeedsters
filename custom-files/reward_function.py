import math


class Reward:
    def __init__(self, verbose=False, debug=False):
        self.first_racingpoint_index = 0
        self.verbose = verbose
        self.debug = debug

    def reward_function(self, params):

        ################## HELPER FUNCTIONS ###################

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
            DISTANCE_MULTIPLE = 1.5
            dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
            distance_reward = max(1e-3, (1 - (dist/(track_width*0.5))**2))
            return dist, distance_reward * DISTANCE_MULTIPLE

        def get_speed_reward():
            SPEED_MULTIPLE = 1.2
            SPEED_DIFF_NO_REWARD = 1.5
            speed_diff = abs(optimals[2] - speed)
            if speed_diff <= SPEED_DIFF_NO_REWARD:
                speed_reward = (1 - (speed_diff/SPEED_DIFF_NO_REWARD)**2)**2
            else:
                speed_reward = 1e-3
            return speed_diff, speed_reward * SPEED_MULTIPLE

        def get_direction_reward():
            direction_diff = racing_direction_diff(optimals[0:2], optimals_second[0:2], [x, y], heading)
            STEERING_ANGLE_THRESHOLD = 20
            if direction_diff > STEERING_ANGLE_THRESHOLD:
                reward = 1e-3
            else:
                reward = (1.0 - direction_diff / STEERING_ANGLE_THRESHOLD)**2
            return direction_diff, reward

        def get_step_reward():
            REWARD_PER_STEP_FOR_FASTEST_TIME = 1.5
            FASTEST_TIME = 15
            STANDARD_TIME = 17.5
            projected_time = get_projected_time(self.first_racingpoint_index, closest_index, steps)
            try:
                steps_prediction = projected_time * 15 + 1
                reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME*(FASTEST_TIME) /
                                               (STANDARD_TIME-FASTEST_TIME))*(steps_prediction-(STANDARD_TIME*15+1)))
                steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction)
            except:
                steps_reward = 0
            return projected_time, steps_reward

        def get_near_edge_penalty():
            EDGE_THRESHOLD = track_width * 0.4
            dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
            if dist > EDGE_THRESHOLD:
                penalty = (dist - EDGE_THRESHOLD) / (track_width * 0.5 - EDGE_THRESHOLD)
                return max(1e-3, 1 - penalty**2)
            return 1.0

        def get_finish_reward():
            STANDARD_TIME = 17.5
            if progress == 100:
                finish_reward = max(1e-3, (STANDARD_TIME*15 - steps)**2)
                if self.verbose:
                    print(f'progress is 100, steps: {steps}, finish_reward: {finish_reward}')
            else:
                finish_reward = 0
            return finish_reward

        #################### RACING LINE ######################

        racing_track = [[-2.29044, -5.90499, 4.0, 0.06577],
                        [-2.03269, -5.94953, 4.0, 0.06539],
                        [-1.77519, -5.99189, 4.0, 0.06524],
                        [-1.51683, -6.03229, 4.0, 0.06537],
                        [-1.2566, -6.07083, 4.0, 0.06577],
                        [-0.99384, -6.1075, 4.0, 0.06633],
                        [-0.72821, -6.14211, 4.0, 0.06697],
                        [-0.45966, -6.17434, 4.0, 0.06762],
                        [-0.18835, -6.2038, 4.0, 0.06823],
                        [0.08563, -6.2298, 4.0, 0.0688],
                        [0.36197, -6.25156, 3.8097, 0.07276],
                        [0.64027, -6.26806, 3.35725, 0.08304],
                        [0.91999, -6.27802, 2.93274, 0.09544],
                        [1.20038, -6.27988, 2.58838, 0.10833],
                        [1.48044, -6.27166, 2.28612, 0.12256],
                        [1.75879, -6.25048, 2.28612, 0.12211],
                        [2.03362, -6.21298, 2.28612, 0.12133],
                        [2.30248, -6.15474, 2.28612, 0.12034],
                        [2.56212, -6.07092, 2.28612, 0.11934],
                        [2.80718, -5.95614, 2.57917, 0.10492],
                        [3.03855, -5.81864, 2.74112, 0.09819],
                        [3.2564, -5.66223, 2.86744, 0.09353],
                        [3.46083, -5.48952, 2.98778, 0.08957],
                        [3.65206, -5.30261, 3.00251, 0.08906],
                        [3.83039, -5.10321, 2.84452, 0.09405],
                        [3.99548, -4.89215, 2.57853, 0.10392],
                        [4.14693, -4.67019, 2.57853, 0.10421],
                        [4.28385, -4.43771, 2.57853, 0.10463],
                        [4.40422, -4.19453, 2.57853, 0.10523],
                        [4.50519, -3.94041, 2.57853, 0.10605],
                        [4.5811, -3.67462, 3.38153, 0.08174],
                        [4.64173, -3.40285, 3.73922, 0.07447],
                        [4.6895, -3.12647, 4.0, 0.07012],
                        [4.7268, -2.84661, 4.0, 0.07058],
                        [4.75534, -2.56404, 3.9208, 0.07244],
                        [4.77669, -2.27953, 3.43612, 0.08303],
                        [4.79129, -1.9982, 3.04495, 0.09252],
                        [4.81153, -1.71887, 2.7091, 0.10338],
                        [4.84047, -1.44244, 2.39562, 0.11602],
                        [4.88103, -1.17002, 2.39562, 0.11497],
                        [4.93629, -0.9029, 2.39562, 0.11386],
                        [5.00964, -0.64271, 2.39562, 0.11285],
                        [5.10518, -0.39166, 2.39562, 0.11213],
                        [5.22834, -0.15323, 2.73921, 0.09797],
                        [5.37236, 0.07469, 2.96699, 0.09087],
                        [5.53416, 0.29297, 3.25305, 0.08353],
                        [5.71089, 0.50267, 3.51513, 0.07802],
                        [5.9005, 0.70448, 3.63277, 0.07623],
                        [6.10175, 0.89847, 3.28217, 0.08517],
                        [6.31298, 1.08425, 2.96246, 0.09496],
                        [6.53131, 1.26041, 2.64731, 0.10597],
                        [6.73752, 1.44208, 2.38605, 0.11518],
                        [6.93189, 1.63141, 2.14772, 0.12634],
                        [7.11199, 1.82943, 2.0, 0.13384],
                        [7.27482, 2.03719, 2.0, 0.13199],
                        [7.41628, 2.25582, 2.0, 0.1302],
                        [7.53153, 2.48597, 2.0, 0.1287],
                        [7.61421, 2.72754, 2.0, 0.12766],
                        [7.65865, 2.97821, 2.00075, 0.12724],
                        [7.66411, 3.23321, 2.11882, 0.12038],
                        [7.63433, 3.48777, 2.25579, 0.11361],
                        [7.57338, 3.7383, 2.47291, 0.10427],
                        [7.48663, 3.98276, 2.64688, 0.098],
                        [7.37766, 4.21986, 2.70252, 0.09655],
                        [7.2478, 4.44799, 2.84979, 0.09211],
                        [7.09968, 4.66642, 2.94278, 0.08968],
                        [6.93501, 4.87436, 3.03931, 0.08727],
                        [6.75541, 5.07129, 2.95432, 0.09021],
                        [6.56227, 5.25672, 2.80573, 0.09543],
                        [6.35639, 5.42984, 2.80573, 0.09587],
                        [6.13858, 5.58978, 2.80573, 0.09631],
                        [5.90964, 5.73559, 2.80573, 0.09674],
                        [5.66927, 5.86433, 2.80573, 0.09718],
                        [5.41807, 5.97285, 2.98596, 0.09164],
                        [5.15907, 6.06258, 3.14774, 0.08708],
                        [4.89443, 6.13484, 3.33529, 0.08225],
                        [4.62583, 6.19122, 3.45522, 0.07943],
                        [4.35437, 6.2326, 3.55565, 0.07723],
                        [4.08094, 6.25969, 3.64008, 0.07549],
                        [3.80622, 6.27304, 3.49495, 0.0787],
                        [3.53081, 6.27312, 3.28629, 0.08381],
                        [3.25524, 6.26039, 3.02241, 0.09128],
                        [2.97998, 6.23505, 2.70858, 0.10205],
                        [2.70564, 6.19609, 2.70858, 0.1023],
                        [2.43303, 6.14195, 2.70858, 0.10261],
                        [2.16331, 6.07066, 2.70858, 0.103],
                        [1.89819, 5.97912, 2.70858, 0.10355],
                        [1.64068, 5.86248, 3.50567, 0.08064],
                        [1.38855, 5.73086, 4.0, 0.0711],
                        [1.14012, 5.5892, 4.0, 0.07149],
                        [0.89363, 5.44216, 4.0, 0.07175],
                        [0.64504, 5.29227, 4.0, 0.07257],
                        [0.39444, 5.1438, 4.0, 0.07282],
                        [0.1418, 4.99749, 4.0, 0.07299],
                        [-0.1129, 4.85411, 4.0, 0.07307],
                        [-0.36964, 4.71428, 4.0, 0.07309],
                        [-0.62845, 4.57853, 4.0, 0.07306],
                        [-0.88945, 4.44737, 4.0, 0.07303],
                        [-1.15276, 4.32125, 4.0, 0.07299],
                        [-1.41845, 4.20057, 4.0, 0.07295],
                        [-1.68656, 4.08559, 4.0, 0.07293],
                        [-1.95704, 3.9764, 4.0, 0.07292],
                        [-2.22977, 3.87288, 4.0, 0.07293],
                        [-2.50454, 3.77476, 4.0, 0.07294],
                        [-2.78109, 3.68161, 4.0, 0.07295],
                        [-3.05911, 3.59291, 4.0, 0.07296],
                        [-3.33821, 3.50811, 4.0, 0.07292],
                        [-3.61797, 3.42663, 4.0, 0.07285],
                        [-3.89795, 3.3479, 4.0, 0.07271],
                        [-4.1778, 3.27132, 4.0, 0.07254],
                        [-4.45737, 3.19621, 3.66955, 0.07889],
                        [-4.73154, 3.12175, 3.17477, 0.08949],
                        [-5.00392, 3.04404, 2.79809, 0.10123],
                        [-5.27286, 2.96017, 2.40934, 0.11693],
                        [-5.53663, 2.8672, 2.25282, 0.12415],
                        [-5.79328, 2.76202, 2.25282, 0.12312],
                        [-6.04032, 2.6411, 2.25282, 0.12209],
                        [-6.27462, 2.5007, 2.25282, 0.12124],
                        [-6.49054, 2.33537, 2.25282, 0.12072],
                        [-6.68303, 2.14378, 2.4037, 0.11299],
                        [-6.85267, 1.9314, 2.61097, 0.10411],
                        [-7.00135, 1.70318, 2.8764, 0.09469],
                        [-7.13167, 1.46332, 3.07774, 0.08869],
                        [-7.2452, 1.21442, 3.31952, 0.08241],
                        [-7.34375, 0.95868, 3.42903, 0.07993],
                        [-7.42855, 0.69761, 3.42903, 0.08005],
                        [-7.49856, 0.43184, 3.6276, 0.07576],
                        [-7.5551, 0.1627, 3.91, 0.07034],
                        [-7.60025, -0.1087, 3.91, 0.07037],
                        [-7.63434, -0.38183, 3.91, 0.0704],
                        [-7.65716, -0.65629, 3.70428, 0.07435],
                        [-7.66801, -0.93164, 3.39402, 0.08119],
                        [-7.66745, -1.20736, 3.39402, 0.08124],
                        [-7.65583, -1.483, 3.39402, 0.08128],
                        [-7.63269, -1.7581, 3.39402, 0.08134],
                        [-7.59617, -2.03196, 3.07784, 0.08977],
                        [-7.54381, -2.30338, 3.07784, 0.08981],
                        [-7.47604, -2.57129, 3.07784, 0.08979],
                        [-7.39495, -2.8353, 3.05825, 0.09031],
                        [-7.29947, -3.09427, 2.74262, 0.10064],
                        [-7.18613, -3.34557, 2.63781, 0.10451],
                        [-7.05622, -3.58822, 2.5996, 0.10588],
                        [-6.91205, -3.82232, 2.5996, 0.10576],
                        [-6.75171, -4.04548, 2.5996, 0.1057],
                        [-6.57222, -4.25369, 2.5996, 0.10574],
                        [-6.37337, -4.44408, 2.5996, 0.1059],
                        [-6.1562, -4.61439, 2.88872, 0.09554],
                        [-5.92544, -4.7672, 3.1711, 0.08728],
                        [-5.68434, -4.90469, 3.46633, 0.08007],
                        [-5.43537, -5.02877, 3.81657, 0.07288],
                        [-5.18064, -5.14134, 4.0, 0.06963],
                        [-4.92164, -5.24376, 4.0, 0.06963],
                        [-4.65961, -5.33708, 4.0, 0.06954],
                        [-4.39557, -5.42227, 4.0, 0.06936],
                        [-4.13041, -5.50021, 4.0, 0.06909],
                        [-3.86488, -5.57173, 4.0, 0.06875],
                        [-3.59959, -5.63761, 4.0, 0.06834],
                        [-3.33504, -5.69856, 4.0, 0.06787],
                        [-3.07163, -5.75525, 4.0, 0.06736],
                        [-2.80966, -5.80823, 4.0, 0.06682],
                        [-2.54928, -5.85801, 4.0, 0.06627]]

        ################## INPUT PARAMETERS ###################

        if self.verbose:
            new_params = {}
            for key, value in params.items():
                if key != 'waypoints':
                    new_params[key] = [value]
            print("params: ", new_params)

        is_offtrack = params['is_offtrack']
        x = params['x']
        y = params['y']
        heading = params['heading']
        progress = params['progress']
        steps = params['steps']
        speed = params['speed']
        track_width = params['track_width']

        ############### OPTIMAL X,Y,SPEED,TIME ################

        closest_index, second_closest_index = closest_2_racing_points_index(racing_track, [x, y])
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        if self.debug:
            self.first_racingpoint_index = 0

        if steps == 2.0:
            print('set first_racingpoint_index to %i' % closest_index)
            self.first_racingpoint_index = closest_index

        ################ REWARD AND PUNISHMENT ################

        reward = 1.0

        dist, distance_reward = get_distance_reward()
        reward *= distance_reward

        speed_diff, speed_reward = get_speed_reward()
        reward *= speed_reward

        direction_diff, direction_reward = get_direction_reward()
        reward *= direction_reward

        edge_penalty = get_near_edge_penalty()
        reward *= edge_penalty

        projected_time, steps_reward = get_step_reward()
        reward += steps_reward

        finish_reward = get_finish_reward()
        reward += finish_reward

        if is_offtrack:
            reward = 1e-3

        ####################### VERBOSE #######################

        if self.verbose:
            print(f"REWARD: {reward:.4f}, DIS_REW: {distance_reward:.4f}, SPD_REW: {speed_reward:.4f}, "
                  f"DIR_REW: {direction_reward:.4f}, EDGE_PEN: {edge_penalty:.4f}, STEP_REW: {steps_reward:.4f}, "
                  f"FIN_REW: {finish_reward:.4f}, ACT_SPD: {speed:.4f}, EXP_SPD: {optimals[2]:.4f}, "
                  f"SPD_DIFF: {speed_diff:.4f}, DIST: {dist:.4f}, DIR_DIFF: {direction_diff:.4f}, "
                  f"STEPS: {steps}, PROGRESS: {progress}")

        #################### RETURN REWARD ####################

        return float(reward)


reward_object = Reward(verbose=True, debug=False)


def reward_function(params):
    return reward_object.reward_function(params)