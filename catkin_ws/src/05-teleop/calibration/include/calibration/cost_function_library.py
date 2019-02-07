import rospy

"""
obj_cost += ( abs(((x_sim[0, i] - x[0, i])) / range_x) +
              abs(((x_sim[1, i] - x[1, i])) / range_y) +
              abs(((x_sim[2, i] - x[2, i])) / range_yaw)
    )

obj_cost += (
             ((x_sim[0, i] - x[0, i])) ** 2 +
             ((x_sim[1, i] - x[1, i])) ** 2 +
             ((x_sim[2, i] - x[2, i])) ** 2
            )

obj_cost += (
        ((x_sim[0, i] - x[0, i]) / range_x) ** 2 +
        ((x_sim[1, i] - x[1, i]) / range_y) ** 2 +
        ((x_sim[2, i] - x[2, i]) / range_yaw) ** 2
)
"""
def nSE(x_predict, x_meas, measurement_coordinate_frame):
    obj_cost = 0.0
    if measurement_coordinate_frame == 'cartesian':
        range_x = float(max(x_meas[0, :]) - min(x_meas[0, :]))
        range_y = float(max(x_meas[1, :]) - min(x_meas[1, :]))
        range_yaw = float(max(x_meas[2, :]) - min(x_meas[2, :]))
        # print('range x: {} range y: {} range yaw: {}'.format(range_x,range_y,range_yaw))
        obj_cost = (((x_predict[0,:] - x_meas[0,:]) / range_x) ** 2 +
                    ((x_predict[1,:] - x_meas[1,:]) / range_y) ** 2 +
                    ((x_predict[2,:] - x_meas[2,:]) / range_yaw) ** 2)
    elif measurement_coordinate_frame == 'polar':
        pass

def cost_fn_selector(cost_name):
    if cost_name == 'nSE':
        return nSE
    else:
        rospy.logwarn('undefined cost function is requested, see cost_fn_library for const function definitions')