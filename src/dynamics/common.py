import numpy as np
from scipy.interpolate import griddata
import scipy as sp



class Dynamics:
    def B(self, v):
        raise NotImplementedError

    def AT(self, v, x):
        raise NotImplementedError
    
    def AV(self, v, x):
        raise NotImplementedError
    
    

def interpolate_2d(point_value_map, query_point, method='linear'):
    """
    Interpolates the value at the query point based on the values at the given points.
    :param point_value_map: A dictionary that maps points to values.
    :param query_point: The point at which to interpolate the value.
    :param method: The interpolation method to use. Can be 'linear', 'nearest', or 'cubic'.
    :return: The interpolated value at the query point.
    """
    points = list(point_value_map.keys())
    values = list(point_value_map.values())
    
    x_query, y_query = query_point
    points = np.array(points)
    values = np.array(values)
    
    # 使用RBF进行插值和外推
    rbf = sp.interpolate.Rbf(points[:,0], points[:,1], values, function=method)
    result = rbf(x_query, y_query)
    
    return float(result)


def interpolate_1d(point_value_map, query_point, method='linear'):
    """
    Interpolates the value at the query point based on the values at the given points.
    :param point_value_map: A dictionary that maps points to values.
    :param query_point: The point at which to interpolate the value.
    :param method: The interpolation method to use. Can be 'linear', 'nearest', or 'cubic'.
    :return: The interpolated value at the query point.
    """
    x_query = query_point
    points = list(point_value_map.keys())
    values = list(point_value_map.values())
    
    result = sp.interpolate.interp1d(points, values, kind=method, fill_value='extrapolate')
    
    return float(result(x_query))