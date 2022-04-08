import sys
import math
import numpy as np

mass_factor_combination = [['e', 'n', 'e'],
                           ['n', 'ne', 'ne'],
                           ['e', 'ne', 'i']]


def fuse(sensor_object,global_object):
    
    fused_mass_factor_existance = None
    fused_mass_factor_nonexistance = None
    fused_mass_factor_uncertainity = None

    fused_probability_existance = None
    fused_probability_nonexistance = None

    """
    Method to perfrom the existence fusion between the sensor object and the global object if the objects are associated.

    :return: fused_probability_existance
    """
    sensor_existance_mass_factors = (sensor_object.list_existance_mass_factor) # need to determine
    global_existance_mass_factors = (global_object.global_predicted_masslist) # need to determine
    

    fused_mass_factor_existance, fused_mass_factor_nonexistance, fused_mass_factor_uncertainity = get_fused_mass_factors(sensor_object,global_object)
    fused_probability_existance = ((fused_mass_factor_existance) + ((1 / 2) * (fused_mass_factor_uncertainity)))
    fused_probability_nonexistance = (1 - (fused_probability_existance))

    return fused_probability_existance

def get_fused_mass_factors(sensor_object,global_object):
    """
    Method to perfrom the fused mass factors.

    :param sensor_existance_mass_factors:
    :param global_existance_mass_factors:
    :param mass_factor_combination:

    :return: fused_mass_factor_existance, fused_mass_factor_nonexistance, fused_mass_factor_uncertainity
    """
    sensor_existance_mass_factors = (sensor_object.list_existance_mass_factor)  # need to determine
    global_existance_mass_factors = (global_object.global_predicted_masslist)  # need to determine

    sum_intersection_existance = 0
    sum_intersection_nonexistance = 0
    sum_intersection_uncertainity = 0
    sum_intersection_null = 0

    for row in range(len(sensor_existance_mass_factors)):
        for column in range(len(global_existance_mass_factors)):
            if mass_factor_combination[row][column] == 'e':
                sum_intersection_existance = sum_intersection_existance + (
                            sensor_existance_mass_factors[row] * global_existance_mass_factors[column])

            elif mass_factor_combination[row][column] == 'n':
                sum_intersection_null = sum_intersection_null + (
                            sensor_existance_mass_factors[row] * global_existance_mass_factors[column])

            elif mass_factor_combination[row][column] == 'ne':
                sum_intersection_nonexistance = sum_intersection_nonexistance + (
                            sensor_existance_mass_factors[row] * global_existance_mass_factors[column])

            elif mass_factor_combination[row][column] == 'i':
                sum_intersection_uncertainity = sum_intersection_uncertainity + (
                            sensor_existance_mass_factors[row] * global_existance_mass_factors[column])

    fused_mass_factor_existance = ((sum_intersection_existance) / (1 - (sum_intersection_null)))
    fused_mass_factor_nonexistance = ((sum_intersection_nonexistance) / (1 - (sum_intersection_null)))
    fused_mass_factor_uncertainity = ((sum_intersection_uncertainity) / (1 - (sum_intersection_null)))

    return (fused_mass_factor_existance, fused_mass_factor_nonexistance, fused_mass_factor_uncertainity)
