#!/usr/bin/env python
'''
Copyright (c) 2016, Antonio Coratelli.
Released under BSD 3-Clause License. See 'LICENSE' file.
'''
import argparse
from tf.transformations import *
from numpy import dot as matrix_product

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('tra_x', action='store', type=str)
    parser.add_argument('tra_y', action='store', type=str)
    parser.add_argument('tra_z', action='store', type=str)
    parser.add_argument('qua_x', action='store', type=str)
    parser.add_argument('qua_y', action='store', type=str)
    parser.add_argument('qua_z', action='store', type=str)
    parser.add_argument('qua_w', action='store', type=str)
    args = parser.parse_args()

    tra_M = translation_matrix([args.tra_x, args.tra_y, args.tra_z])
    qua_M = quaternion_matrix([args.qua_x, args.qua_y, args.qua_z, args.qua_w])
    all_M = matrix_product(tra_M, qua_M)

    print all_M
