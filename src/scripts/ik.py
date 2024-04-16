from logging import exception
from typing import Tuple

import math
import serial


def inverse_kinematics  ( 
                            x: float, y: float, z: float,
                            is_service_mode: bool = False,
                            l1: float = 20, l2: float = 14, l3: float = 0, d2: float = 18
                        ) -> Tuple[float]:
    '''
    l1 -> length of arm1
    l2 -> length of arm2
    l3 -> length of arm3
    d2 -> height of base of ground
    θ1 -> Counter-clockwise Base Angle from Right side of Base
    θ2 -> Angle between arm1 and Horizontal Plane
    θ3 -> Angle between arm1 and arm2

    computation_time --> 1000 test ≈ 2 ms
    '''
    d1 = math.sqrt(x*x + y*y) - l3
    d6 = math.sqrt(d1*d1 + (z - d2)*(z - d2))

    θ1 = math.degrees(math.acos(x/(d1+l3)))
    θ6 = math.acos((l1*l1 + d6*d6 - l2*l2) / (2*l1*d6))
    θ7 = math.acos(d1/d6)
    θ2 = math.degrees(θ6 + θ7)
    θ3 = math.degrees(math.acos((l1*l1 + l2*l2 - d6*d6) / (2*l1*l2)))

    return θ1, θ2, θ3


def ik_tester():
    x, y, z = 0, 27.665097, 10.5186689 + 18
    θ1, θ2, θ3 = inverse_kinematics(x, y, z)


def main():
    port = 'COM17'
    arduino = serial.Serial(port=port, baudrate=9600, timeout=0.1)

    inp = input('Enter x y z: ')
    while inp != '':
        try:
            x, y, z = tuple(map(float, inp.split(' ')))
            print(f"x: {x}\ny: {y}\nz: {z}")
            θ1, θ2, θ3 = tuple(map(int, inverse_kinematics( x, y, z,
                                                            is_service_mode=False,
                                                            l1 = 22, l2 = 16, l3 = 0, d2 = 17)))
                                                            
            print(f"θ1: {θ1}\nθ2: {θ2}\nθ3: {θ3}")
            arduino.write(bytes(f'1{θ2:03d}2{θ3:03d}3{θ1:03d}', 'utf-8'))
        except:
            pass
        inp = input('Enter x y z: ')


if __name__=='__main__':
    main()