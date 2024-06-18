'''
heuristic-algorithm.py

Implementation of heuristic algorithm to solve the problem
of the robot avoiding obstacles.

Copyright (C) 2024 Gonzalo Lope Carrasco

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
'''

import robotica

def avoid(readings,turning):
    wanted_distance = 0.30
    velocity = 2
    global ticks_sincer_turn
    if turning and ( ticks_sincer_turn > 7):
        turning = False
        ticks_sincer_turn = 0
    elif turning:
        ticks_sincer_turn += 1
        return -1, +1, True

    # Avance
    if ( readings[3] > wanted_distance and readings[4] > wanted_distance ):

        if (readings[7] > wanted_distance):
            if (readings[8] < readings[7]):
                sensor_diff = (readings[7] - readings[8])*2 + (readings[8] - wanted_distance) * 3 * velocity
                print("2:",sensor_diff)
                lspeed, rspeed = velocity + sensor_diff, velocity - sensor_diff
            else:
                sensor_diff = (wanted_distance - readings[7]) * velocity * 1/2
                print("3:",sensor_diff)
                lspeed, rspeed = velocity - sensor_diff, velocity + sensor_diff
        elif (readings[7] < wanted_distance ):
            if (readings[8] > readings[7]):
                sensor_diff = (readings[8] - readings[7])*2 + (wanted_distance - readings[7]) * 3 * velocity
                print("1:",sensor_diff)
                lspeed, rspeed = velocity - sensor_diff, velocity + sensor_diff
            else:
                sensor_diff = (readings[7] - wanted_distance) * velocity * 1/2
                print("4:",sensor_diff)
                lspeed, rspeed = velocity + sensor_diff, velocity - sensor_diff
        else:
            lspeed, rspeed = +velocity, +velocity
    # Giros
    else:
        lspeed, rspeed = -1, +1
        turning = True
        ticks_sincer_turn = 0
    return lspeed, rspeed, turning


def main(args=None):
    coppelia = robotica.Coppelia()
    robot = robotica.P3DX(coppelia.sim, 'PioneerP3DX')
    coppelia.start_simulation()
    turning = False
    global ticks_sincer_turn
    ticks_sincer_turn = 0
    while coppelia.is_running():
        readings = robot.get_sonar()
        lspeed, rspeed, turning = avoid(readings,turning)
        robot.set_speed(lspeed, rspeed)
    coppelia.stop_simulation()


if __name__ == '__main__':
    main()