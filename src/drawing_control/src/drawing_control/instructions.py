"""laser_instructions.py: Laser control instructions"""

import numpy as np

# Movement is from x=0,y=0 to x=1,y=1 scaled to drawing space. Velocity is in same units / second
# Velocity must be > 0
class MoveTo(object):
    __NAME__ = 'MoveTo'
    def __init__(self, x, y, vel):
        self.x = float(x)
        self.y = float(y)
        self.vel = float(vel)

    def __repr__(self):
        return '{} {} {} {}'.format(self.__NAME__, self.x, self.y, self.vel)

    @property
    def pos(self):
        return np.array([self.x, self.y])

    @pos.setter
    def pos(self, pos):
        self.x = pos[0]
        self.y = pos[1]

    def write(self, fd):
        fd.write('{}\n'.format(self.__repr__()))

# Turn laser on or off
class SetPower(object):
    __NAME__ = 'SetPower'
    def __init__(self, is_on):
        if type(is_on) is str:
            self.is_on = is_on == "True"
        else:
            self.is_on = bool(is_on)

    def __repr__(self):
        return '{} {}'.format(self.__NAME__, self.is_on)

    def write(self, fd):
        fd.write('{}\n'.format(self.__repr__()))

# Wait for duration seconds
class Wait(object):
    __NAME__ = 'Wait'
    def __init__(self, duration):
        self.duration = float(duration)

    def __repr__(self):
        return '{} {}'.format(self.__NAME__, self.duration)

    def write(self, fd):
        fd.write('{}\n'.format(self.__repr__()))


INSTRUCTIONS = { MoveTo.__NAME__: MoveTo,
                 Wait.__NAME__: Wait,
                 SetPower.__NAME__: SetPower}
def read_instr(line):
    fields = line.split()
    return INSTRUCTIONS[fields[0]](*fields[1:])


def run_test():
    instrs = [ SetPower(False),
               MoveTo(0, 0, .3),
               Wait(1),
               SetPower(True),
               MoveTo(0, 1, .3),
               MoveTo(1, -1, .3),
               MoveTo(1, 0, .3),
               MoveTo(0, 0, .3),
               Wait(1),
               SetPower(False)]

    with open('out/test.mvs', 'w') as fd:
        for instr in instrs:
            instr.write(fd)

    with open('out/test.mvs') as fd:
        for line in fd.readlines():
            print(read_instr(line))


if __name__ == "__main__":
    run_test()