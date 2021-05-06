#!/usr/bin/env python3

class Map:

    # will hold array of values representing the walls in the maze
    # the values will be integers.
    # TopLeftRightBottom convention with values as 8-4-2-1
    # basically a square with say top and bottom walls only will have value = 8(top) + 1(bottom) = 9

    def __init__(self, width, height, start, end, array=None):
        self.width = width
        self.height = height
        if self.check_coords(start) and self.check_coords(end):
            self.start = start
            self.end = end

            if array:
                self.array = array
            else:
                self.array = [ [0]*width for _ in range(height) ]

                for i in range(height):
                    self.add_left_wall((i, 0))
                    self.add_right_wall((i, width-1))

                for i in range(width):
                    self.add_top_wall((0, i))
                    self.add_bottom_wall((height-1, i))

    def check_coords(self, coords):
        if ((coords[0] < 0) or (coords[0] >= self.width)):
            print('Coords ' + str(coords) + ' are out of bounds')
            return False

        if ((coords[1] < 0) or (coords[1] >= self.height)):
            print('Coords ' + str(coords) + ' are out of bounds')
            return False

        return True

    def add_top_wall(self, coords):
        if self.check_coords(coords):
            if self.array[coords[0]][coords[1]] < 8:
                self.array[coords[0]][coords[1]] += 8

    def add_left_wall(self, coords):
        if self.check_coords(coords):
            if (self.array[coords[0]][coords[1]]%8) < 4:
                self.array[coords[0]][coords[1]] += 4

    def add_right_wall(self, coords):
        if self.check_coords(coords):
            if (self.array[coords[0]][coords[1]]%4) < 2:
                self.array[coords[0]][coords[1]] += 2

    def add_bottom_wall(self, coords):
        if self.check_coords(coords):
            if (self.array[coords[0]][coords[1]]%2) < 1:
                self.array[coords[0]][coords[1]] += 1

    def check_top_wall(self, coords):
        return (self.check_coords(coords)) and (self.array[coords[0]][coords[1]] >= 8)

    def check_left_wall(self, coords):
        return (self.check_coords(coords)) and ((self.array[coords[0]][coords[1]]%8) >= 4)

    def check_right_wall(self, coords):
        return (self.check_coords(coords)) and ((self.array[coords[0]][coords[1]]%4) >= 2)

    def check_bottom_wall(self, coords):
        return (self.check_coords(coords)) and ((self.array[coords[0]][coords[1]]%2) >= 1)
