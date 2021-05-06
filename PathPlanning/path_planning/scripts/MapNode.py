#!/usr/bin/env python3

import sys
import rospy

import tkinter

from path_planning.msg import direction, map_detail
from MapClass import Map

class MapNode:
    def __init__(self):
        map_file_name = 'map1.txt' # change this to load in different map for testing

        f = open("src/path_planning/maps/map1.txt", "r")
        dimensions = [int(i) for i in f.readline().split()]
        coords = [int(i) for i in f.readline().split()]
        array = []

        for i in range(dimensions[1]):
            array.append([int(i) for i in f.readline().split()])

        self.map = Map(dimensions[0], dimensions[1], (coords[0],coords[1]), (coords[2],coords[3]), array)
        self.current = self.map.start
        self.walls = Map(self.map.width, self.map.height, self.map.start, self.map.end)
        self.print_root = tkinter.Tk()
        self.print_canvas = tkinter.Canvas(self.print_root, bg="white", height=(100+self.walls.height*50), width=(100+self.walls.width*50))
        self.print_canvas.pack()
        self.update_print()

        self.direction_listener = rospy.Subscriber("/direction", direction, self.direction_callback)
        self.walls_publisher = rospy.Publisher("/walls", map_detail, queue_size=1)

    def direction_callback(self, msg):
        print(msg)
        if msg.direction == 'up':
            if self.current[1] > 1:
                self.current = (self.current[0], self.current[1]-1)
        elif msg.direction == 'left':
            if self.current[0] > 1:
                self.current = (self.current[0]-1, self.current[1])
        elif msg.direction == 'right':
            if self.current[0] < self.map.width-1:
                self.current = (self.current[0], self.current[1]+1)
        elif msg.direction == 'down':
            if self.current[0] < self.map.height-1:
                self.current = (self.current[0]+1, self.current[1])

        if self.current == self.map.end:
            print('Goal reached')

        self.walls.array[self.current[0]][self.current[1]] = self.map.array[self.current[0]][self.current[1]]

        detail = map_detail()
        detail.width = self.map.width
        detail.height = self.map.height
        detail.end_x = self.map.end[0]
        detail.end_y = self.map.end[1]
        detail.current_x = self.current[0]
        detail.current_y = self.current[1]
        detail.current_value = self.map.array[self.current[0]][self.current[1]]
        self.walls_publisher.publish(detail)

        self.update_print()

    def update_print(self):
        self.print_canvas.delete("all")

        temp = self.walls
        self.print_canvas.create_rectangle((50+(temp.end[1]*50)), (50+(temp.end[0]*50)), (50+((temp.end[1]+1)*50)), (50+((temp.end[0]+1)*50)), fill="#00ff00")
        self.print_canvas.create_rectangle((50+(self.current[1]*50)), (50+(self.current[0]*50)), (50+((self.current[1]+1)*50)), (50+((self.current[0]+1)*50)), fill="#0000ff")

        for i in range(temp.width):
            for j in range(temp.height):
                # self.print_canvas.create_text((75+(i*50)), (75+(j*50)), text=str(temp.array[j][i]), fill="#000000")

                if temp.check_top_wall((j,i)):
                    self.print_canvas.create_line((50+(i*50)), (50+(j*50)), (50+((i+1)*50)), (50+(j*50)), fill="#000000", width=2)

                if temp.check_left_wall((j,i)):
                    self.print_canvas.create_line((50+(i*50)), (50+(j*50)), (50+(i*50)), (50+((j+1)*50)), fill="#000000", width=2)

                if temp.check_right_wall((j,i)):
                    self.print_canvas.create_line((50+((i+1)*50)), (50+(j*50)), (50+((i+1)*50)), (50+((j+1)*50)), fill="#000000", width=2)

                if temp.check_bottom_wall((j,i)):
                    self.print_canvas.create_line((50+(i*50)), (50+((j+1)*50)), (50+((i+1)*50)), (50+((j+1)*50)), fill="#000000", width=2)

        self.print_canvas.update()


if __name__ == '__main__':
    rospy.init_node('map_node')
    a = MapNode()
    a.print_root.mainloop()
