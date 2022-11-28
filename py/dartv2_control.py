#!/usr/bin/python
# -*- coding: utf-8 -*-

# elementary functions to control DART V2
#   delta_front_odometers
#   delta_rear_odometers

import sys
import time

class DartV2Control():
    def __init__(self,dart_bot):
        self.__dart_bot = dart_bot
        print ("Control: Dartv2 robot is working as ",self.__dart_bot.exec_robot())
        self.encoders_front_left_mem, self.encoders_front_right_mem = dart_bot.powerboard.get_front_encoders()
        self.encoders_front_left_last, self.encoders_front_right_last = dart_bot.powerboard.get_front_encoders()
        self.encoders_rear_left_mem, self.encoders_rear_right_mem = dart_bot.encoders.read_encoders()
        self.encoders_rear_left_last, self.encoders_rear_right_last = dart_bot.encoders.read_encoders()

    def delta_odometers_without_jumps (self,odo_mem,odo_last):
        deltaOdo = odo_last - odo_mem
        if deltaOdo > 32767:  # check if high positive jumps !!
            deltaOdo -= 65536 #  remove the jump (2^16)
        if deltaOdo < -32767: # same for high negative jumps
            deltaOdo += 65536 #  remove the jump (2^16)
        return deltaOdo

    def get_front_encoders (self,init=False):
        self.encoders_front_left_last, self.encoders_front_right_last = self.__dart_bot.powerboard.get_front_encoders()
        if init:
            self.encoders_front_left_mem = self.encoders_front_left_last
            self.encoders_front_right_mem = self.encoders_front_right_last            
        return self.encoders_front_left_last, self.encoders_front_right_last

    def delta_front_odometers(self,side="both"):
        if side == "both":
            deltaOdoLeft = self.delta_odometers_without_jumps (
                self.encoders_front_left_mem,self.encoders_front_left_last)
            deltaOdoRight = self.delta_odometers_without_jumps (
                self.encoders_front_right_mem,self.encoders_front_right_last)
            self.encoders_front_left_mem = self.encoders_front_left_last
            self.encoders_front_right_mem = self.encoders_front_right_last
            return [deltaOdoLeft,deltaOdoRight]
        elif side == "left":
            deltaOdoLeft = self.delta_odometers_without_jumps (
                self.encoders_front_left_mem,self.encoders_front_left_last)
            self.encoders_front_left_mem = self.encoders_front_left_last
            return deltaOdoLeft
        if side == "right":
            deltaOdoRight = self.delta_odometers_without_jumps (
                self.encoders_front_right_mem,self.encoders_front_right_last)
            self.encoders_front_right_mem = self.encoders_front_right_last
            return deltaOdoRight

    def front_encoders_get_memory (self):
        return self.encoders_front_left_mem, self.encoders_front_right_mem

    def front_encoders_set_memory (self,vleft,vright):
        self.encoders_front_left_mem = vleft
        self.encoders_front_right_mem = vright

    def get_rear_encoders (self,init=False):
        self.encoders_rear_left_last, self.encoders_rear_right_last = self.__dart_bot.encoders.read_encoders()
        if init:
            self.encoders_rear_left_mem = self.encoders_rear_left_last
            self.encoders_rear_right_mem = self.encoders_rear_right_last            
        return self.encoders_rear_left_last, self.encoders_rear_right_last

    def delta_rear_odometers(self,side="both"):
        if side == "both":
            deltaOdoLeft = self.delta_odometers_without_jumps (
                self.encoders_rear_left_mem,self.encoders_rear_left_last)
            deltaOdoRight = self.delta_odometers_without_jumps (
                self.encoders_rear_right_mem,self.encoders_rear_right_last)
            self.encoders_rear_left_mem = self.encoders_rear_left_last
            self.encoders_rear_right_mem = self.encoders_rear_right_last
            return [deltaOdoLeft,deltaOdoRight]
        elif side == "left":
            deltaOdoLeft = self.delta_odometers_without_jumps (
                self.encoders_rear_left_mem,self.encoders_rear_left_last)
            self.encoders_rear_left_mem = self.encoders_rear_left_last
            return deltaOdoLeft
        if side == "right":
            deltaOdoRight = self.delta_odometers_without_jumps (
                self.encoders_rear_right_mem,self.encoders_rear_right_last)
            self.encoders_rear_right_mem = self.encoders_rear_right_last
            return deltaOdoRight

    def rear_encoders_get_memory (self):
        return self.encoders_rear_left_mem, self.encoders_rear_right_mem

    def rear_encoders_set_memory (self,vleft,vright):
        self.encoders_rear_left_mem = vleft
        self.encoders_rear_right_mem = vright

