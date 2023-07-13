#!/usr/bin/env python3

import rospy
import tf
from pickplace_include.srv import pickplace, pickplaceRequest
from bhampick.srv import objectlist, objectlistResponse

class haiya:
    def __init__(self):
        self.objs = []
        #self.objs = [x, y, z, dimx, dimy, dimz]
        self.shelf = [[0.60, 0.95, 0, 1, 0.01, 0.1], [0.60, 0.95, 0, 1, 0.11, 0.2], [0.60, 0.95, 0, 1, 0.21, 0.3],
                      [0.60, 0.95, 0, 1, 0.31, 0.4], [0.60, 0.95, 0, 1, 0.41, 0.5], [0.60, 0.95, 0, 1, 0.51, 0.6], 
                      [0.60, 0.95, 0, 1, 0.61, 0.7]]
        #self.shelf = [[x1, x2, y1, y2, z1, z2], [..]]
        self.free_space = []
        self.tf_listener1 = tf.TransformListener()
        self.dimy = 0
        self.dimx = 0
        self.dimz = 0
        self.client = rospy.ServiceProxy('placeit', pickplace)
        self.service = rospy.Service('objects', objectlist, self.callback)
        self.srv_obj = pickplaceRequest()
        self.setshelf(self.shelf)
        
    def callback(self, data):
        float_list = [round(num, 3) for num in data.objects]
        dat = [float_list[i:i+6] for i in range(0, len(float_list), 6)]
        
        if len(dat) == 1 and data.place:
            self.dimx = dat[0][3]
            self.dimy = dat[0][4]
            self.dimz = dat[0][5]
            print(self.dimx, self.dimy, self.dimz)
            self.get_position(data.floor)
            self.objs.append(dat[0])
        else:
            for i in dat:
                self.objs.append(i)
        self.find_freespace(self.objs, self.shelf)
        x = objectlistResponse()
        return x
    
    def setshelf(self, shelf):
        self.tf_listener1.waitForTransform("base_link", "cr3_base_link", rospy.Time(), rospy.Duration(1.0))
        (trans, rot) = self.tf_listener1.lookupTransform("base_link", "cr3_base_link", rospy.Time(0))
        
        for i in shelf:
            i[4] = round(i[4]-trans[2],4)
            i[5] = round(i[5]-trans[2],4)
        
        print(self.shelf)
        
    def find_level(self, objs, shelf):
        levels = []
        if len(objs) == 0:
            return levels
        for obj in objs:
            posz = obj[2]
            for idx, sh in enumerate(shelf):
                if sh[4] < posz < sh[5]:
                    levels.append(idx)
                    break
        return levels
    
    def find_occupied(self, objs):
        occspace = []
        if len(objs) == 0:
            return occspace
        for i in objs:
            if i[3] > i[4]:
                occspace.append([i[1] - i[3] / 2, i[1] + i[3] / 2])
            else:
                occspace.append([i[1] - i[4] / 2, i[1] + i[4] / 2])
        return occspace
    
    def find_freespace(self, objs, shelf):
        c = self.find_level(objs, shelf)
        objlength = self.find_occupied(objs)
        space = [[[floor[2], floor[3]]] for floor in shelf]
        
        for idx, obj in enumerate(c):
            this_floor = space[obj]
            for idx2, freespace in enumerate(this_floor):
                if objlength[idx][0] >= freespace[0] and objlength[idx][1] <= freespace[1]:
                    this_floor[idx2] = [round(freespace[0],3), round(objlength[idx][0],3)]
                    this_floor.append([round(objlength[idx][1],3), round(freespace[1],3)])
                    this_floor.sort(key=lambda x: x[1])
                elif objlength[idx][0] < freespace[0] and objlength[idx][1] <= freespace[1] and objlength[idx][1] > freespace[0]:
                    this_floor[idx2] = [round(objlength[idx][1],3), round(freespace[1],3)]
                elif objlength[idx][0] >= freespace[0] and objlength[idx][0] < freespace[1] and objlength[idx][1] > freespace[1]:
                    this_floor[idx2] = [round(freespace[0],3), round(objlength[idx][0],3)]
                elif objlength[idx][0] <= freespace[0] and objlength[idx][1] >= freespace[1]:
                    this_floor[idx2].remove(freespace)
                else : pass
            space[obj] = this_floor
        self.free_space = space
        print(self.free_space)
        return space
    
    def get_position(self, floor):
        possible_space = self.free_space[floor]
        for i in possible_space:
            if i[1]-i[0] >= (self.dimy + 0.05):
                self.srv_obj.posercam.header.frame_id = "cr3_base_link"
                self.srv_obj.posercam.pose.position.x = self.shelf[floor][0]+0.05
                self.srv_obj.posercam.pose.position.y = i[0] - self.dimy/2 - 0.05
                self.srv_obj.posercam.pose.position.z = self.shelf[floor][4]
                
                self.srv_obj.posercam.pose.orientation.x = 0
                self.srv_obj.posercam.pose.orientation.y = 0
                self.srv_obj.posercam.pose.orientation.z = 0
                self.srv_obj.posercam.pose.orientation.w = 1
                
                self.srv_obj.dimension.x = self.dimx
                self.srv_obj.dimension.y = self.dimy
                self.srv_obj.dimension.z = self.dimz
                
                self.srv_obj.check = False
                print(self.srv_obj)
                try:
                    rospy.wait_for_service('placeit')
                    response = self.client(self.srv_obj)
                except rospy.ServiceException as e:
                    rospy.logerr("Failed to call service: %s", str(e))
                    exit(1)
                return

if __name__ == "__main__":
    rospy.init_node("posecalculation")
    k = haiya()
    rospy.spin()

