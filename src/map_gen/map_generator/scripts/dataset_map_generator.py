#!/usr/bin/env python3
import math
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry

class DatasetMapGenerator(object):
    def __init__(self):
        self.map = None
        self.points = []
        self.occupied_cells = set()
        self.has_map = False
        
        self.dimx = rospy.get_param("~map/x_size", 100.0)
        self.dimy = rospy.get_param("~map/y_size", 100.0)
        self.dimz = rospy.get_param("~map/z_size", 1.5)
        self.z_origin = rospy.get_param("~map/z_origin", -1.0)
        self.resolution = rospy.get_param("~resolution", 0.5)
        self.map_type = rospy.get_param("~map_type", "clutter")
        self.fill_level = rospy.get_param("~fill_level", 0.1)
        self.all_map_topic = rospy.get_param("~all_map_topic", "random_forest/all_map")
        self.fail_limit = rospy.get_param("~fail_limit", 10000)
        self.gap_width = rospy.get_param("~gap_width", 20)
        
        self.random = np.random
        self.all_map_pub = rospy.Publisher(self.all_map_topic, PointCloud2, queue_size=1)
        self.rate = rospy.Rate(1.0)
        self.floor_bias = 0.0

    def add_box(self, size, position, solid=True):
        ''' size: [x,y,z], position: [x,y,z] --- center position '''
        position[2] -= self.floor_bias
        x_low = math.floor((position[0] - size[0] / 2) / self.resolution) * self.resolution + 0.5 * self.resolution
        x_high = math.floor((position[0] + size[0] / 2) / self.resolution) * self.resolution + 0.5 * self.resolution
        y_low = math.floor((position[1] - size[1] / 2) / self.resolution) * self.resolution + 0.5 * self.resolution
        y_high = math.floor((position[1] + size[1] / 2) / self.resolution) * self.resolution + 0.5 * self.resolution
        z_low = math.floor((position[2] - size[2] / 2) / self.resolution) * self.resolution + 0.5 * self.resolution
        z_high = math.floor((position[2] + size[2] / 2) / self.resolution) * self.resolution + 0.5 * self.resolution

        x = x_low
        while x <= x_high:
            y = y_low
            while y <= y_high:
                self.occupied_cells.add((x, y))
                z = z_low
                while z <= z_high:
                    if solid or (math.fabs(x - x_low) < self.resolution) or (math.fabs(x - x_high) < self.resolution) \
                        or (math.fabs(y - y_low) < self.resolution) or (math.fabs(y - y_high) < self.resolution) \
                        or (math.fabs(z - z_low) < self.resolution) or (math.fabs(z - z_high) < self.resolution):
                        self.points.append([x, y, z])
                    z += self.resolution
                y += self.resolution
            x += self.resolution
        return 0 # Area is tracked globally now

    def add_cylinder(self, size, position, solid=True):
        ''' size: [r, h], position: [x,y,z] --- center position '''
        position[2] -= self.floor_bias
        center_x = position[0]
        center_y = position[1]
        z_low = math.floor((position[2] - size[1] / 2) / self.resolution) * self.resolution + 0.5 * self.resolution
        z_high = math.floor((position[2] + size[1] / 2) / self.resolution) * self.resolution + 0.5 * self.resolution

        radius_num = math.floor(size[0] / self.resolution)
        x = -radius_num
        while x <= radius_num:
            y = -radius_num
            while y <= radius_num:
                radius2 = x ** 2 + y ** 2
                if radius2 < (radius_num + 0.5) ** 2:
                    self.occupied_cells.add((center_x + x * self.resolution, center_y + y * self.resolution))
                    z = z_low
                    while z <= z_high:
                        if solid or radius2 > (radius_num - 0.5) ** 2 or \
                            (math.fabs(z - z_low) < self.resolution) or (math.fabs(z - z_high) < self.resolution):
                            self.points.append([center_x + x * self.resolution, center_y + y * self.resolution, z])
                        z += self.resolution
                y += 1
            x += 1
        return 0

    def add_ushape(self, position, scale, direction):
        ''' direction: 0=+x, 1=-x, 2=+y, 3=-y '''
        vol = 0
        w = 1.0 * scale
        h = 1.0 * scale
        t = 0.2 * scale
        z = 0.0
        wall_height = 3.0
        
        if direction == 0:
            vol += self.add_box([t, h, wall_height], [position[0] - w/2, position[1], z])
            vol += self.add_box([w, t, wall_height], [position[0], position[1] - h/2, z])
            vol += self.add_box([w, t, wall_height], [position[0], position[1] + h/2, z])
        elif direction == 1:
            vol += self.add_box([t, h, wall_height], [position[0] + w/2, position[1], z])
            vol += self.add_box([w, t, wall_height], [position[0], position[1] - h/2, z])
            vol += self.add_box([w, t, wall_height], [position[0], position[1] + h/2, z])
        elif direction == 2:
            vol += self.add_box([w, t, wall_height], [position[0], position[1] - h/2, z])
            vol += self.add_box([t, h, wall_height], [position[0] - w/2, position[1], z])
            vol += self.add_box([t, h, wall_height], [position[0] + w/2, position[1], z])
        elif direction == 3:
            vol += self.add_box([w, t, wall_height], [position[0], position[1] + h/2, z])
            vol += self.add_box([t, h, wall_height], [position[0] - w/2, position[1], z])
            vol += self.add_box([t, h, wall_height], [position[0] + w/2, position[1], z])
        return vol

    def make_map_from_type(self):
        target_area = self.dimx * self.dimy * self.fill_level
        res2 = self.resolution ** 2
        rospy.loginfo("Making map: {} ({} fill)".format(self.map_type, self.fill_level))
        z_size = self.dimz
        z_center = self.z_origin + z_size / 2.0

        # Boundary Boxes
        self.add_box([0.10, self.dimy, z_size], [-self.dimx / 2, 0, z_center], solid=True)
        self.add_box([0.10, self.dimy, z_size], [self.dimx / 2, 0, z_center], solid=True)
        self.add_box([self.dimx, 0.10, z_size], [0, self.dimy / 2, z_center], solid=True)
        self.add_box([self.dimx, 0.10, z_size], [0, -self.dimy / 2, z_center], solid=True)

        fail_limit = self.fail_limit
        fail_count = 0

        if self.map_type == "clutter":
            while len(self.occupied_cells) * res2 < target_area and fail_count < fail_limit:
                sizex = self.random.rand() * 1.5 + 0.5
                sizey = self.random.rand() * 1.5 + 0.5
                posx = (self.random.rand() - 0.5) * (self.dimx - sizex)
                posy = (self.random.rand() - 0.5) * (self.dimy - sizey)
                self.add_box([sizex, sizey, z_size], [posx, posy, z_center])
                fail_count += 1

        elif self.map_type == "narrow":
            # 2 big box obstacles (barriers) spanning from 15% to 85% of map height
            # Between them is a corridor with gap_width = 20
            # Then fill box obstacles = fill level up * remaining volume
            gap_w = 20.0
            wall_y_low = -self.dimy / 2 + 0.15 * self.dimy
            wall_y_high = -self.dimy / 2 + 0.85 * self.dimy
            wall_height = wall_y_high - wall_y_low
            wall_center_y = (wall_y_low + wall_y_high) / 2.0
            
            left_wall_w = (self.dimx - gap_w) / 2.0
            right_wall_w = (self.dimx - gap_w) / 2.0
            
            self.add_box([left_wall_w, wall_height, z_size], [-self.dimx/2 + left_wall_w/2, wall_center_y, z_center], solid=True)
            self.add_box([right_wall_w, wall_height, z_size], [self.dimx/2 - right_wall_w/2, wall_center_y, z_center], solid=True)
            
            cells_after_barriers = len(self.occupied_cells)
            target_fill_area = self.fill_level * (self.dimx * self.dimy - cells_after_barriers * res2)
            
            while (len(self.occupied_cells) - cells_after_barriers) * res2 < target_fill_area and fail_count < fail_limit:
                sizex = self.random.rand() * 1.5 + 0.5
                sizey = self.random.rand() * 1.5 + 0.5
                posx = (self.random.rand() - 0.5) * (self.dimx - sizex)
                posy = (self.random.rand() - 0.5) * (self.dimy - sizey)
                self.add_box([sizex, sizey, z_size], [posx, posy, z_center])
                fail_count += 1

        elif self.map_type == "rooms":
            rows = 3
            cols = 4
            barrier_area = 0
            # Horizontal walls
            for i in range(1, rows):
                y = -self.dimy/2 + i * (self.dimy / rows)
                # Door in each segment
                for j in range(cols):
                    x_start = -self.dimx/2 + j * (self.dimx/cols)
                    x_end = -self.dimx/2 + (j+1) * (self.dimx/cols)
                    x_mid = (x_start + x_end) / 2.0
                    # Left part
                    len_left = (x_end - x_start - self.gap_width) / 2.0
                    barrier_area += self.add_box([len_left, 1.0, z_size], [x_start + len_left/2, y, z_center], solid=True)
                    # Right part
                    barrier_area += self.add_box([len_left, 1.0, z_size], [x_end - len_left/2, y, z_center], solid=True)
            
            # Vertical walls
            for j in range(1, cols):
                x = -self.dimx/2 + j * (self.dimx / cols)
                for i in range(rows):
                    y_start = -self.dimy/2 + i * (self.dimy/rows)
                    y_end = -self.dimy/2 + (i+1) * (self.dimy/rows)
                    y_mid = (y_start + y_end) / 2.0
                    len_low = (y_end - y_start - self.gap_width) / 2.0
                    self.add_box([1.0, len_low, z_size], [x, y_start + len_low/2, z_center], solid=True)
                    self.add_box([1.0, len_low, z_size], [x, y_end - len_low/2, z_center], solid=True)
            
            while len(self.occupied_cells) * res2 < target_area and fail_count < fail_limit:
                sizex = self.random.rand() * 1.5 + 0.5
                sizey = self.random.rand() * 1.5 + 0.5
                posx = (self.random.rand() - 0.5) * (self.dimx - sizex)
                posy = (self.random.rand() - 0.5) * (self.dimy - sizey)
                self.add_box([sizex, sizey, z_size], [posx, posy, z_center])
                fail_count += 1

        elif self.map_type == "blobs":
            while len(self.occupied_cells) * res2 < target_area and fail_count < fail_limit:
                radius = self.random.rand() * 1.5 + 0.5
                posx = (self.random.rand() - 0.5) * (self.dimx - radius*2)
                posy = (self.random.rand() - 0.5) * (self.dimy - radius*2)
                self.add_cylinder([radius, z_size], [posx, posy, z_center], solid=True)
                fail_count += 1

        elif self.map_type == "flappy":
            # 4 set of barriers, small height, gap = gap_width
            # filled = volume of barriers + volume of obstacles
            barrier_area = 0
            for i in range(4):
                posy = -self.dimy/2 + (i + 1) * (self.dimy / 5.0)
                wall_thick = 3.0 # small height
                left_len = self.random.rand() * (self.dimx - self.gap_width - 4.0) + 2.0
                right_len = self.dimx - self.gap_width - left_len
                self.add_box([left_len, wall_thick, z_size], [-self.dimx/2 + left_len/2, posy, z_center], solid=True)
                self.add_box([right_len, wall_thick, z_size], [self.dimx/2 - right_len/2, posy, z_center], solid=True)
            
            while len(self.occupied_cells) * res2 < target_area and fail_count < fail_limit:
                sizex = self.random.rand() * 1.5 + 0.5
                sizey = self.random.rand() * 1.5 + 0.5
                posx = (self.random.rand() - 0.5) * (self.dimx - sizex)
                posy = (self.random.rand() - 0.5) * (self.dimy - sizey)
                self.add_box([sizex, sizey, z_size], [posx, posy, z_center])
                fail_count += 1

        elif self.map_type == "ushape":
            scale = 4.0
            while len(self.occupied_cells) * res2 < target_area and fail_count < fail_limit:
                posx = (self.random.rand() - 0.5) * (self.dimx - scale)
                posy = (self.random.rand() - 0.5) * (self.dimy - scale)
                direction = self.random.randint(0, 4)
                self.add_ushape([posx, posy], scale, direction)
                fail_count += 1
                
        # Transfer to PCL
        self.map = PointCloud2()
        self.map.height = 1
        self.map.width = len(self.points)
        self.map.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)]
        self.map.point_step = 12
        self.map.row_step = 12 * len(self.points)
        self.map.is_bigendian = False
        self.map.is_dense = False
        self.map.data = np.asarray(self.points, np.float32).tobytes()
        self.map.header.frame_id = "map"
        self.map.header.stamp = rospy.Time.now()
        self.has_map = True
        rospy.loginfo("finish making map, final area: {}".format(len(self.occupied_cells) * res2))
        return True

    def publish_map(self):
        if not self.has_map: return False
        self.map.header.stamp = rospy.Time.now()
        self.all_map_pub.publish(self.map)
        return True

def main():
    rospy.init_node('dataset_map_generator_node', anonymous=True)
    map_maker = DatasetMapGenerator()
    map_maker.make_map_from_type()
    while not rospy.is_shutdown():
        map_maker.publish_map()
        map_maker.rate.sleep()

if __name__ == '__main__':
    main()
