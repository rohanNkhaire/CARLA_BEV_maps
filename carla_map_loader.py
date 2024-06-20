import carla
import logging
import pygame
import numpy as np
import math

COLOR_WHITE = pygame.Color(255, 255, 255)
COLOR_BLACK = pygame.Color(0, 0, 0)
COLOR_ALUMINIUM = pygame.Color(136, 138, 133)
COLOR_PLUM = pygame.Color(117, 80, 123)
COLOR_SCARLET = pygame.Color(239, 41, 41)

# Carla environment
class CarlaMapLoader():
	def __init__(self, host, port, town):

		try:
			self.client = carla.Client(host, port)
			self.client.set_timeout(100.0)
			self.client.load_world(map_name=town)
			self.world = self.client.get_world()
			self.map = self.world.get_map()
		except RuntimeError as msg:
			logging.error(msg)

	def initialize_map(self, pixels_per_meter):
		self._pixels_per_meter = float(pixels_per_meter)
		self.scale = 1.0
		self._line_width = 2

		if self._pixels_per_meter < 3:
			self._line_width = 1

		waypoints = self.map.generate_waypoints(2)
		margin = 50
		max_x = max(waypoints, key=lambda x: x.transform.location.x).transform.location.x + margin
		max_y = max(waypoints, key=lambda x: x.transform.location.y).transform.location.y + margin
		min_x = min(waypoints, key=lambda x: x.transform.location.x).transform.location.x - margin
		min_y = min(waypoints, key=lambda x: x.transform.location.y).transform.location.y - margin		
		self.width = max(max_x - min_x, max_y - min_y)
		self._world_offset = (min_x, min_y)
		self.width_in_pixels = int(self._pixels_per_meter * self.width)

		# Initialize Pygame
		pygame.init()
		pygame.font.init()
		self.display = pygame.display.set_mode((self.width_in_pixels, self.width_in_pixels), pygame.HWSURFACE | pygame.DOUBLEBUF)

	def draw_road_map(self):

		self.sidewalk_surface = pygame.Surface((self.width_in_pixels, self.width_in_pixels)).convert()
		self.parking_surface = pygame.Surface((self.width_in_pixels, self.width_in_pixels)).convert()
		self.big_map_surface = pygame.Surface((self.width_in_pixels, self.width_in_pixels)).convert()
		self.big_lane_surface = pygame.Surface((self.width_in_pixels, self.width_in_pixels)).convert()
		self.drivable_surface = pygame.Surface((self.width_in_pixels, self.width_in_pixels)).convert()	
		self.big_map_surface.fill(COLOR_BLACK)
		self.big_lane_surface.fill(COLOR_BLACK)
		self.sidewalk_surface.fill(COLOR_BLACK)
		self.parking_surface.fill(COLOR_BLACK)
		self.drivable_surface.fill(COLOR_BLACK)
		precision = 0.02

		def draw_lane_marking(surface, points, solid=True):
			if solid:
				pygame.draw.lines(surface, COLOR_WHITE, False, points, self._line_width)
			else:
				broken_lines = [x for n, x in enumerate(zip(*(iter(points), ) * 20)) if n % 3 == 0]
				for line in broken_lines:
                    # pygame.draw.lines(surface, COLOR_ORANGE_0, False, line, 2)
					pygame.draw.lines(surface, COLOR_WHITE, False, line, self._line_width)

		def does_cross_solid_line(waypoint, shift):
			w = self.map.get_waypoint(lateral_shift(waypoint.transform, shift), project_to_road=False)
			if w is None or w.road_id != waypoint.road_id:
				return True
			else:
				return (w.lane_id * waypoint.lane_id < 0) or w.lane_id == waypoint.lane_id

		topology = [x[0] for x in self.map.get_topology()]
		topology = sorted(topology, key=lambda w: w.transform.location.z)

		for waypoint in topology:
			waypoints = [waypoint]
			nxt = waypoint.next(precision)[0]
			while nxt.road_id == waypoint.road_id:
				waypoints.append(nxt)
				nxt = nxt.next(precision)[0]	

			left_marking = [lateral_shift(w.transform, -w.lane_width * 0.5) for w in waypoints]
			right_marking = [lateral_shift(w.transform, w.lane_width * 0.5) for w in waypoints]	
			polygon = left_marking + [x for x in reversed(right_marking)]
			polygon = [self.world_to_pixel(x) for x in polygon]

			if len(polygon) > 2:
				pygame.draw.polygon(self.big_map_surface, COLOR_WHITE, polygon)
				pygame.draw.polygon(self.big_map_surface, COLOR_WHITE, polygon)
				pygame.draw.polygon(self.drivable_surface, COLOR_WHITE, polygon)
				pygame.draw.polygon(self.drivable_surface, COLOR_WHITE, polygon)

			if not waypoint.is_intersection:
				if (len(left_marking) == 1):
					continue
				sample = waypoints[int(len(waypoints) / 2)]
				draw_lane_marking(
                    self.big_lane_surface, [self.world_to_pixel(x) for x in left_marking],
                    does_cross_solid_line(sample, -sample.lane_width * 1.1)
                )
				draw_lane_marking(
					self.big_lane_surface, [self.world_to_pixel(x) for x in right_marking],
					does_cross_solid_line(sample, sample.lane_width * 1.1)
                )

			parking = [[], []]
			sidewalk = [[], []]
			stop = [[], []]

			for w in waypoints:
				l = w.get_left_lane()
				while l and l.lane_type != carla.LaneType.Driving:
		
					if l.lane_type == carla.LaneType.Parking:
						parking[0].append(l)				
					if l.lane_type == carla.LaneType.Sidewalk:
						sidewalk[0].append(l)
					if l.lane_type == carla.LaneType.Stop:
						stop[0].append(l)				

					l = l.get_left_lane()

				r = w.get_right_lane()
				while r and r.lane_type != carla.LaneType.Driving:
				
					if r.lane_type == carla.LaneType.Parking:
						parking[1].append(r)					
					if r.lane_type == carla.LaneType.Sidewalk:
						sidewalk[1].append(r)
					if r.lane_type == carla.LaneType.Stop:
						stop[1].append(r)					

					r = r.get_right_lane()

			self.draw_sidewalks(sidewalk)
			self.draw_parking(parking)				

		self.display.blit(self.big_map_surface, (0,0))


		sidewalk_area = save_map(self.sidewalk_surface)
		parking_area = save_map(self.parking_surface)
		driving_area = save_map(self.drivable_surface)
		lanes = save_map(self.big_lane_surface)

		return driving_area, lanes, sidewalk_area, parking_area

	def draw_sidewalks(self, sidewalk):

		for side in sidewalk:
			lane_left_side = [lateral_shift(w.transform, -w.lane_width * 0.5) for w in side]
			lane_right_side = [lateral_shift(w.transform, w.lane_width * 0.5) for w in side]		
			polygon = lane_left_side + [x for x in reversed(lane_right_side)]
			polygon = [self.world_to_pixel(x) for x in polygon]

			if len(polygon) > 2:
				pygame.draw.polygon(self.big_map_surface, COLOR_ALUMINIUM, polygon)
				pygame.draw.polygon(self.big_map_surface, COLOR_ALUMINIUM, polygon)
				pygame.draw.polygon(self.sidewalk_surface, COLOR_ALUMINIUM, polygon)
				pygame.draw.polygon(self.sidewalk_surface, COLOR_ALUMINIUM, polygon)

		return 		

	def draw_parking(self, parking):
		
		for side in parking:
			lane_left_side = [lateral_shift(w.transform, -w.lane_width * 0.5) for w in side]
			lane_right_side = [lateral_shift(w.transform, w.lane_width * 0.5) for w in side]		
			polygon = lane_left_side + [x for x in reversed(lane_right_side)]
			polygon = [self.world_to_pixel(x) for x in polygon]

			if len(polygon) > 2:
				pygame.draw.polygon(self.big_map_surface, COLOR_PLUM, polygon, 5)
				pygame.draw.polygon(self.big_map_surface, COLOR_PLUM, polygon)
				pygame.draw.polygon(self.parking_surface, COLOR_PLUM, polygon, 5)
				pygame.draw.polygon(self.parking_surface, COLOR_PLUM, polygon)

		parking_area = save_map(self.parking_surface)

		return parking_area		

	def draw_crosswalks(self):

		self.crosswalk_surface = pygame.Surface((self.width_in_pixels, self.width_in_pixels)).convert()
		self.crosswalk_surface.fill(COLOR_BLACK)

		crosswalk_area = []
		crosswalks = self.map.get_crosswalks()
		for crswlk in crosswalks:
			if len(crosswalk_area) == 0:
				first_pt = crswlk

			crosswalk_area.append(crswlk)

			if (first_pt.x == crswlk.x and first_pt.y == crswlk.y and len(crosswalk_area) > 1):
				polygon = [self.world_to_pixel(x) for x in crosswalk_area]
				pygame.draw.polygon(self.big_map_surface, COLOR_SCARLET, polygon, 5)
				pygame.draw.polygon(self.big_map_surface, COLOR_SCARLET, polygon)
				pygame.draw.polygon(self.crosswalk_surface, COLOR_SCARLET, polygon, 5)
				pygame.draw.polygon(self.crosswalk_surface, COLOR_SCARLET, polygon)

				crosswalk_area.clear()

		self.display.blit(self.big_map_surface, (0,0))

		crosswalk_area = save_map(self.crosswalk_surface)

		return crosswalk_area
				

	def world_to_pixel(self, location, offset=(0, 0)):
		x = self.scale * self._pixels_per_meter * (location.x - self._world_offset[0])
		y = self.scale * self._pixels_per_meter * (location.y - self._world_offset[1])
		return int(x - offset[0]), int(y - offset[1])
        
	def world_to_pixel_width(self, width):
		return int(self.scale * self._pixels_per_meter * width)
	
	def get_traffic_light_waypoints(self, traffic_light):
		"""
		get area of a given traffic light
		"""
		base_transform = traffic_light.get_transform()
		base_rot = base_transform.rotation.yaw
		area_loc = base_transform.transform(traffic_light.trigger_volume.location)		
		# Discretize the trigger box into points
		area_ext = traffic_light.trigger_volume.extent
		x_values = np.arange(-0.9 * area_ext.x, 0.9 * area_ext.x, 1.0)  # 0.9 to avoid crossing to adjacent lanes		
		area = []
		for x in x_values:
			point = rotate_point(carla.Vector3D(x, 0, area_ext.z), base_rot)
			point_location = area_loc + carla.Location(x=point.x, y=point.y)
			area.append(point_location)

    	# Get the waypoints of these points, removing duplicates
		ini_wps = []
		for pt in area:
			wpx = self.map.get_waypoint(pt)
			# As x_values are arranged in order, only the last one has to be checked
			if not ini_wps or ini_wps[-1].road_id != wpx.road_id or ini_wps[-1].lane_id != wpx.lane_id:
				ini_wps.append(wpx)

    	# Advance them until the intersection
		wps = []
		for wpx in ini_wps:
			while not wpx.is_intersection:
				next_wp = wpx.next(0.5)[0]
				if next_wp and not next_wp.is_intersection:
					wpx = next_wp
				else:
					break
			wps.append(wpx)

		return wps
	
def save_map(x):
	np_mat = pygame.surfarray.array3d(x)
	# Convert to 1d array
	np_mat = np_mat[:, :, 0]
	# Divide by max to get ones and zeros
	np_mat = np_mat/np_mat.max()
	# rid the matrix of non-zero and non-one elements
	rows, cols = np_mat.shape
	for i in range(rows):
		for j in range(cols):
			if np_mat[i][j] < 1.0 and np_mat[i][j] > 0.0:
				np_mat[i][j] = 1.0

	np_mat = np_mat.astype(int)	

	return np_mat


def lateral_shift(transform, shift):
			transform.rotation.yaw += 90
			return transform.location + shift * transform.get_forward_vector()

def rotate_point(point, angle):
    """
    rotate a given point by a given angle
    """
    x_ = math.cos(math.radians(angle)) * point.x - math.sin(math.radians(angle)) * point.y
    y_ = math.sin(math.radians(angle)) * point.x - math.cos(math.radians(angle)) * point.y
    return carla.Vector3D(x_, y_, point.z)