from carla_map_loader import CarlaMapLoader
import argparse
import pygame
import h5py

# Args
parser = argparse.ArgumentParser(description="Connects to Carla Server and extracts the map")
parser.add_argument("--host", default="localhost", type=str, help="IP of the host server (default: 127.0.0.1)")
parser.add_argument("--port", default=2000, type=int, help="TCP port to listen to (default: 2000)")
parser.add_argument("--town", default="Town03", type=str, help="Name of the map in CARLA")
parser.add_argument("--pixels_per_meter", default=2, type=int, help="Pixels per meter")
args = vars(parser.parse_args())


# Load the CARLA map
carla_map = CarlaMapLoader(host=args["host"], port=args["port"], town=args["town"])
carla_map.initialize_map(pixels_per_meter=args["pixels_per_meter"])
driviable_area, lane_marking, sidewalk_area, parking_area = carla_map.draw_road_map()

crosswalk_area = carla_map.draw_crosswalks()

# Save the map as h5 file
hf = h5py.File(args["town"] + '.h5', 'w')
g1 = hf.create_group(args["town"])
g1.create_dataset("drivable_area", data=driviable_area)
g1.create_dataset("lane_markings", data=lane_marking)
g1.create_dataset("sidewalk_area", data=sidewalk_area)
g1.create_dataset("parking_area", data=parking_area)
g1.create_dataset("crosswalk_area", data=crosswalk_area)
hf.close()


# Render map
pygame.display.flip()
print("Created Map")

