import json
import math
import os
import random
import time
from typing import Dict, Optional, Tuple

from attr import dataclass
import cv2
import numpy as np
from enum import Enum

from LaneSegment import LaneSegment, TileType

@dataclass
class Particle:
    
    current_tile: LaneSegment
    #next_tile: LaneSegment
    trajectory: int
    point: int
    weight: float
    x: float
    y: float
    theta: float


class ParticleFilter:

    

    def __init__(self, n_particles=500):
        self.n = n_particles
        self.particles = []
        self.map = {}

    def reset(self, x, y, theta):
        self.particles = []

        for _ in range(self.n):
            p = Particle(
                current_tile=None,
                trajectory=0,
                point=0,
                weight=1.0 / self.n,
                x=x,
                y=y,
                theta=theta
            )
            p.current_tile = LaneSegment((0, 0), theta, random.choice(list(TileType)))
            self.particles.append(p)

    def predict(self, distance, rotation):

        for p in self.particles:

            delta = abs(distance * 0.2)
            d = np.random.normal(distance, delta)
            delta_theta = abs(rotation * 0.2)
            dtheta = np.random.normal(rotation, delta_theta)

            p.x += d * np.cos(p.theta)
            p.y += d * np.sin(p.theta)
            p.theta += dtheta

            

    def normalize(self):

        total = sum(p.weight for p in self.particles)

        if total == 0:
            return

        for p in self.particles:
            p.weight /= total

    def estimate(self):

        tile_counts = {}
        positions = {}
        confidence_tile = {}

        for p in self.particles:
            tile_type = p.current_tile.type
            point = p.point
            map_position = p.current_tile.map_position
            tile_key = (tile_type, map_position, p.current_tile.orientation, point)
            if tile_key not in tile_counts:
                tile_counts[tile_key] = 0
            tile_counts[tile_key] += p.weight
            positions[tile_key] = p.current_tile.points[point]

            if (tile_type, map_position) not in confidence_tile:
                confidence_tile[(tile_type, map_position)] = 0
            confidence_tile[(tile_type, map_position)] += p.weight
                                            

        (tile, map_position, orientation, point_index) = max(tile_counts, key=tile_counts.get)
        total_weight = sum(tile_counts.values())


        confidence = confidence_tile[(tile, map_position)] / total_weight if total_weight else 0


        point = positions[(tile, map_position, orientation, point_index)]
        tile = LaneSegment(map_position, orientation, tile)


        print(f'deciding if saving tile to map: \nalready savedtile at {tile.map_position}: {tile.map_position in self.map}\nhigh enough confidence {confidence}: {confidence > 0.5}\npoint far enough along tile {point_index}: {point_index > 0.75 * len(tile.points)}\n')

        if point_index > 0.75 * len(tile.points) and confidence > 0.75 and tile.map_position not in self.map:
            print(f"\n Adding tile to map: {tile.type}, orientation: {tile.orientation}, position: {tile.map_position}, point: {point_index}, confidence: {confidence} \n")
            self.map[tile.map_position] = tile

        if tile.map_position in self.map:
            print(f'already saved tile in map: {tile.map_position}')

        if point_index <= 0.5 * len(tile.points)  and tile.map_position not in self.map:
            print(f'not adding tile to map: {tile.map_position}, point: {point_index} is not far enough along the tile')

        if  confidence <= 0.2 and tile.map_position not in self.map:
            print(f'not adding tile to map: {tile.map_position}, confidence: {confidence} is too low')

        return tile, point

    def estimate_full_map(self):
        map = self.map.copy()
        current_tile, point = self.estimate()
        map[current_tile.map_position] = current_tile

        print(f'full map estimation: current tile: {current_tile.type}, orientation: {current_tile.orientation}, position: {current_tile.map_position}, point: {point}')

        return map, point

    def plot_particles(self):
        images = {}

        for type in TileType:
            image = np.zeros((60, 60, 3), dtype=np.uint8)
            tile = LaneSegment((0, 0), 0, type)
            for point in tile.points:
                                cv2.circle(image, (int(point[0] * 100 % 60), int(60-point[1] * 100 % 60)), 1, (125, 125, 125), -1)
            

            total_weight = sum(p.weight for p in self.particles if p.current_tile.type == type)
            print(f"Total weight for {type}: {total_weight}")
            for p in self.particles:
                if p.current_tile.type == type:
                    alpha = p.weight * 2
                    image_copy = np.zeros((60, 60, 3), dtype=np.uint8)
                    cv2.circle(image_copy, (int(p.x * 100) % 60, int(60-p.y * 100 % 60)), 1, (0, 255, 0, ), -1)
                    image = cv2.addWeighted(image, 1, image_copy, alpha, 0)

            
                    

            images[type] = image

        return images

        
    
    def calculate_weights(self):

        points_x_dict = {}
        points_y_dict = {}
        points_theta_dict = {}



        for p in self.particles:
            key = (p.current_tile.type, p.current_tile.orientation, p.current_tile.map_position)
            if key not in points_x_dict:
                points_x_dict[key] = np.array([point[0] for point in p.current_tile.points])
                points_y_dict[key] = np.array([point[1] for point in p.current_tile.points])
                points_theta_dict[key] = np.array([point[2] for point in p.current_tile.points])

        
            points_x = points_x_dict[key]
            points_y = points_y_dict[key]
            points_theta = points_theta_dict[key]

            distances = (points_x - p.x) ** 2 + (points_y - p.y) ** 2 
            + ((points_theta - p.theta) % (2 * math.pi)) ** 2
            min_distance_index = np.argmin(distances)
            min_distance = distances[min_distance_index]
            p.point = min_distance_index
            p.weight = 1.0 / ((min_distance) + 1e-6)

            
            if p.point == len(p.current_tile.points) - 1:
                self.set_next_tile(p)

        

    def calculate_weights_better(self):
        for p in self.particles:
            points_x = np.array([point[0] for point in p.current_tile.points])
            points_y = np.array([point[1] for point in p.current_tile.points])

            distances = (points_x - p.x) ** 2 + (points_y - p.y) ** 2
            min_distance_index = np.argmin(distances)
            min_distance = distances[min_distance_index]

            p.point = min_distance_index
            p.weight = 1.0 / (np.sqrt(min_distance) + 1e-6)

            if p.point == len(p.current_tile.points) - 1:
                self.set_next_tile(p)
         
    def calculate_weights_old(self):
        
        for p in self.particles:
            distances = [np.linalg.norm(
                np.array([p.x, 
                          p.y])
                            - np.array([point[0], point[1]] 
                                        ) ) for point in p.current_tile.points]

            min_distance = min(distances)
            p.point = distances.index(min_distance)

            if p.point == len(p.current_tile.points) - 1:
               
                self.set_next_tile(p)
                offset_x = p.current_tile.map_position[0] * 0.6
                offset_y = p.current_tile.map_position[1] * 0.6
                distances = [np.linalg.norm(
                    np.array([p.x, 
                                p.y])
                                - np.array([point[0], point[1]] )) for point in p.current_tile.points]
                
                min_distance = min(distances)
                p.point = distances.index(min_distance)

                

            p.weight = 1.0 / (min_distance + 1e-6)

    def set_next_tile(self, particle):
        map_key = particle.current_tile.next_tile_map_position()
        theta_next = particle.current_tile.next_tile_orientation()

        if map_key in self.map:
            particle.current_tile = self.map[map_key]

        else:
            tile_type = random.choice(list(TileType))

            new_tile = LaneSegment(map_position=map_key, theta=theta_next, type=tile_type)

            particle.current_tile = new_tile


    def resample(self):
        weights = [p.weight for p in self.particles]
        indexes = np.argsort(weights)
        keep_indexes = indexes[-int(self.n*0.66):]  # Keep the top 66% particles
        duplicate_indexes = indexes[-int(self.n*0.34):]  # Duplicate the top 34% particles
        
        new_particles = []
        for idx in duplicate_indexes:
            old_particle = self.particles[idx]
            new_particle = Particle(
                current_tile=old_particle.current_tile,
                trajectory=old_particle.trajectory,
                point=old_particle.point,
                weight=old_particle.weight,
                x=old_particle.x,
                y=old_particle.y,
                theta=old_particle.theta
            )

            if old_particle.current_tile is None:
                print(f"Warning: Particle {idx} has no current tile. Skipping resampling for this particle. This shoud not happen.")
                continue

            current_tile_position = new_particle.current_tile.map_position
            map_position = new_particle.current_tile.map_position
            start_theta = new_particle.current_tile.orientation
            type = new_particle.current_tile.type
            new_particle.current_tile = LaneSegment(
                map_position=map_position,
                theta=start_theta,
                type=type#random.choice(list(TileType))
            )            
            new_particles.append(new_particle)

        for idx in keep_indexes:
            new_particles.append(self.particles[idx])

        self.particles = new_particles

