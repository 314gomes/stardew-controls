import os
import pygame
import math
import random

import rclpy
import rclpy.clock
from rclpy.node import Node
from std_msgs.msg import Float64, Bool

import uuid

import rclpy.action
from ros_fishing_interfaces.action import Fishing
from example_interfaces.action import Fibonacci
from rclpy.action import ActionServer

class FishingProgressBar:
	def __init__(self, scale_factor):
		self.scale_factor = scale_factor
		self.progress_bar = pygame.Rect(0, 0, 4 * self.scale_factor, 0 * self.scale_factor)
		self.progress_bar_bottom_left = pygame.Rect(32 * self.scale_factor, 146 * self.scale_factor, 0, 0)
		self.max_height = 144 * self.scale_factor
		self._progress = 0.0
		# how many pct to increase per second when catching
		self._progress_increase_rate = 1.0 / 5.0
		# how many pct to increase per second when catching
		self.progress_decrease_rate = self._progress_increase_rate * 0.6

	def _set_progress(self, progress:float):
		if progress < 0 or progress > 1:
			raise ValueError("Progress must be between 0 and 1")
		self.progress_bar.height = progress * self.max_height
		self.progress_bar.bottomleft = self.progress_bar_bottom_left.bottomleft

	def tick(self, is_catching:bool, time_delta_s:float):
		if is_catching:
			self._progress += self._progress_increase_rate * time_delta_s
		else:
			self._progress -= self.progress_decrease_rate * time_delta_s
		
		fish_caught_status = self._progress
		self._progress = max(0, min(1, self._progress))

		self._set_progress(self._progress)

		return fish_caught_status

	def draw(self, screen:pygame.SurfaceType):
		color = pygame.Color(0, 0, 0)

		hsva = (120 * self._progress, 100, 100, 100)
		color.hsva = hsva
		
		screen.fill(color, self.progress_bar)


class FishingBarRules:
	def __init__(self):
		self.mass = 1.0
		self.gravity = 3.0
		self.position_range = [0, 1]
		self.speed_range = [-0.3, 0.3]
		self.acc_range = [-1, 1]
		self.restitution = 0.50
		self.speed_epsilon = 0.05
		self.force_ext = 5
		
	def _clamp(self, value, range):
		if value < range[0]:
			return range[0]
		elif value > range[1]:
			return range[1]
		else:
			return value
		
	def apply_epsilon(self, value, epsilon):
		if abs(value) < epsilon:
			return 0
		else:
			return value

	
	def calculate_partially_elastic_collision(self, pos, speed):
		if pos < self.position_range[0] or pos > self.position_range[1]:
			speed = -speed * self.restitution
			speed = FishingBarRules().apply_epsilon(speed, FishingBarRules().speed_epsilon)
			# speed = self._clamp(speed, self.speed_range)
			pos = self._clamp(pos, self.position_range)

		return pos, speed

class FishingReel:
	def __init__(self, fishing_assets, scale_factor):
		self.scale_factor = scale_factor
		self.reel_asset = fishing_assets.subsurface(pygame.Rect(39, 6, 3, 8)).convert_alpha()
		self.reel_asset = pygame.transform.scale_by(self.reel_asset, self.scale_factor)
		# self.reel_asset = pygame.transform

		self.reel_position = self.reel_asset.get_rect()
		self.reel_position = self.reel_position.move(5 * self.scale_factor, 128 * self.scale_factor)
		self.reel_position = self.reel_position.move(-1 * self.scale_factor, -self.reel_asset.get_height() + 1 * self.scale_factor)

		self.angular_position = 0
		# angular velocities in degrees per second
		self.angular_velocity_active = - 360 * 3
		self.angular_velocity_inactive = 180

	def draw(self, screen):
		# screen.blit(self.reel_asset, self.reel_position)
		rotated_reel_asset = pygame.transform.rotate(self.reel_asset, self.angular_position)
		height = self.reel_asset.get_height()
		
		# x_mov = 0
		# y_mov = 0
		
		x_mov = -(height/2) * math.sin(math.radians(self.angular_position))
		y_mov = -(height/2) * math.cos(math.radians(self.angular_position))

		rotated_reel_pos = rotated_reel_asset.get_rect(center = self.reel_position.midbottom)
		rotated_reel_pos = rotated_reel_pos.move(0, -1 * self.scale_factor)
		rotated_reel_pos = rotated_reel_pos.move(x_mov, y_mov)
		
		screen.blit(rotated_reel_asset, rotated_reel_pos)

	def tick(self, time_delta_s:float, is_active:bool):
		if is_active:
			self.angular_position += self.angular_velocity_active * time_delta_s
		else:
			self.angular_position += self.angular_velocity_inactive * time_delta_s

class FishingFish:
	def __init__(self, fishing_assets, scale_factor):
		self.scale_factor = scale_factor
		self.fish_scale_factor = self.scale_factor/2
		self.fish_asset = fishing_assets.subsurface(pygame.Rect(47, 0, 19, 19)).convert_alpha()
		self.fish_asset = pygame.transform.scale_by(self.fish_asset, self.fish_scale_factor)

		# fish physics stuff
		self._pos_pct = 0
		self._pos_px = 508
		self._vel_px = 0
		self._vel_px = 0
		self.FloaterSinkerAcc = 0


		# variables for fish behavior
		# carp parameters by default
		self.difficulty = 15
		self.motionType = 0
		self._target_pos_px = (self.difficulty) / 100 * 548

		self.fish_position = self.fish_asset.get_rect().move(17 * self.scale_factor, 3 * self.scale_factor + 141 * self.scale_factor - self.fish_asset.get_height())
		self.shake = pygame.Rect(0, 0, 0, 0)

	def draw(self, screen):
		screen.blit(self.fish_asset, self.fish_position.move(self.shake.x, self.shake.y))

	def get_bounding_rect(self):
		bounding_rect = pygame.Rect(0, 0, 0, 0)
		bounding_rect.width = self.fish_asset.get_width()
		bounding_rect.height = self.fish_asset.get_height() * 0.3
		bounding_rect.center = self.fish_position.center
		return bounding_rect

	def _set_yoff(self, yoff):
		self._pos_px = yoff
		self.fish_position.y = yoff + 3 * self.scale_factor

	def tick(self, time_delta_s:float, catching: bool):
		# self.motionType {
		# 0: mixed
		# 1: dart
		# 2: smooth
		# 3: sinker
		# 4: floater
		# }
		calculated_pos_px = self._pos_px

		if (random.random() < (self.difficulty * (1 if self.motionType != 2  else 20) / 4000) and (self.motionType != 2 or self._target_pos_px == -1)):
			spaceBelow = 532 - self._pos_px
			spaceAbove = self._pos_px
			percent = min(99, self.difficulty + random.randint(10, 45)) / 100
			self._target_pos_px = self._pos_px + random.uniform(min(0 - spaceAbove, spaceBelow), spaceBelow) * percent

		match (self.motionType):
			case 4:
				self.FloaterSinkerAcc = max(self.FloaterSinkerAcc - 0.01, -1.5)
			case 3:
				self.FloaterSinkerAcc = min(self.FloaterSinkerAcc + 0.01, 1.5)
		
		if (abs(self._pos_px - self._target_pos_px) > (3) and self._target_pos_px != -1):
			bobberAcceleration = (self._target_pos_px - self._pos_px) / (random.randint(10, 30) + (100 - min(100, self.difficulty)))
			self._vel_px += (bobberAcceleration - self._vel_px) / 5
		elif (self.motionType != 2 and random.random() < (self.difficulty / 2000)):
			self._target_pos_px = self._pos_px + (random.uniform(-100, -51) if random.getrandbits(1) else random.uniform(50, 101))
		else:
			self._target_pos_px = -1
		
		if (self.motionType == 1 and random.random() < (self.difficulty / 1000)):
			self._target_pos_px = self._pos_px + (random.randint(-100 - self.difficulty * 2, -51) if random.getrandbits(1) else random.randint(50, 101 + self.difficulty * 2)) /548

		calculated_pos_px += self._vel_px + self.FloaterSinkerAcc
		
		if (calculated_pos_px > 548):
			calculated_pos_px = 548
		elif (calculated_pos_px < 0):
			calculated_pos_px = 0
		
		if (catching):
			self.shake.x = random.randint(-10, 11) / 10
			self.shake.y = random.randint(-10, 11) / 10

		self._set_yoff(calculated_pos_px)

	def get_fish_relative_to_bar_pct(self, bar_height_px):
		min_pos_px = 548 - bar_height_px/2
		max_pos_px = bar_height_px/2 + 3 * self.scale_factor - self.fish_asset.get_height()

		self._pos_pct = (self._pos_px - min_pos_px) / (max_pos_px - min_pos_px)
		self._pos_pct = max(0, min(1, self._pos_pct))

		return self._pos_pct


class FishingBar:
	def __init__(self, height, fishing_assets, scale_factor):
		self.height = height
		self.scale_factor = scale_factor
	
		# Fishing bar physics stuff
		self._pct_pos = 0.0
		self._pct_vel = 0.0
		self._pct_acc = 0.0
		self._pct_force_weight = - FishingBarRules().gravity * FishingBarRules().mass

		self.fishing_bar_top_asset = fishing_assets.subsurface(pygame.Rect(38, 0, 9, 2)).convert_alpha()
		self.fishing_bar_middle_asset = fishing_assets.subsurface(pygame.Rect(38, 2, 9, 1)).convert_alpha()
		self.fishing_bar_bottom_asset = fishing_assets.subsurface(pygame.Rect(38, 3, 9, 2)).convert_alpha()

		self.fishing_bar_top_asset = pygame.transform.scale_by(self.fishing_bar_top_asset, self.scale_factor)
		self.fishing_bar_middle_asset = pygame.transform.scale_by(self.fishing_bar_middle_asset, self.scale_factor)
		self.fishing_bar_middle_asset = pygame.transform.scale_by(self.fishing_bar_middle_asset, [1, self.height])
		self.fishing_bar_bottom_asset = pygame.transform.scale_by(self.fishing_bar_bottom_asset, self.scale_factor)

		self.fishing_bar_position = self.fishing_bar_top_asset.get_rect().move(17 * self.scale_factor, 3 * self.scale_factor)
		
	def set_fishing_bar_pct(self, percent:float = 1.0):
		if(percent < 0 or percent > 1):
			raise ValueError("Percent must be between 0 and 1")
		
		self._pct_pos = percent

		fishing_bar_max_yoff_px = 141 * self.scale_factor - self.get_height_px()
		self._set_yoff((1 - percent) * fishing_bar_max_yoff_px)

	def get_fishing_bar_pct(self):
		return self._pct_pos

	def get_height_px(self):
		return self.fishing_bar_top_asset.get_height() + self.fishing_bar_middle_asset.get_height() + self.fishing_bar_bottom_asset.get_height()

	def get_fishing_bar_bounding_rect(self):
		rect = self.fishing_bar_position.copy()
		rect.height = self.get_height_px()
		return rect

	def _set_yoff(self, yoff):
		self.fishing_bar_position.y = yoff + 3 * self.scale_factor

	def draw(self, screen, alpha = 255):
		self.fishing_bar_top_asset.set_alpha(alpha)
		self.fishing_bar_middle_asset.set_alpha(alpha)
		self.fishing_bar_bottom_asset.set_alpha(alpha)
		screen.blit(self.fishing_bar_top_asset, self.fishing_bar_position)
		screen.blit(self.fishing_bar_middle_asset, self.fishing_bar_position.move(0, self.fishing_bar_top_asset.get_height()))
		screen.blit(self.fishing_bar_bottom_asset, self.fishing_bar_position.move(0, self.fishing_bar_top_asset.get_height() + self.fishing_bar_middle_asset.get_height()))

	def set_external_force(self, force):
		self._pct_force_ext = force

	def tick(self, time_delta_s:float, is_button_pressed = False):
		# Update fishing bar physics
		self._pct_vel += self._pct_acc * time_delta_s
		self._pct_pos += self._pct_vel * time_delta_s
		self._pct_acc = (FishingBarRules().force_ext * is_button_pressed + self._pct_force_weight) / FishingBarRules().mass
		
		self._pct_pos, self._pct_vel = FishingBarRules().calculate_partially_elastic_collision(self._pct_pos, self._pct_vel)

		# Update fishing bar position
		self.set_fishing_bar_pct(self._pct_pos)

		return self._pct_pos

class FishingGame:
	def __init__(self):
		pygame.init()
		self.clock = pygame.time.Clock()
		self.screen = pygame.display.set_mode((800, 600))
		self.height = self.screen.get_height()

		filepath = os.path.dirname(__file__)
		fishing_assets = pygame.image.load(filepath + '/fishing.png')
		pygame.display.set_icon(fishing_assets.convert_alpha().subsurface(pygame.Rect(47, 0, 19, 19)))


		self.background_asset = fishing_assets.subsurface(pygame.Rect(0, 0, 38, 150)).convert_alpha()
		self.scale_factor = self.height/self.background_asset.get_height()
		self.background_asset = pygame.transform.scale_by(self.background_asset, self.scale_factor)
		self.fishing_background_position = self.background_asset.get_rect()

		self.player_bar = FishingBar(30, fishing_assets, self.scale_factor)
		self.fish = FishingFish(fishing_assets, self.scale_factor)
		self.reel = FishingReel(fishing_assets, self.scale_factor)
		self.progress_bar = FishingProgressBar(self.scale_factor)

		self.fish_in_fishing_bar = True
		self.fish_pct = 0.0
		self.bar_pct = 0.0
		self.progress_pct = 0.0

	def _update_fish_collision(self):
		fish_rect = self.fish.get_bounding_rect()
		fishing_bar_rect = self.player_bar.get_fishing_bar_bounding_rect()
		if fish_rect.colliderect(fishing_bar_rect):
			self.fish_in_fishing_bar = True
		else:
			self.fish_in_fishing_bar = False

	def draw(self, screen: pygame.SurfaceType):
		screen.fill("blue")

		screen.blit(self.background_asset, self.fishing_background_position)
		
		if self.fish_in_fishing_bar:
			self.player_bar.draw(screen)
		else:
			self.player_bar.draw(screen, alpha = 100)

		self.reel.draw(screen)

		self.fish.draw(screen)

		self.progress_bar.draw(screen)

		pygame.display.flip()


	def tick(self, is_playerbutton_pressed: bool):
		timedelta_s = self.clock.tick(60) / 1000.0
		# update game logic
		self.bar_pct = self.player_bar.tick(timedelta_s, is_playerbutton_pressed)
		self._update_fish_collision()
		self.reel.tick(timedelta_s, self.fish_in_fishing_bar)
		self.fish.tick(timedelta_s, self.fish_in_fishing_bar)
		self.fish_pct = self.fish.get_fish_relative_to_bar_pct(self.player_bar.get_height_px())

		self.progress_pct = self.progress_bar.tick(self.fish_in_fishing_bar, timedelta_s)

		# update screen
		self.draw(self.screen)
	
	def get_pygame_events(self):
		return pygame.event.get()

	def __del__( self ):
		self.screen = None
		self.background_asset = None
		self.fishing_background_position = None
		self.player_bar = None
		self.fish = None
		self.reel = None
		self.progress_bar = None
		self.fish_in_fishing_bar = None
		self.fish_pct = None
		self.bar_pct = None
		self.progress_pct = None
		self.last_time = None
		pygame.display.quit()
		pygame.quit()

class FishingNode(Node):
	def __init__(self):
		super().__init__('fishing_node')
		self.get_logger().info('Fishing node started')
		# self.timer = self.create_timer(timer_period, self.timer_callback)


		# Initialize action server
		self.fishing_action_server = ActionServer(
			self,	
			Fishing,
			'fish',
			self.execute_callback,
			goal_callback=self.new_goal_callback,
		)

		self.current_goal = None

	def new_goal_callback(self, goal_handle: rclpy.action.server.ServerGoalHandle):
		# only one goal at a time
		if self.current_goal is not None:
			self.get_logger().info('Goal rejected: another goal is already being executed')
			return rclpy.action.server.GoalResponse.REJECT
		self.get_logger().info('Goal accepted')
		self.current_goal = goal_handle
		return rclpy.action.server.GoalResponse.ACCEPT

	def execute_callback(self, goal_handle: rclpy.action.server.ServerGoalHandle):
		# actually start the game and game logic loop
		executing_game = True
		fishing_game = FishingGame()

		# Initialize publishers
		goal_id_string = uuid.UUID(bytes=bytes(goal_handle.goal_id.uuid)).hex
		self.get_logger().info('goal id: %s' % goal_id_string)
		fish_pct_publisher = self.create_publisher(Float64, '/fish_pct/id_' + goal_id_string, 10)
		bar_pct_publisher = self.create_publisher(Float64, '/bar_pct/id_' + goal_id_string, 10)
		fish_in_fishing_bar_publiser = self.create_publisher(Bool, '/fish_in_fishing_bar/id_' + goal_id_string, 10)

		result = Fishing.Result()
		result.message = 'Fish name should go here!'
		is_player_button_pressed = False

		while executing_game:
			for event in fishing_game.get_pygame_events():
				if event.type == pygame.QUIT:
					goal_handle.abort()
					executing_game = False
				if event.type == pygame.KEYUP:
					if event.key == pygame.K_SPACE:
						is_player_button_pressed = False
				if event.type == pygame.KEYDOWN:
					if event.key == pygame.K_ESCAPE:
						goal_handle.abort()
						executing_game = False
					if event.key == pygame.K_SPACE:
						is_player_button_pressed = True

			# TODO: test this
			if goal_handle.is_cancel_requested:
				self.get_logger().info('Goal cancelled')
				goal_handle.canceled()
				executing_game = False
				
			# game logic
			fishing_game.tick(is_player_button_pressed)

			# publish game data
			fish_pct_msg = Float64()
			fish_pct_msg.data = float(fishing_game.fish_pct)
			fish_pct_publisher.publish(fish_pct_msg)
			bar_pct_msg = Float64()
			bar_pct_msg.data = float(fishing_game.bar_pct)
			bar_pct_publisher.publish(bar_pct_msg)

			#this will be published as a goal feedback
			progress_pct_feedback = Fishing.Feedback()
			progress_pct_feedback.progress = fishing_game.progress_pct
			goal_handle.publish_feedback(progress_pct_feedback)

			fish_in_fishing_bar_msg = Bool()
			fish_in_fishing_bar_msg.data = fishing_game.fish_in_fishing_bar
			fish_in_fishing_bar_publiser.publish(fish_in_fishing_bar_msg)

			

			# check if the game is over
			# if fishing_game.progress_pct >= 1.0:
			# 	self.get_logger().info('Game won')
			# 	executing_game = False
			# 	goal_handle.succeed()
			# 	result.success = True
			# elif fishing_game.progress_pct <= 0.0:
			# 	self.get_logger().info('Game lost')
			# 	executing_game = False
			# 	goal_handle.abort()
			# 	result.success = False

		fishing_game = None
		self.get_logger().info('Game ended')

		# remove publishers
		self.destroy_publisher(fish_pct_publisher)
		self.destroy_publisher(bar_pct_publisher)
		self.destroy_publisher(fish_in_fishing_bar_publiser)

		self.current_goal = None
		return result