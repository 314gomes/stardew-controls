import pygame
import math

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

		self.fish_position = self.fish_asset.get_rect().move(17 * self.scale_factor, 3 * self.scale_factor + 60 * self.scale_factor)

	def draw(self, screen):
		screen.blit(self.fish_asset, self.fish_position)

	def get_bounding_rect(self):
		bounding_rect = pygame.Rect(0, 0, 0, 0)
		bounding_rect.width = self.fish_asset.get_width()
		bounding_rect.height = self.fish_asset.get_height() * 0.3
		bounding_rect.center = self.fish_position.center
		return bounding_rect

	def tick(self, time_delta_s:float):
		...

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

class FishingGame:
	def __init__(self, fishing_background_height, xoff, yoff, clock : pygame.time.Clock):
		self.height = fishing_background_height
		self.clock = clock

		fishing_assets = pygame.image.load("assets/fishing.png")
		self.background_asset = fishing_assets.subsurface(pygame.Rect(0, 0, 38, 150)).convert_alpha()
		self.scale_factor = self.height/self.background_asset.get_height()
		self.background_asset = pygame.transform.scale_by(self.background_asset, self.scale_factor)
		self.fishing_background_position = self.background_asset.get_rect()

		self.player_bar = FishingBar(30, fishing_assets, self.scale_factor)
		
		self.fish = FishingFish(fishing_assets, self.scale_factor)

		self.reel = FishingReel(fishing_assets, self.scale_factor)

		self.progress_bar = FishingProgressBar(self.scale_factor)

		self.fish_in_fishing_bar = True

	def _update_fish_collision(self):
		fish_rect = self.fish.get_bounding_rect()
		fishing_bar_rect = self.player_bar.get_fishing_bar_bounding_rect()
		if fish_rect.colliderect(fishing_bar_rect):
			self.fish_in_fishing_bar = True
		else:
			self.fish_in_fishing_bar = False

	def draw(self, screen):
		screen.blit(self.background_asset, self.fishing_background_position)
		
		if self.fish_in_fishing_bar:
			self.player_bar.draw(screen)
		else:
			self.player_bar.draw(screen, alpha = 100)

		self.reel.draw(screen)

		self.fish.draw(screen)

		self.progress_bar.draw(screen)

	def tick(self, is_playerbutton_pressed: bool):
		time_delta = self.clock.get_time() / 1000.0
		self.player_bar.tick(time_delta, is_playerbutton_pressed)
		self._update_fish_collision()
		self.reel.tick(time_delta, self.fish_in_fishing_bar)
		progress_status = self.progress_bar.tick(self.fish_in_fishing_bar, time_delta)

		...

	