import pygame

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
			pos = self._clamp(pos, self.position_range)

		return pos, speed

class FishingFish:
	def __init__(self, fishing_assets, scale_factor):
		self.scale_factor = scale_factor
		self.fish_scale_factor = self.scale_factor/2
		self.fish_asset = fishing_assets.subsurface(pygame.Rect(47, 0, 19, 19)).convert_alpha()
		self.fish_asset = pygame.transform.scale_by(self.fish_asset, self.fish_scale_factor)

		self.fish_position = self.fish_asset.get_rect().move(17 * self.scale_factor, 3 * self.scale_factor)

	def draw(self, screen):
		screen.blit(self.fish_asset, self.fish_position)

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

	def _is_fish_in_fishing_bar(self):
		return False

	def draw(self, screen):
		screen.blit(self.background_asset, self.fishing_background_position)
		self.player_bar.draw(screen)
		self.fish.draw(screen)

	def tick(self, is_playerbutton_pressed: bool):
		time_delta = self.clock.get_time() / 1000.0
		self.player_bar.tick(time_delta, is_playerbutton_pressed)
		...

	