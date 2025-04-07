from . import fishing
import rclpy

def main():
	#spin node

    rclpy.init()
    node = fishing.FishingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
    ...
	# # pygame setup
    # pygame.init()
    # screen = pygame.display.set_mode((800, 600))

    # pygame.display.set_caption("Fishing Game")

    # clock = pygame.time.Clock()
    # running = True

    # fishing_game = fishing.FishingGame(screen.get_height(), 0, 0, clock)

    # while running:
    #     # poll for events
    #     # pygame.QUIT event means the user clicked X to close your window
    #     

    #     clock.tick(60)  # limits FPS

    # pygame.quit()