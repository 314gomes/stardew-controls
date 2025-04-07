# Example file showing a basic pygame "game loop"
import pygame
from . import fishing

def main():
    # pygame setup
    pygame.init()
    screen = pygame.display.set_mode((800, 600))

    pygame.display.set_caption("Fishing Game")

    clock = pygame.time.Clock()
    running = True

    fishing_game = fishing.FishingGame(screen.get_height(), 0, 0, clock)

    while running:
        # poll for events
        # pygame.QUIT event means the user clicked X to close your window
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # fill the screen with a color to wipe away anything from last frame
        screen.fill("blue")

        # game logic
        is_player_button_pressed = pygame.mouse.get_pressed()[0]
        fishing_game.tick(is_player_button_pressed)

        # RENDER YOUR GAME HERE
        fishing_game.draw(screen)

        # flip() the display to put your work on screen
        pygame.display.flip()

        clock.tick(60)  # limits FPS

    pygame.quit()