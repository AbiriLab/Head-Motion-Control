import pygame
import matplotlib.pyplot as plot
from pygame.locals import *
import mouse

fig, ax = plot.subplots()
ax.set_xlim(-1, 1500)
ax.set_ylim(-1, 900)
x, y = [0], [0]

plot.gca().invert_yaxis()

points, = ax.plot([], [], 'o')
background = 0, 0, 0

class SpriteObject(pygame.sprite.Sprite):
    def __init__(self, x, y, color):
        super().__init__()
        self.original_image = pygame.Surface((100, 100), pygame.SRCALPHA)
        pygame.draw.circle(self.original_image, color, (50, 50), 50)
        self.click_image = pygame.Surface((100, 100), pygame.SRCALPHA)
        pygame.draw.circle(self.click_image, color, (50, 50), 50)
        pygame.draw.circle(self.click_image, (0, 0, 0), (50, 50), 50, 10)
        self.image = self.original_image
        self.rect = self.image.get_rect(center = (x, y))
        self.clicked = False
    def update(self, event_list):
        for event in event_list:
            if event.type == pygame.MOUSEBUTTONDOWN:
                if self.rect.collidepoint(event.pos):
                    self.clicked = not self.clicked
                nx, ny = pygame.mouse.get_pos()
                plot.plot(nx, ny, 'bo')
        self.image = self.click_image if self.clicked else self.original_image

def on_move(event):
    x.append(event.xdata)
    y.append(event.ydata)
    points.set_data(x, y)
    fig.canvas.restore_region(background)
    ax.raw_artist(points)
    fig.canvas.blit(0, 0, 0)

pygame.init()
window = pygame.display.set_mode((1500, 900))
clock = pygame.time.Clock()
passed_time = 0
timer_started = False

group = pygame.sprite.Group([
    SpriteObject(window.get_width() // 5, window.get_height() // 5, (255, 0, 0)),
    SpriteObject(window.get_width() * 4 // 5, window.get_height() // 5, (0, 255, 0)),
    SpriteObject(window.get_width() // 5, window.get_height() * 4 // 5, (0, 0, 255)),
    SpriteObject(window.get_width() * 4 // 5, window.get_height() * 4 // 5, (255, 255, 0)),
])

run = True
mouse.move(1920/2,1080/2)

collect_x = []
collect_y = []

while run:
    clock.tick(60)
    event_list = pygame.event.get()
    for event in event_list:
        if event.type == pygame.QUIT:
            run = False
        elif event.type == pygame.KEYDOWN:
            if event.key == K_SPACE:
                print("GUI Test Begin")
                pygame.mouse.set_pos(750, 450)
                collect_x = []
                collect_y = []
                timer_started = not timer_started
                if timer_started:
                    start_time = pygame.time.get_ticks()
            if event.key == K_ESCAPE:
                print("GUI Test Complete")
                run = False
    if timer_started:
        passed_time = pygame.time.get_ticks() - start_time

    new_x, new_y = pygame.mouse.get_pos()
    collect_x.append(new_x)
    collect_y.append(new_y)
    group.update(event_list)

    window.fill((255, 255, 255))
    group.draw(window)
    print(str(passed_time / 1000))
    pygame.display.flip()

plot.plot(collect_x, collect_y)
plot.show()
pygame.quit()
exit()