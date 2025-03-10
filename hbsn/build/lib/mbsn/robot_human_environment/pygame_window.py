import numpy as np
import pygame
from matplotlib import colors
from shapely import Point

EDGE_COLOR = (0, 0, 0)
POLYGON_COLOR = np.array(colors.to_rgba('sienna')[:3]) * 255 # Sienna
ROBOT_POLYGON_COLOR = np.array(colors.to_rgba('orange')[:3]) * 255 
HUMAN_POLYGON_COLOR = np.array(colors.to_rgba('blue')[:3]) * 255 
GOAL_POLYGON_COLOR = np.array(colors.to_rgba('red')[:3]) * 255

class Window():

    def __init__(self, width, height, env):
        self.env = env
        self.width, self.height = width, height

        pygame.init()
        self.clock = None
        self.screen = pygame.display.set_mode((self.width, self.height))
        self.center = self.get_center(self.env.polygonal_map.polygon)
        self.scale = 10

        #################################################

        self.foreground = None
        self.background = None

        self.set_background()                          
        self.set_foreground()                          

        #################################################

        self.sprites_list = []

        self.remove_last_sprite()

        #-----------------------------

        # red text "PAUSE"
        font = pygame.font.SysFont("", 72)
        self.text_pause = font.render("PAUSE", True, (255, 0, 0))

        # center text on screen
        screen_center = self.screen.get_rect().center
        self.text_pause_rect = self.text_pause.get_rect(center=screen_center)

        self.running = True

    #--------------------------

    def add_sprite(self, sprite):
        self.sprites_list.append(sprite)

    #--------------------------

    def remove_last_sprite(self):
        if self.sprites_list:
            del self.sprites_list[-1]

    #--------------------------

    def draw_sprites(self, screen):
        for sprite in self.sprites_list:
            sprite.draw(screen)

    #--------------------------

    def draw_background(self, screen):
        screen.fill((255,255,255)) # clear screen to green
        if self.background:
            screen.blit(self.background, (0,0))

    #--------------------------

    def draw_foreground(self, screen):
        if self.foreground:
            screen.blit(self.foreground, (0,0))

    #--------------------------

    def draw_world(self, image):
        temp = pygame.Surface(self.rect.size, pygame.SRCALPHA, 32).convert_alpha()
        image_rect = image.get_rect()

        for x in range(0, self.rect.width, 60):
            for y in range(0,self.rect.width, 60):
                temp.blit(image,(x,y))

        return temp

    #--------------------------

    def set_foreground(self, image=None):
        if image: 
            img = pygame.image.load(image)
            self.foreground = self.draw_world(img)

    #--------------------------

    def set_background(self, image=None):
        if image: 
            self.background = pygame.image.load(image)

    #--------------------------

    def draw_shapely_polygon(self, canvas, polygon, offset=(0, 0), scale=1, facecolor=POLYGON_COLOR, edgecolor=EDGE_COLOR):
        pygame_points = []

        for x, y in polygon.exterior.coords:
            cx = x + offset[0]
            cy = y + offset[1]
            x_new = scale * x + cx
            y_new = scale * y + cy
            pygame_points.append((x_new, y_new))

        pygame.draw.polygon(canvas, facecolor, pygame_points)
        pygame.draw.lines(canvas, edgecolor, True, pygame_points, 2)  # 'True' pour fermer le polygone

    def draw_text(self, canvas, text, position, offset=(0,0), scale=1, fontsize=30):
        cx = position.x + offset[0]
        cy = position.y + offset[1]
        x_new = scale * position.x + cx - fontsize/4
        y_new = scale * position.y + cy - fontsize/4

        font = pygame.font.SysFont(None, fontsize)
        text_surface = font.render(text, False, (255, 255, 255))
        canvas.blit(text_surface, (x_new,y_new))

    #--------------------------

    def get_center(self, polygon):
        box = polygon.minimum_rotated_rectangle
        # get coordinates of polygon vertices
        x, y = box.exterior.coords.xy

        minx, miny, maxx, maxy = polygon.bounds
        # get length of bounding box edges
        edge_length = (Point(x[0], y[0]).distance(Point(x[1], y[1])), Point(x[1], y[1]).distance(Point(x[2], y[2])))
        # get length of polygon as the longest edge of the bounding box
        poly_height = max(edge_length)
        # get width of polygon as the shortest edge of the bounding box
        poly_width = min(edge_length)

        width, height = pygame.display.get_window_size()
        center = width // 2 - poly_width // 2 - minx, \
                height // 2 - poly_height // 2 - miny

        return center

    #--------------------------

    def draw_mbsn(self, screen):
        for _, polygon in self.env.polygonal_map.grid.items():
            self.draw_shapely_polygon(screen, polygon.polygon, offset=self.center, scale=self.scale)
        

        # ROBOT
        self.draw_shapely_polygon(screen, self.env.robot.position.buffer(0.2), offset=self.center, scale=self.scale, facecolor=ROBOT_POLYGON_COLOR)
        self.draw_text(screen, "R", self.env.robot.position, offset=self.center, scale=self.scale, fontsize=16)

        for h in self.env.humans:
            self.draw_shapely_polygon(screen, h.position.buffer(0.2), offset=self.center, scale=self.scale, facecolor=HUMAN_POLYGON_COLOR)
            self.draw_text(screen, "H", h.position, offset=self.center, scale=self.scale, fontsize=16)

        # GOAL
        self.draw_shapely_polygon(screen, self.env.goal.buffer(0.2), offset=self.center, scale=self.scale, facecolor=GOAL_POLYGON_COLOR)
        self.draw_text(screen, "G", self.env.goal, offset=self.center, scale=self.scale, fontsize=16)
    
    def _render_frame(self):
        if self.clock is None:
            self.clock = pygame.time.Clock()
        self.draw_background(self.screen)
        self.draw_foreground(self.screen)
        self.draw_sprites(self.screen) 
        self.draw_mbsn(self.screen)


    def _deal_with_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

            if event.type == pygame.MOUSEWHEEL:
                self.scale += event.y

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
                elif event.key == pygame.K_SPACE:
                    PAUSED = not PAUSED
                elif event.key == pygame.K_RIGHT:
                    return True
        return False
    
    def update(self, fps=25):
        if not self.running:
            pygame.quit()
        next_step = self._deal_with_events()
        self._render_frame()
        pygame.display.update()
        self.clock.tick(fps)
        return next_step

    def run(self):
        
        RUNNING = True
        PAUSED = False

        while RUNNING:

            #--- events ---

            self._deal_with_events()

            #--- changes ----

            # if not PAUSED:
            #     # change elements position
            #     self.player.update()

            #--- draws ---
            self._render_frame()

            if PAUSED:
                # draw pause string
                self.screen.blit(self.text_pause, self.text_pause_rect.topleft)

            pygame.display.update()

            #--- FPS ---

            self.clock.tick(25) # 25 Frames Per Seconds

        #--- finish ---

        pygame.quit()