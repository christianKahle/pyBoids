import pygame, math, random
pygame.init()
screen_width = 1920
screen_height = 1080
screen = pygame.display.set_mode([screen_width,screen_height])
#pygame.display.toggle_fullscreen()
renderClock = pygame.time.Clock()

boidimg = pygame.image.load("boid.png")
global simulate
simulate = True
draw_rect = False
turn_speed = 5  * (math.pi/180)
max_acceleration = 0.05
min_speed = 5
max_speed = 10
detection_radius = 150
detection_frustum = 150 * (math.pi/360)
num_boids = 65
mouse = (0,0)
follow_mouse = False
frame_rate = 30

def process_events():
    events = pygame.event.get()
    global screen, simulate, mouse, follow_mouse
    for event in events:
        if event.type == pygame.QUIT:
            simulate = False
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                simulate = False
                pygame.display.quit()
        if event.type == pygame.MOUSEMOTION:
            mouse = event.pos 
        if event.type == pygame.MOUSEBUTTONDOWN:
            follow_mouse = not follow_mouse
class vector():
    def __init__(self,magnitude, direction):
        self.magnitude = magnitude
        self.direction = direction
        self.cartesian = (self.magnitude*math.cos(self.direction),self.magnitude*math.sin(self.direction))
    def update_cartesian(self):
        self.cartesian = (self.magnitude*math.cos(self.direction),self.magnitude*math.sin(self.direction))
    def __add__(self, other):
        x = self.magnitude * math.cos(self.direction) + other.magnitude * math.cos(other.direction)
        y = self.magnitude * math.sin(self.direction) + other.magnitude * math.sin(other.direction)
        return vector(math.sqrt(x*x + y*y), math.atan2(y,x))
    __iadd__ = __add__

    @staticmethod
    def addall(vectors):
        x = 0
        y = 0
        for v in vectors:
            x += v.magnitude * math.cos(v.magnitude)
            y += v.magnitude * math.sin(v.magnitude)
        return vector(math.sqrt(x*x + y*y), math.atan2(y,x))
    def __mul__(self,other):
        if isinstance(other, vector):
            x = (self.magnitude * math.cos(self.direction)) * (other.magnitude * math.cos(other.direction))
            y = (self.magnitude * math.sin(self.direction)) * (other.magnitude * math.sin(other.direction))
            return vector(math.sqrt(x*x + y*y), math.atan2(y,x))
        return vector(self.magnitude * other, self.direction)
    __imul__ = __mul__

    def __truediv__(self,other):
        if isinstance(other, vector):
            x = (self.magnitude * math.cos(self.direction)) / (other.magnitude * math.cos(other.direction))
            y = (self.magnitude * math.sin(self.direction)) / (other.magnitude * math.sin(other.direction))
            return vector(math.sqrt(x*x + y*y), math.atan2(y,x))
        return vector(self.magnitude / other, self.direction)
    __itruediv__ = __truediv__

    @staticmethod
    def avgall(vectors):
        return vector.addall(vectors) / len(vectors)
class boid():
    def __init__(self,position = (0,0),velocity = vector(0,0)):
        self.position = position
        self.velocity = velocity
    @staticmethod
    def create_random(minwidth,maxwidth,minheight,maxheight,minvelocity,maxvelocity,):
        x = random.random()*(maxwidth-minwidth)+minwidth
        y = random.random()*(maxheight-minheight)+minheight
        s = random.random()*(maxvelocity-minvelocity)+minvelocity
        d = (random.random()+random.random())/2*math.tau
        return boid((x,y),vector(s,d))
    def move(self,bounds):
        moving = self.velocity.cartesian
        self.position = ((self.position[0]+moving[0]) % bounds[0],(self.position[1]+moving[1]) % bounds[1])
    def render(self,surface,image_surface):
        angle = -(self.velocity.direction/math.pi * 180)
        image_surface = pygame.transform.rotate(image_surface,angle)
        x = int(self.position[0] - image_surface.get_width()/2)
        y = int(self.position[1] - image_surface.get_height()/2)
        surface.blit(image_surface,(x,y))
    def turn(self,turn):
        self.velocity.direction += turn
        self.velocity.update_cartesian()
    def centerbehavior(self,boids):
        if len(boids) > 0:
            center = avgpos([b.position for b in boids])
            self.followposition(center)
        else:
            self.velocity.magnitude = max(min_speed,min((random.random()*2-1)*(max_acceleration)+self.velocity.magnitude,max_speed))
    def followposition(self,position):
        angle = math.pi - (self.velocity.direction-math.atan2((self.position[1]-position[1]),(self.position[0]-position[0])))%math.tau
        self.turn(2*absmin(turn_speed*angle,math.copysign(turn_speed,angle)))
    def relativepos(self,other):
        return ((self.position[0]-other.position[0]),(self.position[1]-other.position[1]))
    def distance(self,other):
        rpos = self.relativepos(other)
        return math.sqrt(rpos[0]**2 + rpos[1]**2)
    def avoidancebehavior(self,boids):
        boidistances = [(b,self.distance(b)) for b in boids]
        detection_radius = 35
        left = 0
        right = 0
        for bd in boidistances:
            if bd[1] < detection_radius and bd[1] > 0.01:
                rpos = self.relativepos(bd[0])
                if (((self.velocity.direction-math.atan2(rpos[1],rpos[0]))%math.tau > math.pi)):
                    right += detection_radius/bd[1]
                else:
                    left += detection_radius/bd[1]
        self.turn((right-left)*turn_speed)
    def followbehavior(self,boids):
        if len(boids) > 0:
            avgvel = vector.avgall([b.velocity for b in boids])
            dirdif = (self.velocity.direction - avgvel.direction) % math.tau - math.pi
            if abs(dirdif) > 0.1 * math.pi:
                self.turn(2*absmin(turn_speed*math.pi/dirdif,math.copysign(turn_speed,dirdif)))
def absmax(this,that):
    if abs(this) > abs(that):
        return this
    else:
        return that
def absmin(this,that):
    if abs(this) < abs(that):
        return this
    else:
        return that
def addpos(positions):
    x = 0
    y = 0
    for p in positions:
        x += p[0]
        y += p[1]
    return (x,y)
def avgpos(positions):
    added = addpos(positions)
    l = len(positions)
    return (added[0]/l,added[1]/l)
def update(boids):
    screen.fill((15,15,35))
    for b in boids:
        boidistances = [(b2,b.distance(b2)) for b2 in boids]
        visible_boids = []
        for bd in boidistances:
            rpos = b.relativepos(bd[0])
            if bd[1] < detection_radius and bd[1] > 0.01:# and abs((b.velocity.direction-math.atan2(rpos[1],rpos[0]))%math.tau - math.pi) < detection_frustum:
                visible_boids += [bd[0]]
        if follow_mouse:
            b.followposition(mouse)
        else:
            b.centerbehavior(visible_boids)
            b.followbehavior(visible_boids)
        b.avoidancebehavior(visible_boids)
       
        b.move((screen_width,screen_height))
        b.render(screen,boidimg)
boids = [boid.create_random(10,screen_width-10,10,screen_height-10,min_speed,max_speed) for n in range(0,num_boids)]
while simulate:
    renderClock.tick(frame_rate)
    update(boids)
    pygame.display.update()
    process_events()
pygame.display.quit()