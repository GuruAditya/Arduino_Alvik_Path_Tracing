from time import sleep_ms
from arduino_alvik import ArduinoAlvik

IMG_WIDTH = 16
IMG_HEIGHT = 16
image = [
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0],
    [0,0,1,0,0,0,0,0,0,0,0,0,0,1,0,0],
    [0,0,1,0,0,0,0,0,0,0,0,0,0,1,0,0],
    [0,0,1,0,0,0,0,0,0,0,0,0,0,1,0,0],
    [0,0,1,0,0,0,0,0,0,0,0,0,0,1,0,0],
    [0,0,1,0,0,0,0,0,0,0,0,0,0,1,0,0],
    [0,0,1,0,0,0,0,0,0,0,0,0,0,1,0,0],
    [0,0,1,0,0,0,0,0,0,0,0,0,0,1,0,0],
    [0,0,1,0,0,0,0,0,0,0,0,0,0,1,0,0],
    [0,0,1,0,0,0,0,0,0,0,0,0,0,1,0,0],
    [0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
]
kernel = [
    [1, 1, 1],
    [1, -8, 1],
    [1, 1, 1]
]

def print_image(img_map):
    for y in range(IMG_HEIGHT):
        for x in range(IMG_WIDTH):
            print(img_map[y][x], end=" ")
        print()

def apply_convolution(input_image):
    edge_map = [[0 for _ in range(IMG_WIDTH)] for _ in range(IMG_HEIGHT)]
    for y in range(1, IMG_HEIGHT - 1):
        for x in range(1, IMG_WIDTH - 1):
            sum_val = 0
            for ky in range(-1, 2):
                for kx in range(-1, 2):
                    sum_val += input_image[y + ky][x + kx] * kernel[ky + 1][kx + 1]
            edge_map[y][x] = 1 if sum_val != 0 and input_image[y][x]==1 else 0
    return edge_map

MOVE_FORWARD = 'FORWARD'
MOVE_TURN_LEFT = 'TURN_LEFT'
MOVE_TURN_RIGHT = 'TURN_RIGHT'
MOVE_STOP = 'STOP'

def find_path(edge_map):
    path = []
    
    def find_start():
        for y in range(IMG_HEIGHT):
            for x in range(IMG_WIDTH):
                if edge_map[y][x] == 1:
                    return x, y
        return None, None

    start_x, start_y = find_start()
    if start_x is None:
        print("No path found!")
        return []
    
    current_x, current_y = start_x, start_y
    direction = 0 # 0:Right, 1:Down, 2:Left, 3:Up
    
    visited_map = [row[:] for row in edge_map]

    while True:
        if current_x == start_x and current_y == start_y and len(path) > 0:
            path.append(MOVE_STOP)
            break
        if len(path) > (IMG_WIDTH * IMG_HEIGHT):
            print("Path is too long, stopping.")
            path.append(MOVE_STOP)
            break

        visited_map[current_y][current_x] = 0
        
        forward_x, forward_y = current_x, current_y
        if direction == 0: forward_x += 1
        elif direction == 1: forward_y += 1
        elif direction == 2: forward_x -= 1
        elif direction == 3: forward_y -= 1
        
        if 0 <= forward_y < IMG_HEIGHT and 0 <= forward_x < IMG_WIDTH and visited_map[forward_y][forward_x] == 1:
            path.append(MOVE_FORWARD)
            current_x, current_y = forward_x, forward_y
        else:
            right_dir = (direction + 1) % 4
            right_x, right_y = current_x, current_y
            if right_dir == 0: right_x += 1
            elif right_dir == 1: right_y += 1
            elif right_dir == 2: right_x -= 1
            elif right_dir == 3: right_y -= 1

            if 0 <= right_y < IMG_HEIGHT and 0 <= right_x < IMG_WIDTH and visited_map[right_y][right_x] == 1:
                path.append(MOVE_TURN_RIGHT)
                direction = right_dir
            else:
                path.append(MOVE_STOP)
                direction = (direction - 1 + 4) % 4
                break
                
    return path


class Alvik:
    def __init__(self, alvik_instance):
        self.abc = alvik_instance
        print("Initializing Alvik Robot...")
    
    def move(self, cm):
        print(f"Moving forward {cm} cm")
        self.abc.set_wheels_speed(20,20)
        sleep_ms(2850)

    def turn(self, degrees):
        if(degrees==10): # Left
            print("Turning left")
            self.abc.set_wheels_speed(0,20)
            sleep_ms(4300)
        else: # Right
            print("Turning right")
            self.abc.set_wheels_speed(20,0)
            sleep_ms(4300)

    def stop(self):
        print("Stopping")
        self.abc.set_wheels_speed(0,0)
        sleep_ms(100)

PIXEL_TO_CM = 1

def execute_path(robot, path):
    print("\n--- Executing Path ---")
    forward_count = 0
    for move in path:
        if move == MOVE_FORWARD:
            forward_count += 1
        else:
            if forward_count > 0:
                robot.move(forward_count * PIXEL_TO_CM)
                forward_count = 0
            
            if move == MOVE_TURN_LEFT:
                robot.turn(10)
            elif move == MOVE_TURN_RIGHT:
                robot.turn(11)
            elif move == MOVE_STOP:
                print("Drawing complete!")
                robot.stop()
                break
    sleep_ms(100)

if __name__ == "__main__":
    abc = ArduinoAlvik()
    abc.begin()

    print("--- Original Image ---")
    print_image(image)
    
    print("\n--- Applying Convolution ---")
    edge_map = apply_convolution(image)
    print_image(edge_map)
    
    print("\n--- Finding Path ---")
    drawing_path = find_path(edge_map)
    print("\n--- Generated Path ---")
    print(", ".join(drawing_path))
    
    print("\nDrawing will start in 5 seconds...")
    sleep_ms(5000)
    
    my_alvik = Alvik(abc)
    execute_path(my_alvik, drawing_path)
