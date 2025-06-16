import pygame
import sys
import math

# Initialize Pygame and the joystick module.
pygame.init()
pygame.joystick.init()

# Check for connected joysticks.
if pygame.joystick.get_count() == 0:
    print("No joystick detected. Please connect your Xbox controller.")
    sys.exit()

# Use the first joystick.
joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Detected joystick: {joystick.get_name()}")

# Define sector names for 8 equally divided sectors (each 45°).
sector_names = ["East", "Northeast", "North", "Northwest",
                "West", "Southwest", "South", "Southeast"]

try:
    while True:
        # Process event queue.
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                raise KeyboardInterrupt

            if event.type == pygame.JOYAXISMOTION:
                # Retrieve axis values.
                x = joystick.get_axis(0)          # left/right axis (x remains the same)
                y = -joystick.get_axis(1)         # invert the y-axis

                # Compute polar coordinates.
                R = math.sqrt(x**2 + y**2)
                theta = math.atan2(y, x)          # theta in radians
                theta_deg = math.degrees(theta)   # convert to degrees

                # Normalize theta to be in the [0, 360) range.
                if theta_deg < 0:
                    theta_deg += 360

                # Check if the joystick is pushed beyond the neutral threshold.
                if R > 0.5:
                    # Divide the 360° plane into 8 sectors (each 45° wide).
                    sector_index = int(theta_deg // 45)
                    action_sector = sector_names[sector_index]
                    # print(f"Joystick in polar form: R = {R:.2f}, θ = {theta_deg:.2f}° -> Sector: {action_sector}")
                    if action_sector == 'North':
                        print('move forward')
                    elif action_sector == 'Northeast':
                        print('rotate right')
                    elif action_sector == 'South':
                        print('move back')
                    elif action_sector == 'Southeast':
                        print('rotate right')
                    elif action_sector == 'East':
                        print('move right')
                    elif action_sector == 'West':
                        print('move left')
                    elif action_sector == 'Southwest':
                        print('rotate left')
                    elif action_sector == 'Northwest':
                        print('rotate left')
                else:
                    # Joystick is in neutral position.
                    print("stop")

            if event.type == pygame.JOYBUTTONDOWN:
                button = event.button
                print(f"Button {button} pressed")
            
            if event.type == pygame.JOYBUTTONUP:
                button = event.button
                print(f"Button {button} released")
        
        # Add a small delay to avoid over-polling.
        pygame.time.wait(50)

except KeyboardInterrupt:
    print("Exiting program.")

finally:
    joystick.quit()
    pygame.quit()
