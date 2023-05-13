


def main():

    # TODO modify joint_limits.yaml for this test
    # TODO update the environment and scene
    # TODO place camera on Apple Proxy
    # TODO organize electronics of apple proxy

    # Initialize Class

    # --- Step 1: Place robot at waypoint (preliminary position)

    # --- Step 2: Obtain info from user
    # Info about gripper: pressure
    # Info about apple: stiffness -- type of springs, apples used
    # Pass these properties to the class

    # --- Step 3: Check that the vacuum circuit is free of holes
    # Turn on valve
    # Place flat object
    # Plot and check all sensors are @20KPa
    # Check camera, and all signals

    # --- Step 4: Pick the desired experiment
    # Exp 1: Real Apples
    # Exp 2: Apple Proxy

    ...


def proxy_picks():

    # --- Experiment Parameters ---
    # Number of experiments
    # Amount of noise

        # Measure Apple Position (simply add a fiducial marker)

        # Move to Starting Position

        # Start Recording Rosbag file

        # Add noise

        # Open Valve (apply vacuum)

        # Approach Surface

        # Label number of cups

        # Retreieve

        # Label result

        # Close Valve (stop vacuum)

        # Stop Recording Rosbag file

        # Save metadata in yaml file

        # Plot results, and decide to toss experiment away or not



    ...


def real_picks():
    ...



class RoboticGripper():

    def __init__(self):

    def go_to_preliminary_position(self):

    def go_to_starting_position(self):

    def add_cartesian_noise(self):

    def place_marker_text(self):

    def save_metadata(self):

    def publish_event(self):

    def check_real_noise(self):











if __name__ == '__main__':
    main()

