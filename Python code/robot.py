class Robot:
    """ Class to represent worker robots 
    
    Attributes:
        id (int): A unique number for identifying robots
        storage (str[list]): What is currently on the robot
    """

    def __init__(self, unique_id):
        """ Robot initialize method

        Arguments:
            unique_id (int): Initializes the ID of the robot
        """
        self.id = unique_id
        # Storage is empty when the robot is initialized
        self.storage = []

    def add_part_to_storage(self, part):
        """ Adds a part to the robots storage

        Args:
            part (str): A part to add to storage
        """

        # Checks if the robot is already full
        if len(self.storage) < 2:
            # Adds the part to the robots storage
            self.storage.append(part)
            return "Added " + part + " to robot with id: " + str(self.id)
        else:
            # Returns message that robot is full
            return"Robot with id: " + str(self.id) + " is full"

    def unload_parts(self, assembly_line):
        """ Unloads parts at specific assembly line

        Args:
            assembly_line (AssemblyLine): Where the parts should be unloaded
        """
        # TODO: unload parts at assembly line

    def go_to_location(self, location):
        """ Sends the robot to a specific location

        Args:
            location (Location): A location to go to
        """
        # TODO: Make the robot drive to location

    def fetch(self, list_of_parts):
        """ Adds a part to the robots storage

        Args:
            list_of_parts (str[list]): A part to add to storage
        """
        # TODO: check if robot is full. IF yes -> go_to_location(assembly)
        # TODO: check for next missing part -> get it if the robot doesn't have it
        # TODO: if robot has the next part in storage get part after that


# Will only run if this is the main file being run
if __name__ == '__main__':
    # Creates an Robot object with the ID of 99
    mc_turner = Robot(99)

    print(mc_turner.add_part_to_storage("C1"))
    print(mc_turner.add_part_to_storage("C1"))
    print(mc_turner.add_part_to_storage("C1"))