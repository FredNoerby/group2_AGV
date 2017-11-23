class Robot:
    """ Class to represent worker robots 
    
    Attributes:
        id (int): A unique number for identifying robots
        storage (str[list]): the current state of the product
    """

    def __init__(self, id):
        """ Robot initialize method

        Arguments:
            id (int): A unique number for identifying products
        """
        self.id = id
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


# Will only run if this is the main file being run
if __name__ == '__main__':
    # Creates an Robot object with the ID of 99
    mc_turner = Robot(99)

    print(mc_turner.add_part_to_storage("C1"))
    print(mc_turner.add_part_to_storage("C1"))
    print(mc_turner.add_part_to_storage("C1"))
