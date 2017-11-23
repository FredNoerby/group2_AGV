class Robot:
    """ Class to represent worker robots 
    
    Attributes:
        id (int): A unique number for identifying robots
        storage_slots (str): the current state of the product 
    """

    def __init__(self, id):
        """ Product init method

        Args:
            id (int): A unique number for identifying products
            type (str): the type of product (P1, P2, P3, or P4)
            status (Optional[str]): the current state of the product 
        """
        self.id = id
        self.storage = []


    def add_part_to_storage(part):
        if len(self.storage) < 2:
            print("all good")
            self.storage.append(part)
        else:
            print("Robot with id: " + self.id + " is full")

# Will only run if this is the main file being runned
if __name__ == '__main__':
    # Creates an Robot object with the ID of 99
    mc_turner = Robot(99)

    mc_turner.add_part_to_storage("C1")
    mc_turner.add_part_to_storage("C1")
    mc_turner.add_part_to_storage("C1")
