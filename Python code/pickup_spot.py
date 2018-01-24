class PickupSpot:
    """ Class to represent pickup spots for components.

    Attributes:
        id (int): A unique number for identifying a pickup spot
        type (str): Type of components to pick up at location
        location (Location): Where the pickup spot is located
        storage (int): How many components that the pickup spot has stored
    """

    def __init__(self, unique_id, component_type, location, in_storage=1000):
        """ Location initialize method

            Arguments:
                id (int): Initializes the ID of the pickup spot
                type (str): Sets the type of components to pick up at location
                location (Location): Initializes the location of the pickup spot
                in_storage (Optional(int)): Initializes number of components in storage
        """
        self.id = unique_id
        self.type = component_type
        self.location = location
        self.storage = in_storage

    def remove_component(self):
        """ Removes one component from storage if storage is not empty
        """
        if self.storage == 0:
            print("Pickup spot with ID: " + str(self.id) + " is empty.")
        else:
            self.storage -= 1

# Will only run if this is the main file being run
if __name__ == '__main__':
    # Creates a new instance of PickupSpot
    c6_spot = PickupSpot(123, "C6", "location")

    print("In storage:")
    print(c6_spot.storage)

    c6_spot.remove_component()
    c6_spot.remove_component()
    c6_spot.remove_component()

    print("In storage:")
    print(c6_spot.storage)

    for x in range(1000):
        c6_spot.remove_component()

    print("In storage:")
    print(c6_spot.storage)
