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

    def add_to_storage(self, component):
        """ Adds a component to the robots storage

        Args:
            component (str): A part to add to storage
        """

        # Checks if the robot is already full
        if len(self.storage) < 2:
            # Adds the component to the robots storage
            self.storage.append(component)
            return "Added " + component + " to robot with id: " + str(self.id)
        else:
            # Returns message that robot is full
            return"Robot with id: " + str(self.id) + " is full"

    def check_storage_for(self, component_list):
        """ Checks the assembly line storage for a list of parts and returns missing components

        Args:
            component_list (str[list]): List of the components to check for
        """
        # Used for storing which components are still missing
        still_needed = []
        # Creates a cloned list of what is in storage
        temp = list(self.storage)
        # Goes through the components in the list
        for component in component_list:
            # If the component is not in storage append it to the still needed list
            if component not in temp:
                still_needed.append(component)
            # If the component is in storage remove it from the temporary storage clone
            else:
                temp.remove(component)
        # Returns the components still needed
        if still_needed:
            return still_needed
        else:
            return "none"

    def unload_components(self, assembly_line):
        """ Unloads components at specific assembly line

        Args:
            assembly_line (AssemblyLine): Where the components should be unloaded
        """
        # Goes through the robot's storage and adds the components to the assembly line
        for component in self.storage:
            print(assembly_line.add_to_storage(component))
        # Clears the robot's storage
        self.storage[:] = []

    def go_to(self, location):
        """ Sends the robot to a specific location

        Args:
            location (Location): A location to go to
        """
        # TODO: Make the robot drive to location


# Will only run if this is the main file being run
if __name__ == '__main__':
    # Creates an Robot object with the ID of 99
    mc_turner = Robot(99)

    print(mc_turner.add_to_storage("C1"))
    print(mc_turner.add_to_storage("C1"))
    print(mc_turner.add_to_storage("C1"))
