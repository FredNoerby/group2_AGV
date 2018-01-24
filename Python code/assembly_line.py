class AssemblyLine:
    """ Class to represent assembly lines

    Attributes:
        id (int): A unique number for identifying assembly line
        storage (str[list]): what is currently in the assembly line
        location (Location): where the assembly line is located
    """

    def __init__(self, unique_id, location, in_storage=[]):
        """ Robot initialize method

        Arguments:
            id (int): Initializes the ID of the assembly line
            location (Location): Initializes the location of the assembly line
            in_storage (Optional(str[list])): Initializes the parts in storage
        """
        self.id = unique_id
        # The storage is set to empty unless a list of parts is passed to it
        self.storage = in_storage
        self.location = location

    def add_to_storage(self, component):
        """ Adds a component to the assembly lines storage

        Args:
            component (str): A component to add to storage
        """
        self.storage.append(component)
        return "Stored " + component + " in assembly line with id: " + str(self.id)

    def check_storage_for(self, component_list):
        """ Checks the assembly line storage for a list of components and returns missing components

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

    def assemble(self, product):
        """ Assembles specific product

        Args:
            product (Product): A product to assemble
        """
        for component in product.components:
            if component in self.storage:
                self.storage.remove(component)

        print("Product with ID: " + str(product.id) + " assembled in assembly line " + str(self.id))
        product.status = "Ready for QC"


# Will only run if this is the main file being run
if __name__ == '__main__':
    # Creates an AssemblyLine object with the ID of 10
    assembly_line_copenhagen = AssemblyLine(10, "placeholder_location")

    print(assembly_line_copenhagen.id)
    print(assembly_line_copenhagen.location)
    print(assembly_line_copenhagen.storage)
    print(assembly_line_copenhagen.add_to_storage("C1"))
    print(assembly_line_copenhagen.add_to_storage("C4"))
    print(assembly_line_copenhagen.add_to_storage("C3"))
    print(assembly_line_copenhagen.add_to_storage("C2"))
    print("In storage:")
    print(assembly_line_copenhagen.storage)

    list_components = ["C1", "C2", "C3", "C4", "C2"]
    print("Check if storage has the following:")
    print(list_components)
    print("Storage is missing:")
    print(assembly_line_copenhagen.check_storage_for(list_components))
