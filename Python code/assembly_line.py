

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

    def add_part_to_storage(self, part):
        """ Adds a part to the assembly lines storage

        Args:
            part (str): A part to add to storage
        """
        self.storage.append(part)
        return "Stored " + part + " in assembly line with id: " + str(self.id)

    def assemble(self, product):
        """ Assemble specific product

        Args:
            product (Product): A product to assemble
        """
        # TODO: Assemble the product


# Will only run if this is the main file being run
if __name__ == '__main__':
    # Creates an AssemblyLine object with the ID of 10
    assembly_line_copenhagen = AssemblyLine(10, "placeholder_location")

    print(assembly_line_copenhagen.id)
    print(assembly_line_copenhagen.location)
    print(assembly_line_copenhagen.storage)
    print(assembly_line_copenhagen.add_part_to_storage("C1"))
    print(assembly_line_copenhagen.add_part_to_storage("C1"))
    print(assembly_line_copenhagen.add_part_to_storage("C1"))
    print(assembly_line_copenhagen.storage)
