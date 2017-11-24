class Location:
    """ Class to represent locations in the map

    Attributes:
        id (int): A unique number for identifying a location
        position (float[list]): x, y, & z values for the location
        orientation (float[list]): quaternion representing rotation
    """
    #TODO: EVERYTHING
    def __init__(self, unique_id, position, orientation):
        """ Location initialize method

            Arguments:
                id (int): Initializes the ID of the location
                position (Position): Initializes the position
                orientation (Orientation): Initializes the orientation
            """
        self.id = unique_id
        self.position = position
        self.orientation = orientation

    translation = [x, y, z]
