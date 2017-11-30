class Location:
    """ Class to represent locations in the map

    Attributes:
        id (int): A unique number for identifying a location
        position (float[list]): x, y, & z values for the location
        orientation (float[list]): quaternion representing rotation
    """

    def __init__(self, unique_id, position=[0.0, 0.0, 0.0], orientation=[0.0, 0.0, 0.0, 1.0]):
        """ Location initialize method

            Arguments:
                id (int): Initializes the ID of the location
                position (float[list]): Initializes the position of the location
                orientation (float[list]): Initializes the orientation of the location
            """
        self.id = unique_id
        self.position = position
        self.orientation = orientation

# Will only run if this is the main file being run
if __name__ == '__main__':
    # Creates a new instance of Location
    group_room = Location(1)

    print("X:", end=" ")
    print(group_room.position[0])
    print("Y:", end=" ")
    print(group_room.position[1])
    print("Z:", end=" ")
    print(group_room.position[2])

    print("Rotation around X-axis:", end=" ")
    print(group_room.orientation[0])
    print("Rotation around Y-axis:", end=" ")
    print(group_room.orientation[1])
    print("Rotation around Z-axis:", end=" ")
    print(group_room.orientation[2])
    print("Scalar:", end=" ")
    print(group_room.orientation[3])
