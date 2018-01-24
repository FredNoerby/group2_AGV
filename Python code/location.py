class Location:
    """ Class to represent locations in the map

    Attributes:
        position (float[list]): x, y, & z values for the location
        orientation (float[list]): quaternion representing rotation
    """

    def __init__(self, position=[0.0, 0.0, 0.0], orientation=[0.0, 0.0, 0.0, 1.0]):
        """ Location initialize method

            Arguments:
                position (float[list]): Initializes the position of the location
                orientation (float[list]): Initializes the orientation of the location
            """
        self.position = position
        self.orientation = orientation

# Will only run if this is the main file being run
if __name__ == '__main__':
    # Creates a new instance of Location
    group_room = Location()

    print("X:")
    print(group_room.position[0])
    print("Y:")
    print(group_room.position[1])
    print("Z:")
    print(group_room.position[2])

    print("Rotation around X-axis:")
    print(group_room.orientation[0])
    print("Rotation around Y-axis:")
    print(group_room.orientation[1])
    print("Rotation around Z-axis:")
    print(group_room.orientation[2])
    print("Scalar:")
    print(group_room.orientation[3])
