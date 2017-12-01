class Product:
    """ Class to represent products to be made 
    
    Attributes:
        id (int): A unique number for identifying products
        type (str): The type of product (P1, P2, P3, or P4)
        parts (str[list]): The parts the product consists of
        status (str): The current state of the product
    """

    def __init__(self, unique_id, type_of_product, status="Waiting"):
        """ Product init method

        Args:
            unique_id (int): Initializes the product ID
            type_of_product (str): Initializes the type of the product
            status (Optional[str]): Initializes the current status of the product
        """
        self.id = unique_id
        self.type = type_of_product
        self.status = status
        # The parts needed to make the product is fetched from the convertProductsToParts library
        self.parts = self.get_parts()

    def get_parts(self):
        """ Returns the parts of a specific product type
        """

        # Dictionary of products
        product_dict = {"P1": ["C1", "C3", "C4", "C4"],
                        "P2": ["C1", "C2", "C5", "C6"],
                        "P3": ["C3", "C3", "C5"],
                        "P4": ["C2", "C3", "C4"]}

        # Checks if the type is in the dictionary
        if self.type in product_dict:
            # Returns the parts needed for the product
            return product_dict[self.type]

# Will only run if this is the main file being run
if __name__ == '__main__':
    # Creates an Product object with the ID of 2112 and the type of P2
    new_product = Product(2112, "P4")

    # Prints all the stored information for the Product object
    print(new_product.id)
    print(new_product.type)
    print(new_product.parts)
    print(new_product.status)
