# Imports the convertProductsToParts library used to get the parts of an product type
import convertProductsToParts


class Product:
    """ Class to represent products to be made 
    
    Attributes:
        id (int): A unique number for identifying products
        type (str): the type of product (P1, P2, P3, or P4)
        parts (str[list]): the parts needed to assembly the product
        status (str): the current state of the product 
    """

    def __init__(self, unique_id, type_of_product, status="Waiting"):
        """ Product init method

        Args:
            id (int): A unique number for identifying products
            type (str): the type of product (P1, P2, P3, or P4)
            status (Optional[str]): the current state of the product 
        """
        self.id = unique_id
        self.type = type_of_product
        self.status = status
        self.parts = convertProductsToParts.return_parts(type)

# Will only run if this is the main file being run
if __name__ == '__main__':
    # Creates an Product object with the ID of 2112 and the type of P2
    new_product = Product(2112, "P2")

    # Prints all the stored information for the Product object
    print(new_product.id)
    print(new_product.type)
    print(new_product.parts)
    print(new_product.status)