import convertProductsToParts

class Product:
    """ Class to represent products to be made 
    
    Attributes:
        id (int): A unique number for identifying products
        type (str): the type of product (P1, P2, P3, or P4)
        Parts (str[list]): the parts needed to assembly the product
        status (str): the current state of the product 
    """

    def __init__(self, id, type, status="waiting"):
        """ Product init method

        Args:
            id (int): A unique number for identifying products
            type (str): the type of product (P1, P2, P3, or P4)
            status (Optional[str]): the current state of the product 
        """
        self.id = id
        self.type = type
        self.status = status
        self.parts = convertProductsToParts.return_parts(type)


if __name__ == '__main__':
    new_product = Product(2112, "P2")

    print(new_product.id)
    print(new_product.type)
    print(new_product.parts)
    print(new_product.status)