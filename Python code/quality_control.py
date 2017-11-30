from product import Product


class QualityControl:
    """ Class to represent quality control units

    Attributes:
        id (int): A unique number for identifying a quality control unit
    """

    def __init__(self, unique_id):
        """ Quality control init method

    Args:
        unique_id (int): Initializes the product ID"""
        self.id = unique_id

    def pass_product(self, product):
        """ Changes status of product to 'passed'

        Args:
             product (Product) : product to pass
        """

        product.status = "Passed from QC " + str(self.id)

    def fail_product(self, product):
        """ Changes status of product to 'failed'

        Args:
            product (Product) : product to fail
        """

        product.status = "Failed from QC " + str(self.id)


# Will only run if this is the main file being run
if __name__ == '__main__':
    # Creates a new QualityControl
    new_p = QualityControl(63)

    # Creates a new Product with ID of 1 and type P1
    new_product = Product(1, "P1")

    print("Product ID: " + str(new_product.id))
    print("Status received: " + new_product.status)
    new_p.pass_product(new_product)
    print("Current status: " + new_product.status)
