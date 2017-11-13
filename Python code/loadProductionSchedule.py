#!/usr/bin/env python
import openpyxl
from products import Product

""" Goes through the production plan excel sheet and returns a list of products"""
def load_production_plan():
    more_products = True
    incrementer = 3

    production_list = []

    # Adds all the cells that have a value to the production list
    while more_products:
        if sheet_one.cell(row=incrementer, column=1).value:

            sheet_one.cell(row=incrementer, column=1).value
            production_list.append( Product((incrementer-2), str(sheet_one.cell(row=incrementer, column=1).value)))
        else:
            more_products = False

        incrementer += 1

    return production_list

# Uses the openpyxl library to open the production plan excel file
workbook = openpyxl.load_workbook('/home/sev-ros/catkin_ws/src/group2_AGV/Daily_Production_Plan.xlsx')
type(workbook)

# Stores the first sheet of the excel file
sheet_one = workbook.get_sheet_by_name('Plan')

# If this is run as main the cells will be printed
if __name__ == "__main__":
    
    printerthings = load_production_plan()

    for product in printerthings:
        print (product.id, product.type, product.parts, product.status)