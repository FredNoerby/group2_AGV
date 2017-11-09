#!/usr/bin/env python

# Dictionary of products
product = { "P1": ["C1", "C3", "C4", "C4"],
			"P2": ["C1", "C2", "C5", "C6"],
			"P3": ["C3", "C3", "C5"],
			"P4": ["C2", "C3", "C4"]}


""" Takes in a product name and returns the parts"""
def print_parts(name):
	if name in product:
		print("Product " + name + " consists of:")
		parts = product[name]
		for part in parts:
			print(part)
	else:
		print(name + " not found in product range.")


""" Takes in a product and returns the parts as a list"""
def return_parts(name):
	part_list = []
	if name in product:
		parts = product[name]
		for part in parts:
			part_list.append(part)
		return part_list


""" Takes in a list of products and returns a list of parts"""
def get_parts_to_fetch(products):
	parts_joined = []
	for product in products:
		parts_joined += return_parts(product)
	return parts_joined


if __name__ == "__main__":
	#return_parts('P1')
	listerern = ["P1", "P3", "P2", "P4"]
	get_parts_to_fetch(listerern)
	