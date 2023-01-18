#!/usr/bin/env python  
import yaml

with open(r'/home/crbt/misiones/m5.yaml') as file:
    # The FullLoader parameter handles the conversion from YAML
    # scalar values to Python the dictionary format
    fruits_list = yaml.load(file, Loader=yaml.FullLoader)
    #maquinas = fruits_list["maquinas"]["ID"]
    print(fruits_list["datos"]["numero_maquinas"])
    #print(type(maquinas[0]))