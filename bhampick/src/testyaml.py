#!/usr/bin/env python3

import yaml

class read_dimension_yaml:
    def __init__(self):
        with open('/home/bham/ws_moveit/src/bhampick/conf/test.yml', 'r') as file:
            self.config_data = yaml.safe_load(file)

    def get_dimension_from_name(self, name):
        for item in self.config_data:
            if item["name"] == name:
                return item["vector"]["x"], item["vector"]["y"], item["vector"]["z"]
            
if __name__ == "__main__":
    x, y, z = read_dimension_yaml().get_dimension_from_name("conflakes box")
    print(x, y, z)