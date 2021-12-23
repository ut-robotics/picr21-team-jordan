import json
import os


ID = "001TRT"
with open("command.json", "r") as json_file:
    json_obj = json.load(json_file)

signal = json_obj["signal"]
if signal == "start":
    index = json_obj["targets"].index(ID)
    target_basket = json_obj["baskets"][index]
    
