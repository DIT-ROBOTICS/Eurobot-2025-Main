import xml.etree.ElementTree as ET
import copy
import os, os.path, sys
import glob

### Create XML element tree
# root = ET.Element("root", {"BTCPP_format": "4"})
# main_tree = ET.ElementTree(root)

output_file_name = 'bot1_blue_e.xml'
points_list = []
dock_type_list = ['dock_y_slow_precise', 'dock_x_slow_precise', 'dock_x_slow_precise', 'dock_y_slow_precise', 'dock_y_slow_precise', 'dock_y_slow_precise', 'dock_y_slow_precise', 'dock_x_slow_precise', 'dock_x_slow_precise', 'dock_y_slow_precise', 'dock_y_slow_precise', 'dock_x_slow_precise', 'dock_y_slow_precise', 'dock_y_slow_precise', 'dock_y_slow_precise', 'dock_x_slow_precise', 'dock_y_slow_precise', 'dock_y_slow_precise', 'dock_y_slow_precise', 'dock_y_slow_precise']
offset_list = ['0.1', '-0.1', '-0.1', '-0.1', '-0.1', '-0.1', '-0.1', '-0.1']

def read_input(root):
    i = 0
    list_size = 5 + 3

    color = input('input team color: ')
    print('input points: ')
    for i in range(0, list_size):
        pt = int(input())
        if (color == 'blue'):
            if (pt < 10):
                pt = 9 - pt
            else:
                pt = pt + 4
        points_list.append(str(pt))

    i = 0
    for elem in root:
        if (elem.tag == "BehaviorTree" and elem.get('ID') == "MissionPointOne"):
            subtree = elem[0][1]
            for action in subtree:
                if (action.tag == "VisionCheck" and action.get('base_index') != "-1"):
                    action.set('base_index', points_list[i])
                    action.set('dock_type', dock_type_list[int(points_list[i])])
                    i += 1
            subtree = elem[0][2]
            for action in subtree[0][1][1]:
                if (action.tag == "VisionCheck" and action.get('base_index') != "-1"):
                    action.set('base_index', points_list[i])
                    action.set('dock_type', dock_type_list[int(points_list[i])])
                    i += 1
            subtree = elem[0][3]
            for action in subtree[0][0][0]:
                if (action.tag == "MissionNearRival"):
                    action.set('base_index', points_list[i])
                    i += 1
                    docking = action[0]
                    docking.set('dock_type', dock_type_list[int(points_list[i - 1])])
                    docking.set('offset', offset_list[int(points_list[i - 1]) - 11])
            for action in subtree:
                if (action.tag == "SubTree" and action.get('ID') == "ThreeLevelsBot1new"):
                    action.set('dock_type', dock_type_list[int(points_list[i - 1])])
                if (action.tag == "MissionSuccess"):
                    action.set('base_index', points_list[i - 1])
        if (elem.tag == "BehaviorTree" and elem.get('ID') == "MissionPointTwo"):
            subtree = elem[0][0]
            for action in subtree[0]:
                if (action.tag == "VisionCheck" and action.get('base_index') != "-1"):
                    action.set('base_index', points_list[i])
                    action.set('dock_type', dock_type_list[int(points_list[i])])
                    i += 1
            subtree = elem[0][1]
            for action in subtree[0]:
                if (action.tag == "MissionNearRival"):
                    action.set('base_index', points_list[i])
                    i += 1
                    docking = action[0]
                    docking.set('dock_type', dock_type_list[int(points_list[i - 1])])
                    docking.set('offset', offset_list[int(points_list[i - 1]) - 11])
            action = subtree[1]
            if (action.tag == "SubTree" and action.get('ID') == "ThreeLevelsBot1new"):
                action.set('dock_type', dock_type_list[int(points_list[i - 1])])
            action = subtree[2]
            if (action.tag == "MissionSuccess"):
                action.set('base_index', points_list[i - 1])
        if (elem.tag == "BehaviorTree" and elem.get('ID') == "MissionPointThree"):
            subtree = elem[0][0]
            for action in subtree:
                if (action.tag == "VisionCheck" and action.get('base_index') != "-1"):
                    action.set('base_index', points_list[i])
                    action.set('dock_type', dock_type_list[int(points_list[i])])
                    i += 1
            subtree = elem[0][2]
            for action in subtree[0]:
                if (action.tag == "VisionCheck" and action.get('base_index') != "-1"):
                    action.set('base_index', points_list[i])
                    action.set('dock_type', dock_type_list[int(points_list[i])])
                    i += 1
            subtree = elem[0][3]
            for action in subtree[0]:
                if (action.tag == "MissionNearRival"):
                    action.set('base_index', points_list[i])
                    i += 1
                    docking = action[0]
                    docking.set('dock_type', dock_type_list[int(points_list[i - 1])])
                    docking.set('offset', offset_list[int(points_list[i - 1]) - 11])
            action = subtree[1]
            if (action.tag == "SubTree" and action.get('ID') == "ThreeLevelsBot1new"):
                action.set('dock_type', dock_type_list[int(points_list[i - 1])])
            action = subtree[2]
            if (action.tag == "MissionSuccess"):
                action.set('base_index', points_list[i - 1])
    tree = ET.ElementTree(root)
    tree.write(output_file_name, encoding="utf-8", xml_declaration=True)

def create_tree():
    input_tree = ET.parse('bot1_blue_a.xml')
    root_in = input_tree.getroot()
    root_out = ET.Element("root", {"BTCPP_format": "4"})
    root_out.clear()

    # Find element to copy 
    # Create a copy
    # Append the copy 
    while (root_in.find("BehaviorTree") != None):
        bt = root_in.find("BehaviorTree")
        # bt.set('t') = "t"
        root_in.remove(root_in.find("BehaviorTree"))
        new_bt = copy.deepcopy(bt)
        root_out.append(new_bt)
    tnm = root_in.find("TreeNodesModel")
    new_tnm = copy.deepcopy(tnm)
    root_out.append(new_tnm)

    read_input(root_out)
    
# Defining main function
def main():
    create_tree()

# Using the special variable 
# __name__
if __name__=="__main__":
    main()