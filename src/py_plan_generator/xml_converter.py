import xml.etree.ElementTree as ET
import copy
import os, os.path, sys
import glob

### Create XML element tree
# root = ET.Element("root", {"BTCPP_format": "4"})
# main_tree = ET.ElementTree(root)

output_file_name = 'bot1_blue_e.xml'
points_list = []
yellow_home = '0.35'
blue_home = '2.65'
banner = []
yellow_banner = ['2.75', '1.25', '0.75']
blue_banner = ['0.25', '1.75', '2.25']
dock_type_list = ['dock_y_slow_precise', 'dock_x_slow_precise', 'dock_x_slow_precise', 'dock_y_slow_precise', 'dock_y_slow_precise', 'dock_y_slow_precise', 'dock_y_slow_precise', 'dock_x_slow_precise', 'dock_x_slow_precise', 'dock_y_slow_precise', 'dock_y_slow_precise', 'dock_x_slow_precise', 'dock_y_slow_precise', 'dock_y_slow_precise', 'dock_y_slow_precise', 'dock_x_slow_precise', 'dock_y_slow_precise', 'dock_y_slow_precise', 'dock_y_slow_precise', 'dock_y_slow_precise']
offset_list = ['0.1', '-0.1', '-0.1', '-0.1', '-0.1', '-0.1', '-0.1', '-0.1']

def read_input(root):
    i = 0
    list_size = 5 + 3

    color = input('input team color: ')
    output_file_name = input('input file name: ')
    if (color == 'blue'):
        banner = blue_banner
        home = blue_home
    else:
        banner = yellow_banner
        home = yellow_home
    print('input points: ')
    points_list.clear()
    for i in range(0, list_size):
        pt = int(input())
        if (color == 'blue'):
            if (pt < 10):
                pt = 9 - pt
            else:
                pt = pt + 4
        points_list.append(str(pt))
    banner_pt = input()
    print(points_list)

    i = 0
    j = 0
    k = 0
    l = 0
    for elt in root.iter():
        if (elt.tag == "BehaviorTree" and elt.get('ID') == "BannerMission"):
            for dock in elt.iter():
                if (dock.tag == "Docking" and l < 11):
                    dock.set('base', banner[l % 3] + ', 0.3, 0')
                    l += 1
                    if (l == 3 or l == 7):
                        l += 1
        elif (elt.tag == "BehaviorTree" and elt.get('ID') == "MainTree"):
            for dock in elt.iter():
                if (dock.tag == "Docking"):
                    dock.set('base', home + ', 1.6, 0')
        elif (elt.tag == "BehaviorTree" and (elt.get('ID') == "MissionPoint1" or elt.get('ID') == "MissionPoint2" or elt.get('ID') == "MissionPoint3")):
            for node in elt.iter():
                if (node.tag == "MaterialChecker" and node.get('base_index') != "-1"):
                    node.set('base_index', points_list[i])
                    print(points_list[i])
                    i += 1
                if (node.tag == "MissionChecker"):
                    node.set('base_index', points_list[i])
                    print(points_list[i])
                    i += 1
                    docking = node[0]
                    docking.set('dock_type', dock_type_list[int(points_list[i - 1])])
                    docking.set('offset', offset_list[int(points_list[i - 1]) - 11])
                if (node.tag == "SubTree" and node.get('ID') == "ThreeLevelsBot1new"):
                    node.set('index', points_list[i - 1])
                if (node.tag == "SubTree" and node.get('ID') == "BannerMission"):
                    node.set('BANNER_PLACE', banner_pt)
    tree = ET.ElementTree(root)
    tree.write(output_file_name, encoding="utf-8", xml_declaration=True)

def create_tree():
    input_tree = ET.parse('bot1_blue_a_template.xml')
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
    repeat = int(input('repeat times: '))
    for i in range(0, repeat):
        create_tree()

# Using the special variable 
# __name__
if __name__=="__main__":
    main()