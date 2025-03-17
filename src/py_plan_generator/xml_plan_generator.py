import xml.etree.ElementTree as ET
import os, os.path, sys
import glob

### Create XML element tree
root = ET.Element("root", {"BTCPP_format": "4"})
main_tree = ET.ElementTree(root)

type_list = []
number_list = []
points_list = []

def read_input():
    input_tree = ET.parse('input_data.xml')
    root = input_tree.getroot()
    for elem in root:
        # print(elem.tag, elem[1].text, elem.attrib)
        for i in elem:
            print(i.text)
            if (i.tag == "type"):
                type_list.append(i.text)
            if (i.tag == "number"):
                number_list.append(i.text)
            if (i.tag != "type" and i.tag != "number"):
                points_list.append(i.text)
    print(points_list)

def import_subtree_and_nodes():
    tree_a = ET.parse('test_output.xml') # get plan tree file
    root_a = tree_a.getroot() # get root of plan tree
    empty_model = ET.SubElement(root_a, "TreeNodesModel") # create area for storaging models
    data_b = ET.parse('subtrees.xml').getroot() # get root of models file
    # root = tree.getroot()
    for TreeNodesModel in data_b.iter('TreeNodesModel'): # find the target to be insert
        insertion_point = tree_a.findall("./TreeNodesModel")[0] # find the insert point in plan tree
        insertion_point.extend(TreeNodesModel)
    tree_a.write("test_output.xml", encoding="utf-8", xml_declaration=True)

def create_tree(name, point_numbers):
    print(name, point_numbers)
    BehaviorTree = ET.SubElement(root, "BehaviorTree", {"ID": "mission"+str(name)})
    Sequence = ET.SubElement(BehaviorTree, "Sequence")
    for i in range(0, point_numbers):
        if (points_list[(i-1)*3+2] == "0.0" or points_list[(i-1)*3+2] == "2.0"):
            dock_type = "mission_dock_x"
        else:
            dock_type = "mission_dock_y"
        Docking = ET.SubElement(Sequence, "Docking", {"final_pose": "{PrevGoal}", "isPureDocking": "0", "dock_type": dock_type, "shift": "0", "offset": "-0.2", "base": str(points_list[(i-1)*3])+", "+str(points_list[(i-1)*3+1])+", "+str(points_list[(i-1)*3+2])})

def divid_sub_tree():
    mission_number = 0
    point_numbers = 0
    for i in type_list:
        point_numbers += 1
        if (i == "mission"):
            create_tree(mission_number, point_numbers)
            mission_number += 1
            point_numbers = 0
    
# Defining main function
def main():
    print("start reading")
    read_input()
    divid_sub_tree()

    # ### Write XML element tree to file
    main_tree.write("test_output.xml")
    import_subtree_and_nodes()



# Using the special variable 
# __name__
if __name__=="__main__":
    main()