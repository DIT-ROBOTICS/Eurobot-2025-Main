import xml.etree.ElementTree as ET
import os, os.path, sys
import glob
import queue

### Create XML element tree
root = ET.Element("root", {"BTCPP_format": "4"})
main_tree = ET.ElementTree(root)

type_list = []
number_list = []
points_list = []

mis_linked_dict = {10: [11, 2, 30, 31], 11: [2, 30, 31], 2: [30, 31], 30: [31, 4], 31: [4]}
mis_linked_check = [[0, 0, 0, 0], [0, 0, 0], [0, 0], [0, 0], [0]]
mis_total_num = 3
mis_current_num = 0
mat_linked_dict = {1: [2, 3, 4], 2: [3, 4], 3: [4, 5, 6, 9], 4: [6], 5: [6, 7, 8, 9], 6: [7, 8, 9], 7: [8, 9], 8: [9]}
mat_linked_check = [[0, 0, 0], [0, 0], [0, 0, 0, 0], [0], [0, 0, 0, 0], [0, 0, 0], [0, 0], [0]]
mis_total_num = 5
mis_current_num = 0
result_mats_que = queue.Queue()
result_miss_que = queue.Queue()
x = -1
y = -1
z = -1

def visit_linked_list(num):
    if ()

def generate_plans():
    i = 0
    j = 0
    while (mis_linked_check[i][j] == 0):
        mis_current_num = 0
        # while (mis_current_num < mis_total_num):

        mis_linked_check[i][j] = 1
        mis_current_num += 1
        result_miss_que.push(mis_linked_dict[str(i)][j])
        i

        


def get_source_point(type, num):
    input_tree = ET.parse('source_points.xml')
    root = input_tree.getroot()
    materials = root[0]
    missions = root[1]
    if (type == "material"):
        x = materials[num][0]
        y = materials[num][1]
        z = materials[num][2]
        offset = materials[num][3]
        # for mat in materials:
    elif (type == "mission"):
        x = missions[num][0]
        y = missions[num][1]
        z = missions[num][2]
        offset = missions[num][3]
        # for mis in missions:

def import_subtree_and_nodes():
    tree_a = ET.parse('compter_plan.xml') # get plan tree file
    root_a = tree_a.getroot() # get root of plan tree
    empty_model = ET.SubElement(root_a, "TreeNodesModel") # create area for storaging models
    data_b = ET.parse('subtrees.xml').getroot() # get root of models file
    # root = tree.getroot()
    for TreeNodesModel in data_b.iter('TreeNodesModel'): # find the target to be insert
        insertion_point = tree_a.findall("./TreeNodesModel")[0] # find the insert point in plan tree
        insertion_point.extend(TreeNodesModel)
    tree_a.write("compter_plan.xml", encoding="utf-8", xml_declaration=True)

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
    main_tree.write("compter_plan.xml")
    import_subtree_and_nodes()



# Using the special variable 
# __name__
if __name__=="__main__":
    main()