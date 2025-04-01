import xml.etree.ElementTree as ET
import os, os.path, sys
import glob

### Create XML element tree
root = ET.Element("root", {"BTCPP_format": "4"})
main_tree = ET.ElementTree(root)

type_list = []
number_list = []
points_list = []
mis_dict_name = [10, 11, 2, 30, 31]
mis_linked_dict = {10: [11, 2, 30, 31], 11: [2, 30, 31], 2: [30, 31], 30: [31, 4], 31: [4]}
mis_linked_check = {10: [0, 0, 0, 0], 11: [0, 0, 0], 2: [0, 0], 30: [0, 0], 31: [0]}
mat_dict_name = [1, 2, 3, 4, 5, 6, 7, 8]
mat_linked_dict = {1: [2, 3, 4], 2: [3, 4], 3: [4, 5, 6, 9], 4: [6], 5: [6, 7, 8, 9], 6: [7, 8, 9], 7: [8, 9], 8: [9]}
mat_linked_check = {1: [0, 0, 0], 2: [0, 0], 3: [0, 0, 0, 0], 4: [0], 5: [0, 0, 0, 0], 6: [0, 0, 0], 7: [0, 0], 8: [0]}
result_mats_que = []
result_miss_que = []

def sort_mission_points():
    for name in mis_dict_name:
        i = name
        j = 0
        for index in mis_dict_name:
            for sub_index in range(0, len(mis_linked_check[index])):
                mis_linked_check[index][sub_index] = 0
        set = []
        set.append(name)
        while len(set) > 0:
            if mis_linked_check[i][j] == 0:
                mis_linked_check[i][j] = 1
                set.append(mis_linked_dict[i][j])
                i = mis_linked_dict[i][j]
                j = 0
            if len(set) == 3:
                print(set)
                result_miss_que.append(set)
                temp = []
                for item in set:
                    temp.append(item)
                result_miss_que.append(temp.reverse())
                # set = temp
                set.pop()
                i = set[len(set) - 1]
                j = 0
            if i == 4:
                set.pop()
                if (len(set) > 0):
                    i = set[len(set) - 1]
                j = 0
            if mis_linked_check[i][j] == 1 and j < len(mis_linked_check[i]) - 1:
                j += 1
                index = len(mis_dict_name) - 1
                while mis_dict_name[index] != i:
                    for sub_index in range(0, len(mis_linked_check[mis_dict_name[index]])):
                        mis_linked_check[mis_dict_name[index]][sub_index] = 0
                    index -= 1
            elif mis_linked_check[i][j] == 1 and j >= len(mis_linked_check[i]) - 1:
                set.pop()
                if (len(set) > 0):
                    i = set[len(set) - 1]
                j = 0

def sort_material_points():
    for name in mat_dict_name:
        i = name
        j = 0
        for index in mat_dict_name:
            for sub_index in range(0, len(mat_linked_check[index])):
                mat_linked_check[index][sub_index] = 0
        set = []
        set.append(name)
        while len(set) > 0:
            if mat_linked_check[i][j] == 0:
                mat_linked_check[i][j] = 1
                set.append(mat_linked_dict[i][j])
                i = mat_linked_dict[i][j]
                j = 0
            if len(set) == 5:
                print(set)
                result_mats_que.append(set)
                temp = []
                for item in set:
                    temp.append(item)
                result_miss_que.append(temp.reverse())
                set.pop()
                i = set[len(set) - 1]
                j = 0
            if i == 4:
                set.pop()
                if (len(set) > 0):
                    i = set[len(set) - 1]
                j = 0
            if mat_linked_check[i][j] == 1 and j < len(mat_linked_check[i]) - 1:
                j += 1
                index = len(mat_dict_name) - 1
                while mat_dict_name[index] != i:
                    for sub_index in range(0, len(mat_linked_check[mat_dict_name[index]])):
                        mat_linked_check[mat_dict_name[index]][sub_index] = 0
                    index -= 1
            elif mat_linked_check[i][j] == 1 and j >= len(mat_linked_check[i]) - 1:
                set.pop()
                if (len(set) > 0):
                    i = set[len(set) - 1]
                j = 0

def get_source_point(type, num):
    input_tree = ET.parse('source_points.xml')
    root = input_tree.getroot()
    materials = root[0]
    missions = root[1]
    if (type == "material"):
        x = materials[num][0].text
        y = materials[num][1].text
        z = materials[num][2].text
        offset = materials[num][3].text
    elif (type == "mission"):
        x = missions[num][0].text
        y = missions[num][1].text
        z = missions[num][2].text
        offset = missions[num][3].text
    return x, y, z, offset

def create_tree(mission_set, material_set):
    docking_input = []
    BehaviorTree = ET.SubElement(root, "BehaviorTree", {"ID": "mission1"})
    Sequence = ET.SubElement(BehaviorTree, "Sequence")
    x, y, z, offset = get_source_point("material", mat_dict_name.index(material_set[0]))
    if (z == "0.0" or z == "2.0"):
        dock_type = "mission_dock_x"
    else:
        dock_type = "mission_dock_y"
    Docking = ET.SubElement(Sequence, "Docking", {"final_pose": "{PrevGoal}", "isPureDocking": "0", "dock_type": dock_type, "shift": "0", "offset": str(offset), "base": str(x)+", "+str(y)+", "+str(z)})
    x, y, z, offset = get_source_point("material", mat_dict_name.index(material_set[0]))
    if (z == "0.0" or z == "2.0"):
        dock_type = "mission_dock_x"
    else:
        dock_type = "mission_dock_y"
    Docking = ET.SubElement(Sequence, "Docking", {"final_pose": "{PrevGoal}", "isPureDocking": "0", "dock_type": dock_type, "shift": "0", "offset": str(offset), "base": str(x)+", "+str(y)+", "+str(z)})
    x, y, z, offset = get_source_point("material", mat_dict_name.index(material_set[0]))
    if (z == "0.0" or z == "2.0"):
        dock_type = "mission_dock_x"
    else:
        dock_type = "mission_dock_y"
    Docking = ET.SubElement(Sequence, "Docking", {"final_pose": "{PrevGoal}", "isPureDocking": "0", "dock_type": dock_type, "shift": "0", "offset": str(offset), "base": str(x)+", "+str(y)+", "+str(z)})
    

    x, y, z, offset = get_source_point("material", mat_dict_name.index(material_set[i * 2 + 1]))
    x, y, z, offset = get_source_point("mission", mis_dict_name.index(mission_set[i]))

def generate_plans():
    for mission_set in result_miss_que:
        for material_set in result_mats_que:
            create_tree(mission_set, material_set)

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
    
# Defining main function
def main():
    print("start reading")
    sort_mission_points()
    sort_material_points()
    generate_plans()
    import_subtree_and_nodes()

    # ### Write XML element tree to file
    # main_tree.write("compter_plan.xml")
    # import_subtree_and_nodes()

# Using the special variable 
# __name__
if __name__=="__main__":
    main()