import json
import os
from grutopia.core.config import SimulatorConfig
from grutopia.core.env import BaseEnv
from grutopia.core.util.container import is_in_container
file_path = './GRUtopia/demo/configs/h1_locomotion.yaml'
sim_config = SimulatorConfig(file_path)

headless = False
webrtc = False

if is_in_container():
    headless = True
    webrtc = True

env = BaseEnv(sim_config, headless=headless, webrtc=webrtc)

import numpy as np
import tqdm

# from omni.isaac.kit import SimulationApp
# simulation_app = SimulationApp({"headless": True})

from pxr import Usd, UsdPhysics, UsdGeom
from pxr_utils.usd_physics import set_collider, set_rigidbody, remove_collider, activate_rigid, deactivate_rigid, copyfile, remove_rigid
# import sys
# sys.path.append("/home/percy/.local/share/ov/pkg/isaac-sim-2023.1.1/exts/omni.isaac.core")
from omni.isaac.core.utils.semantics import add_update_semantics
# print(os.path.dirname(omni.isaac.core.utils.semantics.__file__))
def bind_rigid(prim):
    remove_collider(prim)
    remove_rigid(prim)
    set_collider(prim, "sdf")
    set_rigidbody(prim)

def bind_static(prim):
    remove_collider(prim)
    remove_rigid(prim)
    set_collider(prim, "meshSimplification")

def bind_articulation(prim, pickable=False):
    remove_collider(prim)
    remove_rigid(prim)
    children = prim.GetChildren()
    # print(prim)
    for child in children:
        childName = str(child.GetName())
        # print(childName)

        if not childName.lower() in ['group_static', 'group_00'] and child.IsA(UsdGeom.Xform):
            # print("part", child)
            bind_rigid(child)

        if childName.lower() == 'group_00':
            # print("static part", child)
            if pickable:
                bind_rigid(child)
            else:
                bind_static(child)
        
        if childName.lower() == 'group_static':
            # print("static part", child)
            bind_static(child)
        
        if child.IsA(UsdPhysics.PrismaticJoint) or child.IsA(UsdPhysics.RevoluteJoint):
            attr = child.GetAttribute("physics:jointEnabled")
            # print(attr.Get())
            attr.Set(True)
            # flag = True




PICKABLE_OBJECTS = [
    "backpack", # 背包
    "basket", # 篮子
    "pan", # 平底锅
    "pot", # 锅
    "bottle", # 瓶子
    "clock", # 钟
    "laptop", # 笔记本电脑
    "mouse", # 鼠标
    "pen", # 笔
    "pillow", # 枕头
    "towel", # 毛巾
    "blanket", # 毯子
    "bowl", # 碗
    "light", # 灯
    "Musical_instrument", # 乐器
    "toy", # 玩具
    "keyboard", # 键盘
    "plate", # 盘子
    "telephone", # 电话
    "cup", # 杯子
    "picture", # 图片
    "tray" # 托盘
]

COLLIDER_FREE_OBJECTS = [
    "plant",
    "curtain",
    "clothes"
]


def bind_part_rigid(instance, pickable=True):
    children = instance.GetChildren()
    for child in children:
        childName = str(child.GetName())
        # print(childName)

        if not childName.lower() in ['group_static', 'group_00'] and child.IsA(UsdGeom.Xform):
            # print(childName)
            set_rigidbody(child)
            set_collider(child)

        if childName.lower() == 'group_00' and pickable:
            set_rigidbody(child)
            set_collider(child)
        
        if child.IsA(UsdPhysics.PrismaticJoint) or child.IsA(UsdPhysics.RevoluteJoint):
            attr = child.GetAttribute("physics:jointEnabled")
            # print(attr.Get())
            attr.Set(True)
            # flag = True

def set_semantic_label(prim, label):
    if prim.GetTypeName() == "Mesh":
        add_update_semantics(prim, semantic_label=label, type_label="class")

    all_children = prim.GetAllChildren()
    for child in all_children:
        set_semantic_label(child, label)




def bind_collider_remove_door_add_semantic():
    split = 'validation'
    with open('splits.json','r') as fp:
        data = json.load(fp)

    with open(f"doors_{split}.json",'r') as fp:
        main_door = json.load(fp)
    
    scenes = data[split]
    for scene in tqdm.tqdm(scenes):
        main_door_path = main_door[scene]
        scene_path = f"splits/{split}/scenes/{scene}/start_result_fix.usd"
        if not os.path.exists(scene_path):
            scene_path = f"splits/{split}/scenes/{scene}/start_result_new.usd"
        dest_scene_path = f"splits/{split}/scenes/{scene}/start_result_interaction_new.usd"
        copyfile(scene_path, dest_scene_path)

        stage = Usd.Stage.Open(dest_scene_path)
        door_path = "/Root/Meshes/BaseAnimation/door"
        for child in stage.GetPrimAtPath(door_path).GetChildren():
            cp = str(child.GetPrimPath())
            if cp != main_door_path:
                stage.RemovePrim(cp)

        meshes = stage.GetPrimAtPath("/Root/Meshes")
        set_collider(meshes)
        

        for scope in meshes.GetChildren():
            scope_name = str(scope.GetName())
            for cate in scope.GetChildren():
                cate_name = str(cate.GetName())
                for instance in cate.GetChildren():
                    if cate_name in PICKABLE_OBJECTS:
                        activate_rigid(instance)
                    
                    instname = str(instance.GetName())
                    label = f"{cate_name}/{instname}"
                    set_semantic_label(instance, label)


        ground = stage.GetPrimAtPath("/Root/Meshes/Base/ground")
        remove_collider(ground)
        set_collider(ground, UsdPhysics.Tokens.none)

        
        stage.GetRootLayer().Save()
        
        
    pass


def solve_scene(scene_path, dest_scene_path):
    copyfile(scene_path, dest_scene_path)

    stage = Usd.Stage.Open(dest_scene_path)
    meshes = stage.GetPrimAtPath("/Root/Meshes")
    # set_collider(meshes)
    total_active_objects = 0
    articulated_objects = 0
    max_articulated_obj = 40
    for scope in meshes.GetChildren():
        scope_name = str(scope.GetName())
        for cate in scope.GetChildren():
            cate_name = str(cate.GetName())
            for instance in tqdm.tqdm(cate.GetChildren(), desc=f"Scope {scope_name} Category {cate_name}"):
                if scope_name in ["BaseAnimation", "Animation"]:
                    pickable = cate_name in PICKABLE_OBJECTS
                    instance_prim = instance.GetPrimAtPath("Instance")
                    bind_articulation(instance_prim, pickable)
                    if articulated_objects > max_articulated_obj:
                        deactivate_rigid(instance_prim)
                    articulated_objects += 1
                else:
                    if cate_name in PICKABLE_OBJECTS:
                        # activate_rigid(instance)
                        bind_rigid(instance)
                        deactivate_rigid(instance)
                        total_active_objects += 1
                    else:
                        # deactivate_rigid(instance)
                        bind_static(instance)
                
                # instname = str(instance.GetName())
                # label = f"{cate_name}/{instname}"
                # set_semantic_label(instance, label)

    # ground = stage.GetPrimAtPath("/Root/Meshes/Base/ground")
    # remove_collider(ground)
    # set_collider(ground, UsdPhysics.Tokens.none)

    stage.GetRootLayer().Save()
    return total_active_objects, articulated_objects

def test_solve_scene():
    scene_path = "/home/percy/princeton/IsaacLab/Simple_Warehouse/full_warehouse.usd" # /home/percy/princeton/GRUtopia/assets/scenes/demo_house/demo_house.usd
    dest_scene_path = "/home/percy/princeton/IsaacLab/Simple_Warehouse/full_warehouse_w_rigid_body.usd"
    # scene_path = "/home/percy/princeton/GRUtopia/assets/scenes/demo_house/demo_house.usd"
    # dest_scene_path = "/home/percy/princeton/GRUtopia/assets/scenes/demo_house/demo_house_w_rigid_body.usd"
    total_active_objects, articulated_objects = solve_scene(scene_path, dest_scene_path)
    print(total_active_objects, "total_active_objects", articulated_objects, "articulated_objects")

test_solve_scene()
# bind_collider_remove_door_add_semantic()