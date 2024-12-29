from pxr import Usd, UsdGeom, UsdPhysics, Sdf

# Load the USD stage
stage = Usd.Stage.Open("/home/percy/princeton/IsaacLab/Simple_Warehouse/full_warehouse.usd")

# Define default physics scene (this is needed for physics simulation)
physicsScenePath = Sdf.Path("/World/physicsScene")
physicsScene = UsdPhysics.Scene.Define(stage, physicsScenePath)

# Traverse all prims in the scene
for prim in stage.Traverse():
    # Check if the prim is a mesh or other geometry type that should have a rigid body
    if prim.IsA(UsdGeom.Xform) or prim.IsA(UsdGeom.Mesh):
        # Define a PhysicsRigidBodyAPI on the prim
        UsdPhysics.RigidBodyAPI.Apply(prim)
        
        # Optionally, define mass properties if needed
        massAPI = UsdPhysics.MassAPI.Apply(prim)
        massAPI.CreateMassAttr(1.0)  # Set mass to 1 unit
        
        # You can also set other attributes like velocity or collision properties here

# Specify the new file path to save the modified USD file
new_file_path = "/home/percy/princeton/IsaacLab/Simple_Warehouse/full_warehouse_w_rigid_body.usd"

# Save the stage to the new path
stage.GetRootLayer().Export(new_file_path)

print(f"Modified USD scene saved to {new_file_path}")
