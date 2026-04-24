import time
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.duration import Duration

from pymoveit2 import MoveIt2
from tf2_ros import Buffer, TransformListener

from moveit_msgs.msg import CollisionObject, PlanningScene, AttachedCollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

DEFAULT_CUBE = {
    "cube_id": "cube",
    "size": 0.10,
    "cube_pos": [0.35, 0.0, 0.05]
}

MAX_ALLOWED_PLANNING_TIME = 30.0

class SimplePickPlaceNode(Node):
    def __init__(self):
        super().__init__("simple_pick_place_node")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.apply_planning_scene_client = self.create_client(
            ApplyPlanningScene,
            "/apply_planning_scene",
        )

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=[
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
            base_link_name="base_link",
            end_effector_name="tool0",
            group_name="ur_manipulator",
        )

        # specify the planner to use
        self.moveit2.pipeline_id = "move_group"
        self.moveit2.planner_id = "RRTConnectkConfigDefault"

        # specify planning time allowed
        self.moveit2.allowed_planning_time = MAX_ALLOWED_PLANNING_TIME

        # scale motion
        self.moveit2.max_velocity = 0.2
        self.moveit2.max_acceleration = 0.2

        # self.moveit2.pipeline_id = "pilz_industrial_motion_planner"
        # self.moveit2.planner_id = "PTP" 
        
        self.planning_scene_pub = self.create_publisher(
            PlanningScene,
            "/planning_scene",
            10
        )

        while not self.apply_planning_scene_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /apply_planning_scene service....")

    def add_floor(self):
        """
        Add the floor as a collision object to our planning scene.
        """
        floor = CollisionObject()
        floor.header.frame_id = "base_link" # be sure to express in base_link frame
        floor.id = "floor"

        # remove if present
        self.moveit2.remove_collision_object(floor.id)
        time.sleep(0.5)

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [2.0, 2.0, 0.02]

        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = -0.02
        pose.orientation.w = 1.0

        floor.primitives.append(primitive)
        floor.primitive_poses.append(pose)
        floor.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(floor)

        self.planning_scene_pub.publish(scene)
        self.get_logger().info("Added floor collision object to MoveIt planning scene.")

        time.sleep(1.0)

    def add_wall(self,
                 wall_id="wall",
                 frame_id="base_link",
                 origin = [0.8, 0.0, 0.5],
                 height=1.0,
                 length=1.0,
                 thickness=0.20):
        """
        Add a wall to our planning scene. 
        x: length/2 + 0.3 at a minimum to allow room for the robot to reach the home position.
        y: depends on what areas you want the wall to divide.
        z: height/2 - 0.01 if you want the wall touching the floor, or height/2 if you're okay
        # with a small gap.
        """
        [x, y, z] = origin
        obj = CollisionObject()
        obj.header.frame_id = frame_id
        obj.id = wall_id

        self.moveit2.remove_collision_object(wall_id)

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [length, thickness, height]

        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.w = 1.0

        obj.primitives.append(primitive)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(obj)

        self.planning_scene_pub.publish(scene)
        self.get_logger().info("Added wall collision object to MoveIt planning scene.")

        time.sleep(1.0)



    def add_cube_to_planning_scene(
            self,
            cube_id="cube",
            frame_id="base_link",
            size=0.20,
            position=(0.45, 0.45, 0.10)):
        """
        Add cube with id = cube_id at position to our planning scene.
        """
        obj = CollisionObject()
        obj.header.frame_id = frame_id
        obj.id = cube_id

        # remove if already exists in scene
        self.moveit2.remove_collision_object(cube_id)
        time.sleep(0.5)
        
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [size, size, size]

        pose = Pose()
        pose.position.x = float(position[0])
        pose.position.y = float(position[1])
        pose.position.z = float(position[2])
        pose.orientation.w = 1.0

        obj.primitives.append(primitive)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.is_diff = True # this is important if we plan on adding multiple cubes to the same scene
        scene.world.collision_objects.append(obj)

        self.planning_scene_pub.publish(scene)
        self.get_logger().info(
            f"Added cube '{cube_id}' to planning scene at {position} in frame '{frame_id}'."
        )

        # allow just a bit of time to make sure scene is updated before running any other commands
        time.sleep(1.0)
    
    def remove_cube_from_planning_scene(
            self,
            cube_id="cube",
            frame_id="base_link"):
        """
        Remove cube with id = cube_id from the planning scene.
        """
        obj = CollisionObject()
        obj.header.frame_id = frame_id
        obj.id = cube_id
        obj.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(obj)

        self.planning_scene_pub.publish(scene)
        self.get_logger().info(f"Removed cube '{cube_id}' from planning scene.")

        time.sleep(1.0)

    def apply_scene_diff(self, scene: PlanningScene):
        request = ApplyPlanningScene.Request()
        request.scene = scene

        future = self.apply_planning_scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response is None:
            self.get_logger().error("No response from /apply_planning_scene")
            return False
        
        return response.success

    def attach_cube_to_tool(
            self,
            cube_id="cube",
            tool_link="tool0",
            size=0.20,
            safety_buffer = 0.01):
        """
        Attach the cube to our tool link for geometrically accurate planning.
        """
        # remove cube from world collision objects so we can add it to 
        remove_obj = CollisionObject()
        remove_obj.header.frame_id = "base_link"
        remove_obj.id = cube_id
        remove_obj.operation = CollisionObject.REMOVE

        # create attached collision object
        obj = AttachedCollisionObject()
        obj.link_name = tool_link
        obj.touch_links = [
            tool_link,
            "wrist_3_link",
        ]

        obj.object.header.frame_id = tool_link
        obj.object.id = cube_id

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [size, size, size]

        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = size / 2.0 + safety_buffer
        pose.orientation.w = 1.0

        obj.object.primitives.append(primitive)
        obj.object.primitive_poses.append(pose)
        obj.object.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.is_diff = True
        
        # we could call remove_cube_from_planning_scene() but the way its currently implemented
        # would lead to more time between when the cube is removed from scene.world and added
        # to scene.robot_state
        scene.world.collision_objects.append(remove_obj)
        scene.robot_state.attached_collision_objects.append(obj)

        # self.planning_scene_pub.publish(scene)
        ok = self.apply_scene_diff(scene)
        self.get_logger().info(f"Attached cube '{cube_id}' to '{tool_link}': {ok}.")

        time.sleep(1.0)
    
    def detach_cube_from_tool(
            self,
            position,
            cube_id="cube",
            frame_id="base_link",
            tool_link="tool0",
            size=0.20):
        """
        Remove the cube from the tool link for geometrically accurate planning.
        """
        # remove attached object from tool
        detach_obj = AttachedCollisionObject()
        detach_obj.link_name = tool_link
        detach_obj.object.id = cube_id
        detach_obj.object.operation = CollisionObject.REMOVE

        # add cube back to world
        obj = CollisionObject()
        obj.header.frame_id = frame_id
        obj.id = cube_id

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [size, size, size]

        pose = Pose()
        pose.position.x = float(position[0])
        pose.position.y = float(position[1])
        pose.position.z = float(position[2])
        pose.orientation.w = 1.0

        obj.primitives.append(primitive)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.is_diff = True # this is important if we plan on adding multiple cubes to the same scene
        scene.robot_state.is_diff = True

        scene.robot_state.attached_collision_objects.append(detach_obj)
        scene.world.collision_objects.append(obj)

        # self.planning_scene_pub.publish(scene)
        ok = self.apply_scene_diff(scene)
        self.get_logger().info(
            f"Added cube '{cube_id}' to planning scene at {position} in frame '{frame_id}': {ok}."
        )

    def get_tool0_pose_in_base(self):
        """
        Helper function used for getting the tool pose at any given time.
        """
        try:
            ok = self.tf_buffer.can_transform(
                "base_link",
                "tool0",
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            )
            if not ok:
                self.get_logger().warn("Transform base_link -> tool0 not yet available.")
                return None
            
            t = self.tf_buffer.lookup_transform(
                "base_link",
                "tool0",
                rclpy.time.Time(),
            )
            p = t.transform.translation
            q = t.transform.rotation
            return (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)
        
        except Exception as e:
            self.get_logger().warn(f"Failed to get transform base_link -> tool0: {e}")
            return None

    def move_pose(self, pos, quat, cartesian, target_link="tool0", frame_id="base_link"):
        """
        Move the robot arm to position = pos, with orientation = quat. Specify cartesian=True
        if you want cartesian motion.
        """
        tol_pos = 0.005
        tol_orient = 0.02
        # if planning is still too difficult relax the tolerances
        # tol_pos = 0.01
        # tol_orient = 0.05

        if cartesian:
            self.get_logger().info(f"Moving to {pos}, cartesian=True")
            self.moveit2.move_to_pose(
                position=pos, 
                quat_xyzw=quat, 
                target_link=target_link,
                frame_id=frame_id,
                tolerance_position=tol_pos,
                tolerance_orientation=tol_orient,
                cartesian=True)
            self.moveit2.wait_until_executed()
        else:
            self.get_logger().info(f"Moving to {pos}, cartesian=False")
            self.moveit2.move_to_pose(
                position=pos, 
                quat_xyzw=quat, 
                target_link=target_link,
                frame_id=frame_id,
                tolerance_position=tol_pos,
                tolerance_orientation=tol_orient,
                cartesian=False)
            self.moveit2.wait_until_executed()
        
        # time.sleep(5.0)

    def activate_gripper(self):
        """
        Simulate picking up objects.
        """
        # for not just sleep, eventually this will be the function that controls the gripper.
        time.sleep(2.0)
    
    def deactivate_gripper(self):
        """
        Simulate dropping objects.
        """
        time.sleep(2.0)

    def go_home(self):
        """"
        For the home position use joint states to easily position the end effector
            facing down.
        """
        self.get_logger().info(f"Moving to home position")
        home_joints = [0.0, -135/180*np.pi, 110/180*np.pi, -65/180*np.pi, -np.pi/2, 0.0]
        self.moveit2.move_to_configuration(home_joints)
        self.moveit2.wait_until_executed()

    def pick(self, cube_id, cube_position, cube_size, safety_buffer, down_quat):
        """
        Pick up object with id=cube_id at position=cube_position.
        """
        # assume pose of cube is always orthogonal to ground plane and
        # assume that whenever we begin a pick action, our current robot pose is at home
        
        # cub_position, is the coordinate of the center of the cube, if we want the coordinate
        # of the top of the cube, we will need to add cube_size/2 to the z-coordinate.
        # Also, we will add an additional buffer/safety zone so that the robot will move
        # to within buffer distance above the cube, and we will consider this close enough
        # when determining whether or not the robot can successfully pick up the box.
        self.get_logger().info(f"Picking up object at: {cube_position}")
        # set the pre grasp position to be 5cm above the 'top' of the cube
        pre_grasp_z = 0.05
        x, y, z = cube_position

        # 1) Home -> Pre-grasp
        self.get_logger().info(f"Moving to pre-grasp position.")
        pre_z = float(z + cube_size/2 + safety_buffer + pre_grasp_z)
        self.move_pose([x, y, pre_z], down_quat, cartesian=False)

        # 2) Pre-grasp -> Grasp
        self.get_logger().info(f"Moving to grasp position.")
        grasp_z = float(z + cube_size/2 + safety_buffer)
        pose = self.get_tool0_pose_in_base()
        if pose is None:
            self.get_logger().warn("Grasp descend failed. TF is unavailable.")
            return
        current_pos, current_quat = pose
        grasp_pos = [current_pos[0], current_pos[1], grasp_z]
        self.move_pose(pos=grasp_pos, quat=current_quat, cartesian=True,)

        # 3) Activate Gripper
        self.activate_gripper()
        # in our sim, we will also need to attach the cube to the robot now
        self.attach_cube_to_tool(cube_id=cube_id, size=cube_size, safety_buffer=safety_buffer)

        # 4) Grasp -> Post-grasp
        self.get_logger().info(f"Moving to post-grasp position.")
        post_z = float(z + cube_size/2 + safety_buffer + pre_grasp_z)
        # self.move_pose(pos=[x, y, post_z], cartesian=True)
        pose = self.get_tool0_pose_in_base()
        if pose is None:
            self.get_logger().warn("Grasp ascend failed. TF is unavailable.")
            return
        current_pos, current_quat = pose
        grasp_pos = [current_pos[0], current_pos[1], post_z]
        self.move_pose(pos=grasp_pos, quat=current_quat, cartesian=True)
        
        # 5) Post-grasp -> Home
        # maybe comment this out depending on how well it can work, maybe go straight to
        # place position.
        # self.go_home()

    def place(self, cube_id, goal_pos, size, safety_buffer, quat):
        """
        Place object with id=cube_id at goal position=goal_pos
        """
        self.get_logger().info(f"Placing object at: {goal_pos}")
        # set the pre grasp position to be 5cm above the 'top' of the cube
        pre_grasp_z = 0.05
        x, y, z = goal_pos

        # 1) Home -> Pre-place
        self.get_logger().info(f"Moving to pre-place position.")
        pre_z = float(z + size/2 + safety_buffer + pre_grasp_z)
        self.move_pose([x, y, pre_z], quat, cartesian=False)

        # 2) Pre-place -> Place
        self.get_logger().info(f"Moving to place position.")
        grasp_z = float(z + size/2 + safety_buffer)
        pose = self.get_tool0_pose_in_base()
        if pose is None:
            self.get_logger().warn("Place descend failed. TF is unavailable.")
            return
        current_pos, current_quat = pose
        grasp_pos = [current_pos[0], current_pos[1], grasp_z]
        self.move_pose(pos=grasp_pos, quat=current_quat, cartesian=True,)

        # 3) De-activate Gripper
        self.deactivate_gripper()
        # in our sim, detach cube from robot, and add it back to the scene as a collision object
        self.detach_cube_from_tool(position=goal_pos, cube_id=cube_id, size=size)

        # 4) Place -> Post-place
        self.get_logger().info(f"Moving to post-grasp position.")
        post_z = float(z + size/2 + safety_buffer + pre_grasp_z)
        # self.move_pose(pos=[x, y, post_z], cartesian=True)
        pose = self.get_tool0_pose_in_base()
        if pose is None:
            self.get_logger().warn("Grasp ascend failed. TF is unavailable.")
            return
        current_pos, current_quat = pose
        grasp_pos = [current_pos[0], current_pos[1], post_z]
        self.move_pose(pos=grasp_pos, quat=current_quat, cartesian=True)
        
        # 5) Post-grasp -> Home
        # maybe comment this out depending on how well it can work, maybe go straight to
        # place position.
        # self.go_home()

    def reset_world(self):
        """ 
        Reset collision objects in our planning scene.
        """
        # reset world
        future = self.moveit2.clear_all_collision_objects()
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().success:
            self.get_logger().info("Cleared all collision objects from planning scene.")
        else:
            self.get_logger().error("Failed to clear planning scene.")

    def pick_and_place(self):
        """
        Pick-and-place demo function.
        """
        self.reset_world()

        ########################## CUBE PARAMETERS ##########################
        grasp_quat = [0.0, 0.0, 0.0, 1.0]   # example only

        pick = [0.45, 0.10, 0.12]
        place = [0.35, -0.20, 0.12]
        lift_z = 0.25

        pick_pre = [pick[0], pick[1], lift_z]
        place_pre = [place[0], place[1], lift_z]

        # 0. initialize the Planning Scene
        self.add_floor()

        # cube_id0 = DEFAULT_CUBE["cube_id"]
        # size0 = DEFAULT_CUBE["size"]
        # pos0 = DEFAULT_CUBE["cube_pos"]
        cube_id0 = "cube0"
        size0 = 0.10
        pos0 = [0.35, 0.35, 0.05]
        self.add_cube_to_planning_scene(cube_id=cube_id0, size=size0, position=pos0)

        # cube_id1 = "cube1"
        # size1 = 0.10
        # pos1 = [0.35, -0.35, 0.05]
        # self.remove_cube_from_planning_scene(cube_id=cube_id0)
        
        # 1. Move robot to home position from initial load position (where all joints angles = 0)
        self.go_home()

        # 2. set a path orientation constraint for moveit so that the orientation of the
        # end stays within a tolerance throughout the entire path, not just at the goal position
        down_quat = (0.707, -0.707, 0.00, 0.00)

        # TODO for now we will not force a path constraint to keep the orientation of the
        # end effector pointing down.
        # self.moveit2.clear_path_constraints()
        # self.moveit2.set_path_orientation_constraint(
        #     quat_xyzw=down_quat,
        #     frame_id="base_link",
        #     target_link="tool0",
        #     tolerance=(3.14, 0.5, 0.5),
        #     parameterization=1,
        # )
        # self.move_pose(pos=(0.50, 0.50, 0.40), 
        #                quat=down_quat, 
        #                target_link="tool0", 
        #                frame_id="base_link",
        #                cartesian=False)
        # self.moveit2.clear_path_constraints()
        
        # 3. call pick()
        # TODO change this if actually using gripper.
        safety_buffer = 0.01 # cannot be 0 or else will be considered a collision with box!
        self.pick(cube_id0, pos0, size0, safety_buffer, down_quat)

        goal_pos0 = [0.35, -0.35, 0.05]
        self.place(cube_id0, goal_pos0, size0, safety_buffer, down_quat)

        self.go_home()

    def pick_and_place_wall(self):
        """
        Pick-and-place demo with wall dividing shortest path from cube start state to goal state.
        """
        ########################## CUBE PARAMETERS ##########################
        # reset world
        self.reset_world()

        # 0. initialize the Planning Scene
        self.add_floor()

        wall_id0 = "wall0"
        self.add_wall(wall_id=wall_id0, origin=[0.8, 0.0, 0.5], height = 1.0, length=1.0,
                      thickness=0.05)
        
        # cube_id0 = DEFAULT_CUBE["cube_id"]
        # size0 = DEFAULT_CUBE["size"]
        # pos0 = DEFAULT_CUBE["cube_pos"]
        cube_id0 = "cube0"
        size0 = 0.10
        pos0 = [0.35, 0.35, 0.05]
        self.add_cube_to_planning_scene(cube_id=cube_id0, size=size0, position=pos0)

        
        
        # 1. Move robot to home position from initial load position (where all joints angles = 0)
        self.go_home()

        # 2. set a path orientation constraint for moveit so that the orientation of the
        # end stays within a tolerance throughout the entire path, not just at the goal position
        down_quat = (0.707, -0.707, 0.00, 0.00)

        # TODO for now we will not force a path constraint to keep the orientation of the
        # end effector pointing down.
        # self.moveit2.clear_path_constraints()
        # self.moveit2.set_path_orientation_constraint(
        #     quat_xyzw=down_quat,
        #     frame_id="base_link",
        #     target_link="tool0",
        #     tolerance=(3.14, 0.5, 0.5),
        #     parameterization=1,
        # )
        # self.move_pose(pos=(0.50, 0.50, 0.40), 
        #                quat=down_quat, 
        #                target_link="tool0", 
        #                frame_id="base_link",
        #                cartesian=False)
        # self.moveit2.clear_path_constraints()
        
        # 3. call pick()
        # TODO change this if actually using gripper.
        safety_buffer = 0.01 # cannot be 0 or else will be considered a collision with box!
        self.pick(cube_id0, pos0, size0, safety_buffer, down_quat)

        goal_pos0 = [0.35, -0.35, 0.05]
        self.place(cube_id0, goal_pos0, size0, safety_buffer, down_quat)

        self.go_home()
    
    def stack_single_box(self):
        """
        Box stacking demo.
        """
        # reset world
        self.reset_world()
        ########################## CUBE PARAMETERS ##########################
        # 0. initialize the Planning Scene
        self.add_floor()

        cube_id0 = "cube0"
        size0 = 0.10
        pos0 = [0.35, 0.35, 0.05]
        self.add_cube_to_planning_scene(cube_id=cube_id0, size=size0, position=pos0)

        cube_id1 = "cube1"
        size1 = 0.10
        pos1 = [0.35, -0.35, 0.05]
        self.add_cube_to_planning_scene(cube_id=cube_id1, size=size1, position=pos1)
        
        # 1. Move robot to home position from initial load position (where all joints angles = 0)
        self.go_home()

        # 2. set a path orientation constraint for moveit so that the orientation of the
        # end stays within a tolerance throughout the entire path, not just at the goal position
        down_quat = (0.707, -0.707, 0.00, 0.00)
        
        # 3. call pick()
        # TODO change this if actually using gripper.
        safety_buffer = 0.01 # cannot be 0 or else will be considered a collision with box!
        self.pick(cube_id0, pos0, size0, safety_buffer, down_quat)

        goal_pos0 = [0.35, -0.35, 0.15]
        self.place(cube_id0, goal_pos0, size0, safety_buffer, down_quat)

        self.go_home()
        


def main():
    rclpy.init()
    node = SimplePickPlaceNode()
    time.sleep(1.0)
    try:
        # DEMOS 
        # uncomment the demo you wish to run:
        node.pick_and_place()
        # node.pick_and_place_wall()
        # node.stack_single_box()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == "__main__":
    main()