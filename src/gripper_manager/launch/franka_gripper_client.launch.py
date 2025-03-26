from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  args=[]
  nodes=[
    Node(
      package='gripper_manager',
      executable='franka_gripper_client',
      name='franka_gripper_client',
      output='screen',
      parameters=[
        {'grasp_action_topic': '/fr3_gripper/grasp'},
        {'homing_action_topic': '/fr3_gripper/homing'},
        {'gripper_command_topic': '/gripper_client/target_gripper_width_percent'},
        {'joint_states_topic': '/fr3_gripper/joint_states'}, 
        {'gripper_epsilon_inner': 0.08}, # max gripper width for default fingers
        {'gripper_epsilon_outer': 0.08}, # max gripper width for default fingers
        {'gripper_speed': 1.0}, # max gripper speed
        {'gripper_force': 70.0} # max gripper force
      ]
    )
  ]

  return LaunchDescription(args+nodes)