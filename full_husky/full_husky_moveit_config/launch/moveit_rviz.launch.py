from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch
from ament_index_python.packages import get_package_share_directory
from ur_moveit_config.launch_common import load_yaml

def generate_launch_description():
    # Load MoveIt configuration
    moveit_config = MoveItConfigsBuilder("full_husky", package_name="full_husky_moveit_config").to_moveit_configs()
    
    # Load the OMPL planning configuration from YAML
    ompl_planning_yaml = load_yaml("full_husky_moveit_config", "config/ompl_planning.yaml")
    
    # Set the OMPL planner config to the MoveItConfig
    moveit_config.planning_pipelines['ompl'] = ompl_planning_yaml

    return generate_moveit_rviz_launch(moveit_config)