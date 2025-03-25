from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from ament_index_python.packages import get_package_share_directory
from ur_moveit_config.launch_common import load_yaml

def generate_launch_description():
    # Resolve the paths to the necessary files
    robot_description_path = get_package_share_directory('full_husky_description') + '/urdf/full_husky.urdf.xacro'
    srdf_path = get_package_share_directory('full_husky_moveit_config') + '/config/full_husky.srdf'
    controllers_path = get_package_share_directory('full_husky_moveit_config') + '/config/moveit_controllers.yaml'
    ompl_planning_yaml_path = get_package_share_directory('full_husky_moveit_config') + '/config/ompl_planning.yaml'

    # Load the OMPL planning configuration
    ompl_planning_yaml = load_yaml("full_husky_moveit_config", "config/ompl_planning.yaml")

    # Build the MoveIt configuration using the modular builder pattern
    moveit_config = (
        MoveItConfigsBuilder("full_husky", package_name="full_husky_moveit_config")
        .robot_description(file_path=robot_description_path)
        .robot_description_semantic(file_path=srdf_path)
        .trajectory_execution(file_path=controllers_path)
        .planning_pipelines(pipelines=["ompl","pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    # Define additional OMPL planning configuration
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization "
                                "default_planner_request_adapters/FixWorkspaceBounds "
                                "default_planner_request_adapters/FixStartStateBounds "
                                "default_planner_request_adapters/FixStartStateCollision "
                                "default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
            "default_planner_id": "RRTConnectkConfigDefault",
        }
    }

    # Combine the OMPL planning YAML with additional configurations
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Set the combined configuration to the MoveItConfigs object
    moveit_config.planning_pipelines['ompl'] = ompl_planning_pipeline_config['move_group']


    return generate_move_group_launch(moveit_config)