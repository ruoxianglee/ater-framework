import subprocess

def publish_modified_cycle_time(x):
    command = f'ros2 topic pub /modified_cycle_time std_msgs/msg/Float32 "data: {x}" -1'
    
    # Execute the command in the shell
    try:
        subprocess.run(command, shell=True, check=True)
        # print(f"Successfully published modified cycle time: {x}")
    except subprocess.CalledProcessError as e:
        print(f"Error occurred while publishing modified cycle time: {e}")