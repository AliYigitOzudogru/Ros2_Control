#!/usr/bin/env python3
import subprocess
import yaml
import sys
import re

# Mapping table (ROS -> Gazebo)
ROS_GZ_MAP = {
    "builtin_interfaces/msg/Time": "ignition.msgs.Time",
    "std_msgs/msg/Bool": "ignition.msgs.Boolean",
    "std_msgs/msg/ColorRGBA": "ignition.msgs.Color",
    "std_msgs/msg/Empty": "ignition.msgs.Empty",
    "std_msgs/msg/Float32": "ignition.msgs.Float",
    "std_msgs/msg/Float64": "ignition.msgs.Double",
    "std_msgs/msg/Header": "ignition.msgs.Header",
    "std_msgs/msg/Int32": "ignition.msgs.Int32",
    "std_msgs/msg/UInt32": "ignition.msgs.UInt32",
    "std_msgs/msg/String": "ignition.msgs.StringMsg",
    "geometry_msgs/msg/Wrench": "ignition.msgs.Wrench",
    "geometry_msgs/msg/WrenchStamped": "ignition.msgs.Wrench",
    "geometry_msgs/msg/Quaternion": "ignition.msgs.Quaternion",
    "geometry_msgs/msg/Vector3": "ignition.msgs.Vector3d",
    "geometry_msgs/msg/Point": "ignition.msgs.Vector3d",
    "geometry_msgs/msg/Pose": "ignition.msgs.Pose",
    "geometry_msgs/msg/PoseArray": "ignition.msgs.Pose_V",
    "geometry_msgs/msg/PoseWithCovariance": "ignition.msgs.PoseWithCovariance",
    "geometry_msgs/msg/PoseStamped": "ignition.msgs.Pose",
    "geometry_msgs/msg/Transform": "ignition.msgs.Pose",
    "geometry_msgs/msg/TransformStamped": "ignition.msgs.Pose",
    "geometry_msgs/msg/Twist": "ignition.msgs.Twist",
    "geometry_msgs/msg/TwistStamped": "ignition.msgs.Twist",
    "geometry_msgs/msg/TwistWithCovariance": "ignition.msgs.TwistWithCovariance",
    "geometry_msgs/msg/TwistWithCovarianceStamped": "ignition.msgs.TwistWithCovariance",
    "gps_msgs/msg/GPSFix": "ignition.msgs.NavSat",
    "nav_msgs/msg/Odometry": "ignition.msgs.Odometry",
    "ros_gz_interfaces/msg/Altimeter": "ignition.msgs.Altimeter",
    "ros_gz_interfaces/msg/Contact": "ignition.msgs.Contact",
    "ros_gz_interfaces/msg/Contacts": "ignition.msgs.Contacts",
    "ros_gz_interfaces/msg/Dataframe": "ignition.msgs.Dataframe",
    "ros_gz_interfaces/msg/Entity": "ignition.msgs.Entity",
    "ros_gz_interfaces/msg/EntityWrench": "ignition.msgs.EntityWrench",
    "ros_gz_interfaces/msg/Float32Array": "ignition.msgs.Float_V",
    "ros_gz_interfaces/msg/GuiCamera": "ignition.msgs.GUICamera",
    "ros_gz_interfaces/msg/JointWrench": "ignition.msgs.JointWrench",
    "ros_gz_interfaces/msg/Light": "ignition.msgs.Light",
    "ros_gz_interfaces/msg/SensorNoise": "ignition.msgs.SensorNoise",
    "ros_gz_interfaces/msg/StringVec": "ignition.msgs.StringMsg_V",
    "ros_gz_interfaces/msg/TrackVisual": "ignition.msgs.TrackVisual",
    "ros_gz_interfaces/msg/VideoRecord": "ignition.msgs.VideoRecord",
    "ros_gz_interfaces/msg/WorldControl": "ignition.msgs.WorldControl",
    "rosgraph_msgs/msg/Clock": "ignition.msgs.Clock",
    "sensor_msgs/msg/BatteryState": "ignition.msgs.BatteryState",
    "sensor_msgs/msg/CameraInfo": "ignition.msgs.CameraInfo",
    "sensor_msgs/msg/FluidPressure": "ignition.msgs.FluidPressure",
    "sensor_msgs/msg/Imu": "ignition.msgs.IMU",
    "sensor_msgs/msg/Image": "ignition.msgs.Image",
    "sensor_msgs/msg/JointState": "ignition.msgs.Model",
    "sensor_msgs/msg/Joy": "ignition.msgs.Joy",
    "sensor_msgs/msg/LaserScan": "ignition.msgs.LaserScan",
    "sensor_msgs/msg/MagneticField": "ignition.msgs.Magnetometer",
    "sensor_msgs/msg/NavSatFix": "ignition.msgs.NavSat",
    "sensor_msgs/msg/PointCloud2": "ignition.msgs.PointCloudPacked",
    "sensor_msgs/msg/Range": "ignition.msgs.LaserScan",
    "tf2_msgs/msg/TFMessage": "ignition.msgs.Pose_V",
    "trajectory_msgs/msg/JointTrajectory": "ignition.msgs.JointTrajectory",
    "vision_msgs/msg/Detection3D": "ignition.msgs.AnnotatedOriented3DBox",
    "vision_msgs/msg/Detection3DArray": "ignition.msgs.AnnotatedOriented3DBox_V",
    # Added manually
    "geometry_msgs/msg/PointStamped": "ignition.msgs.Vector3d",
    "geometry_msgs/msg/PoseWithCovarianceStamped": "ignition.msgs.PoseWithCovariance"
}

GZ_ROS_MAP = {v: k for k, v in ROS_GZ_MAP.items()}

def run_cmd(cmd):
    result = subprocess.run(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if result.returncode != 0:
        print(f"Error running {cmd}: {result.stderr}")
        sys.exit(1)
    return result.stdout.strip()

def get_ros_topics():
    out = run_cmd("ros2 topic list -t")
    topics = []
    for line in out.splitlines():
        if not line.strip():
            continue
        topic, typ = line.split()
        typ = typ[1:]
        typ = typ[:-1]
        topics.append((topic, typ))
    return topics

def get_ign_topics():
    out = run_cmd("ign topic -l")
    topics = []
    for line in out.splitlines():
        if not line.strip():
            continue
        topics.append(line.strip())
    return topics

def get_ign_type(topic):
    out = run_cmd(f"ign topic -i -t {topic}")
    match = re.search(r",\s+(\S+)$", out)
    if match:
        return match.group(1)
    else:
        return None

def main():
    ros_topics = get_ros_topics()
    ign_topics = get_ign_topics()

    bridges = []
    for ros_topic, ros_type in ros_topics:
        if ros_type not in ROS_GZ_MAP:
            print(f"ERROR: No mapping found for ROS type {ros_type} (ROS topic: {ros_topic})")
            answer = input("Break or ignore? [B/I] ")[0].upper()
            if answer != "I":
                sys.exit(1)
            continue

        gz_type = ROS_GZ_MAP[ros_type]

        # try to match ROS topic name to ign topic name (heuristic: same name or suffix)
        matched_ign = None
        for ign_topic in ign_topics:
            if ign_topic.endswith(ros_topic) or ros_topic.endswith(ign_topic):
                matched_ign = ign_topic
                break


        if matched_ign:
            ign_type = get_ign_type(matched_ign)
            if ign_type != gz_type:
                print(f"ERROR: Type mismatch for {ros_topic} <-> {matched_ign}")
                print(f"  Expected {gz_type}, but Ignition has {ign_type}")
                sys.exit(1)

        if not matched_ign:
            print(f"WARNING: No matching Ignition topic found for {ros_topic} (ROS type: {ros_type}). Adding anyway...")
            # answer = input("Add anyway, continue, or break? [A/C/B] ").upper()
            answer = "A"
            if answer == "A":
                pass
            elif answer == "C":
                continue
            else:
                sys.exit(1)

            matched_ign = ros_topic

        bridges.append({
            "ros_topic_name": ros_topic,
            "gz_topic_name": matched_ign,
            "ros_type_name": ros_type,
            "gz_type_name": gz_type,
        })
    
    # Add the Ignition topics
    for ign_topic in ign_topics:
        gz_type = get_ign_type(ign_topic)
        if gz_type not in GZ_ROS_MAP:
            print(f"ERROR: No mapping found for Ignition type {gz_type} (Ignition topic: {ign_topic})")
            answer = input("Break or ignore? B/[I] ").upper()
            if answer != "I" and len(answer) != 0:
                sys.exit(1)
            continue

        ros_type = GZ_ROS_MAP[gz_type]

        # if there is a corresponding ROS topic, it's already been processed
        hasMatch = None
        for ros_topic, _ in ros_topics:
            if ign_topic.endswith(ros_topic) or ros_topic.endswith(ign_topic):
                hasMatch = True
                break
        if hasMatch:
            continue

        bridges.append({
            "ros_topic_name": ign_topic,
            "gz_topic_name": ign_topic,
            "ros_type_name": ros_type,
            "gz_type_name": gz_type,
            "direction": "GZ_TO_ROS"
        })

    with open("bridge.yaml", "w") as f:
        yaml.dump(bridges, f, default_flow_style=False)

    print(f"✅ bridge.yaml generated successfully with {len(bridges)} bridges.")

if __name__ == "__main__":
    main()
