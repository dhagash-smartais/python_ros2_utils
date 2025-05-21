import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from rclpy.qos import qos_profile_sensor_data
import numpy as np


class CloudRotator(Node):
    """
    Subscribes to /camera/depth/points in camera_frame (X right, Y down, Z forward),
    rotates each point into new_base_frame (X forward, Y left, Z up), updates the header,
    and republishes on /cloud_reoriented.
    """

    def __init__(self):
        super().__init__("cloud_rotator")
        # Rotation matrix: camera_frame -> new_base_frame
        # new_x =  0*x + 0*y + 1*z
        # new_y = -1*x + 0*y + 0*z
        # new_z =  0*x - 1*y + 0*z
        self.R = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)

        self.target_frame = "new_base_frame"
        self.sub = self.create_subscription(
            PointCloud2, "/camera/depth/color/points", self.cloud_callback, qos_profile_sensor_data
        )
        self.pub = self.create_publisher(PointCloud2, "/cloud_reoriented", qos_profile_sensor_data)

    def cloud_callback(self, msg: PointCloud2):

        if not msg:
            return
        # Read points including rgb (uint32)
        data = point_cloud2.read_points(
            msg, field_names=["x", "y", "z", "rgb"], skip_nans=True
        )

        
        # Separate XYZ and RGB
        xyz = np.stack((data["x"], data["y"], data["z"]), axis=-1)
        rgb = np.array(data["rgb"], dtype=np.uint32)  # shape (N,)

        # Rotate XYZ: (N,3)
        rotated_xyz = (self.R @ xyz.T).T

        # Build structured array to preserve types
        N = rotated_xyz.shape[0]
        dt = np.dtype(
            [
                ("x", np.float32),
                ("y", np.float32),
                ("z", np.float32),
                ("rgb", np.uint32),
            ]
        )
        cloud_struct = np.empty(N, dtype=dt)
        cloud_struct["x"] = rotated_xyz[:, 0]
        cloud_struct["y"] = rotated_xyz[:, 1]
        cloud_struct["z"] = rotated_xyz[:, 2]
        cloud_struct["rgb"] = rgb

        
        # Create new header and fields for only x,y,z,rgb
        header = msg.header
        header.frame_id = self.target_frame
        new_fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)
        ]

        # Create and publish rotated cloud
        new_cloud = point_cloud2.create_cloud(
            header,
            new_fields,
            cloud_struct.tolist())

        # Convert to list of tuples
        self.pub.publish(new_cloud)


def main(args=None):
    rclpy.init(args=args)
    node = CloudRotator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
