import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. หาเส้นทางไปยังโฟลเดอร์ที่ติดตั้งแพ็กเกจ bartbot_nav
    pkg_dir = get_package_share_directory('bartbot_nav')

    # 2. ระบุเส้นทางไปยังไฟล์แผนที่ .pcd ของคุณ
    map_file_path = os.path.join(pkg_dir, 'maps', 'my_map.pcd')

    # 3. สร้าง Node สำหรับอ่านไฟล์ .pcd และปล่อยออกมาเป็น Topic
    pcd_map_node = Node(
        package='pcl_ros',
        executable='pcd_to_pointcloud',
        name='pcd_map_loader',
        parameters=[{
            'file_name': map_file_path,
            'tf_frame': 'map',    # ผูกแผนที่นี้ไว้กับจุดอ้างอิง (Frame) ที่ชื่อว่า 'map'
            'latch': True         # ให้ส่งข้อมูลแผนที่ค้างไว้ตลอดเวลา ไม่ต้องส่งซ้ำๆ 
        }],
        remappings=[
            # เปลี่ยนชื่อ Topic ที่ปล่อยออกมาให้เป็นชื่อที่เข้าใจง่าย
            ('cloud_pcd', 'map_3d_cloud') 
        ]
    )

    # 4. ส่ง Node เข้าไปใน Launch Description เพื่อให้ ROS 2 นำไปรัน
    return LaunchDescription([
        pcd_map_node
    ])
