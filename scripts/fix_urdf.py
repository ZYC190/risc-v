import os

# 定位到你旧时代 URDF 模型的绝对路径
urdf_file = os.path.expanduser('~/robot2/src/table_streeing_arm/urdf/table_streeing_arm.urdf')

with open(urdf_file, 'r') as f:
    content = f.read()

if '<ros2_control' in content:
    print("✅ 你的 URDF 已经包含 ros2_control 标签，无需重复添加！")
else:
    # 动态生成 11 个关节的虚拟控制器 XML 标签
    mock_xml = '\n  <ros2_control name="MockSystem" type="system">\n    <hardware>\n      <plugin>mock_components/GenericSystem</plugin>\n    </hardware>\n'
    for i in range(1, 12):
        mock_xml += f'    <joint name="joint_{i}">\n      <command_interface name="position"/>\n      <state_interface name="position"/>\n      <state_interface name="velocity"/>\n    </joint>\n'
    mock_xml += '  </ros2_control>\n\n</robot>'
    
    # 强行替换到文件末尾
    new_content = content.replace('</robot>', mock_xml)
    
    with open(urdf_file, 'w') as f:
        f.write(new_content)
    print("🔥 破甲成功！虚拟电机硬件标签已强行注入 URDF 模型！")