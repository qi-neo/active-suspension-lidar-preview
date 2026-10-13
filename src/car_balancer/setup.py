from setuptools import find_packages, setup

package_name = 'car_balancer'

setup(
    name=package_name,
    # 与 package.xml 的 version 保持一致
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='qi-neo',
    maintainer_email='qi-neo@users.noreply.github.com',
    description='早期姿态平衡联调包：订阅 /unilidar/imu 并周期性打印四元数、'
                '角速度与线加速度，用于验证雷达 IMU 数据通路是否符合预期',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 原先两条入口之间缺逗号，被 Python 隐式拼接成一条，两个命令都不存在。
            # balancer_node 模块未随包提供，一并去掉，只留下实际可用的那条
            'test_imu_node = car_balancer.test_imu_node:main',
        ],
    },
)
