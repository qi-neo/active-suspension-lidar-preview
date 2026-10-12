from setuptools import find_packages, setup

package_name = 'car_balancer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 原先两条入口之间缺逗号，被 Python 隐式拼接成一条，两个命令都不存在。
            # balancer_node 模块未随包提供，一并去掉，只留下实际可用的那条
            'test_imu_node = car_balancer.test_imu_node:main',
        ],
    },
)
