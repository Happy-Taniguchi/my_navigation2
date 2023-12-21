from setuptools import find_packages, setup

package_name = 'my_navigation2'

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
    maintainer='ubuntu',
    maintainer_email='c1213803@st.kanazawa-it.ac.jp',
    description='navigation',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navi_coord = script.navi_coord:main',
            'navi_location = script.navi_location:main',
            'set_location = script.set_location:main'
        ],
    },
)
