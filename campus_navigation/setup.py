from setuptools import setup

package_name = 'campus_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','networkx', 'matplotlib'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'visualization_node = campus_navigation.visualization_node:main',
            'ci_agent = campus_navigation.ci_agent:main',
            'bi_agent = campus_navigation.bi_agent:main',
            'visitor_agent = campus_navigation.visitor_agent:main',
        ],
    },
)
    