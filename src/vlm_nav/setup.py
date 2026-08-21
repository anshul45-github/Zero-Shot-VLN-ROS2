from setuptools import find_packages, setup

package_name = 'vlm_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anish',
    maintainer_email='garg110118@gmail.com',
    description='VLM semantic mapping + navigation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'vlm_detection = vlm_nav.vlm_detection:main',
            'semantic_nav = vlm_nav.semantic_nav:main',
        ],
    },
)
