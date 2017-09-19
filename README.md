# ros_industrial_ssm
Repository for development of state machines for ROS-I components

##### Package structure #####
```
pkg_name 
|-- resources    
|   |-- scxml  
|   |   └-- *.scxml
|   └-- skills.xml
|-- src
|   └-- pkg_name
|       |-- __init__.py
|       └-- node.py
|-- CMakeLists.txt
|-- package.xml
└-- setup.py
```

##### setup.py #####
Modify the below contents (python pkg):

```
d = generate_distutils_setup(
	packages=['pkg_name'],
	package_dir={'': 'src'},
)
```
The global datamodel should have a data field with:
```
id*  : skill_file
expr : ${pkg_name}/resources/skills.xml
```

##### Usage #####
Make sure the working workspace overlays airbus_coop workspace

```
$ roscore
$ rosrun airbus_ssm_plugin ssm_interface_node.py
```
Select the *.scxml from the resources folder
