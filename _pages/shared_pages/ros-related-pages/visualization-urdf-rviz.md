## Visualization
Rviz, URDF
- What is an XML file
- What is a URDF
- What is xacro

Get simple things from old notes

Later:

```python
<link name='base_link'/>

  <joint name='base_to_chassis' type='fixed'>
    <parent link="base_link"/>
    <child link="chassis"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
```