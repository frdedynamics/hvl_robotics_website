The ROS-Matlab communication is way easier than many of you might think. What you need is just [MATLAB ROS Toolbox](https://www.mathworks.com/products/ros.html). Please make sure that you have the toolbox installed: Home > Add-Ons > Manage Add-Ons:

![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/ros/matlab-toolbox.png)

# Prepare Virtual Machine

## Connections
1. Select your virtual machine on the left bar
2. Click Edit virtual machine settings
3. Select Network Adapter
4. Select the first option **Bridged: Connected directly to the physical network** also check the **Replicate physical network connection state**
5. Go to Configure Adapters and ONLY select the wireless adapter which your PC has. In our case it is “Killer(R) Wi-Fi 6 AX1650 160MHz Wireless Network Adapter”
6. Save everything and start your virtual machine.

![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/vm/VM-settings.png)

![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/vm/vm_bridge_settings.png)

Next:
1. Start your virtual machine and open a new terminal: ``Ctrl + Alt + T`
2. Open the **.bashrc** using your favorite text editor: `gedit .bashrc`
3. Find the line where **ROS_DOMAIN_ID** is set: (for me: line 121: ``export ROS_DOMAIN_ID=24``)
4. Note the number somewhere. You will use this number in MATLAB.

## Start a node to communicate
At this point, you are quite free to choose what you want to control. It can be `turtlesim`, your custom robot or Open Manipulator joints. For simplicity, we will only control the `turtlesim` here but the concept is the same for all.

1. Start your node that you want to communicate: `ros2 run turtlesim turtlesim node`
2. Note the ROS topic that you want to publish/subscribe: `ros2 topic list`

![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/ros/turtlesim-topic-list.png)
