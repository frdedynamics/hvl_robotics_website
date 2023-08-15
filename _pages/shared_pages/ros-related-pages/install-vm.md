In this course, we are using [Ubuntu 20.04 LTS](https://releases.ubuntu.com/focal/) and [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html) as development tools. We will talk about what these *tools* mean later but for now you just need to know that these are not regular programs which you simply write the name in Microsoft Store (or App Store) and get. Therefore, we are using virtual machines.

Virtual machines are basically some software which run just like a
regular PC but without any physical components. They use the *host*\'s
hardware equipments as a *guest*. Since ROS works best on Linux based
operating systems but we assume most of you have Windows PCs, we provide
this ready-to-use solution for you to start ROS as smooth as possible.

There will be two components:

1. The virtual appliance (the file you open in a virtual machine player)
2. A software to run the virtual appliance (VMware)

The admin password inside the virtual appliance is: **ros**

## Virtual Appliance

A virtual appliance is a copy of a working operating system and its
programs. We provide you ready-to-use virtual copy of what you need. To
download it, use [this
link](https://frdedynamics.github.io/hvl_robotics_website/custom_404)
with your HVL credentials. After downloading, extract the folder inside
the .zip file.

## Virtual Appliance Player

A virtual appliance player is a piece of software on which you can run your
virtual appliance (aka. a virtual copy of a system). We suggest you download VMware for this purpose.

### VMware Install

You can download VMware here: [Windows/Linux
download](https://www.vmware.com/products/workstation-player/workstation-player-evaluation.html),
[Mac
download](https://www.vmware.com/products/fusion/fusion-evaluation.html).
After installing it you can import the virtual appliance by clicking
**Open a Virtual Machine** and choose the .vmx file from inside the
previously downloaded and extracted folder.

After you finished importing the virual appliance, go to **Edit virtual
machine settings** and in Display settings enable **Accelerate 3D
graphics** and choose recommended Graphics Memory from the dropdown box
as shown in the pictures.

![image-center]({{ site.url }}{{ site.baseurl}}/assets/images/shared/vm/VM-settings.png)

Also make sure, that in the **Network Adapter** settings, under
**Network connection**, **NAT** is selected, as shown in the pictures
below.

![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/vm/vm_network_settings.PNG)

When first opening the virtual machine, the following window will
pop-up. Select **I Copied It** to continue.

![image-center]({{ site.url }}{{ site.baseurl }}/assets/images/shared/vm/vm_installation_popup.PNG)

## Troubleshooting

{: .notice--info}

For those who don't use the given virtual copy and choose to install
the necessary software and packages by themselves use the following
links: [Ubuntu 20.04 LTS](https://releases.ubuntu.com/20.04/), [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html), [Necessary
Turtlebot
packages](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/),
[MATLAB
2020a](https://se.mathworks.com/products/new_products/release2020a.html).
