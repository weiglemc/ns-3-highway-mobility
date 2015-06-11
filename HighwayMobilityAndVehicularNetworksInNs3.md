

Note: This software is no longer being actively maintained. The last updates were made in December 2011.

# Introduction #
Vehicular ad-hoc networks (VANETs) are networks in which each node is a vehicle. Such systems aim to provide communications between individual vehicles and between vehicles and nearby fixed equipment, or roadside units. The goal of VANETs, and more broadly vehicular networks, is to improve traffic safety by providing timely information to drivers and concerned authorities. The development of VANETs has received much attention from the automotive industry and government agencies, including the US Department of Transportation (DOT) which has launched the IntelliDrive initiative (US-DOT 2010). The US DOT reports that in 2008, 37,000 people died in traffic accidents in the US. The agency sees the promise of IntelliDrive, and VANETs in general, to be able to significantly reduce that number.

In order to provide applications that can fulfill this vision, approaches must be thoroughly evaluated. There are a limited number of testbeds with instrumented vehicles and roadside units. As this is prohibitively expensive for most academic researchers, the majority of evaluation studies have been performed via simulation. VANET simulations have typically been segregated into traffic simulations and network simulations. Traffic simulators, such as CORSIM (Halati et al. 1997), SUMO (Krajzewicz et al. 2006), VISSIM (PTV America 2010), and VanetMobiSim (Fiore et al. 2006) have been used to generate realistic mobility traces of vehicle traffic. These traces would then be fed into well-known network simulators such as ns-2 (Breslau et al. 2000), QualNet (Scalable Network Technologies 2010), OPNET (2010), or GloMoSim (Zeng et al. 1998) to measure network performance. VANET tools such as TraNS (Piorkowski et al. 2008) and MOVE (Karnadi et al. 2007) have been used to facilitate this interaction between traffic and network simulators. More recently, researchers have developed integrated simulators such as ASH (Ibrahim and Weigle 2008) and Gorgorin et al. (2006) that allow feedback between the applications using the network and the traffic model. This is important because the goal of most VANET applications is to provide drivers with information that may change their driving behavior or allow them to make more informed decisions (e.g., start braking now, or take the next exit to avoid a traffic jam). Interested readers can find detailed comparisons of various VANET simulators in Hassan (2009) and Yan et al. (2009).

The problem with integrated simulators is that often either the mobility model is overly simplified or the network model is overly simplified. In order to study important networking properties of VANETs, a high quality network simulator is essential. We have chosen to balance these two concerns by taking the latest version of the highly-regarded network simulator, ns-3 (Henderson et al. 2006), and adding a wellknown traffic mobility model in order to provide an integrated simulator for VANET research. ns-3 is a discrete-event network simulator written in C++, targeted primarily for research and educational use, and intended as a replacement for the popular ns-2 simulator. ns-3 promises to be a more efficient and more accurate simulator than its predecessor (especially for wireless protocols). In addition, during the first quarter of 2010, ns-3 averaged almost 7000 downloads per month (http://www.nsnam.org). For this reason, we were interested in using ns-3 to perform our VANET simulations. ns-3 provides various mobility models, but none are appropriate to simulate the mobility of vehicles. The mobility of a node in the mobility models included in ns-3 depends only on the node itself. In realistic vehicular mobility, the mobility of the node must depend on the surrounding nodes and the conditions on the road. Furthermore, this node dependency becomes essential when messages in the network can affect the mobility of the nodes on the roads. For example, the receipt of a safety message may result in a speed reduction. Fiore and Harri (2008) and Fiore (2009) investigated the effects of node mobility on network characteristics. They found that realistic mobility, especially at intersections, has a great impact on networking connectivity metrics and that car-following models, such as the Intelligent Driver Model (IDM) (Treiber et al. 2000), provide realistic movement. In addition, they found that multi-lane scenarios are important when considering network-level clustering.

We have implemented IDM and the MOBIL lane change model (Treiber and Helbing 2002) in ns-3. In addition, we have provided a Highway class to represent a straight multi-lane, roadway. Highway classes can be joined with other Highway classes to create intersections.  A TrafficLightGenerator class aids in creating stop-lights.  In our simulations, the Highway object is the “brain” of the system and efficiently manages the behavior of vehicles and their mobility on the road. Each vehicle is a fully-fledged wireless node in ns-3. In this way, vehicles can move with realistic mobility and communicate with each other to form a VANET. In our network and mobility combined design, a user can simulate VANETs in highways with customized road-side and on-board units. Users can create user-defined actions and event handlers to customize simulation scenarios, allowing them to study vehicular traffic, network traffic, or both.

Future work will add the ability to read in and use detailed maps, in addition to implementing and developing the WAVE/DSRC standard in ns-3. This will allow users to simulate realistic wireless communication for VANETs based on the standard, which includes multi-channel operation. We hope that this addition to ns-3 along with our future work will allow researchers to easily perform high-quality VANET simulations.

# Installation #

## Step 1 ##
  1. Get a local copy of the ns-3 from repository
```
hg clone http://code.nsnam.org/ns-3.8 my-ns-3-directory
```

## Step 2 ##
  1. Create a diretory named "vanet-highway" inside "my-ns-3-directory" and copy the source code, header files, and subfolders found in [vanet-highway.zip](http://code.google.com/p/ns-3-highway-mobility/downloads/detail?name=vanet-highway-version2.zip&can=2&q=) from [download section](http://code.google.com/p/ns-3-highway-mobility/downloads/list).


## Step 3 ##
  1. Add this line to the wscript file inside "my-ns-3-directory"
```
bld.add_subdirs('vanet-highway') 
```

# How To Run #
ns-3 experiments can be run using Python waf command.
Experiment names are usually defined in their corresponding wscripts usually exist inside their directory.

## Example 1 ##
The following command will simply run a pre-existing ns-3 application called hello-simulator. Inside my-ns-3-directory:
```
./waf --run "hello-simulator"
```

Use this command to see list of all existing experiments:
```
./waf --run all
```

## Example 2 ##
We have provided an example code and test. Our example code accepts command line arguments. The example can be run with the following command:
```
./waf --run 'vanet-highway-test --project=./vanet-highway/Examples/SimpleStraightHighway.xml --enablevehiclereceive=1'
```
This will run highway for 30 seconds.  Two files will be created in "my-ns-3-directory".  "vehicleTrace.csv" will contain information on vehicle location.  "networkTrace.csv" will contain information on messages received by vehicles.
vehicleTrace.csv has the following columns:
```
Simulation Time (Nanoseconds)
Vehicle ID (Integer)
Type ID (0 for Vehicle, 1 for Obstacle, 2 for traffic light)
X Position (double)
Y Position (double)
Direction (double)
Velocity (double)
Acceleration (double)
```

networkTrace.csv has the following columns:
```
Simulation Time (Nanoseconds)
Vehicle ID (Integer)
Message (String)
User Index (an integer based on the type of message)
```

### Command Line Arguments ###
Use this command to see list of all command line arguments defined for vanet-highway-test.
```
./waf --run "vanet-highway-test --PrintHelp"
```


# ns-3 Versions Supported #

This version of the code has been developed and tested on ns-3.9.

# Documentation #
  1. It is included in the download zip in a folder called Documentation.  It includes the original Master's paper and presentation which documents the architecture of the code.
  1. The PDF "SimpleHighwayExample" details the example code.
  1. The download also includes a much more complex XML configuration file in the "Examples" folder for a four-way intersection.

# Original Code #
The wiki page documenting how to use the original version is available at [HighwaySimulatorLegacy](http://code.google.com/p/ns-3-highway-mobility/wiki/HighwaySimulatorLegacy)

# Vehicle Viewer #
There is a utility Java project for visualizing vehicle simulations.  It is available for [download](http://code.google.com/p/ns-3-highway-mobility/downloads/detail?name=VehicleViewer.zip&can=2&q=) from [download list](http://code.google.com/p/ns-3-highway-mobility/downloads/list).  Note that the project is currently in beta and not all functions are available.  See the included README file for more information.

## Citation Request ##
If you use this in your work, please cite:
Hadi Arbabi and Michele C. Weigle, "[Highway Mobility and Vehicular Ad-Hoc Networks in ns-3](http://www.cs.odu.edu/~mweigle/papers/arbabi-wsc10.pdf)," In Proceedings of the Winter Simulation Conference. Baltimore, MD, December 2010. ([BibTeX](http://www.cs.odu.edu/~inets/Public/Publications?action=bibentry&bibfile=Public/Bibtex&bibref=arbabi-wsc10))