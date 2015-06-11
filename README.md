# ns-3-highway-mobility
*Automatically exported from code.google.com/p/ns-3-highway-mobility*

The study of vehicular ad-hoc networks (VANETs) requires efficient and accurate simulation tools. As the mobility of vehicles 
and driver behavior can be affected by network messages, these tools must include a vehicle mobility model integrated with a 
quality network simulator. We present the first implementation of a well-known vehicle mobility model to ns-3, the next 
generation of the popular ns-2 networking simulator. Vehicle mobility and network communication are integrated through events. 
User-created event handlers can send network messages or alter vehicle mobility each time a network message is received and 
each time vehicle mobility is updated by the model. To aid in creating simulations, we have implemented a straight highway 
model that manages vehicle mobility, while allowing for various user customizations. We show that the results of our 
implementation of the mobility model matches that of the model’s author and provide an example of using our implementation in 
ns-3.

Note: The source code is delivered 'as-is', and we cannot offer any support. Code last updated December 2011.

Citation Request: If you use this in your work, please cite 

> Hadi Arbabi and Michele C. Weigle, "Highway Mobility and Vehicular Ad-Hoc Networks in ns-3," In *Proceedings of the Winter 
> Simulation Conference*. Baltimore, MD, December 2010, pp. 2991-3003.
 * http://dx.doi.org/10.1109/WSC.2010.5678993
 * http://www.cs.odu.edu/~mweigle/papers/arbabi-wsc10.pdf
