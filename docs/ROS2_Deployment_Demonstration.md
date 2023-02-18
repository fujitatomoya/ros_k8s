# ROS 2 Deployment Demonstration

This section provides several example deployments with ROS 2, starting with basic deployment to complicated configuration with diagrams.

## ROS Rolling Demonstration

***<span style="color: red"> Please Use WeaveNet CNI Plugin </span>***

For ROS Rolling, we can only use any CNI implementation Weave but Flannel/Cilium, since ROS 2 uses DDS underneath as RMW implementation for transport framework.
DDS uses multicast (depends on DDS implementation), but some CNI implementation cannot support multicast packets, in that case ROS 2 discovery process cannot work as expected.

If we can be sure that RMW implementation does not rely on UDP multicast, we can of course use other CNI plugins.
Most likely this case applied to use case ofr discovery server and so on, using TCP/UDP unicast packets.

### XXX

### XXX

### XXX

### XXX
