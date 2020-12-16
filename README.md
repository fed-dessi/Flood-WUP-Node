# Flood-WUP-Node

An implementation of the FLOOD-WUP protocol using FreeRTOS and a Silicon labs Mighty Gecko board. This particular code is for a node of a WSN.

The node will wait in Low Power Mode to receive a signal using RFSENSE with which we will simulate receiving a Wake Up Radio packet on a sub Ghz band (868mhz). Once we've woken up we will receive the data packet that has a header containing a packet Sequence# and WUP Sequence, in this case it's a simple enum (Wa or Wb).
The node will process the data packet, add it to its retransmission buffer, switch it's own WUP Sequence from Wa to Wb (or from Wb to Wa), and then retransmit the packet to the rest of the WSN.

If the node sees that it lost a packet (the Seq# of the last received packet is too high) then it will ask all sorrounding nodes for the lost packet with a special sequence (Wr).
