# virginMediaCec

## CEC Control of Virgin Media through Pioneer/Onkyo AV receiver

This works with my Samsung UE50KU series TV and Pioneer VSX 1131 reciever. This might work with other Receivers and TVs, I have no idea.

For use with a Raspberry PI connected to the AV receiver. When the input is switched to the Virgin Media input, it triggers some input changes that cause CEC requests to be routed to the Pi. It then sends the CEC request messages over the network to the Virgin Media box.

Requires the Pulse-Eight CEC library for the Pi (and it's based on the sample client code).

You'll need to change some hard-coded IP addresses and input numbers stuff to make it work.

## Building

```
cmake CMakeLists.txt

make
```





