# ReconROS_Vitis

This repository provides an example of the Vitis (Vision) library used in a ReconROS application. 


# Build

First, you have to adapt the path to the Vitis library (e.g., version in this repository) in the build.cfg file. After that, you can build the bitstream and the software binary.

### Bitstream 

```
faketime '2021-11-11 11:11:11' rdk export_hw && rdk build_hw
```

### Software binary 
```
rdk export_sw && rdk build_sw
```


