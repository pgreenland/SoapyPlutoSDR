# Building

```
export SRSRAN_INSTALL=${HOME}/srsRAN
mkdir build
cd build
cmake .. -DCMAKE_PREFIX_PATH=${SRSRAN_INSTALL}/share/cmake
make
LD_LIBRARY_PATH=${SRSRAN_INSTALL}/lib ./test_timestamp_loopback
```
