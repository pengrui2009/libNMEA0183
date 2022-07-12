# libNMEA0183

gps message prase library, support:

- [x] RMC
- [x] GSA
- [x] GGA

# Usage

Add below into CMakeLists.txt file:
```
find_package(libnmea0183 REQUIRED)

target_link_libraries(xxx nmea0183)
```
