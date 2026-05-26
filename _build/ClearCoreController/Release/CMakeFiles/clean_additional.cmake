# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "")
  file(REMOVE_RECURSE
  "/Users/isaacmiller/Downloads/Apple-Picking-Robot/Linear-Slider/linear-slider/out/ClearCoreController/Release.bin"
  "/Users/isaacmiller/Downloads/Apple-Picking-Robot/Linear-Slider/linear-slider/out/ClearCoreController/Release.eep"
  "/Users/isaacmiller/Downloads/Apple-Picking-Robot/Linear-Slider/linear-slider/out/ClearCoreController/Release.hex"
  "/Users/isaacmiller/Downloads/Apple-Picking-Robot/Linear-Slider/linear-slider/out/ClearCoreController/Release.lss"
  "/Users/isaacmiller/Downloads/Apple-Picking-Robot/Linear-Slider/linear-slider/out/ClearCoreController/Release.srec"
  )
endif()
