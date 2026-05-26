# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "")
  file(REMOVE_RECURSE
  "/Users/isaacmiller/Downloads/Apple-Picking-Robot/Linear-Slider/linear-slider/out/ClearCoreController/Debug.bin"
  "/Users/isaacmiller/Downloads/Apple-Picking-Robot/Linear-Slider/linear-slider/out/ClearCoreController/Debug.eep"
  "/Users/isaacmiller/Downloads/Apple-Picking-Robot/Linear-Slider/linear-slider/out/ClearCoreController/Debug.hex"
  "/Users/isaacmiller/Downloads/Apple-Picking-Robot/Linear-Slider/linear-slider/out/ClearCoreController/Debug.lss"
  "/Users/isaacmiller/Downloads/Apple-Picking-Robot/Linear-Slider/linear-slider/out/ClearCoreController/Debug.srec"
  )
endif()
