# The following variables contains the files used by the different stages of the build process.
set(ClearCoreController_Debug_Debug_arm_gcc_FILE_TYPE_assemble)
set_source_files_properties(${ClearCoreController_Debug_Debug_arm_gcc_FILE_TYPE_assemble} PROPERTIES LANGUAGE ASM)

# For assembly files, add "." to the include path for each file so that .include with a relative path works
foreach(source_file ${ClearCoreController_Debug_Debug_arm_gcc_FILE_TYPE_assemble})
        set_source_files_properties(${source_file} PROPERTIES INCLUDE_DIRECTORIES "$<PATH:NORMAL_PATH,$<PATH:REMOVE_FILENAME,${source_file}>>")
endforeach()

set(ClearCoreController_Debug_Debug_arm_gcc_FILE_TYPE_assembleWithPreprocess)
set_source_files_properties(${ClearCoreController_Debug_Debug_arm_gcc_FILE_TYPE_assembleWithPreprocess} PROPERTIES LANGUAGE ASM)

# For assembly files, add "." to the include path for each file so that .include with a relative path works
foreach(source_file ${ClearCoreController_Debug_Debug_arm_gcc_FILE_TYPE_assembleWithPreprocess})
        set_source_files_properties(${source_file} PROPERTIES INCLUDE_DIRECTORIES "$<PATH:NORMAL_PATH,$<PATH:REMOVE_FILENAME,${source_file}>>")
endforeach()

set(ClearCoreController_Debug_Debug_arm_gcc_FILE_TYPE_compile "${CMAKE_CURRENT_SOURCE_DIR}/../../../clearcore-controller-isaac-position-control/Device_Startup/startup_same53.c")
set_source_files_properties(${ClearCoreController_Debug_Debug_arm_gcc_FILE_TYPE_compile} PROPERTIES LANGUAGE C)
set(ClearCoreController_Debug_Debug_arm_gcc_FILE_TYPE_compile_cpp
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../clearcore-controller-isaac-position-control/main.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../clearcore-controller-isaac-position-control/src/ClearPathMC.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../clearcore-controller-isaac-position-control/src/EthUDP.cpp")
set_source_files_properties(${ClearCoreController_Debug_Debug_arm_gcc_FILE_TYPE_compile_cpp} PROPERTIES LANGUAGE CXX)
set(ClearCoreController_Debug_Debug_arm_gcc_FILE_TYPE_link)
set(ClearCoreController_Debug_Debug_arm_gcc_FILE_TYPE_objcopy_ihex)
set(ClearCoreController_Debug_Debug_arm_gcc_FILE_TYPE_objcopy_eep)
set(ClearCoreController_Debug_Debug_arm_gcc_FILE_TYPE_objcopy_lss)
set(ClearCoreController_Debug_Debug_arm_gcc_FILE_TYPE_objcopy_srec)

# The linker script used for the build.
set(ClearCoreController_Debug_LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/../../../clearcore-controller-isaac-position-control/Device_Startup/flash_with_bootloader.ld")
set(ClearCoreController_Debug_image_name "Debug.elf")
set(ClearCoreController_Debug_image_base_name "Debug")

# The output directory of the final image.
set(ClearCoreController_Debug_output_dir "${CMAKE_CURRENT_SOURCE_DIR}/../../../out/ClearCoreController")

# The full path to the final image.
set(ClearCoreController_Debug_full_path_to_image ${ClearCoreController_Debug_output_dir}/${ClearCoreController_Debug_image_name})

# Potential output file extensions
set(output_extensions
    .hex
    .lss
    .eep
    .bin
    .srec)
list(TRANSFORM output_extensions PREPEND "${ClearCoreController_Debug_output_dir}/${ClearCoreController_Debug_image_base_name}")
