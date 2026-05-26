include("${CMAKE_CURRENT_LIST_DIR}/rule.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/file.cmake")

set(ClearCoreController_Debug_library_list )

# Handle files with suffix s, for group Debug-arm-gcc
if(ClearCoreController_Debug_Debug_arm_gcc_FILE_TYPE_assemble)
add_library(ClearCoreController_Debug_Debug_arm_gcc_assemble OBJECT ${ClearCoreController_Debug_Debug_arm_gcc_FILE_TYPE_assemble})
    ClearCoreController_Debug_Debug_arm_gcc_assemble_rule(ClearCoreController_Debug_Debug_arm_gcc_assemble)
    list(APPEND ClearCoreController_Debug_library_list "$<TARGET_OBJECTS:ClearCoreController_Debug_Debug_arm_gcc_assemble>")

endif()

# Handle files with suffix S, for group Debug-arm-gcc
if(ClearCoreController_Debug_Debug_arm_gcc_FILE_TYPE_assembleWithPreprocess)
add_library(ClearCoreController_Debug_Debug_arm_gcc_assembleWithPreprocess OBJECT ${ClearCoreController_Debug_Debug_arm_gcc_FILE_TYPE_assembleWithPreprocess})
    ClearCoreController_Debug_Debug_arm_gcc_assembleWithPreprocess_rule(ClearCoreController_Debug_Debug_arm_gcc_assembleWithPreprocess)
    list(APPEND ClearCoreController_Debug_library_list "$<TARGET_OBJECTS:ClearCoreController_Debug_Debug_arm_gcc_assembleWithPreprocess>")

endif()

# Handle files with suffix [cC], for group Debug-arm-gcc
if(ClearCoreController_Debug_Debug_arm_gcc_FILE_TYPE_compile)
add_library(ClearCoreController_Debug_Debug_arm_gcc_compile OBJECT ${ClearCoreController_Debug_Debug_arm_gcc_FILE_TYPE_compile})
    ClearCoreController_Debug_Debug_arm_gcc_compile_rule(ClearCoreController_Debug_Debug_arm_gcc_compile)
    list(APPEND ClearCoreController_Debug_library_list "$<TARGET_OBJECTS:ClearCoreController_Debug_Debug_arm_gcc_compile>")

endif()

# Handle files with suffix cpp, for group Debug-arm-gcc
if(ClearCoreController_Debug_Debug_arm_gcc_FILE_TYPE_compile_cpp)
add_library(ClearCoreController_Debug_Debug_arm_gcc_compile_cpp OBJECT ${ClearCoreController_Debug_Debug_arm_gcc_FILE_TYPE_compile_cpp})
    ClearCoreController_Debug_Debug_arm_gcc_compile_cpp_rule(ClearCoreController_Debug_Debug_arm_gcc_compile_cpp)
    list(APPEND ClearCoreController_Debug_library_list "$<TARGET_OBJECTS:ClearCoreController_Debug_Debug_arm_gcc_compile_cpp>")

endif()

# Handle files with suffix elf, for group Debug-arm-gcc
if(ClearCoreController_Debug_Debug_arm_gcc_FILE_TYPE_objcopy_ihex)
add_library(ClearCoreController_Debug_Debug_arm_gcc_objcopy_ihex OBJECT ${ClearCoreController_Debug_Debug_arm_gcc_FILE_TYPE_objcopy_ihex})
    ClearCoreController_Debug_Debug_arm_gcc_objcopy_ihex_rule(ClearCoreController_Debug_Debug_arm_gcc_objcopy_ihex)
    list(APPEND ClearCoreController_Debug_library_list "$<TARGET_OBJECTS:ClearCoreController_Debug_Debug_arm_gcc_objcopy_ihex>")

endif()

# Handle files with suffix elf, for group Debug-arm-gcc
if(ClearCoreController_Debug_Debug_arm_gcc_FILE_TYPE_objcopy_eep)
add_library(ClearCoreController_Debug_Debug_arm_gcc_objcopy_eep OBJECT ${ClearCoreController_Debug_Debug_arm_gcc_FILE_TYPE_objcopy_eep})
    ClearCoreController_Debug_Debug_arm_gcc_objcopy_eep_rule(ClearCoreController_Debug_Debug_arm_gcc_objcopy_eep)
    list(APPEND ClearCoreController_Debug_library_list "$<TARGET_OBJECTS:ClearCoreController_Debug_Debug_arm_gcc_objcopy_eep>")

endif()

# Handle files with suffix elf, for group Debug-arm-gcc
if(ClearCoreController_Debug_Debug_arm_gcc_FILE_TYPE_objcopy_lss)
add_library(ClearCoreController_Debug_Debug_arm_gcc_objcopy_lss OBJECT ${ClearCoreController_Debug_Debug_arm_gcc_FILE_TYPE_objcopy_lss})
    ClearCoreController_Debug_Debug_arm_gcc_objcopy_lss_rule(ClearCoreController_Debug_Debug_arm_gcc_objcopy_lss)
    list(APPEND ClearCoreController_Debug_library_list "$<TARGET_OBJECTS:ClearCoreController_Debug_Debug_arm_gcc_objcopy_lss>")

endif()

# Handle files with suffix elf, for group Debug-arm-gcc
if(ClearCoreController_Debug_Debug_arm_gcc_FILE_TYPE_objcopy_srec)
add_library(ClearCoreController_Debug_Debug_arm_gcc_objcopy_srec OBJECT ${ClearCoreController_Debug_Debug_arm_gcc_FILE_TYPE_objcopy_srec})
    ClearCoreController_Debug_Debug_arm_gcc_objcopy_srec_rule(ClearCoreController_Debug_Debug_arm_gcc_objcopy_srec)
    list(APPEND ClearCoreController_Debug_library_list "$<TARGET_OBJECTS:ClearCoreController_Debug_Debug_arm_gcc_objcopy_srec>")

endif()


# Main target for this project
add_executable(ClearCoreController_Debug_image_gQXGiE3M ${ClearCoreController_Debug_library_list})

set_target_properties(ClearCoreController_Debug_image_gQXGiE3M PROPERTIES
    OUTPUT_NAME "Debug"
    SUFFIX ".elf"
    ADDITIONAL_CLEAN_FILES "${output_extensions}"
    RUNTIME_OUTPUT_DIRECTORY "${ClearCoreController_Debug_output_dir}")
target_link_libraries(ClearCoreController_Debug_image_gQXGiE3M PRIVATE ${ClearCoreController_Debug_Debug_arm_gcc_FILE_TYPE_link})

#Add objcopy steps
ClearCoreController_Debug_objcopy_ihex_rule(ClearCoreController_Debug_image_gQXGiE3M)
ClearCoreController_Debug_objcopy_eep_rule(ClearCoreController_Debug_image_gQXGiE3M)
ClearCoreController_Debug_objcopy_lss_rule(ClearCoreController_Debug_image_gQXGiE3M)
ClearCoreController_Debug_objcopy_srec_rule(ClearCoreController_Debug_image_gQXGiE3M)
# Add the link options from the rule file.
ClearCoreController_Debug_link_rule( ClearCoreController_Debug_image_gQXGiE3M)


# The following step will be performed after each build if final image is rebuilt
add_custom_command(TARGET ClearCoreController_Debug_image_gQXGiE3M POST_BUILD
    COMMAND $\(SolutionDir\)\\..\\Tools\\uf2-builder\\Release\\uf2-builder.exe \"$\(OutputDirectory\)\\$\(OutputFileName\).bin\" \"$\(OutputDirectory\)\\$\(OutputFileName\).uf2\"
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/../../../..)
