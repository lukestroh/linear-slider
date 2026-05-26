# The following functions contains all the flags passed to the different build stages.

set(PACK_REPO_PATH "/Users/isaacmiller/.mchp_packs" CACHE PATH "Path to the root of a pack repository.")

function(ClearCoreController_Release_Release_arm_gcc_assemble_rule target)
    set(options
        "-g"
        "${ASSEMBLER_PRE}"
        "-mcpu=cortex-m4"
        "-mfpu=fpv4-sp-d16"
        "-mthumb"
        "-Wa,--defsym=__MPLAB_BUILD=1${MP_EXTRA_AS_POST}")
    list(REMOVE_ITEM options "")
    target_compile_options(${target} PRIVATE "${options}")
    target_compile_definitions(${target} PRIVATE "__SAME53N19A__")
endfunction()
function(ClearCoreController_Release_Release_arm_gcc_assembleWithPreprocess_rule target)
    set(options
        "-x"
        "assembler-with-cpp"
        "-g"
        "-gdwarf-2"
        "-mcpu=cortex-m4"
        "-mfpu=fpv4-sp-d16"
        "-mthumb"
        "-Wa,--defsym=__MPLAB_BUILD=1${MP_EXTRA_AS_POST}")
    list(REMOVE_ITEM options "")
    target_compile_options(${target} PRIVATE "${options}")
    target_compile_definitions(${target}
        PRIVATE "__SAME53N19A__"
        PRIVATE "Release=Release")
endfunction()
function(ClearCoreController_Release_Release_arm_gcc_compile_rule target)
    set(options
        "-g"
        "-gdwarf-2"
        "-mcpu=cortex-m4"
        "-mfpu=fpv4-sp-d16"
        "-x"
        "c"
        "-mthumb"
        "-O3"
        "-ffunction-sections"
        "-fdata-sections"
        "-Wall"
        "-std=gnu11"
        "SHELL:--param max-inline-insns-single=50"
        "-MMD"
        "-mcpu=cortex-m4"
        "-mfloat-abi=hard"
        "-mfpu=fpv4-sp-d16")
    list(REMOVE_ITEM options "")
    target_compile_options(${target} PRIVATE "${options}")
    target_compile_definitions(${target}
        PRIVATE "__SAME53N19A__"
        PRIVATE "NDEBUG"
        PRIVATE "Release=Release")
endfunction()
function(ClearCoreController_Release_Release_arm_gcc_compile_cpp_rule target)
    set(options
        "-g"
        "-gdwarf-2"
        "-x"
        "c++"
        "-mcpu=cortex-m4"
        "-mfpu=fpv4-sp-d16"
        "-mthumb"
        "-O3"
        "-ffunction-sections"
        "-fno-rtti"
        "-fno-exceptions"
        "-Wall"
        "-std=gnu++11"
        "-fno-threadsafe-statics"
        "-nostdlib"
        "SHELL:--param max-inline-insns-single=500"
        "-MMD"
        "-mcpu=cortex-m4"
        "-mfloat-abi=hard"
        "-mfpu=fpv4-sp-d16")
    list(REMOVE_ITEM options "")
    target_compile_options(${target} PRIVATE "${options}")
    target_compile_definitions(${target}
        PRIVATE "__SAME53N19A__"
        PRIVATE "NDEBUG"
        PRIVATE "Release=Release")
    target_include_directories(${target}
        PRIVATE "libClearCore/inc"
        PRIVATE "LwIP/LwIP/src/include"
        PRIVATE "LwIP/LwIP/port/include")
endfunction()
function(ClearCoreController_Release_link_rule target)
    set(options
        "-gdwarf-2"
        "${MP_EXTRA_LD_PRE}"
        "-T/Users/isaacmiller/Downloads/Apple-Picking-Robot/Linear-Slider/linear-slider/clearcore-controller-isaac-position-control/Device_Startup/flash_with_bootloader.ld"
        "-mthumb"
        "-Wl,-Map=mem.map"
        "--specs=rdimon.specs"
        "-Wl,--defsym=__MPLAB_BUILD=1${MP_EXTRA_LD_POST}"
        "-L${CMAKE_CURRENT_SOURCE_DIR}/../../../clearcore-controller-isaac-position-control/Device_Startup"
        "-Wl,--gc-sections"
        "-Tsame53n19a_flash.ld"
        "-mfloat-abi=hard"
        "-mfpu=fpv4-sp-d16")
    list(REMOVE_ITEM options "")
    target_link_options(${target} PRIVATE "${options}")
    target_compile_definitions(${target}
        PRIVATE "__SAME53N19A__"
        PRIVATE "Release=Release")
    target_link_libraries(${target}
        PRIVATE "m"
        PRIVATE "arm_cortexM4lf_math")
endfunction()
function(ClearCoreController_Release_objcopy_ihex_rule target)
    add_custom_command(
        TARGET ${target}
        POST_BUILD
        COMMAND ${OBJCOPY}
        ARGS --output-target=ihex --remove-section=.eeprom --remove-section=.fuse --remove-section=.lock --remove-section=.signature ${ClearCoreController_Release_image_name} ${ClearCoreController_Release_image_base_name}.hex
        WORKING_DIRECTORY ${ClearCoreController_Release_output_dir})
endfunction()
function(ClearCoreController_Release_objcopy_eep_rule target)
    add_custom_command(
        TARGET ${target}
        POST_BUILD
        COMMAND ${OBJCOPY}
        ARGS --only-section=.eeprom --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0 --no-change-warnings --output-target=binary ${ClearCoreController_Release_image_name} ${ClearCoreController_Release_image_base_name}.eep
        WORKING_DIRECTORY ${ClearCoreController_Release_output_dir})
endfunction()
function(ClearCoreController_Release_objcopy_lss_rule target)
    add_custom_command(
        TARGET ${target}
        POST_BUILD
        COMMAND ${OBJDUMP}
        ARGS --disassemble --wide --demangle --line-numbers --section-headers --source ${ClearCoreController_Release_image_name} > ${ClearCoreController_Release_image_base_name}.lss
        WORKING_DIRECTORY ${ClearCoreController_Release_output_dir})
endfunction()
function(ClearCoreController_Release_objcopy_srec_rule target)
    add_custom_command(
        TARGET ${target}
        POST_BUILD
        COMMAND ${OBJCOPY}
        ARGS --output-target=srec --remove-section=.eeprom --remove-section=.fuse --remove-section=.lock --remove-section=.signature ${ClearCoreController_Release_image_name} ${ClearCoreController_Release_image_base_name}.srec
        WORKING_DIRECTORY ${ClearCoreController_Release_output_dir})
endfunction()
