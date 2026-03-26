add_subdirectory(dynamics)
add_subdirectory(module_test)

set(EXAMPLES
        trapezoidal_motion_generator
        so3
        optimal_control
        velocity_estimation

        teleoperation_with_joint_mapping
        teleoperation_with_joint_mapping_kgs

        kinematic_calibration
        kinematic_calibration2

        log
        reset_odometry

        # Even-numbered Python example counterparts (02–32)
        02_power_command
        04_get_robot_state
        06_cancel_control
        06_stop_command
        08_zero_pose
        10_get_pid_gain
        12_fatory_default_pid_gain
        14_led
        16_battery_config
        18_check_firmware_version
        20_wifi
        22_joint_impedance_control
        24_serial_communication
        26_joint_group_command
        28_real_time_control_command
        30_dynamics_robot
        32_log_stream

        get_robot_state2
        gravity_compensation
        # self_collision_checking

        mobility_command

        rtcomm_protocol

        break_test
        set_pid_gain

        demo_motion_with_master_arm
)

foreach (SRC IN LISTS EXAMPLES)
    set(EXECUTABLE_NAME example_${SRC})
    add_executable(${EXECUTABLE_NAME}
            ${SRC}.cpp
    )
    target_link_libraries(${EXECUTABLE_NAME} PRIVATE
            rby1-sdk
        
            DynamixelSDK
            OsqpEigen::OsqpEigen
    )
    target_compile_definitions(${EXECUTABLE_NAME} PUBLIC
            -DPATH="${CMAKE_CURRENT_SOURCE_DIR}"
            -DMODELS_PATH="${CMAKE_SOURCE_DIR}/models/"
    )
    message(STATUS ${EXECUTABLE_NAME})
endforeach ()
