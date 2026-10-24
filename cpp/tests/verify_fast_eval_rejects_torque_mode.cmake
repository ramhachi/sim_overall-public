if(NOT DEFINED LAPTIME_SIM)
    message(FATAL_ERROR "LAPTIME_SIM is required")
endif()

if(NOT DEFINED SOURCE_ROOT)
    message(FATAL_ERROR "SOURCE_ROOT is required")
endif()

set(TORQUE_PARAMS "${SOURCE_ROOT}/matlab/parameter_presets/params_torque_steering.json")

execute_process(
    COMMAND "${LAPTIME_SIM}"
            --track acceleration
            --driver pid
            --eval-mode fast
            --params_file "${TORQUE_PARAMS}"
            --allow-defaults
            --output_json
    WORKING_DIRECTORY "${SOURCE_ROOT}"
    RESULT_VARIABLE run_rc
    OUTPUT_VARIABLE run_stdout
    ERROR_VARIABLE run_stderr
)

if(NOT run_rc EQUAL 1)
    message(FATAL_ERROR
        "Expected fast torque-mode rejection to exit with code 1, got ${run_rc}\n"
        "stdout:\n${run_stdout}\n"
        "stderr:\n${run_stderr}")
endif()

string(FIND "${run_stdout}" "\"status\":\"failed\"" status_pos)
if(status_pos EQUAL -1)
    message(FATAL_ERROR
        "Expected failed status in JSON output.\nstdout:\n${run_stdout}\n"
        "stderr:\n${run_stderr}")
endif()

string(FIND "${run_stdout}" "\"failure_code\":\"fast_eval_unsupported\"" code_pos)
if(code_pos EQUAL -1)
    message(FATAL_ERROR
        "Expected fast_eval_unsupported failure code in JSON output.\nstdout:\n${run_stdout}\n"
        "stderr:\n${run_stderr}")
endif()
