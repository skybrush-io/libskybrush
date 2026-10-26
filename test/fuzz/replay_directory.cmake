# Runs the replay driver given in REPLAY on all the files in the directory
# given in SEEDS. The list of files is known only after the seeds have been
# generated, therefore it cannot be passed on the command line in CMakeLists.
file(GLOB seeds "${SEEDS}/*")
if(NOT seeds)
    message(FATAL_ERROR "No seeds found in ${SEEDS}")
endif()

execute_process(COMMAND "${REPLAY}" ${seeds} RESULT_VARIABLE result)
if(NOT result EQUAL 0)
    message(FATAL_ERROR "Replay failed: ${result}")
endif()
