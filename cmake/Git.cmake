# ------------------------------------------------------------
# --------------------- Git cmake module ---------------------
# ------------------------------------------------------------
# Obtain Git info of the project repo

option(GIT_INFO "Obtain Git info of the project repo." ON)

if (GIT_INFO)

    find_package(Git REQUIRED)

    # Extract git version
    string(REGEX MATCH "^[0-9]+\\.[0-9]+\\.[0-9]+" GIT_VERSION "${GIT_VERSION_STRING}")

    set(REQUIRED_VERSION "2.8")
    if (${GIT_VERSION} VERSION_LESS ${REQUIRED_VERSION})
        message(STATUS "Git Found: ${GIT_VERSION} (should be >= ${REQUIRED_VERSION})")
    else ()
        message(STATUS "Git Found: ${GIT_VERSION}")
    endif ()

    set(GIT_WORKING_DIR ${CMAKE_SOURCE_DIR})
    message(STATUS "Git working directory for commit: ${GIT_WORKING_DIR}")

    # Get the latest commit hash of the working branch
    execute_process(
            COMMAND ${GIT_EXECUTABLE} rev-parse --abbrev-ref HEAD
            WORKING_DIRECTORY ${GIT_WORKING_DIR}
            OUTPUT_VARIABLE GIT_BRANCH
            OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    # Extract working branch
    message(STATUS "Git working branch: ${GIT_BRANCH}")

    # Get the latest commit hash of the working branch
    execute_process(
            COMMAND ${GIT_EXECUTABLE} log --oneline -n 1
            WORKING_DIRECTORY ${GIT_WORKING_DIR}
            OUTPUT_VARIABLE GIT_COMMIT_MESSAGE
            OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    message(STATUS "Git last commit: ${GIT_COMMIT_MESSAGE}")

    # Get the latest commit hash of the working branch
    execute_process(
            COMMAND ${GIT_EXECUTABLE} describe --always --dirty --abbrev=16
            WORKING_DIRECTORY ${GIT_WORKING_DIR}
            OUTPUT_VARIABLE GIT_DESCRIBE_STRING
            OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    # Extract the abbreviated commit hash
    string(REGEX MATCH "(([0-9a-zA-Z]+)(-dirty)?)$" GIT_COMMIT_HASH "${GIT_DESCRIBE_STRING}")
    set(PREFIX "GIT_COMMIT_HASH=[")
    set(SUFFIX "]")
    set(GIT_COMMIT ${PREFIX}${GIT_COMMIT_HASH}${SUFFIX})

    # define ${GIT_COMMIT} in sources so man can recognize the commit of compiled library
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/include/Repository/Git.h.in ${CMAKE_CURRENT_SOURCE_DIR}/include/Repository/Git.h @ONLY)
    #message(STATUS "Git commit hash: ${GIT_COMMIT_HASH}")
    message(--)

endif ()