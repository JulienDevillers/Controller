set(CMAKE_C_COMPILER_WORKS 1)
set(CMAKE_C_COMPILER gcc)
set(CMAKE_CXX_COMPILER_WORKS 1)
set(CMAKE_CXX_COMPILER g++)
set(CMAKE_ASM_COMPILER  gcc)
set(CMAKE_AR ar)
set(CMAKE_OBJCOPY objcopy)
set(CMAKE_OBJDUMP objdump)
set(SIZE size)

if (NOT CMAKE_HOST_WIN32)
    set(FREERTOS_DIR_X86 ${LOWLEVEL_SRC_DIR}/FreeRTOS-Sim/)
    set(FREERTOS_SRC_DIR_X86 ${FREERTOS_DIR_X86}/Source/)
    set(FREERTOS_INC_DIR_X86 ${FREERTOS_DIR_X86}/Source/include/)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pthread")
    set(FREERTOS_SRC_X86
            ${LOWLEVEL_SRC_DIR}/vassertcall.c
            ${FREERTOS_SRC_DIR_X86}/croutine.c
            ${FREERTOS_SRC_DIR_X86}/event_groups.c
            ${FREERTOS_SRC_DIR_X86}/list.c
            ${FREERTOS_SRC_DIR_X86}/queue.c
            ${FREERTOS_SRC_DIR_X86}/tasks.c
            ${FREERTOS_SRC_DIR_X86}/timers.c
            ${FREERTOS_SRC_DIR_X86}/portable/MemMang/heap_3.c
            ${FREERTOS_SRC_DIR_X86}/portable/GCC/POSIX/port.c)

    include_directories(${LOWLEVEL_SRC_DIR} ${FREERTOS_INC_DIR_X86} ${FREERTOS_SRC_DIR_X86} ${FREERTOS_SRC_DIR_X86}/portable/GCC/POSIX/)
    target_sources(${EXECUTABLE} PRIVATE ${SOURCES} ${FREERTOS_SRC_X86})
endif()
