IF (UNIX)
	SET (libplatform_SRCS posix/uffs_os.c posix/uffs_config.h)
	INCLUDE_DIRECTORIES(${uffs_SOURCE_DIR}/src/platform/posix)
ENDIF()

IF (WIN32)
	SET (libplatform_SRCS win32/uffs_os.c win32/uffs_config.h)
	INCLUDE_DIRECTORIES(${uffs_SOURCE_DIR}/src/platform/win32)
ENDIF()

set (srcs)
foreach(file ${libplatform_SRCS})
	set(srcs ${srcs} ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach(file)

SET (lib_SRCS ${lib_SRCS} ${srcs} PARENT_SCOPE)

INCLUDE_DIRECTORIES(${uffs_SOURCE_DIR}/src/inc)
ADD_LIBRARY(platform STATIC ${libplatform_SRCS})

