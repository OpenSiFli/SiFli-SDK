CMAKE_MINIMUM_REQUIRED(VERSION 2.6 )
PROJECT( uffs )

SET(CMAKE_CXX_FLAGS_VGL "-O0 -g")
SET(CMAKE_C_FLAGS_VGL "-O0 -g")

SET(CMAKE_C_FLAGS_GCOV "-g -O0 -Wall -fprofile-arcs -ftest-coverage")
SET(CMAKE_CXX_FLAGS_GCOV "-g -O0 -Wall -fprofile-arcs -ftest-coverage")
SET(CMAKE_EXE_LINKER_FLAGS_GCOV "${CMAKE_EXE_LINKER_FLAGS} -fprofile-arcs -ftest-coverage -lgcov")

IF (CMAKE_COMPILER_IS_GNUCC)
IF (APPLE)
    SET(CMAKE_CXX_FLAGS "-fPIC -Wall -Werror -g")
    SET(CMAKE_C_FLAGS "-fPIC -Wall -Werror -g")
ELSEIF (CYGWIN)
	SET(CMAKE_CXX_FLAGS "-Wall -Werror -Wclobbered -Wempty-body -Wignored-qualifiers -Wmissing-parameter-type -Woverride-init -Wtype-limits -Wuninitialized -g")
	SET(CMAKE_C_FLAGS "-Wall -Werror -Wclobbered -Wempty-body -Wignored-qualifiers -Wmissing-parameter-type -Woverride-init -Wtype-limits -Wuninitialized -g	")
ELSE ()
	SET(CMAKE_CXX_FLAGS "-fPIC -Wall -Werror -Wclobbered -Wempty-body -Wignored-qualifiers -Wmissing-parameter-type -Woverride-init -Wtype-limits -Wuninitialized -g -rdynamic")
	SET(CMAKE_C_FLAGS "-fPIC -Wall -Werror -Wclobbered -Wempty-body -Wignored-qualifiers -Wmissing-parameter-type -Woverride-init -Wtype-limits -Wuninitialized -g -rdynamic")
ENDIF()
ENDIF()

IF (APPLE)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DOS_APPLE")
	SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DOS_APPLE")
ENDIF()

IF (UNIX)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUNIX")
	SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DUNIX")
ENDIF()

ADD_SUBDIRECTORY( src )

