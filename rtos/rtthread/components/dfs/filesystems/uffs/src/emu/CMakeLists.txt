SET (libemu_SRCS 
		cmdline.c
		cmdline.h
		helper_cmds.c
		uffs_fileem.c
		uffs_fileem_share.c
		uffs_fileem_wrap.c
		uffs_fileem_ecc_soft.c
		uffs_fileem_ecc_hw.c
		uffs_fileem_ecc_hw_auto.c
		uffs_fileem.h
		test_cmds.c
	 )

set (srcs)
foreach(file ${libemu_SRCS})
	set(srcs ${srcs} ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach(file)

SET (lib_SRCS ${lib_SRCS} ${srcs} PARENT_SCOPE)

ADD_LIBRARY(emu STATIC ${libemu_SRCS} )

