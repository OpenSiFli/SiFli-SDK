SET (flist 
		uffs_badblock.c 
		uffs_blockinfo.c 
		uffs_buf.c 
		uffs_debug.c 
		uffs_device.c 
		uffs_ecc.c 
		uffs_fd.c 
		uffs_fs.c 
		uffs_init.c 
		uffs_mem.c 
		uffs_pool.c 
		uffs_public.c 
		uffs_tree.c 
		uffs_utils.c
		uffs_mtb.c
		uffs_find.c
		uffs_flash.c
		uffs_version.c
		uffs_crc.c
	 )
	 
set (srcs)
foreach(file ${flist})
	set(srcs ${srcs} ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach(file)

SET (lib_SRCS ${lib_SRCS} ${srcs} PARENT_SCOPE)

SET (flist 
		uffs.h
		uffs_badblock.h
		uffs_blockinfo.h
		uffs_buf.h 
		uffs_core.h 
		uffs_device.h
		uffs_ecc.h
		uffs_fd.h
		uffs_fs.h
		uffs_mem.h
        uffs_os.h
        uffs_pool.h
        uffs_public.h
		uffs_tree.h
		uffs_types.h
		uffs_utils.h
		uffs_mtb.h
		uffs_find.h
		uffs_flash.h
		uffs_version.h
		uffs_crc.h
     )
	 
set (hdrs)
foreach(file ${hdrs})
	set(hdrs ${hdrs} ${CMAKE_CURRENT_SOURCE_DIR}/../inc/${file})
endforeach(file)


SET (libuffs_HEADS ${libuffs_HEADS} PARENT_SCOPE)


ADD_LIBRARY( uffs STATIC ${srcs} ${hdrs} )
