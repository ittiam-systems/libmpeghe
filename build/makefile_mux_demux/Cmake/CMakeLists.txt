cmake_minimum_required (VERSION 3.5.1)

project (MPEG-H_Mux_Demux VERSION 1.0)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_SUPPRESS_REGENERATION true)


add_executable (MPEGH_mux
	../../../test/impeghe_error.c
	../../../test/impegh_mux_rom.c
	../../../test/impegh_mp4_mux.c
	../../../test/impegh_mp4_mux_utils.c
	../../../test/impeghe_mp4_writer.c	
	)
add_executable (MPEGH_demux
	../../../test/impegh_error_demux.c
	../../../test/impegh_mp4_demux.c
	../../../test/impegh_mp4_demux_utils.c
	../../../test/impegh_mp4_file.c
	../../../test/impegh_mp4_file_wrapper.c
	../../../test/impegh_mp4_init.c
	../../../test/impegh_mp4_parser.c
	../../../test/impegh_mp4_utils.c
	../../../test/impegh_mp4_video.c
	)
if(MSVC)
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /STACK:16277216")
	target_compile_options(
		MPEGH_mux
		PRIVATE
		-D__USE_40__
        -D_CRT_SECURE_NO_WARNINGS
        -DWIN32
        -D_DEBUG
        -D_CONSOLE
        -D_WINDOWS
		)
	target_compile_options(
		MPEGH_demux
		PRIVATE
		-D__USE_40__
        -D_CRT_SECURE_NO_WARNINGS
        -DWIN32
        -D_DEBUG
        -D_CONSOLE
        -D_WINDOWS
		
		)
else()
	if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "amd64")
		target_compile_options(MPEGH_mux 
			PRIVATE
			-c -O3 -Wall -Wsequence-point 
			-Wunused-function 
			)
		target_compile_options(MPEGH_demux 
			PRIVATE
			-c -O3 -Wall -Wsequence-point 
			-Wunused-function 
			)
	
	elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "i686")	
	    set_target_properties(MPEGH_mux PROPERTIES LINK_FLAGS "-m32")
		target_compile_options(MPEGH_mux 
			PRIVATE
			-c -O3 -m32 -Wall -Wsequence-point 
			-Wunused-function 
			)
	    set_target_properties(MPEGH_demux PROPERTIES LINK_FLAGS "-m32")
		target_compile_options(MPEGH_demux 
			PRIVATE
			-c -O3 -m32 -Wall -Wsequence-point 
			-Wunused-function 
			)	
	elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "armv7l")
		target_compile_options(MPEGH_mux
			PRIVATE
			-c -O3 -mcpu=cortex-a8 -mfloat-abi=hard -Wall 
			-Wsequence-point -Wunused-function --std=gnu99
			)
		target_compile_options(MPEGH_demux
			PRIVATE
			-c -O3 -mcpu=cortex-a8  -Wall -Wsequence-point 
			-Wunused-function -Wno-strict-aliasing --std=gnu99
			-mfloat-abi=hard 
			)
	elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
		target_compile_options(MPEGH_mux 
			PRIVATE
		    -c -O3 -march=armv8-a -Wall -Wsequence-point 
			-Wunused-function -Wno-strict-aliasing -flto 
			)
		target_compile_options(MPEGH_demux 
			PRIVATE
		    -c -O3 -march=armv8-a -Wall -Wsequence-point 
			-Wunused-function -Wno-strict-aliasing -flto 
			)
	endif()
endif()

target_include_directories (
        MPEGH_mux
        PUBLIC 
	${CMAKE_CURRENT_LIST_DIR}/../../../encoder 
	${CMAKE_CURRENT_LIST_DIR}/../../../test
        )
target_include_directories (
        MPEGH_demux
        PUBLIC 
	${CMAKE_CURRENT_LIST_DIR}/../../../encoder 
	${CMAKE_CURRENT_LIST_DIR}/../../../test
        )
