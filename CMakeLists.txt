cmake_minimum_required (VERSION 3.5.1)

project (MPEG-H_Encoder VERSION 1.0)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_SUPPRESS_REGENERATION true)
set(EXECUTABLE_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/build/bin)

# Add Library Target
add_library (
        MPEGHEncoderLib 
        STATIC	
	encoder/impeghe_acelp_enc.c
    encoder/impeghe_acelp_rom.c
	encoder/impeghe_acelp_tools.c
	encoder/impeghe_api.c
	encoder/impeghe_arith_enc.c
	encoder/impeghe_asc_write.c
	encoder/impeghe_avq_enc.c
	encoder/impeghe_avq_rom.c
	encoder/impeghe_bitbuffer.c
	encoder/impeghe_block_switch.c
	encoder/impeghe_cplx_pred.c
	encoder/impeghe_crc.c
	encoder/impeghe_dmx_cicp2geometry.c
	encoder/impeghe_dmx_matrix_common.c
	encoder/impeghe_dmx_matrix_enc.c
	encoder/impeghe_dmx_matrix_enc_api.c
	encoder/impeghe_drc_api.c
	encoder/impeghe_drc_enc.c
	encoder/impeghe_drc_gain_calculator.c
	encoder/impeghe_drc_gain_enc.c
	encoder/impeghe_drc_mux.c
    encoder/impeghe_drc_tables.c
    encoder/impeghe_drc_uni_drc_eq.c
    encoder/impeghe_drc_uni_drc_filter_bank.c
    encoder/impeghe_enc_fac.c
    encoder/impeghe_enc_main.c
    encoder/impeghe_enc_mct.c
	encoder/impeghe_enc_mct_rom.c
    encoder/impeghe_fdp_enc.c
    encoder/impeghe_fd_enc.c
    encoder/impeghe_fd_fac.c
    encoder/impeghe_fd_mdct.c
    encoder/impeghe_fd_qc_adjthr.c
    encoder/impeghe_fd_qc_util.c
    encoder/impeghe_fd_quant.c
    encoder/impeghe_fft.c
    encoder/impeghe_hoa_amb_comp_mod.c
    encoder/impeghe_hoa_ch_assignment.c
	encoder/impeghe_hoa_common_functions.c
	encoder/impeghe_hoa_config.c
	encoder/impeghe_hoa_decomposition.c
	encoder/impeghe_hoa_dir_based_pre_dom_sound_syn.c
	encoder/impeghe_hoa_dir_vec_estimation.c
	encoder/impeghe_hoa_dyn_correction.c
	encoder/impeghe_hoa_encoder.c
	encoder/impeghe_hoa_fft.c
	encoder/impeghe_hoa_frame.c
	encoder/impeghe_hoa_init.c
	encoder/impeghe_hoa_matrix.c
	encoder/impeghe_hoa_pre_dom_sound_syn.c
	encoder/impeghe_hoa_rom.c
	encoder/impeghe_hoa_spatial_encoder.c
	encoder/impeghe_hoa_vector_based_predom_sound_syn.c
	encoder/impeghe_huffman_rom.c
	encoder/impeghe_igf_enc.c
	encoder/impeghe_lpc.c
	encoder/impeghe_lpc_avq.c
	encoder/impeghe_lpd_enc.c
	encoder/impeghe_lpd_rom.c
	encoder/impeghe_lpd_utils.c
	encoder/impeghe_ltpf_enc.c
	encoder/impeghe_ltpf_rom.c
	encoder/impeghe_mae_write.c
	encoder/impeghe_mhas_write.c
	encoder/impeghe_ms.c
	encoder/impeghe_nf.c
	encoder/impeghe_oam_enc.c
	encoder/impeghe_oam_enc_ld.c
	encoder/impeghe_oam_enc_utils.c
	encoder/impeghe_psy_mod.c
	encoder/impeghe_psy_rom.c
	encoder/impeghe_psy_utils.c
	encoder/impeghe_resampler.c
	encoder/impeghe_resampler_rom.c
	encoder/impeghe_rom.c
	encoder/impeghe_signal_classifier.c
	encoder/impeghe_signal_classifier_rom.c
	encoder/impeghe_simple_matrix.c
	encoder/impeghe_stereo_lpd.c
	encoder/impeghe_stereo_lpd_rom.c
	encoder/impeghe_tbe_enc.c
	encoder/impeghe_tbe_rom.c
	encoder/impeghe_tcx_enc.c
	encoder/impeghe_tcx_mdct.c
	encoder/impeghe_tns_usac.c
	encoder/impeghe_windowing.c
	encoder/impeghe_write_bitstream.c
	
        )

add_executable (MPEGHEncoder
	test/impeghe_error.c
	test/impeghe_drc_user_config.c
	test/impeghe_mp4_writer.c
	test/impeghe_testbench.c
    test/impeghe_mae_user_config.c	
	)

if(MSVC)
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /STACK:16277216")
	target_compile_options(
		MPEGHEncoderLib	
		PRIVATE
		-D__USE_40__
		-D_CRT_SECURE_NO_WARNINGS
		-DWIN32
		-D_DEBUG
		-D_CONSOLE
		-D_LIB
		-D_WINDOWS

		)
	target_compile_options(
		MPEGHEncoder
		PRIVATE
		-D_CRT_SECURE_NO_WARNINGS
		-D__USE_40__
		-D_DEBUG
		-D_WINDOWS
		)
else()
	if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "amd64")
		target_compile_options(
			MPEGHEncoderLib	
			PRIVATE
			-D_X86_
			-D_LIB
			-c -O3 -Wall -Wsequence-point
			-Wunused-function 
			)
		target_compile_options(MPEGHEncoder 
			PRIVATE
			-D_DEBUG
			-c -O3 -Wall -Wsequence-point 
			-Wunused-function 
			)
	elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "i686")
		target_compile_options(
			MPEGHEncoderLib	
			PRIVATE
			-D_X86_
			-DWIN32
			-D_LIB
			-c -m32 -O3 -Wall -Wsequence-point
			-Wunused-function
			
			)

		set_target_properties(MPEGHEncoder PROPERTIES LINK_FLAGS "-m32")

		target_compile_options(MPEGHEncoder 
			PRIVATE   
			-D_DEBUG
			-c -m32 -O3 -Wall -Wsequence-point 
			-Wunused-function 
			)
	elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "armv7l")
		target_compile_options(
			MPEGHEncoderLib
			PRIVATE
			-D_ARM_
			-D_LIB
			-c -O3 -mcpu=cortex-a8 -mfloat-abi=hard -Wall 
			-Wsequence-point -Wunused-function --std=gnu99
			)
		target_compile_options(MPEGHEncoder 
			PRIVATE
			-D_DEBUG
			-c -O3 -mcpu=cortex-a8  -Wall -Wsequence-point 
			-Wunused-function -Wno-strict-aliasing --std=gnu99
			-mfloat-abi=hard 
			)
	elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
		target_compile_options(
			MPEGHEncoderLib
			PRIVATE
			-D_ARM_
			-D_LIB
			-c -O3 -march=armv8-a -Wall -Wsequence-point 
			-Wunused-function --std=gnu99
			)
		target_compile_options(MPEGHEncoder 
			PRIVATE
			-D_DEBUG
		    -c -O3 -march=armv8-a -Wall -Wsequence-point 
			-Wunused-function -Wno-strict-aliasing -flto 
			)
	endif()
endif()

target_include_directories (
        MPEGHEncoderLib
        PUBLIC 
	${CMAKE_CURRENT_LIST_DIR}/encoder 
	${CMAKE_CURRENT_LIST_DIR}/test
        )


target_include_directories (
        MPEGHEncoder 
        PUBLIC 
	${CMAKE_CURRENT_LIST_DIR}/encoder 
	${CMAKE_CURRENT_LIST_DIR}/test
	${CMAKE_CURRENT_LIST_DIR}/test/mp4
        )

if(MSVC)
	target_link_libraries (
		MPEGHEncoder 
		LINK_PUBLIC
		MPEGHEncoderLib
		)
else()
	target_link_libraries (
		MPEGHEncoder 
		LINK_PUBLIC
		MPEGHEncoderLib
		m
		)
endif()

add_subdirectory(build/makefile_mux_demux/Cmake)
