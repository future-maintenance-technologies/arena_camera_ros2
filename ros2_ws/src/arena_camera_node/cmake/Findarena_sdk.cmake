
set(_LOG_LVL DEBUG) # does not filter as it should 
set(_LOG_LVL_FRMT "-- [ ${_LOG_LVL} ] ")

# Resolve the Arena SDK root:
#   1. Prefer the ARENA_SDK_ROOT environment variable (set per-machine in ~/.bashrc).
#   2. Fall back to reading Arena_SDK.conf (path relative to this file).
if(DEFINED ENV{ARENA_SDK_ROOT})
	set(arena_sdk_installation_root "$ENV{ARENA_SDK_ROOT}")
	message(${_LOG_LVL_FRMT} "arena_sdk_installation_root from ARENA_SDK_ROOT env var = ${arena_sdk_installation_root}")
else()
	set(_arena_sdk_conf "${CMAKE_CURRENT_LIST_DIR}/../../../resources/ArenaSDK/Arena_SDK.conf")
	if(NOT EXISTS ${_arena_sdk_conf})
		message(FATAL_ERROR "ARENA_SDK_ROOT env var not set and Arena_SDK.conf not found at ${_arena_sdk_conf}. "
			"Set ARENA_SDK_ROOT in your environment (e.g. in ~/.bashrc) to the ArenaSDK installation root.")
	endif()
	execute_process(
		COMMAND bash -c "dirname $(head -n 1 \"${_arena_sdk_conf}\")"
		OUTPUT_VARIABLE arena_sdk_installation_root)
	string(STRIP ${arena_sdk_installation_root} arena_sdk_installation_root)
	message(${_LOG_LVL_FRMT} "arena_sdk_installation_root from Arena_SDK.conf = ${arena_sdk_installation_root}")
endif()

	######### -----------------------------------------------------------------
	# INCLUDE
	#########

	set(arena_sdk_INCLUDE_DIRS
		${arena_sdk_installation_root}/GenICam/library/CPP/include
		${arena_sdk_installation_root}/include/Arena)
	set (arena_sdk_INCLUDES ${arena_sdk_INCLUDE_DIRS})

	#message(${_LOG_LVL_FRMT} "arena_sdk_INCLUDE_DIRS = ${arena_sdk_INCLUDE_DIRS}")

	###### --------------------------------------------------------------------
	# LIBS
	######
	
	if(EXISTS ${arena_sdk_installation_root}/GenICam/library/lib/Linux64_x64/libGCBase_gcc421_v3_0.so)
		set(ArenaSDK_Build "Linux64_x64_pre_54")
	elseif(EXISTS ${arena_sdk_installation_root}/GenICam/library/lib/Linux64_x64/libGCBase_gcc54_v3_3_LUCID.so)
		set(ArenaSDK_Build "Linux64_x64_54")
	elseif(EXISTS ${arena_sdk_installation_root}/GenICam/library/lib/Linux64_ARM/libGCBase_gcc54_v3_3_LUCID.so)
		set(ArenaSDK_Build "Linux64_ARM")
		
	else()
		message( FATAL_ERROR "LUCID GenICam not found. Please reisntall ArenaSDK "
					 "If having issues, contact: "
					 "LUCID support team (support@thinklucid.com). ")
	endif()
	
	if("${ArenaSDK_Build}" STREQUAL "Linux64_x64_pre_54")
		set(arena_sdk_LIBS

		## ArenaSDK

		## release
		${arena_sdk_installation_root}/lib64/libarena.so
		${arena_sdk_installation_root}/lib64/libsave.so
		${arena_sdk_installation_root}/lib64/libgentl.so
		${arena_sdk_installation_root}/lib64/liblucidlog.so

		## debug
		#${arena_sdk_installation_root}/lib64/libarenad.so
		#${arena_sdk_installation_root}/lib64/libsaved.so
		#${arena_sdk_installation_root}/lib64/libgentld.so
		#${arena_sdk_installation_root}/lib64/liblucidlogd.so

		## GenICam
		${arena_sdk_installation_root}/GenICam/library/lib/Linux64_x64/libGCBase_gcc421_v3_0.so
		${arena_sdk_installation_root}/GenICam/library/lib/Linux64_x64/libGenApi_gcc421_v3_0.so
		#${arena_sdk_installation_root}/GenICam/library/lib/Linux64_x64/liblog4cpp_gcc421_v3_0.so
		#${arena_sdk_installation_root}/GenICam/library/lib/Linux64_x64/libLog_gcc421_v3_0.so
		#${arena_sdk_installation_root}/GenICam/library/lib/Linux64_x64/libMathParser_gcc421_v3_0.so
		#${arena_sdk_installation_root}/GenICam/library/lib/Linux64_x64/libNodeMapData_gcc421_v3_0.so
		#${arena_sdk_installation_root}/GenICam/library/lib/Linux64_x64/libXmlParser_gcc421_v3_0.so
		## fmpeg
		#${arena_sdk_installation_root}/ffmpeg/libavcodec.so
		#${arena_sdk_installation_root}/ffmpeg/libavformat.so
		#${arena_sdk_installation_root}/ffmpeg/libavutil.so
		#${arena_sdk_installation_root}/ffmpeg/libswresample.so
		)
	elseif("${ArenaSDK_Build}" STREQUAL "Linux64_x64_54")
		set(arena_sdk_LIBS

		## ArenaSDK

		## release
		${arena_sdk_installation_root}/lib64/libarena.so
		${arena_sdk_installation_root}/lib64/libsave.so
		${arena_sdk_installation_root}/lib64/libgentl.so
		${arena_sdk_installation_root}/lib64/liblucidlog.so

		## debug
		#${arena_sdk_installation_root}/lib64/libarenad.so
		#${arena_sdk_installation_root}/lib64/libsaved.so
		#${arena_sdk_installation_root}/lib64/libgentld.so
		#${arena_sdk_installation_root}/lib64/liblucidlogd.so

		## GenICam
		${arena_sdk_installation_root}/GenICam/library/lib/Linux64_x64/libGCBase_gcc54_v3_3_LUCID.so
		${arena_sdk_installation_root}/GenICam/library/lib/Linux64_x64/libGenApi_gcc54_v3_3_LUCID.so
		#${arena_sdk_installation_root}/GenICam/library/lib/Linux64_x64/liblog4cpp_gcc54_v3_3_LUCID.so
		#${arena_sdk_installation_root}/GenICam/library/lib/Linux64_x64/libLog_gcc54_v3_3_LUCID.so
		#${arena_sdk_installation_root}/GenICam/library/lib/Linux64_x64/libMathParser_gcc54_v3_3_LUCID.so
		#${arena_sdk_installation_root}/GenICam/library/lib/Linux64_x64/libNodeMapData_gcc54_v3_3_LUCID.so
		#${arena_sdk_installation_root}/GenICam/library/lib/Linux64_x64/libXmlParser_gcc54_v3_3_LUCID.so
		
		## fmpeg
		#${arena_sdk_installation_root}/ffmpeg/libavcodec.so
		#${arena_sdk_installation_root}/ffmpeg/libavformat.so
		#${arena_sdk_installation_root}/ffmpeg/libavutil.so
		#${arena_sdk_installation_root}/ffmpeg/libswresample.so
		)
	elseif("${ArenaSDK_Build}" STREQUAL "Linux64_ARM")
		set(arena_sdk_LIBS

		## ArenaSDK

		## release
		${arena_sdk_installation_root}/lib/libarena.so
		${arena_sdk_installation_root}/lib/libsave.so
		${arena_sdk_installation_root}/lib/libgentl.so
		#${arena_sdk_installation_root}/lib/liblucidlog.so

		## debug
		#${arena_sdk_installation_root}/lib/libarenad.so
		#${arena_sdk_installation_root}/lib/libsaved.so
		#${arena_sdk_installation_root}/lib/libgentld.so
		#${arena_sdk_installation_root}/lib/liblucidlogd.so

		## GenICam
		${arena_sdk_installation_root}/GenICam/library/lib/Linux64_ARM/libGCBase_gcc54_v3_3_LUCID.so
		${arena_sdk_installation_root}/GenICam/library/lib/Linux64_ARM/libGenApi_gcc54_v3_3_LUCID.so
		#${arena_sdk_installation_root}/GenICam/library/lib/Linux64_ARM/liblog4cpp_gcc54_v3_3_LUCID.so
		#${arena_sdk_installation_root}/GenICam/library/lib/Linux64_ARM/libLog_gcc54_v3_3_LUCID.so
		#${arena_sdk_installation_root}/GenICam/library/lib/Linux64_ARM/libMathParser_gcc54_v3_3_LUCID.so
		#${arena_sdk_installation_root}/GenICam/library/lib/Linux64_ARM/libNodeMapData_gcc54_v3_3_LUCID.so
		#${arena_sdk_installation_root}/GenICam/library/lib/Linux64_ARM/libXmlParser_gcc54_v3_3_LUCID.so
		
		## fmpeg
		#${arena_sdk_installation_root}/ffmpeg/libavcodec.so
		#${arena_sdk_installation_root}/ffmpeg/libavformat.so
		#${arena_sdk_installation_root}/ffmpeg/libavutil.so
		#${arena_sdk_installation_root}/ffmpeg/libswresample.so
		)
	endif()

	set(arena_sdk_LIBRARIES ${arena_sdk_LIBS})
	#set(arena_sdk_DEFINITIONS GENICAM_USER_ACCEPTS_ANY_COMPILER)

	#message(${_LOG_LVL_FRMT} "arena_sdk_LIBRARIES = ${arena_sdk_LIBRARIES}")

	###### --------------------------------------------------------------------
	# LIBS DIRS
	######
	# NOT NEEDED AS _LIBS GIVES THE ABSOLUTE PATH TO THE SOs
	#set(arena_sdk_LIBRARIES_DIRS
	#	# ArenaSDK
	#	${arena_sdk_installation_root}/lib64
	#	${arena_sdk_installation_root}/GenICam/library/lib/Linux64_x64/
	#	${arena_sdk_installation_root}/ffmpeg/
	#)

	#message(${_LOG_LVL_FRMT} "arena_sdk_LIBRARIES_DIRS = ${arena_sdk_LIBRARIES_DIRS}")

	####### -------------------------------------------------------------------
	# FOUND
	#######

	set(arena_sdk_FOUND true)

	#message(${_LOG_LVL_FRMT} "arena_sdk_FOUND = ${arena_sdk_FOUND}")
