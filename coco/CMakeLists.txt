set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
add_library(${PROJECT_NAME})
target_sources(${PROJECT_NAME}
	PUBLIC FILE_SET headers TYPE HEADERS FILES
		I2cMaster.hpp
	PRIVATE
		I2cMaster.cpp
)

if(${PLATFORM} STREQUAL "native" OR ${PLATFORM} STREQUAL "emu")
	# native platform (Windows, MacOS, Linux)
	target_sources(${PROJECT_NAME}
		PUBLIC FILE_SET platform_headers TYPE HEADERS BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/native FILES
			native/coco/platform/I2cMaster_cout.hpp
		PRIVATE
			native/coco/platform/I2cMaster_cout.cpp
	)
elseif(${PLATFORM} MATCHES "^nrf52")
#	target_sources(${PROJECT_NAME}
#		PUBLIC FILE_SET platform_headers TYPE HEADERS BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/nrf52 FILES
#			nrf52/coco/platform/I2cMaster_TWIM0.hpp
#		PRIVATE
#			nrf52/coco/platform/I2cMaster_TWIM0.cpp
#	)
elseif(${PLATFORM} MATCHES "^stm32f4")

elseif(${PLATFORM} MATCHES "^stm32")
	target_sources(${PROJECT_NAME}
		PUBLIC FILE_SET platform_headers TYPE HEADERS BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/stm32 FILES
			stm32/coco/platform/I2cMaster_I2C_DMA.hpp
		PRIVATE
			stm32/coco/platform/I2cMaster_I2C_DMA.cpp
	)
endif()

target_link_libraries(${PROJECT_NAME}
	coco::coco
	coco-loop::coco-loop
	coco-device::coco-device
)

target_include_directories(${PROJECT_NAME}
	PUBLIC
		..
)

# install the library
install(TARGETS ${PROJECT_NAME}
	FILE_SET headers DESTINATION include/coco
	FILE_SET platform_headers DESTINATION include
)
