set(SOURCES
  calibrated_image.cpp
  field.cpp
  monitoring_manager.cpp
  opencv_image_provider.cpp
  replay_image_provider.cpp
  utils.cpp
  )

if (HL_MONITORING_USES_FLYCAPTURE)
  set(SOURCES "${SOURCES}" 
    flycap_image_provider.cpp
  )
endif(HL_MONITORING_USES_FLYCAPTURE)
