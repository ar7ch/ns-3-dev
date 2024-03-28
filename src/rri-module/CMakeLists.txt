build_lib(
  LIBNAME rri-module
  SOURCE_FILES
    model/rri-module.cc
  HEADER_FILES
    model/rri-module.h
  LIBRARIES_TO_LINK ${libnetwork}
)
