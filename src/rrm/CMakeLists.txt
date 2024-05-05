build_lib(
  LIBNAME rrm
  SOURCE_FILES
    model/rrm.cc
  HEADER_FILES
    model/rrm.h
  LIBRARIES_TO_LINK ${libnetwork}
)
