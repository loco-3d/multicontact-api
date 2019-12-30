
SET(${PROJECT_NAME}_GEOMETRY_HEADERS
  geometry/curve-map.hpp
  geometry/ellipsoid.hpp
  geometry/fwd.hpp
  geometry/linear-cone.hpp
  geometry/second-order-cone.hpp
  )

SET(${PROJECT_NAME}_SERIALIZATION_HEADERS
  serialization/aligned-vector.hpp
  serialization/archive.hpp
  serialization/eigen-matrix.hpp
  serialization/spatial.hpp
  )

SET(${PROJECT_NAME}_SCENARIO_HEADERS
  scenario/contact-model-planar.hpp
  scenario/contact-patch.hpp
  scenario/contact-phase.hpp
  scenario/contact-sequence.hpp
  scenario/fwd.hpp
  )


SET(${PROJECT_NAME}_PYTHON_HEADERS
  bindings/python/geometry/ellipsoid.hpp
  bindings/python/geometry/expose-geometry.hpp
  bindings/python/geometry/linear-cone.hpp
  bindings/python/geometry/second-order-cone.hpp
  bindings/python/serialization/archive.hpp
  bindings/python/scenario/contact-model-planar.hpp
  bindings/python/scenario/contact-patch.hpp
  bindings/python/scenario/contact-phase.hpp
  bindings/python/scenario/contact-sequence.hpp
  bindings/python/scenario/expose-scenario.hpp
  bindings/python/utils/printable.hpp
  )

SET(${PROJECT_NAME}_HEADERS
  ${${PROJECT_NAME}_GEOMETRY_HEADERS}
  ${${PROJECT_NAME}_SERIALIZATION_HEADERS}
  ${${PROJECT_NAME}_SCENARIO_HEADERS}
  ${${PROJECT_NAME}_PYTHON_HEADERS}
  )

ADD_HEADER_GROUP(${${PROJECT_NAME}_HEADERS})

FOREACH(HEADER ${${PROJECT_NAME}_HEADERS})
  GET_FILENAME_COMPONENT(HEADER_PATH ${HEADER} PATH)
  INSTALL(FILES ${HEADER} DESTINATION include/${PROJECT_NAME}/${HEADER_PATH})
ENDFOREACH(HEADER ${${PROJECT_NAME}_HEADERS})
