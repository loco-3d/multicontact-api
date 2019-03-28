SET(${PROJECT_NAME}_MATH_HEADERS
  math/cumsum.hpp
  math/factorial.hpp
  math/nrand.hpp
  math/search.hpp
  )

SET(${PROJECT_NAME}_GEOMETRY_HEADERS
  geometry/ellipsoid.hpp
  geometry/fwd.hpp
  geometry/linear-cone.hpp
  geometry/second-order-cone.hpp
  )

SET(${PROJECT_NAME}_SERIALIZATION_HEADERS
  serialization/aligned-vector.hpp
  serialization/archive.hpp
  serialization/eigen-matrix.hpp
  serialization/fwd.hpp
  serialization/spatial.hpp
  serialization/xml.hpp
  )

SET(${PROJECT_NAME}_SCENARIO_HEADERS
  scenario/constraint.hpp
  scenario/constraint-gmm.hpp
  scenario/constraint-soc.hpp
  scenario/contact-constraint.hpp
  scenario/contact-constraint-planar.hpp
  scenario/contact-model-planar.hpp
  scenario/contact-patch.hpp
  scenario/contact-phase.hpp
  scenario/contact-phase-humanoid.hpp
  scenario/contact-sequence.hpp
  scenario/fwd.hpp
  scenario/ms-interval.hpp
  )


SET(${PROJECT_NAME}_TRAJECTORIES_HEADERS
  trajectories/cubic-hermite-spline.hpp
  trajectories/fwd.hpp
  )

SET(${PROJECT_NAME}_CONTAINER_HEADERS
  container/ref.hpp
  )

SET(${PROJECT_NAME}_PYTHON_HEADERS
  bindings/python/container/array.hpp
  bindings/python/container/reference-wrapper.hpp
  bindings/python/container/visitor.hpp
  bindings/python/fwd.hpp
  bindings/python/geometry/ellipsoid.hpp
  bindings/python/geometry/expose-geometry.hpp
  bindings/python/geometry/linear-cone.hpp
  bindings/python/geometry/second-order-cone.hpp
  bindings/python/serialization/archive.hpp
  bindings/python/scenario/contact-model-planar.hpp
  bindings/python/scenario/contact-patch.hpp
  bindings/python/scenario/contact-phase.hpp
  bindings/python/scenario/contact-phase-humanoid.hpp
  bindings/python/scenario/contact-sequence.hpp
  bindings/python/scenario/enums.hpp
  bindings/python/scenario/expose-scenario.hpp
  bindings/python/scenario/ms-interval.hpp
  bindings/python/trajectories/cubic-hermite-spline.hpp
  bindings/python/trajectories/expose-trajectories.hpp
  bindings/python/utils/base.hpp
  bindings/python/utils/printable.hpp
  )

SET(${PROJECT_NAME}_HEADERS
  ${${PROJECT_NAME}_MATH_HEADERS}
  ${${PROJECT_NAME}_GEOMETRY_HEADERS}
  ${${PROJECT_NAME}_SERIALIZATION_HEADERS}
  ${${PROJECT_NAME}_SCENARIO_HEADERS}
  ${${PROJECT_NAME}_TRAJECTORIES_HEADERS}
  ${${PROJECT_NAME}_CONTAINER_HEADERS}
  ${${PROJECT_NAME}_PYTHON_HEADERS}
  )

ADD_HEADER_GROUP(${${PROJECT_NAME}_HEADERS})

FOREACH(HEADER ${${PROJECT_NAME}_HEADERS})
  GET_FILENAME_COMPONENT(HEADER_PATH ${HEADER} PATH)
  INSTALL(FILES ${HEADER} DESTINATION include/locomote/${HEADER_PATH})
ENDFOREACH(HEADER ${${PROJECT_NAME}_HEADERS})
