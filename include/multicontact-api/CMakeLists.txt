set(${PROJECT_NAME}_GEOMETRY_HEADERS
    geometry/curve-map.hpp geometry/ellipsoid.hpp geometry/fwd.hpp
    geometry/linear-cone.hpp geometry/second-order-cone.hpp)

set(${PROJECT_NAME}_SERIALIZATION_HEADERS
    serialization/aligned-vector.hpp serialization/archive.hpp
    serialization/eigen-matrix.hpp serialization/spatial.hpp)

set(${PROJECT_NAME}_SCENARIO_HEADERS
    scenario/contact-model.hpp scenario/contact-patch.hpp
    scenario/contact-phase.hpp scenario/contact-sequence.hpp scenario/fwd.hpp)

set(${PROJECT_NAME}_PYTHON_HEADERS
    bindings/python/fwd.hpp
    bindings/python/geometry/ellipsoid.hpp
    bindings/python/geometry/expose-geometry.hpp
    bindings/python/geometry/linear-cone.hpp
    bindings/python/geometry/second-order-cone.hpp
    bindings/python/serialization/archive.hpp
    bindings/python/scenario/contact-model.hpp
    bindings/python/scenario/contact-patch.hpp
    bindings/python/scenario/contact-phase.hpp
    bindings/python/scenario/contact-sequence.hpp
    bindings/python/scenario/expose-scenario.hpp
    bindings/python/utils/printable.hpp)

set(${PROJECT_NAME}_HEADERS
    ${${PROJECT_NAME}_GEOMETRY_HEADERS}
    ${${PROJECT_NAME}_SERIALIZATION_HEADERS}
    ${${PROJECT_NAME}_SCENARIO_HEADERS} ${${PROJECT_NAME}_PYTHON_HEADERS})

add_header_group(${${PROJECT_NAME}_HEADERS})

foreach(HEADER ${${PROJECT_NAME}_HEADERS})
  get_filename_component(HEADER_PATH ${HEADER} PATH)
  install(FILES ${HEADER} DESTINATION include/${PROJECT_NAME}/${HEADER_PATH})
endforeach(HEADER ${${PROJECT_NAME}_HEADERS})
