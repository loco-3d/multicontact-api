set(${PROJECT_NAME}_PYTHON_TESTS trivial geometry scenario)

foreach(TEST ${${PROJECT_NAME}_PYTHON_TESTS})
  add_python_unit_test("py-${TEST}" "unittest/python/${TEST}.py"
                       "bindings/python")
endforeach(TEST ${${PROJECT_NAME}_PYTHON_TESTS})
