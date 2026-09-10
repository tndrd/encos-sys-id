cmake_minimum_required(VERSION 3.20)
project(encos-sys-id CXX)
set(PYTHON_MODULE_NAME encosid)

# Bindings
add_subdirectory(lib/pybind11)

# Tenso driver
add_subdirectory(lib/tilkom-t42-driver)

pybind11_add_module(${PYTHON_MODULE_NAME} 
  src/binding.cpp
  src/encos.cpp
  src/generator.cpp
  ${TILKOM_BINDING_SOURCES}
)
target_include_directories(${PYTHON_MODULE_NAME} PUBLIC ${TILKOM_BINDING_INCLUDE_DIR})
target_link_libraries(${PYTHON_MODULE_NAME} PUBLIC pthread tilkom_t42)

target_compile_definitions(${PYTHON_MODULE_NAME} PUBLIC 
  PYTHON_MODULE_NAME=${PYTHON_MODULE_NAME}
)

execute_process(COMMAND python3 -c "import site; print(site.getsitepackages()[0])" OUTPUT_VARIABLE PYTHON3_SITEDIR OUTPUT_STRIP_TRAILING_WHITESPACE)
message(STATUS " Python site-packages directory: ${PYTHON3_SITEDIR}")
install(TARGETS ${PYTHON_MODULE_NAME} DESTINATION ${PYTHON3_SITEDIR})