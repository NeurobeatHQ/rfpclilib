# ESP-IDF component registration. Lets this library be consumed as an ESP-IDF
# component — by the component manager (an idf_component.yml git or path
# dependency), a git submodule, or a plain components/ dir. The Arduino build
# ignores this file and compiles the library per library.properties.
#
# Uses only common components (freertos, libc, C++ STL), so no REQUIRES needed.
idf_component_register(SRCS "rfpclilib.cpp"
                       INCLUDE_DIRS ".")
