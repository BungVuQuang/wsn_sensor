# set(srcs "main.c"
# "board.c")

# idf_component_register(SRCS "${srcs}"
# INCLUDE_DIRS  ".")
set(COMPONENT_SRCS ds18b20.c main.c)
set(COMPONENT_ADD_INCLUDEDIRS ".")

register_component()

# set(pri_req ble_mesh_example_nvs)
# idf_component_register(SRCS "main.c" "board.c"
# INCLUDE_DIRS "."
# REQUIRES ${pri_req})
