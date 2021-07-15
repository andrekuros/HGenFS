# CMakeList.txt: projeto de CMake para QLGen, incluir origem e definir
# lógica específica a um projeto aqui.
#
cmake_minimum_required (VERSION 3.8)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package (Threads REQUIRED)

# Adicione a origem ao executável deste projeto.
add_executable (QLGen "QLGen.cpp" "QLGen.h" "individual.cpp" "individual.h")

# TODO: Adicione testes e instale destinos, se necessário.
target_link_libraries(QLGen Threads::Threads)
