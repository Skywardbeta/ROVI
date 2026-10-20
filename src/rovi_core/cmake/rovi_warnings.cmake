# Project warning policy applied to every ROVI C++ target.
function(rovi_target_warnings target)
  target_compile_options(${target} PRIVATE
    -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow -Wold-style-cast
    -Wcast-align -Wnon-virtual-dtor -Woverloaded-virtual -Wnull-dereference
    -Wdouble-promotion -Wformat=2 -Wimplicit-fallthrough
    "$<$<CXX_COMPILER_ID:GNU>:-Wuseless-cast;-Wduplicated-cond;-Wduplicated-branches;-Wlogical-op>"
    -Werror)
endfunction()
