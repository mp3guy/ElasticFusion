# Taken from https://github.com/BVLC/caffe/blob/master/cmake/Cuda.cmake
################################################################################################
# A function for automatic detection of GPUs installed  (if autodetection is enabled)
# Usage:
#   detect_installed_gpus(out_variable)
function(detect_installed_gpus out_variable)
  if(NOT CUDA_gpu_detect_output)
    set(__cufile ${PROJECT_BINARY_DIR}/detect_cuda_archs.cu)

    file(WRITE ${__cufile} ""
      "#include <cstdio>\n"
      "int main()\n"
      "{\n"
      "  int count = 0;\n"
      "  if (cudaSuccess != cudaGetDeviceCount(&count)) return -1;\n"
      "  if (count == 0) return -1;\n"
      "  for (int device = 0; device < count; ++device)\n"
      "  {\n"
      "    cudaDeviceProp prop;\n"
      "    if (cudaSuccess == cudaGetDeviceProperties(&prop, device))\n"
      "      std::printf(\"%d.%d;\", prop.major, prop.minor);\n"
      "  }\n"
      "  return 0;\n"
      "}\n")

    execute_process(COMMAND "${CUDA_NVCC_EXECUTABLE}" "-Wno-deprecated-gpu-targets" "--run" "${__cufile}"
                    WORKING_DIRECTORY "${PROJECT_BINARY_DIR}/CMakeFiles/"
                    RESULT_VARIABLE __nvcc_res OUTPUT_VARIABLE __nvcc_out
                    ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)

    if(__nvcc_res EQUAL 0)
      string(REGEX REPLACE "\\." "" __nvcc_out "${__nvcc_out}")
      string(REGEX MATCHALL "[0-9;]+" __nvcc_out "${__nvcc_out}")
      list(REMOVE_DUPLICATES __nvcc_out)
      set(CUDA_gpu_detect_output ${__nvcc_out} CACHE INTERNAL "Returned GPU architectures from detect_gpus tool" FORCE)
    endif()
  endif()

  if(NOT CUDA_gpu_detect_output)
    message(STATUS "Automatic GPU detection failed. Is CUDA properly installed? .")
  else()
    set(${out_variable} ${CUDA_gpu_detect_output} PARENT_SCOPE)
  endif()
endfunction()
