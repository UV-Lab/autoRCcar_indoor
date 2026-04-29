function(generate_proto_cpp_file src_proto_dir dst_cpp_dir)
  # Check the dst cpp dir exist or not.
  if(EXISTS ${dst_cpp_dir})
    message("${dst_cpp_dir} exists. Generate the cpp files.")
  else()
    message("${dst_cpp_dir} doesn't exist, create it.")
    file(MAKE_DIRECTORY ${dst_cpp_dir})
  endif()

  file(COPY ${src_proto_dir}/ DESTINATION ${dst_cpp_dir})
  file(GLOB_RECURSE src_proto_files
    ${dst_cpp_dir}/*.proto
  )
  foreach(ele_proto IN LISTS src_proto_files)
    message("Generate the cpp file for ${ele_proto}")
    get_filename_component(ele_proto_path ${ele_proto} DIRECTORY)
    execute_process(COMMAND protoc --proto_path=${dst_cpp_dir} --cpp_out=${dst_cpp_dir} ${ele_proto})
    # file(REMOVE ${ele_proto})
  endforeach()

  foreach(ele_proto IN LISTS src_proto_files)
    message("Remove the proto files in dst folder.")
    file(REMOVE ${ele_proto})
  endforeach()

endfunction()