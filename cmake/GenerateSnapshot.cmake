# CMake script to generate a single file with all project code

set(PROJECT_SOURCE_DIR ${ARGV0})
set(SNAPSHOT_FILE "${PROJECT_SOURCE_DIR}/project_snapshot.txt")

message(STATUS "Generating project snapshot to: ${SNAPSHOT_FILE}")

# --- AI Instructions ---
set(AI_INSTRUCTIONS "--- AI INSTRUCTIONS ---

IMPORTANT:
1. ALWAYS write complete files. Do NOT use '...' or '// ... (rest of the code)' or similar abbreviations. Check ALL files you were asked to modify.
2. Use ONLY English comments in the code (// English comment). Do NOT use Russian comments.
3. Adhere to the existing coding style and structure.
4. At the end of your response, ALWAYS include the standard command sequence for the user (see USER COMMANDS section below for format).

--- END AI INSTRUCTIONS ---")

# --- User Commands ---
set(USER_COMMANDS "--- USER COMMANDS (Standard Workflow) ---

# 1. Clean Build (if CMakeLists.txt or dependencies changed, or if problems occur)
#    Run these in Developer Command Prompt for VS
# C: && cd C:\\Users\\illia\\OneDrive\\Документы\\programming\\Urbaxio
# rmdir /s /q build
# mkdir build
# cd build
# cmake .. -G \"Visual Studio 17 2022\" -A x64 -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake
# cmake --build . --config Debug -- /p:VcpkgEnableManifest=true

# 2. Incremental Build (if only .cpp/.h files changed)
#    Run these in Developer CommandPrompt for VS (inside the existing 'build' directory)
# cd /d C:\\Users\\illia\\OneDrive\\Документы\\programming\\Urbaxio\\build
# cmake --build . --config Debug -- /p:VcpkgEnableManifest=true

# 3. Run Application (after successful build)
#    Run in Developer Command Prompt for VS
# cd /d C:\\Users\\illia\\OneDrive\\Документы\\programming\\Urbaxio\\build\\shell\\Debug
# .\\Urbaxio.exe
#    (Or find Urbaxio.exe in build\\bin\\Debug or build\\shell\\Debug and double-click)

# 4. Generate Code Snapshot (run when needed for AI)
#    Run in Developer Command Prompt for VS (inside the 'build' directory)
# cd /d C:\\Users\\illia\\OneDrive\\Документы\\programming\\Urbaxio\\build
# cmake --build . --target generate_snapshot

--- END USER COMMANDS ---")

# Assemble the full header
set(SNAPSHOT_HEADER "${AI_INSTRUCTIONS}\n\n${USER_COMMANDS}\n\n--- Project Code Snapshot ---\n\n")

# Clear the file and write the header (in UTF8)
file(WRITE ${SNAPSHOT_FILE} "${SNAPSHOT_HEADER}" ENCODING UTF8)

# --- Source Directories to Scan ---
set(SOURCE_DIRS
    "${PROJECT_SOURCE_DIR}"                          # Project root
    "${PROJECT_SOURCE_DIR}/cad_kernel/include/cad_kernel" # CAD Kernel headers
    "${PROJECT_SOURCE_DIR}/cad_kernel/src"           # CAD Kernel sources
    "${PROJECT_SOURCE_DIR}/engine/include/engine"     # Engine headers
    "${PROJECT_SOURCE_DIR}/engine/src"               # Engine sources
    "${PROJECT_SOURCE_DIR}/shell"                    # Shell root (for CMakeLists.txt)
    "${PROJECT_SOURCE_DIR}/shell/include"            # Shell headers (NEW)
    "${PROJECT_SOURCE_DIR}/shell/src"                # Shell sources (NEW)
    "${PROJECT_SOURCE_DIR}/cmake"                    # CMake helper scripts
)

# --- File Extensions/Names to Include ---
set(SOURCE_EXTENSIONS
    "*.h" "*.cpp" "*.cmake" "CMakeLists.txt" "*.json" ".gitignore"
)

# --- Function to Add a File to the Snapshot ---
function(add_file_to_snapshot filepath)
    if(EXISTS ${filepath})
        get_filename_component(filename ${filepath} NAME)
        file(RELATIVE_PATH rel_path ${PROJECT_SOURCE_DIR} ${filepath})
        # Ensure consistent path separators (forward slashes)
        string(REPLACE "\\" "/" rel_path ${rel_path})
        file(APPEND ${SNAPSHOT_FILE} "ENCODINGUTF8--- File: ${rel_path} ---\n" ENCODING UTF8) # Add prefix
        file(READ ${filepath} file_content ENCODING UTF8)
        # Replace CRLF with LF for consistency (optional but good)
        # string(REGEX REPLACE "\r\n" "\n" file_content "${file_content}")
        file(APPEND ${SNAPSHOT_FILE} "${file_content}\n" ENCODING UTF8) # Add newline after content
        message(STATUS "  Added: ${rel_path}")
    else()
        message(WARNING "  Snapshot: File not found - ${filepath}")
    endif()
endfunction()

# --- Main Loop to Find and Add Files ---
foreach(dir ${SOURCE_DIRS})
    message(STATUS "Processing directory: ${dir}")
    # Find files matching the patterns in the current directory only (no recursion needed with explicit dirs)
    file(GLOB current_files LIST_DIRECTORIES false RELATIVE "${dir}" "${dir}/*") # Use RELATIVE to get paths relative to dir
    foreach(filename_rel ${current_files})
         set(filepath_abs "${dir}/${filename_rel}") # Construct absolute path
         set(include_file FALSE)
         get_filename_component(filename_only ${filepath_abs} NAME)
         get_filename_component(fileext ${filepath_abs} EXT)
         string(TOLOWER "${fileext}" fileext_lower)

         # Check against specific filenames and extensions
         foreach(ext_pattern ${SOURCE_EXTENSIONS})
             if (ext_pattern STREQUAL filename_only) # Match specific filenames like CMakeLists.txt
                  set(include_file TRUE)
                  break()
             elseif (IS_DIRECTORY ${filepath_abs}) # Skip directories
                 continue()
             elseif (ext_pattern MATCHES "^\\*\\.") # Match extensions like *.h
                 string(REPLACE "*." "." ext_match ${ext_pattern})
                 if (fileext_lower STREQUAL ext_match)
                     set(include_file TRUE)
                     break()
                 endif()
             endif()
         endforeach()

         # Add the file if it matched and is not in the build directory
         if(include_file)
             string(FIND "${filepath_abs}" "/build/" build_pos)
             string(FIND "${filepath_abs}" "\\build\\" build_pos_win)
             if(build_pos EQUAL -1 AND build_pos_win EQUAL -1)
                 add_file_to_snapshot(${filepath_abs})
             else()
                  message(STATUS "  Skipped (in build dir): ${filepath_abs}")
             endif()
         # else()
             # message(STATUS "  Skipped (no match): ${filepath_abs}") # Uncomment for verbose logging
         endif()
     endforeach()
endforeach()

message(STATUS "Project snapshot generation finished.") 