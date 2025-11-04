# CMake script to generate a single file with project code.
# Can generate a full snapshot OR a partial one based on input.

# The project source dir must be defined with -DPROJECT_SOURCE_DIR=...
if(NOT DEFINED PROJECT_SOURCE_DIR)
    message(FATAL_ERROR "PROJECT_SOURCE_DIR must be defined on the command line, e.g., -DPROJECT_SOURCE_DIR=${CMAKE_SOURCE_DIR}")
endif()

# --- Determine Mode: Partial or Full Snapshot ---
# If FORCE_FULL_SNAPSHOT is set, always generate full snapshot regardless of snapshot_files.txt
set(INPUT_FILE_LIST_PATH "${PROJECT_SOURCE_DIR}/snapshot_files.txt")
set(IS_PARTIAL_SNAPSHOT FALSE)

if(DEFINED FORCE_FULL_SNAPSHOT AND FORCE_FULL_SNAPSHOT)
    # Force full snapshot mode
    message(STATUS "FORCE_FULL_SNAPSHOT is set - generating FULL snapshot (ignoring snapshot_files.txt if present)")
elseif(EXISTS ${INPUT_FILE_LIST_PATH})
    # Partial snapshot mode - snapshot_files.txt exists
    set(IS_PARTIAL_SNAPSHOT TRUE)
    file(READ ${INPUT_FILE_LIST_PATH} FILE_LIST_STRING ENCODING UTF8)
    string(STRIP "${FILE_LIST_STRING}" FILE_LIST_STRING) # Remove leading/trailing whitespace
    string(REPLACE ";" " " FILE_LIST_STRING_SPACES "${FILE_LIST_STRING}")
    string(REGEX MATCHALL "[^ ]+" FILES_TO_SNAPSHOT_LIST "${FILE_LIST_STRING_SPACES}")
endif()

# --- AI Instructions ---

set(AI_INSTRUCTIONS "--- AI INSTRUCTIONS ---



IMPORTANT:

1. Adhere to the existing coding style and structure.

2. Use ONLY English comments in the code (// English comment).

3. After every successful change, create a Git commit by providing the user with the necessary commands.

4. At the end of your response, ALWAYS include the standard command sequence for the user (see USER COMMANDS section below for format).

5. **If a PARTIAL snapshot was provided, ONLY write the changed files.** Do NOT write all files.

6. **If a FULL snapshot was provided, you MUST write ALL modified files.** Do NOT use abbreviations like '...' or '// ...'.



--- END AI INSTRUCTIONS ---")

# --- User Commands ---

set(USER_COMMANDS "--- USER COMMANDS (Standard Workflow) ---



# 1. Clean Build (if needed)

# cd /d C:/Users/illia/OneDrive/Документы/programming/Urbaxio

# rmdir /s /q build && mkdir build && cd build

# cmake .. -G \"Visual Studio 17 2022\" -A x64 -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake

# cmake --build . --config Debug -- /p:VcpkgEnableManifest=true



# 2. Incremental Build (most common)

# cd /d C:/Users/illia/OneDrive/Документы/programming/Urbaxio/build

# cmake --build . --config Debug -- /p:VcpkgEnableManifest=true



# 3. Run Application

# cd /d C:/Users/illia/OneDrive/Документы/programming/Urbaxio/build/shell/Debug

# .\\Urbaxio.exe



# 4. Rollback to Previous Version (if needed before generating snapshot)

# cd /d C:/Users/illia/OneDrive/Документы/programming/Urbaxio

# git reset --hard HEAD



# 5. Generate Code for AI

#    A) Generate Project Structure (if AI doesn't know the project)

#       Run this and send the content of 'project_structure.txt'.

#       cd /d C:/Users/illia/OneDrive/Документы/programming/Urbaxio/build

#       cmake --build . --target generate_structure

#

#    B) Generate Partial Snapshot (after AI requests specific files)

#       1. Open 'snapshot_files.txt' in project root and PASTE the file list from AI.

#       2. Run:

#          cd /d C:/Users/illia/OneDrive/Документы/programming/Urbaxio/build

#          cmake --build . --target generate_partial_snapshot

#

#    C) Generate Full Snapshot (for large changes or if requested)

#       cd /d C:/Users/illia/OneDrive/Документы/programming/Urbaxio/build

#       cmake --build . --target generate_snapshot



# 6. Git Commit & Push

# cd /d C:/Users/illia/OneDrive/Документы/programming/Urbaxio

# git add . && git commit -m \"A descriptive commit message\" && git push origin main



--- END USER COMMANDS ---")

# --- Set Output File and Header ---
if(IS_PARTIAL_SNAPSHOT)
    set(SNAPSHOT_FILE "${PROJECT_SOURCE_DIR}/partial_snapshot.txt")
    set(SNAPSHOT_HEADER "--- Partial Project Code Snapshot ---\n\n")
    message(STATUS "Generating PARTIAL snapshot to: ${SNAPSHOT_FILE}")
else()
    set(SNAPSHOT_FILE "${PROJECT_SOURCE_DIR}/project_snapshot.txt")
    set(SNAPSHOT_HEADER "--- Project Code Snapshot ---\n\n")
    message(STATUS "Generating FULL snapshot to: ${SNAPSHOT_FILE}")
endif()

# Assemble the full header content
set(FULL_HEADER "${AI_INSTRUCTIONS}\n\n${USER_COMMANDS}\n\n${SNAPSHOT_HEADER}")

# Clear the file and write the header (in UTF8)
file(WRITE ${SNAPSHOT_FILE} "${FULL_HEADER}" ENCODING UTF8)

# --- Function to Add a File to the Snapshot ---
function(add_file_to_snapshot filepath)
    if(EXISTS ${filepath})
        file(RELATIVE_PATH rel_path ${PROJECT_SOURCE_DIR} ${filepath})
        string(REPLACE "\\" "/" rel_path ${rel_path}) # Normalize path separators
        file(APPEND ${SNAPSHOT_FILE} "ENCODINGUTF8--- File: ${rel_path} ---\n" ENCODING UTF8)
        file(READ ${filepath} file_content ENCODING UTF8)
        file(APPEND ${SNAPSHOT_FILE} "${file_content}\n" ENCODING UTF8)
        message(STATUS "  Added: ${rel_path}")
    else()
        message(WARNING "  Snapshot: File not found - ${filepath}")
    endif()
endfunction()

# --- Main Logic ---
if(IS_PARTIAL_SNAPSHOT)
    # --- PARTIAL MODE ---
    message(STATUS "Files to include in partial snapshot (from ${INPUT_FILE_LIST_PATH}):")
    foreach(rel_path ${FILES_TO_SNAPSHOT_LIST})
        message(STATUS "  - ${rel_path}")
        add_file_to_snapshot("${PROJECT_SOURCE_DIR}/${rel_path}")
    endforeach()
    
    message(STATUS "Partial snapshot generated. The file 'snapshot_files.txt' was NOT deleted.")
else()
    # --- FULL MODE ---
    # Source Directories to Scan
    set(SOURCE_DIRS
        "${PROJECT_SOURCE_DIR}"
        "${PROJECT_SOURCE_DIR}/cad_kernel/include/cad_kernel"
        "${PROJECT_SOURCE_DIR}/cad_kernel/src"
        "${PROJECT_SOURCE_DIR}/engine/include/engine"
        "${PROJECT_SOURCE_DIR}/engine/src"
        "${PROJECT_SOURCE_DIR}/engine/include/engine/commands"
        "${PROJECT_SOURCE_DIR}/engine/src/commands"
        "${PROJECT_SOURCE_DIR}/shell"
        "${PROJECT_SOURCE_DIR}/shell/include"
        "${PROJECT_SOURCE_DIR}/shell/src"
        "${PROJECT_SOURCE_DIR}/shell/include/ui"
        "${PROJECT_SOURCE_DIR}/shell/src/ui"
        "${PROJECT_SOURCE_DIR}/shell/include/tools"
        "${PROJECT_SOURCE_DIR}/shell/src/tools"
        "${PROJECT_SOURCE_DIR}/cmake"
    )
    # File Extensions/Names to Include
    set(SOURCE_PATTERNS "*.h" "*.cpp" "*.cmake" "CMakeLists.txt" "*.json" ".gitignore")
    
    foreach(dir ${SOURCE_DIRS})
        foreach(pattern ${SOURCE_PATTERNS})
            file(GLOB files "${dir}/${pattern}")
            foreach(filepath ${files})
                add_file_to_snapshot(${filepath})
            endforeach()
        endforeach()
    endforeach()
endif()

message(STATUS "Snapshot generation finished.")
