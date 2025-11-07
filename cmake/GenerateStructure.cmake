# CMake script to generate a file with the project's file structure.

set(PROJECT_SOURCE_DIR ${ARGV0})
string(REPLACE "\\" "/" PROJECT_SOURCE_DIR_NORMALIZED "${PROJECT_SOURCE_DIR}")
set(STRUCTURE_FILE "${PROJECT_SOURCE_DIR}/project_structure.txt")

message(STATUS "Generating project structure to: ${STRUCTURE_FILE}")

# --- Header with instructions for the AI ---
# Using bracket notation [[...]] to avoid escape sequence issues with backticks

set(FILE_HEADER [[--- AI WORKFLOW INSTRUCTIONS & PROJECT STRUCTURE ---

TO THE AI:

Your primary goal is to assist the user by requesting the MINIMUM amount of code necessary. Use the file list below to determine which files are relevant.

YOUR WORKFLOW:

1. ANALYZE the user's request and the file list below.

2. IDENTIFY the specific files you need.

3. CONSTRUCT a response for the user that contains TWO parts:

   a. Instructions to open/edit a file named 'snapshot_files.txt'.

   b. A code block containing the semicolon-separated list of files to paste into that file.

   c. The simple, final command for the user to run.

4. PROVIDE this complete response to the user.

--- AI RESPONSE TEMPLATE ---

Okay, I see the issue. To fix it, I need to see the relevant files. Please follow these steps:

1.  In your project's root folder (`C:/dev/Urbaxio`), open the file named `snapshot_files.txt` (or create it if it doesn't exist).

2.  Replace its entire content with the following line:

    ```
    <file1>;<file2>;...
    ```

3.  Run the following command in your Developer Command Prompt (from your 'build' directory):

    ```bash
    # Run in Developer Command Prompt for VS (inside the 'build' directory)
    cd /d C:/dev/Urbaxio/build
    cmake --build . --target generate_partial_snapshot
    ```

4.  Send me the content of the newly generated `partial_snapshot.txt` file.

--- END OF TEMPLATE ---

EXAMPLE FOR FULL SNAPSHOT:

--- START OF AI-GENERATED COMMAND ---

# Run in Developer Command Prompt for VS (inside the 'build' directory)

cd /d C:/dev/Urbaxio/build

cmake --build . --target generate_snapshot

--- END OF AI-GENERATED COMMAND ---

--- PROJECT FILE LIST ---

]])

# Clear the file and write the header
file(WRITE ${STRUCTURE_FILE} "${FILE_HEADER}")

# --- Source Directories to Scan ---

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
    "${PROJECT_SOURCE_DIR}/resources"
)

# --- File Patterns to Include ---

set(SOURCE_PATTERNS "*.h" "*.cpp" "*.cmake" "CMakeLists.txt" "*.json" ".gitignore" "*.png" "*.obj" "*.ttf" "*.ico")

# --- Collect all file paths ---

set(ALL_FILES "")
foreach(dir ${SOURCE_DIRS})
    foreach(pattern ${SOURCE_PATTERNS})
        file(GLOB files RELATIVE "${PROJECT_SOURCE_DIR}" "${dir}/${pattern}")
        if(files)
            list(APPEND ALL_FILES ${files})
        endif()
    endforeach()
endforeach()

# Sort the list for consistent output and remove duplicates
list(SORT ALL_FILES)
list(REMOVE_DUPLICATES ALL_FILES)

# --- Write file paths to the structure file ---

foreach(rel_path ${ALL_FILES})
    string(REPLACE "\\" "/" rel_path ${rel_path}) # Normalize path separators
    file(APPEND ${STRUCTURE_FILE} "${rel_path}\n")
endforeach()

message(STATUS "Project structure generation finished.")

