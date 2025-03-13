# List all source files from the Src directory
set(SOURCES 
    "Src/PController.cpp"
    "Src/PIController.cpp"
    "Src/PDController.cpp"
    "Src/PIDController.cpp"
)

# Register the component with ESP-IDF
idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS "Inc"  # Point to the Inc directory where headers are located
    REQUIRES driver     # Maintain dependency on driver component
)