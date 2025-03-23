# List all source files from the Src directory
set(SOURCES 
    "Src/WebSocketHandler.cpp"
    "Src/RobotEndpoints.cpp"
    "Src/MiniSumoEndpoints.cpp"
    "Src/FollowerEndpoints.cpp"
    "Src/JsonHelper.cpp"
    "Src/Server.cpp"
)

# Register the component with ESP-IDF
idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS "Inc"  # Point to the Inc directory where headers are located
    REQUIRES driver esp_wifi nvs_flash esp_http_server json esp_timer  # Maintain dependency on driver component
)