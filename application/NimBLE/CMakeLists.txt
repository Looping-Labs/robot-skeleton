# List all source files from the Src directory
set(SOURCES 
    "Src/NimBLEDevice.cpp"
    "Src/NimBLEServer.cpp"
    "Src/NimBLEService.cpp"
    "Src/NimBLECharacteristic.cpp"
    "Src/NimBLEAdvertising.cpp"
)

# Register the component with ESP-IDF
idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS "Inc"                          # Point to the Inc directory where headers are located
    REQUIRES driver nvs_flash bt                # Maintain dependency on driver component
)