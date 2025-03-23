#pragma once

#include "cJSON.h"
#include "esp_err.h"
#include <map>
#include <functional>
#include <string>
#include <vector>

/**
 * @brief Helper class for JSON processing using cJSON library
 *
 * This class provides utilities for parsing and creating JSON messages
 * used in WebSocket communication with clients.
 */
class JsonHelper {
public:
  /**
   * @brief Constructs a new Json Helper object
   */
  JsonHelper() = default;

  /**
   * @brief Destroys the Json Helper object
   */
  ~JsonHelper() = default;

  /**
   * @brief Parses a JSON string into key-value pairs
   *
   * @param json The JSON string to parse
   * @param output Map to store the parsed key-value pairs
   * @return esp_err_t ESP_OK on success, otherwise error code
   */
  esp_err_t parse(const std::string &json, std::map<std::string, std::string> &output);

  /**
   * @brief Creates a JSON string from key-value pairs
   *
   * @param data Map of key-value pairs to convert to JSON
   * @return std::string The resulting JSON string
   */
  std::string createJson(const std::map<std::string, std::string> &data);

  /**
   * @brief Creates a simple JSON string with a single key-value pair
   *
   * @param key The key
   * @param value The value
   * @return std::string The resulting JSON string
   */
  std::string createSimpleJson(const std::string &key, const std::string &value);

  /**
   * @brief Creates a JSON string with an array of values
   *
   * @param key The key for the array
   * @param values Vector of values to include in array
   * @return std::string The resulting JSON string
   */
  std::string createJsonArray(const std::string &key, const std::vector<std::string> &values);

  /**
   * @brief Extracts a value from a parsed JSON map
   *
   * @param data The parsed JSON map
   * @param key The key to look for
   * @param defaultVal Default value to return if key not found
   * @return std::string The value associated with the key or defaultVal
   */
  std::string getValue(const std::map<std::string, std::string> &data,
                       const std::string &key,
                       const std::string &defaultVal = "");

  /**
   * @brief Converts a numeric value to string
   *
   * @tparam T Numeric type
   * @param value The value to convert
   * @return std::string The string representation
   */
  template <typename T>
  std::string toString(T value);

  /**
   * @brief Creates JSON directly using a cJSON object
   *
   * This method allows more complex JSON structures than the map-based methods
   *
   * @param createFunc Function that populates a cJSON object
   * @return std::string The resulting JSON string
   */
  std::string createJsonWithBuilder(std::function<void(cJSON *)> createFunc);

  /**
   * @brief Parse a JSON string into a cJSON object
   *
   * @param json The JSON string to parse
   * @return cJSON* The parsed cJSON object (must be freed by caller with cJSON_Delete)
   */
  cJSON *parseToObject(const std::string &json);
};