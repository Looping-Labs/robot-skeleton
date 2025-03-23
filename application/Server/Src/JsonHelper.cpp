#include "JsonHelper.h"
#include "esp_log.h"
#include <cstring>
#include <iomanip>
#include <sstream>

static const char *TAG = "JsonHelper";

esp_err_t JsonHelper::parse(const std::string &json, std::map<std::string, std::string> &output) {
  output.clear();

  // Parse the JSON string
  cJSON *root = cJSON_Parse(json.c_str());
  if (root == nullptr) {
    const char *error_ptr = cJSON_GetErrorPtr();
    if (error_ptr != nullptr) {
      ESP_LOGE(TAG, "JSON parsing error: %s", error_ptr);
    }
    return ESP_FAIL;
  }

  // Iterate through all object items
  cJSON *item = nullptr;
  cJSON_ArrayForEach(item, root) {
    if (cJSON_IsString(item)) {
      output[item->string] = item->valuestring;
    } else if (cJSON_IsNumber(item)) {
      // Convert number to string
      output[item->string] = toString(item->valuedouble);
    } else if (cJSON_IsBool(item)) {
      output[item->string] = item->valueint ? "true" : "false";
    } else if (cJSON_IsNull(item)) {
      output[item->string] = "null";
    }
    // Skip arrays and objects for the simple key-value interface
  }

  // Clean up
  cJSON_Delete(root);

  return ESP_OK;
}

std::string JsonHelper::createJson(const std::map<std::string, std::string> &data) {
  cJSON *root = cJSON_CreateObject();

  for (const auto &kv : data) {
    // Try to determine the type of the value
    if (kv.second == "true") {
      cJSON_AddBoolToObject(root, kv.first.c_str(), true);
    } else if (kv.second == "false") {
      cJSON_AddBoolToObject(root, kv.first.c_str(), false);
    } else if (kv.second == "null") {
      cJSON_AddNullToObject(root, kv.first.c_str());
    } else {
      // Try to parse as number
      char *endptr = nullptr;
      double value = std::strtod(kv.second.c_str(), &endptr);

      // If the entire string was parsed as a number and it's not empty
      if (*endptr == '\0' && kv.second.length() > 0) {
        cJSON_AddNumberToObject(root, kv.first.c_str(), value);
      } else {
        // Treat as string
        cJSON_AddStringToObject(root, kv.first.c_str(), kv.second.c_str());
      }
    }
  }

  // Convert to string
  char *json_str = cJSON_Print(root);
  std::string result = json_str;

  // Clean up
  cJSON_free(json_str);
  cJSON_Delete(root);

  return result;
}

std::string JsonHelper::createSimpleJson(const std::string &key, const std::string &value) {
  cJSON *root = cJSON_CreateObject();

  // Add the key-value pair
  cJSON_AddStringToObject(root, key.c_str(), value.c_str());

  // Convert to string
  char *json_str = cJSON_Print(root);
  std::string result = json_str;

  // Clean up
  cJSON_free(json_str);
  cJSON_Delete(root);

  return result;
}

std::string JsonHelper::createJsonArray(const std::string &key, const std::vector<std::string> &values) {
  cJSON *root = cJSON_CreateObject();
  cJSON *array = cJSON_AddArrayToObject(root, key.c_str());

  for (const auto &value : values) {
    // Try to determine the type of the value
    if (value == "true") {
      cJSON_AddItemToArray(array, cJSON_CreateBool(true));
    } else if (value == "false") {
      cJSON_AddItemToArray(array, cJSON_CreateBool(false));
    } else if (value == "null") {
      cJSON_AddItemToArray(array, cJSON_CreateNull());
    } else {
      // Try to parse as number
      char *endptr = nullptr;
      double number = std::strtod(value.c_str(), &endptr);

      // If the entire string was parsed as a number and it's not empty
      if (*endptr == '\0' && value.length() > 0) {
        cJSON_AddItemToArray(array, cJSON_CreateNumber(number));
      } else {
        // Treat as string
        cJSON_AddItemToArray(array, cJSON_CreateString(value.c_str()));
      }
    }
  }

  // Convert to string
  char *json_str = cJSON_Print(root);
  std::string result = json_str;

  // Clean up
  cJSON_free(json_str);
  cJSON_Delete(root);

  return result;
}

std::string JsonHelper::getValue(const std::map<std::string, std::string> &data,
                                 const std::string &key,
                                 const std::string &defaultVal) {
  auto it = data.find(key);
  if (it != data.end()) {
    return it->second;
  }
  return defaultVal;
}

template <typename T>
std::string JsonHelper::toString(T value) {
  std::stringstream ss;
  ss << value;
  return ss.str();
}

std::string JsonHelper::createJsonWithBuilder(std::function<void(cJSON *)> createFunc) {
  cJSON *root = cJSON_CreateObject();

  // Call the provided function to build the JSON object
  createFunc(root);

  // Convert to string
  char *json_str = cJSON_Print(root);
  std::string result = json_str;

  // Clean up
  cJSON_free(json_str);
  cJSON_Delete(root);

  return result;
}

cJSON *JsonHelper::parseToObject(const std::string &json) {
  return cJSON_Parse(json.c_str());
}

// Explicit template instantiations for common types
template std::string JsonHelper::toString<int>(int value);
template std::string JsonHelper::toString<float>(float value);
template std::string JsonHelper::toString<double>(double value);
template std::string JsonHelper::toString<long>(long value);