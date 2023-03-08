#pragma once

#include <future>
#include <optional>
#include <regex>
#include <string>

#include <httplib.h>
#include <nlohmann/json.hpp>

namespace fetchjson {
inline std::pair<std::string, std::string> split_url(const std::string &url) {
  static std::regex exp{R"(^(http[s]?:\/\/.+?)(\/.*|)$)"};
  std::smatch match_result;
  std::regex_match(url, match_result, exp);
  std::string host = match_result.str(1);
  std::string path = match_result.str(2).empty() ? "/" : match_result.str(2);
  return std::pair{host, path};
}

inline std::future<std::optional<nlohmann::json>> get(const std::string &url, const nlohmann::json &headers = {}) {
  return std::async(std::launch::async, [url, headers]() -> std::optional<nlohmann::json> {
    auto [host, path] = split_url(url);
    httplib::Client client{host};
    httplib::Result response = client.Get(path, headers.get<httplib::Headers>());
    if (response.error() != httplib::Error::Success) {
      return std::nullopt;
    }

    std::optional<nlohmann::json> result;
    try {
      result = nlohmann::json::parse(response->body);
    } catch (...) {
      result = std::nullopt;
    }
    return result;
  });
}

template <class... Args>
inline std::future<std::optional<nlohmann::json>> post(const std::string &url, const nlohmann::json &body,
                                                       const nlohmann::json &headers = {}) {
  return std::async(std::launch::async, [url, headers, body]() -> std::optional<nlohmann::json> {
    auto [host, path] = split_url(url);
    httplib::Client client{host};
    httplib::Result response = client.Post(path, headers, body.dump(), "application/json");
    if (response.error() != httplib::Error::Success) {
      return std::nullopt;
    }

    std::optional<nlohmann::json> result;
    try {
      result = nlohmann::json::parse(response->body);
    } catch (...) {
      result = std::nullopt;
    }
    return result;
  });
}
} // namespace fetchjson
