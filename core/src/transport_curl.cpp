#include "c3d/transport.hpp"

#include <curl/curl.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>

#include "sigpipe_block.hpp"

namespace c3d {
namespace {

std::once_flag g_curl_init;

std::size_t WriteBody(char* ptr, std::size_t size, std::size_t nmemb, void* userdata) {
  auto* out = static_cast<std::string*>(userdata);
  out->append(ptr, size * nmemb);
  return size * nmemb;
}

std::string LowerCase(const std::string& text) {
  std::string lower;
  lower.reserve(text.size());
  for (char c : text) lower.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
  return lower;
}

std::size_t ReadHeader(char* buffer, std::size_t size, std::size_t nitems, void* userdata) {
  auto* response = static_cast<HttpResponse*>(userdata);
  const std::size_t total = size * nitems;
  const std::string line(buffer, total);
  const std::string lower = LowerCase(line);
  // A 200 that arrives without this header did not come from the Cognitive3D
  // gateway. It came from a captive portal or an intercepting proxy, both of
  // which are routine on a robot on site wifi.
  if (lower.rfind("cvr-request-time", 0) == 0) response->saw_request_time_header = true;
  static constexpr char kRetryAfter[] = "retry-after:";
  if (lower.rfind(kRetryAfter, 0) == 0) {
    std::string value = line.substr(sizeof(kRetryAfter) - 1);
    const auto first = value.find_first_not_of(" \t");
    const auto last = value.find_last_not_of(" \t\r\n");
    value = first == std::string::npos ? std::string() : value.substr(first, last - first + 1);
    response->retry_after_s = RetryAfterSeconds(
        value, std::chrono::duration<double>(
                   std::chrono::system_clock::now().time_since_epoch()).count());
  }
  return total;
}

// Days from 1970-01-01 to a proleptic Gregorian date.
std::int64_t DaysFromCivil(std::int64_t year, unsigned month, unsigned day) {
  year -= month <= 2 ? 1 : 0;
  const std::int64_t era = (year >= 0 ? year : year - 399) / 400;
  const unsigned year_of_era = static_cast<unsigned>(year - era * 400);
  const unsigned day_of_year = (153 * (month + (month > 2 ? -3 : 9)) + 2) / 5 + day - 1;
  const unsigned day_of_era = year_of_era * 365 + year_of_era / 4 - year_of_era / 100 + day_of_year;
  return era * 146097 + static_cast<std::int64_t>(day_of_era) - 719468;
}

bool ParseDigits(const std::string& text, std::size_t at, std::size_t count, int* out) {
  if (at + count > text.size()) return false;
  int value = 0;
  for (std::size_t i = at; i < at + count; ++i) {
    if (text[i] < '0' || text[i] > '9') return false;
    value = value * 10 + (text[i] - '0');
  }
  *out = value;
  return true;
}

// Called by libcurl through a transfer, at least once a second. Returning
// non-zero aborts it, which is how Interrupt() reaches a request already blocked
// inside curl_easy_perform.
int AbortIfInterrupted(void* userdata, curl_off_t, curl_off_t, curl_off_t, curl_off_t) {
  return static_cast<std::atomic<bool>*>(userdata)->load() ? 1 : 0;
}

}  // namespace

double RetryAfterSeconds(const std::string& value, double now_epoch_s) {
  if (value.empty()) return -1.0;
  if (value.find_first_not_of("0123456789") == std::string::npos) {
    if (value.size() > 9) return 1e9;
    return static_cast<double>(std::stoll(value));
  }
  // IMF-fixdate, the only HTTP-date form a sender may generate: "Sun, 06 Nov 1994 08:49:37 GMT".
  static constexpr const char* kMonths[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
                                            "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
  if (value.size() != 29 || value.compare(3, 2, ", ") != 0 || value.compare(25, 4, " GMT") != 0) {
    return -1.0;
  }
  int day = 0, year = 0, hour = 0, minute = 0, second = 0;
  unsigned month = 0;
  for (unsigned m = 0; m < 12; ++m) {
    if (value.compare(8, 3, kMonths[m]) == 0) month = m + 1;
  }
  if (month == 0 || !ParseDigits(value, 5, 2, &day) || !ParseDigits(value, 12, 4, &year) ||
      !ParseDigits(value, 17, 2, &hour) || !ParseDigits(value, 20, 2, &minute) ||
      !ParseDigits(value, 23, 2, &second) || value[7] != ' ' || value[11] != ' ' ||
      value[16] != ' ' || value[19] != ':' || value[22] != ':' || day < 1 || day > 31 ||
      hour > 23 || minute > 59 || second > 60) {
    return -1.0;
  }
  const double at = static_cast<double>(DaysFromCivil(year, month, static_cast<unsigned>(day))) *
                        86400.0 +
                    hour * 3600.0 + minute * 60.0 + second;
  return at > now_epoch_s ? at - now_epoch_s : 0.0;
}

HttpTimeouts UploadTimeouts(const Config& config) {
  HttpTimeouts t;
  t.connect_s = config.http_connect_timeout_s;
  t.total_s = config.http_timeout_s;
  t.low_speed_bytes_per_s = config.http_low_speed_limit_bytes_per_s;
  t.low_speed_time_s = config.http_low_speed_time_s;
  return t;
}

struct CurlTransport::Impl {
  CURL* handle = nullptr;
  HttpTimeouts timeouts;
  std::atomic<bool> interrupted{false};
  // curl's own account of a failure. curl_easy_strerror names only the class --
  // "Timeout was reached" for the connect timeout, the ceiling and the stall
  // detector alike -- and which of the three fired is the operator's first
  // question.
  char error[CURL_ERROR_SIZE] = {0};

  // Everything Get and Post share. The caller has already called
  // curl_easy_reset, which clears every option set here.
  void ApplyCommonOptions(HttpResponse* response) {
    curl_easy_setopt(handle, CURLOPT_TIMEOUT, static_cast<long>(timeouts.total_s));
    curl_easy_setopt(handle, CURLOPT_CONNECTTIMEOUT, static_cast<long>(timeouts.connect_s));
    if (timeouts.low_speed_bytes_per_s > 0 && timeouts.low_speed_time_s > 0) {
      curl_easy_setopt(handle, CURLOPT_LOW_SPEED_LIMIT, timeouts.low_speed_bytes_per_s);
      curl_easy_setopt(handle, CURLOPT_LOW_SPEED_TIME,
                       static_cast<long>(timeouts.low_speed_time_s));
    }
    error[0] = '\0';
    curl_easy_setopt(handle, CURLOPT_ERRORBUFFER, error);
    curl_easy_setopt(handle, CURLOPT_NOPROGRESS, 0L);
    curl_easy_setopt(handle, CURLOPT_XFERINFOFUNCTION, AbortIfInterrupted);
    curl_easy_setopt(handle, CURLOPT_XFERINFODATA, &interrupted);
    curl_easy_setopt(handle, CURLOPT_WRITEFUNCTION, WriteBody);
    curl_easy_setopt(handle, CURLOPT_WRITEDATA, &response->body);
    curl_easy_setopt(handle, CURLOPT_HEADERFUNCTION, ReadHeader);
    curl_easy_setopt(handle, CURLOPT_HEADERDATA, response);
    // NOSIGNAL also turns off libcurl's own SIGPIPE handling, which swaps the process-wide
    // disposition and is not safe beside other threads. libcurl 7's OpenSSL backend still writes
    // to the socket through OpenSSL's own calls, which raise SIGPIPE on a connection the peer has
    // reset, so every call that can write to a socket -- curl_easy_perform and curl_easy_cleanup --
    // runs under a ScopedSigpipeBlock instead.
    curl_easy_setopt(handle, CURLOPT_NOSIGNAL, 1L);
    // No redirect following: a redirect would re-send the Authorization header to
    // whatever host the response names.
    curl_easy_setopt(handle, CURLOPT_FOLLOWLOCATION, 0L);
    curl_easy_setopt(handle, CURLOPT_ACCEPT_ENCODING, "");
  }

  void Perform(HttpResponse* response) {
    if (interrupted.load()) {
      response->status = 0;
      response->error = "transport interrupted";
      return;
    }
    const ScopedSigpipeBlock sigpipe_block;
    const CURLcode rc = curl_easy_perform(handle);
    if (rc != CURLE_OK) {
      response->status = 0;
      response->error = error[0] != '\0' ? std::string(error) : curl_easy_strerror(rc);
    } else {
      long code = 0;
      curl_easy_getinfo(handle, CURLINFO_RESPONSE_CODE, &code);
      response->status = code;
    }
  }
};

CurlTransport::CurlTransport(int timeout_s)
    : CurlTransport(HttpTimeouts{timeout_s, timeout_s, 0, 0}) {}

CurlTransport::CurlTransport(const HttpTimeouts& timeouts) : impl_(new Impl) {
  std::call_once(g_curl_init, []() { curl_global_init(CURL_GLOBAL_DEFAULT); });
  impl_->handle = curl_easy_init();
  impl_->timeouts = timeouts;
}

void CurlTransport::Interrupt() { impl_->interrupted.store(true); }

CurlTransport::~CurlTransport() {
  if (impl_ && impl_->handle) {
    const ScopedSigpipeBlock sigpipe_block;
    curl_easy_cleanup(impl_->handle);
  }
}

HttpResponse CurlTransport::Get(const std::string& url,
                                const std::vector<std::string>& headers) {
  HttpResponse response;
  if (!impl_->handle) {
    response.error = "curl handle not initialised";
    return response;
  }

  struct curl_slist* header_list = nullptr;
  for (const auto& h : headers) header_list = curl_slist_append(header_list, h.c_str());

  curl_easy_reset(impl_->handle);
  curl_easy_setopt(impl_->handle, CURLOPT_URL, url.c_str());
  curl_easy_setopt(impl_->handle, CURLOPT_HTTPGET, 1L);
  curl_easy_setopt(impl_->handle, CURLOPT_HTTPHEADER, header_list);
  impl_->ApplyCommonOptions(&response);
  impl_->Perform(&response);

  if (header_list) curl_slist_free_all(header_list);
  return response;
}

HttpResponse CurlTransport::Post(const std::string& url, const std::string& body,
                                 const std::vector<std::string>& headers) {
  HttpResponse response;
  if (!impl_->handle) {
    response.error = "curl handle not initialised";
    return response;
  }

  struct curl_slist* header_list = nullptr;
  for (const auto& h : headers) header_list = curl_slist_append(header_list, h.c_str());

  curl_easy_reset(impl_->handle);
  curl_easy_setopt(impl_->handle, CURLOPT_URL, url.c_str());
  curl_easy_setopt(impl_->handle, CURLOPT_POST, 1L);
  curl_easy_setopt(impl_->handle, CURLOPT_POSTFIELDS, body.c_str());
  curl_easy_setopt(impl_->handle, CURLOPT_POSTFIELDSIZE, static_cast<long>(body.size()));
  curl_easy_setopt(impl_->handle, CURLOPT_HTTPHEADER, header_list);
  impl_->ApplyCommonOptions(&response);
  impl_->Perform(&response);

  if (header_list) curl_slist_free_all(header_list);
  return response;
}

}  // namespace c3d
