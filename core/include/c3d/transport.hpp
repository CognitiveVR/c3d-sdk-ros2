#pragma once

#include <memory>
#include <string>
#include <vector>

#include "c3d/config.hpp"

namespace c3d {

struct HttpResponse {
  long status = 0;                    // 0 means the request never completed
  bool saw_request_time_header = false;
  // What a `Retry-After` response header asked for, in seconds from now, or negative when the
  // response carried none that parsed.
  double retry_after_s = -1.0;
  std::string body;
  std::string error;

  // Any 2xx the gateway stamped with the `cvr-request-time` response header. A 2xx
  // without it is a captive portal, not a success. The Unity SDK rewrites this case
  // to 307; we treat it as a retryable failure, which is the same decision by a
  // different name.
  bool IsSuccess() const { return status >= 200 && status < 300 && saw_request_time_header; }

  // The credentials were refused, by the collector (401, 403) or by a proxy on the way (407).
  // Nothing is wrong with the payload, and every other upload would meet the same answer.
  bool IsAuthenticationRejection() const {
    return status == 401 || status == 403 || status == 407;
  }

  // The payload itself was refused. The only answers that discard data, because replaying the
  // same bytes can never succeed.
  bool IsPayloadRejection() const {
    return status == 400 || status == 413 || status == 415 || status == 422;
  }

  // Everything else is kept and tried again after a back-off: no response at all, a captive
  // portal, a redirect (never followed: it would carry the key to the host it names), 404 (a
  // wrong route is a configuration problem, not a bad payload), 408, 429, every 5xx, and any
  // status not named above. Session decides which of these can count against the chunk.
  bool IsRetryable() const {
    return !IsSuccess() && !IsAuthenticationRejection() && !IsPayloadRejection();
  }
};

// A `Retry-After` value -- delay-seconds or an HTTP-date -- as seconds from `now_epoch_s`, or a
// negative number when it is neither. A date in the past is 0.
double RetryAfterSeconds(const std::string& value, double now_epoch_s);

class ITransport {
 public:
  virtual ~ITransport() = default;
  // Called only from the session's upload pump: its uploader thread, or a caller of
  // Session::PumpUploads. Blocking here is fine and expected; blocking on a recording
  // thread is not, which is why Record* never reaches this.
  virtual HttpResponse Post(const std::string& url,
                            const std::string& body,
                            const std::vector<std::string>& headers) = 0;

  // Abandon any request in flight and fail every later one at once. A Session
  // calls this as it shuts down, so joining its upload thread does not wait out a
  // slow upload that may legitimately run for minutes. Irreversible, and for
  // every user of this transport, so a transport must not be shared with anything
  // that outlives the session. The default does nothing, for a transport that
  // never blocks for long.
  virtual void Interrupt() {}
};

// Reads. Kept apart from ITransport on purpose: the session worker never reads, and a
// capability the worker does not have is one it cannot block on. A caller of Get runs on
// its own thread, with its own timeout, and owns the wait.
class IHttpGet {
 public:
  virtual ~IHttpGet() = default;
  virtual HttpResponse Get(const std::string& url,
                           const std::vector<std::string>& headers) = 0;
};

// How long a request may take, in the terms Config::http_* describes. A
// low_speed value of 0 turns stall detection off, which leaves only the ceiling.
struct HttpTimeouts {
  int connect_s = 10;
  int total_s = 10;
  long low_speed_bytes_per_s = 0;
  int low_speed_time_s = 0;
};

// The upload timeouts a Config asks for.
HttpTimeouts UploadTimeouts(const Config& config);

// libcurl-backed transport. One blocking easy handle, used from the single
// upload thread for Post; Get shares the handle and the timeouts, so a transport
// that reads must not be the one the uploader is posting through.
class CurlTransport : public ITransport, public IHttpGet {
 public:
  // One number for connect and for the whole request, and no stall detection:
  // the shape a short read wants, where the whole exchange is a few hundred bytes.
  explicit CurlTransport(int timeout_s = 10);
  explicit CurlTransport(const HttpTimeouts& timeouts);
  ~CurlTransport() override;
  HttpResponse Post(const std::string& url,
                    const std::string& body,
                    const std::vector<std::string>& headers) override;
  HttpResponse Get(const std::string& url, const std::vector<std::string>& headers) override;
  void Interrupt() override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace c3d
