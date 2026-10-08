#pragma once
#include <chrono>
#include <condition_variable>
#include <functional>
#include <optional>
#include <mutex>
#include <thread>
#include <string>
#include <vector>

#include "c3d/transport.hpp"

namespace c3d {

// Programmable transport. Records every request and replays a scripted sequence
// of responses, so the retry, backoff and discard paths are testable without a
// network.
class FakeTransport : public ITransport, public IHttpGet {
 public:
  struct Request {
    std::string url;
    std::string body;
    std::vector<std::string> headers;
  };

  HttpResponse Post(const std::string& url, const std::string& body,
                    const std::vector<std::string>& headers) override {
    std::unique_lock<std::mutex> lock(mu_);
    requests_.push_back(Request{url, body, headers});
    if (post_delay_.count() > 0) {
      // Released while sleeping: a test observing the SDK mid-pump must be able
      // to read requests() without blocking behind this.
      lock.unlock();
      std::this_thread::sleep_for(post_delay_);
      lock.lock();
    }
    if (stalled_) {
      ++in_flight_;
      stall_cv_.notify_all();
      stall_cv_.wait(lock, [this] { return !stalled_; });
      --in_flight_;
    }
    if (offline_) {
      HttpResponse r;
      r.status = 0;
      r.error = "simulated network down";
      return r;
    }
    if (responder_) {
      if (const auto answer = responder_(requests_.back())) return *answer;
    }
    if (!scripted_.empty()) {
      HttpResponse r = scripted_.front();
      scripted_.erase(scripted_.begin());
      return r;
    }
    HttpResponse ok;
    ok.status = 200;
    ok.saw_request_time_header = true;
    return ok;
  }

  HttpResponse Get(const std::string& url, const std::vector<std::string>& headers) override {
    std::lock_guard<std::mutex> lock(mu_);
    gets_.push_back(Request{url, "", headers});
    if (offline_) {
      HttpResponse r;
      r.status = 0;
      r.error = "simulated network down";
      return r;
    }
    if (!scripted_gets_.empty()) {
      HttpResponse r = scripted_gets_.front();
      scripted_gets_.erase(scripted_gets_.begin());
      return r;
    }
    HttpResponse ok;
    ok.status = 200;
    ok.saw_request_time_header = true;
    ok.body = "{}";
    return ok;
  }

  // Holds every POST from now on, once it is recorded, until Unstall or Interrupt: an upload
  // stuck on a dead link, so a test can show what does not wait for one.
  void Stall() {
    std::lock_guard<std::mutex> lock(mu_);
    stalled_ = true;
  }
  void Unstall() {
    std::lock_guard<std::mutex> lock(mu_);
    stalled_ = false;
    stall_cv_.notify_all();
  }
  // Waits until a POST is held by Stall, or `timeout` passes.
  bool WaitUntilStalled(std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mu_);
    return stall_cv_.wait_for(lock, timeout, [this] { return in_flight_ > 0; });
  }
  std::size_t stalled_posts() const {
    std::lock_guard<std::mutex> lock(mu_);
    return in_flight_;
  }
  // A session interrupts its transport as it shuts down; a held POST is let go.
  void Interrupt() override { Unstall(); }

  void SetOffline(bool offline) {
    std::lock_guard<std::mutex> lock(mu_);
    offline_ = offline;
  }
  // Makes each POST take a measurable amount of time, so a test can observe what
  // the SDK does WHILE it is uploading rather than only after. The heartbeat
  // starvation this exists to test is invisible to an instant transport.
  void SetPostDelay(std::chrono::milliseconds delay) {
    std::lock_guard<std::mutex> lock(mu_);
    post_delay_ = delay;
  }
  // Consulted before the script, for every POST: an answer chosen by the request itself, such
  // as a gateway that fails one particular payload every time. No answer falls through.
  void SetResponder(std::function<std::optional<HttpResponse>(const Request&)> responder) {
    std::lock_guard<std::mutex> lock(mu_);
    responder_ = std::move(responder);
  }
  void Script(HttpResponse r) {
    std::lock_guard<std::mutex> lock(mu_);
    scripted_.push_back(std::move(r));
  }
  void ScriptGet(HttpResponse r) {
    std::lock_guard<std::mutex> lock(mu_);
    scripted_gets_.push_back(std::move(r));
  }
  std::vector<Request> gets() const {
    std::lock_guard<std::mutex> lock(mu_);
    return gets_;
  }
  std::vector<Request> requests() const {
    std::lock_guard<std::mutex> lock(mu_);
    return requests_;
  }
  std::size_t count() const {
    std::lock_guard<std::mutex> lock(mu_);
    return requests_.size();
  }

 private:
  mutable std::mutex mu_;
  std::vector<Request> requests_;
  std::vector<HttpResponse> scripted_;
  std::vector<Request> gets_;
  std::vector<HttpResponse> scripted_gets_;
  bool offline_ = false;
  bool stalled_ = false;
  std::size_t in_flight_ = 0;
  std::condition_variable stall_cv_;
  std::chrono::milliseconds post_delay_{0};
  std::function<std::optional<HttpResponse>(const Request&)> responder_;
};

}  // namespace c3d
