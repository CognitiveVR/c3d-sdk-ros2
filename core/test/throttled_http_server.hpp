#pragma once

// A loopback HTTP/1.1 server that reads request bodies at a capped byte rate, or
// stops reading them altogether, so the real CurlTransport can be driven through
// a slow uplink and a dead one inside a unit test.
//
// The receive buffer is set SMALL on purpose, before listen() so every accepted
// socket inherits it. Loopback otherwise autotunes it to megabytes, which lets
// curl hand over a whole 1.5 MB body in one burst and then sit waiting for the
// response: the throttle would then be measuring the kernel, not the client.
//
// Answers every request 200 with the `cvr-request-time` header the SDK requires,
// sends `100 Continue` when asked, and serves several requests per connection,
// because a curl easy handle keeps its connection alive between POSTs.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <limits>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace c3d {
namespace testing_support {

class ThrottledHttpServer {
 public:
  struct Options {
    // Body read rate. 0 reads as fast as the socket delivers.
    double bytes_per_s = 0.0;
    // After this many body bytes of a request, stop reading and hold the
    // connection open without answering: a link that died mid-upload.
    std::size_t stall_after_bytes = std::numeric_limits<std::size_t>::max();
    int receive_buffer_bytes = 16 * 1024;
    // The whole response sent to every request, status line to body. Empty sends the 200 with
    // the `cvr-request-time` header the SDK requires.
    std::string response;
  };

  struct Received {
    std::string path;
    // The request line and header lines, each ending in CRLF as sent.
    std::string headers;
    std::string body;
    double seconds = 0.0;  // first body byte to last
  };

  explicit ThrottledHttpServer(Options options)
      : options_(options), bytes_per_s_(options.bytes_per_s) {
    listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    int one = 1;
    ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    int rcvbuf = options_.receive_buffer_bytes;
    ::setsockopt(listen_fd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;
    ::bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
    ::listen(listen_fd_, 16);
    socklen_t len = sizeof(addr);
    ::getsockname(listen_fd_, reinterpret_cast<sockaddr*>(&addr), &len);
    port_ = ntohs(addr.sin_port);
    acceptor_ = std::thread([this] { AcceptLoop(); });
  }

  ~ThrottledHttpServer() {
    stop_.store(true);
    if (acceptor_.joinable()) acceptor_.join();
    {
      std::lock_guard<std::mutex> lock(mu_);
      for (int fd : connection_fds_) ::shutdown(fd, SHUT_RDWR);
    }
    for (auto& t : connections_) {
      if (t.joinable()) t.join();
    }
    // Closed only here, after every Serve() has returned, so a descriptor number
    // is never shut down after the kernel has handed it to something else.
    for (int fd : connection_fds_) ::close(fd);
    ::close(listen_fd_);
  }

  ThrottledHttpServer(const ThrottledHttpServer&) = delete;
  ThrottledHttpServer& operator=(const ThrottledHttpServer&) = delete;

  // Changes the read rate for bodies read from now on; 0 lifts the throttle. A
  // test lifts it before teardown so the session's closing drain is quick.
  void SetBytesPerSecond(double rate) { bytes_per_s_.store(rate); }

  int port() const { return port_; }
  std::string host() const { return "127.0.0.1:" + std::to_string(port_); }

  std::vector<Received> received() const {
    std::lock_guard<std::mutex> lock(mu_);
    return received_;
  }
  std::size_t completed() const {
    std::lock_guard<std::mutex> lock(mu_);
    return received_.size();
  }
  // Requests now held without an answer, past `stall_after_bytes`.
  std::size_t stalled() const { return stalled_.load(); }

 private:
  void AcceptLoop() {
    while (!stop_.load()) {
      pollfd p{listen_fd_, POLLIN, 0};
      if (::poll(&p, 1, 20) <= 0) continue;
      const int fd = ::accept(listen_fd_, nullptr, nullptr);
      if (fd < 0) continue;
      std::lock_guard<std::mutex> lock(mu_);
      connection_fds_.push_back(fd);
      connections_.emplace_back([this, fd] { Serve(fd); });
    }
  }

  // Appends at most `max_chunk` bytes to `buffer`. False when the peer closed or
  // the server is stopping; polls so a stop is noticed while a client is idle.
  bool ReadMore(int fd, std::string* buffer, std::size_t max_chunk) {
    while (!stop_.load()) {
      pollfd p{fd, POLLIN, 0};
      const int ready = ::poll(&p, 1, 20);
      if (ready < 0) return false;
      if (ready == 0) continue;
      char tmp[4096];
      const std::size_t n = std::min(sizeof(tmp), max_chunk);
      const ssize_t got = ::recv(fd, tmp, n, 0);
      if (got <= 0) return false;
      buffer->append(tmp, static_cast<std::size_t>(got));
      return true;
    }
    return false;
  }

  static std::string Lower(std::string s) {
    for (auto& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
  }

  void Serve(int fd) {
    std::string pending;
    while (!stop_.load()) {
      // Headers.
      std::size_t header_end;
      while ((header_end = pending.find("\r\n\r\n")) == std::string::npos) {
        if (!ReadMore(fd, &pending, 4096)) {
          return;
        }
      }
      const std::string headers = pending.substr(0, header_end);
      pending.erase(0, header_end + 4);
      const std::string lower = Lower(headers);
      std::size_t content_length = 0;
      const auto cl = lower.find("content-length:");
      if (cl != std::string::npos) content_length = std::stoul(lower.substr(cl + 15));
      const auto first_space = headers.find(' ');
      const auto second_space = headers.find(' ', first_space + 1);
      Received request;
      request.path = headers.substr(first_space + 1, second_space - first_space - 1);
      request.headers = headers + "\r\n";
      if (lower.find("expect: 100-continue") != std::string::npos) {
        constexpr std::string_view go = "HTTP/1.1 100 Continue\r\n\r\n";
        ::send(fd, go.data(), go.size(), MSG_NOSIGNAL);
      }

      // Body, throttled.
      const auto start = std::chrono::steady_clock::now();
      std::string body = pending.substr(0, std::min(pending.size(), content_length));
      pending.erase(0, body.size());
      while (true) {
        // Checked before the completeness test: a small body arrives with its
        // headers, and would otherwise be answered without ever stalling.
        if (options_.stall_after_bytes < content_length &&
            body.size() >= options_.stall_after_bytes) {
          ++stalled_;
          while (!stop_.load()) std::this_thread::sleep_for(std::chrono::milliseconds(10));
          --stalled_;
          return;
        }
        if (body.size() >= content_length) break;
        std::size_t chunk = std::min<std::size_t>(content_length - body.size(), 4096);
        chunk = std::min(chunk, options_.stall_after_bytes - body.size());
        const double rate = bytes_per_s_.load();
        if (rate > 0.0) {
          const auto due = start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                       std::chrono::duration<double>(
                                           static_cast<double>(body.size()) / rate));
          std::this_thread::sleep_until(due);
        }
        std::string got;
        if (!ReadMore(fd, &got, chunk)) {
          return;
        }
        body += got;
      }
      request.seconds =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
      request.body = std::move(body);
      {
        std::lock_guard<std::mutex> lock(mu_);
        received_.push_back(std::move(request));
      }
      constexpr std::string_view ok =
          "HTTP/1.1 200 OK\r\ncvr-request-time: 1\r\nContent-Length: 2\r\n\r\nok";
      const std::string_view answer =
          options_.response.empty() ? ok : std::string_view(options_.response);
      ::send(fd, answer.data(), answer.size(), MSG_NOSIGNAL);
    }
  }

  Options options_;
  std::atomic<double> bytes_per_s_;
  int listen_fd_ = -1;
  int port_ = 0;
  std::atomic<bool> stop_{false};
  std::atomic<std::size_t> stalled_{0};
  std::thread acceptor_;
  mutable std::mutex mu_;
  std::vector<int> connection_fds_;
  std::vector<std::thread> connections_;
  std::vector<Received> received_;
};

}  // namespace testing_support
}  // namespace c3d
