// The real CurlTransport against a loopback server that reads slowly, stops
// reading, or trickles. An upload is judged by whether it is moving: a slow one
// that keeps moving must be allowed to finish, a dead one must be abandoned, and a
// trickle that never quite stalls must still meet the ceiling. And against a peer
// that resets the connection mid-handshake, which must fail the request, not the
// process.

#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <thread>

#include "c3d/config.hpp"
#include "c3d/transport.hpp"
#include "throttled_http_server.hpp"

using namespace c3d;
using c3d::testing_support::ThrottledHttpServer;

namespace {

std::string Body(std::size_t n) {
  std::mt19937 rng(11);
  std::string s(n, '\0');
  for (auto& c : s) c = static_cast<char>(rng() & 0xFF);
  return s;
}

std::string UrlFor(const ThrottledHttpServer& server) {
  return "http://" + server.host() + "/v0/camera";
}

double SecondsSince(std::chrono::steady_clock::time_point t) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - t).count();
}

bool RobotScale() {
  const char* v = std::getenv("C3D_STALL_REPRO_ROBOT");
  return v != nullptr && std::string(v) == "1";
}

}  // namespace

// A robot's case, with the shipped defaults: one 1.5 MB camera part over a link
// that needs well over ten seconds to carry it. A fixed ten-second request
// timeout fails this part on every attempt.
TEST(CurlTransportTimeouts, ASlowButSteadyUploadIsAllowedToFinishUnderTheDefaults) {
  ThrottledHttpServer::Options options;
  options.bytes_per_s = 120e3;
  ThrottledHttpServer server(options);
  CurlTransport transport(UploadTimeouts(Config{}));

  const std::string body = Body(1500 * 1000);
  const auto start = std::chrono::steady_clock::now();
  const HttpResponse response = transport.Post(UrlFor(server), body, {});
  const double took = SecondsSince(start);

  EXPECT_TRUE(response.IsSuccess()) << "status " << response.status << ": " << response.error;
  ASSERT_EQ(server.completed(), 1u);
  EXPECT_EQ(server.received()[0].body.size(), body.size());
  // Proof the link really was slower than a fixed ten-second timeout allows, or
  // the success above proves nothing.
  EXPECT_GT(took, 10.0) << "the throttle did not hold; this upload was never slow";
}

// One number for the connect and for the whole request: the reader's shape, and
// shown here failing the same part on the same link, which is why uploads use
// stall detection instead. Slow, so it runs with the robot-scale reproduction
// only.
TEST(CurlTransportTimeouts, TheSingleFixedTimeoutFailsThatSameUpload) {
  if (!RobotScale()) GTEST_SKIP() << "set C3D_STALL_REPRO_ROBOT=1 to run";
  ThrottledHttpServer::Options options;
  options.bytes_per_s = 120e3;
  ThrottledHttpServer server(options);
  CurlTransport transport(10);

  const HttpResponse response = transport.Post(UrlFor(server), Body(1500 * 1000), {});
  EXPECT_FALSE(response.IsSuccess());
  EXPECT_EQ(response.status, 0);
  std::cout << "[single fixed 10 s timeout] " << response.error << "\n";
}

TEST(CurlTransportTimeouts, AStalledUploadIsAbandonedByTheStallDetector) {
  ThrottledHttpServer::Options options;
  options.stall_after_bytes = 64 * 1024;  // the link dies mid-part and never answers
  ThrottledHttpServer server(options);
  HttpTimeouts timeouts;
  timeouts.connect_s = 2;
  timeouts.total_s = 60;
  timeouts.low_speed_bytes_per_s = 1024;
  timeouts.low_speed_time_s = 2;
  CurlTransport transport(timeouts);

  const auto start = std::chrono::steady_clock::now();
  const HttpResponse response = transport.Post(UrlFor(server), Body(1500 * 1000), {});
  const double took = SecondsSince(start);

  EXPECT_FALSE(response.IsSuccess());
  EXPECT_EQ(response.status, 0);
  EXPECT_TRUE(response.IsRetryable()) << "a stall is the network's fault, never the chunk's";
  EXPECT_NE(response.error.find("too slow"), std::string::npos)
      << "the error must say it was the stall detector, not the ceiling: " << response.error;
  EXPECT_GE(took, 2.0);
  // Well inside the 60 s ceiling: the detector, not the ceiling, ended it.
  EXPECT_LT(took, 20.0) << "a dead link was held for " << took << " s";
}

TEST(CurlTransportTimeouts, ATrickleThatNeverQuiteStallsStillMeetsTheCeiling) {
  ThrottledHttpServer::Options options;
  options.bytes_per_s = 8e3;  // above the stall limit below, forever
  ThrottledHttpServer server(options);
  HttpTimeouts timeouts;
  timeouts.connect_s = 2;
  timeouts.total_s = 4;
  timeouts.low_speed_bytes_per_s = 1024;
  timeouts.low_speed_time_s = 3;
  CurlTransport transport(timeouts);

  const auto start = std::chrono::steady_clock::now();
  const HttpResponse response = transport.Post(UrlFor(server), Body(1500 * 1000), {});
  const double took = SecondsSince(start);

  EXPECT_FALSE(response.IsSuccess());
  EXPECT_NE(response.error.find("timed out"), std::string::npos) << response.error;
  EXPECT_GE(took, 3.5);
  EXPECT_LT(took, 8.0);
}

TEST(CurlTransportTimeouts, InterruptAbandonsARequestInFlightAndRefusesTheNext) {
  ThrottledHttpServer::Options options;
  options.stall_after_bytes = 64 * 1024;
  ThrottledHttpServer server(options);
  HttpTimeouts timeouts;
  timeouts.connect_s = 2;
  timeouts.total_s = 120;
  CurlTransport transport(timeouts);  // no stall detection: only Interrupt can end it early

  std::thread interrupter([&] {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    transport.Interrupt();
  });
  const auto start = std::chrono::steady_clock::now();
  const HttpResponse response = transport.Post(UrlFor(server), Body(1500 * 1000), {});
  const double took = SecondsSince(start);
  interrupter.join();

  EXPECT_FALSE(response.IsSuccess());
  EXPECT_LT(took, 5.0) << "Interrupt did not reach a request blocked inside curl";

  const auto again = std::chrono::steady_clock::now();
  const HttpResponse next = transport.Post(UrlFor(server), "x", {});
  EXPECT_FALSE(next.IsSuccess());
  EXPECT_LT(SecondsSince(again), 0.5);
  EXPECT_EQ(server.completed(), 0u);
}

TEST(UploadTimeoutsConfig, TheDefaultsValidateAndReachTheTransport) {
  Config cfg;
  cfg.application_key = "K";
  cfg.scene_id = "S";
  cfg.scene_version = "1";
  cfg.device_id = "d";
  cfg.spool_dir = "/tmp/x";
  ASSERT_EQ(cfg.Validate(), "");
  cfg.http_connect_timeout_s = 7;
  cfg.http_timeout_s = 123;
  cfg.http_low_speed_limit_bytes_per_s = 2048;
  cfg.http_low_speed_time_s = 45;
  const HttpTimeouts t = UploadTimeouts(cfg);
  EXPECT_EQ(t.connect_s, 7);
  EXPECT_EQ(t.total_s, 123);
  EXPECT_EQ(t.low_speed_bytes_per_s, 2048);
  EXPECT_EQ(t.low_speed_time_s, 45);
}

TEST(UploadTimeoutsConfig, EveryBoundIsRefusedWhenItWouldSilentlyDisappear) {
  auto base = [] {
    Config cfg;
    cfg.application_key = "K";
    cfg.scene_id = "S";
    cfg.scene_version = "1";
    cfg.device_id = "d";
    cfg.spool_dir = "/tmp/x";
    return cfg;
  };
  auto refuses = [](Config cfg, const std::string& names) {
    const std::string why = cfg.Validate();
    EXPECT_NE(why.find(names), std::string::npos) << "expected a refusal naming " << names
                                                  << ", got '" << why << "'";
  };
  {
    Config c = base();
    c.http_connect_timeout_s = 0;
    refuses(c, "http_connect_timeout_s");
  }
  {
    Config c = base();
    c.http_connect_timeout_s = 400;  // past the 300 s ceiling
    refuses(c, "http_connect_timeout_s");
  }
  {
    Config c = base();
    c.http_low_speed_limit_bytes_per_s = 0;
    refuses(c, "http_low_speed_limit_bytes_per_s");
  }
  {
    Config c = base();
    c.http_low_speed_time_s = 0;
    refuses(c, "http_low_speed_time_s");
  }
  {
    Config c = base();
    c.http_low_speed_time_s = 300;  // equal to the ceiling: can never fire first
    refuses(c, "http_low_speed_time_s");
  }
  {
    Config c = base();
    c.http_timeout_s = 0;
    refuses(c, "http_timeout_s");
  }
  {
    Config c = base();
    c.retry_min_s = 0.0;
    refuses(c, "retry window");
  }
  {
    Config c = base();
    c.retry_min_s = 100.0;
    c.retry_max_s = 50.0;
    refuses(c, "retry window");
  }
  {
    Config c = base();
    c.retry_max_s = std::numeric_limits<double>::infinity();
    refuses(c, "retry_min_s and retry_max_s");
  }
}

// ---------------------------------------------------------------------------
// What an answer means for the data, and what the server asked for
// ---------------------------------------------------------------------------

TEST(HttpResponsePolicy, OnlyAPayloadRejectionDiscardsAndOnlyCredentialsPause) {
  auto with = [](long status) {
    HttpResponse r;
    r.status = status;
    r.saw_request_time_header = true;
    return r;
  };
  for (long status : {400L, 413L, 415L, 422L}) {
    EXPECT_TRUE(with(status).IsPayloadRejection()) << status;
    EXPECT_FALSE(with(status).IsRetryable()) << status;
    EXPECT_FALSE(with(status).IsAuthenticationRejection()) << status;
  }
  for (long status : {401L, 403L, 407L}) {
    EXPECT_TRUE(with(status).IsAuthenticationRejection()) << status;
    EXPECT_FALSE(with(status).IsRetryable()) << status;
    EXPECT_FALSE(with(status).IsPayloadRejection()) << status;
  }
  // Kept and retried: no answer, every redirect, a wrong route, a timeout, a rate limit, every
  // server error, and statuses nobody expects.
  for (long status : {0L, 199L, 300L, 301L, 302L, 303L, 307L, 308L, 404L, 405L, 408L, 409L, 410L,
                      425L, 429L, 500L, 502L, 503L, 504L, 599L}) {
    EXPECT_TRUE(with(status).IsRetryable()) << status;
    EXPECT_FALSE(with(status).IsSuccess()) << status;
    EXPECT_FALSE(with(status).IsPayloadRejection()) << status;
    EXPECT_FALSE(with(status).IsAuthenticationRejection()) << status;
  }
  // Any 2xx the gateway stamped is delivered; the same status unstamped is a captive portal.
  for (long status : {200L, 201L, 202L, 204L, 299L}) {
    EXPECT_TRUE(with(status).IsSuccess()) << status;
    EXPECT_FALSE(with(status).IsRetryable()) << status;
    HttpResponse portal;
    portal.status = status;
    EXPECT_FALSE(portal.IsSuccess()) << status;
    EXPECT_TRUE(portal.IsRetryable()) << status;
  }
}

TEST(RetryAfter, ReadsDelaySecondsAndAnImfFixdate) {
  const double now = 1790856000.0;  // 2026-10-01T12:00:00Z
  EXPECT_EQ(RetryAfterSeconds("0", now), 0.0);
  EXPECT_EQ(RetryAfterSeconds("120", now), 120.0);
  EXPECT_EQ(RetryAfterSeconds("Thu, 01 Oct 2026 12:00:30 GMT", now), 30.0);
  EXPECT_EQ(RetryAfterSeconds("Thu, 01 Oct 2026 11:00:00 GMT", now), 0.0) << "a date in the past";
  EXPECT_EQ(RetryAfterSeconds("Sun, 06 Nov 1994 08:49:37 GMT", 784111777.0 - 7.0), 7.0);
  EXPECT_EQ(RetryAfterSeconds("Tue, 29 Feb 2000 23:59:59 GMT", 951868799.0 - 1.0), 1.0);
  EXPECT_GE(RetryAfterSeconds("99999999999999999999", now), 1e9) << "huge, not wrapped";
  for (const char* junk : {"", "soon", "-5", "1.5", "Thu, 01 Oct 2026 12:00:30 UTC",
                           "Thu, 01 Xyz 2026 12:00:30 GMT", "Thu, 32 Oct 2026 12:00:30 GMT"}) {
    EXPECT_LT(RetryAfterSeconds(junk, now), 0.0) << "'" << junk << "'";
  }
}

TEST(RetryAfter, TheTransportReadsTheHeaderOffTheWire) {
  ThrottledHttpServer::Options options;
  options.response =
      "HTTP/1.1 429 Too Many Requests\r\nRetry-After:  17 \r\nContent-Length: 0\r\n\r\n";
  ThrottledHttpServer server(options);
  CurlTransport transport(UploadTimeouts(Config{}));
  const HttpResponse response = transport.Post(UrlFor(server), "{}", {});
  EXPECT_EQ(response.status, 429);
  EXPECT_EQ(response.retry_after_s, 17.0);

  ThrottledHttpServer::Options plain;
  plain.response = "HTTP/1.1 503 Service Unavailable\r\nContent-Length: 0\r\n\r\n";
  ThrottledHttpServer without(plain);
  EXPECT_LT(transport.Post(UrlFor(without), "{}", {}).retry_after_s, 0.0);
}

// ---------------------------------------------------------------------------
// A peer that resets the connection in the middle of the TLS handshake
// ---------------------------------------------------------------------------

// The peer reads the head of the ClientHello and resets the connection with the rest unread, as a
// middlebox or a dying server can. Where libcurl's TLS backend writes through OpenSSL's own socket
// calls -- libcurl 7, as on Ubuntu 22.04 -- the client's next write to that socket raises SIGPIPE,
// whose default action ends the process: the node's, or the integrator's container it is loaded
// into. The request must fail, retryably, and the process must live. The transfer runs in a child,
// with SIGPIPE at its default action whatever the suite was started with, so a death is a result
// rather than the end of the suite.
TEST(CurlTransportSigpipe, AnHttpsPeerResettingMidHandshakeFailsTheRequestAndNotTheProcess) {
  const int listener = ::socket(AF_INET, SOCK_STREAM, 0);
  ASSERT_GE(listener, 0);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  ASSERT_EQ(::bind(listener, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)), 0);
  ASSERT_EQ(::listen(listener, 4), 0);
  socklen_t length = sizeof(addr);
  ::getsockname(listener, reinterpret_cast<sockaddr*>(&addr), &length);
  const std::string url = "https://127.0.0.1:" + std::to_string(ntohs(addr.sin_port)) + "/v0/x";

  std::string head;
  std::thread peer([listener, &head] {
    pollfd ready{listener, POLLIN, 0};
    if (::poll(&ready, 1, 10000) != 1) return;
    const int fd = ::accept(listener, nullptr, nullptr);
    if (fd < 0) return;
    timeval wait{5, 0};
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &wait, sizeof(wait));
    char buf[5];
    const ssize_t got = ::recv(fd, buf, sizeof(buf), 0);
    if (got > 0) head.assign(buf, static_cast<std::size_t>(got));
    linger reset{1, 0};
    ::setsockopt(fd, SOL_SOCKET, SO_LINGER, &reset, sizeof(reset));
    ::close(fd);
  });

  int said_fds[2];
  ASSERT_EQ(::pipe(said_fds), 0);
  const pid_t child = ::fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    ::signal(SIGPIPE, SIG_DFL);
    sigset_t none;
    sigemptyset(&none);
    ::pthread_sigmask(SIG_SETMASK, &none, nullptr);
    ::close(said_fds[0]);
    HttpResponse response;
    {
      CurlTransport transport(5);
      response = transport.Get(url, {});
    }
    const std::string said = "status " + std::to_string(response.status) + ": " + response.error;
    (void)!::write(said_fds[1], said.data(), said.size());
    ::_exit(response.status == 0 && !response.error.empty() && response.IsRetryable() ? 0 : 1);
  }
  ::close(said_fds[1]);
  std::string said;
  char buf[256];
  ssize_t n;
  while ((n = ::read(said_fds[0], buf, sizeof(buf))) > 0) {
    said.append(buf, static_cast<std::size_t>(n));
  }
  ::close(said_fds[0]);
  int status = 0;
  ::waitpid(child, &status, 0);
  peer.join();
  ::close(listener);

  ASSERT_FALSE(WIFSIGNALED(status)) << "the process was killed by signal " << WTERMSIG(status);
  EXPECT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0)
      << "the request did not fail retryably: " << said;
  ASSERT_FALSE(head.empty()) << "the client never reached the handshake: " << said;
  EXPECT_EQ(head[0], '\x16') << "the client's first bytes were not a TLS handshake record";
}
