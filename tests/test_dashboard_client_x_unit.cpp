// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2022 Universal Robots A/S
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
// -- END LICENSE BLOCK ------------------------------------------------

#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <future>
#include <iterator>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <urcl_3rdparty/httplib/httplib.h>
#include <urcl_3rdparty/nlohmann_json/json.hpp>

#include <ur_client_library/exceptions.h>
#include <ur_client_library/ur/dashboard_client_implementation.h>
#include <ur_client_library/ur/dashboard_client_implementation_x.h>

using json = nlohmann::json;

using namespace urcl;

// ---------------------------------------------------------------------------
// Mock tests: httplib::Server replaces a real robot so the actual
// DashboardClientImplX code (endpoint routing, streaming download,
// response parsing, temp-file lifecycle) is exercised without hardware.
// Response bodies follow the OpenAPI spec at /universal-robots/robot-api/openapi.json.
// ---------------------------------------------------------------------------

// Minimal OpenAPI response: version 5.0.107 satisfies the minimum required by
// both generate_flight_report and download_support_files.
static constexpr const char* MOCK_OPENAPI_RESPONSE = R"({"info":{"version":"5.0.107"}})";
// POST /supportfiles/v1  → Generate Flight Report
// GET  /supportfiles/v1/ → Download Support Files (zip)
static constexpr const char* MOCK_SUPPORTFILES_ENDPOINT = "/universal-robots/robot-api/supportfiles/v1";
static constexpr const char* MOCK_OPENAPI_ENDPOINT = "/universal-robots/robot-api/openapi.json";

// Thin subclass used only in tests to expose the protected is_connected_ field.
class TestableDashboardClientMock : public DashboardClientImplX
{
public:
  using DashboardClientImplX::DashboardClientImplX;
  bool isConnected() const
  {
    return is_connected_;
  }
};

class DashboardClientImplXMockTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    server_.Get(MOCK_OPENAPI_ENDPOINT, [](const httplib::Request&, httplib::Response& res) {
      res.set_content(MOCK_OPENAPI_RESPONSE, "application/json");
    });

    port_ = server_.bind_to_any_port("127.0.0.1");
    server_thread_ = std::thread([this]() { server_.listen_after_bind(); });

    while (!server_.is_running())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    impl_ = std::make_unique<DashboardClientImplX>("127.0.0.1:" + std::to_string(port_));
    impl_->connect();
  }

  void TearDown() override
  {
    server_.stop();
    if (server_thread_.joinable())
    {
      server_thread_.join();
    }
  }

  httplib::Server server_;
  int port_ = 0;
  std::thread server_thread_;
  std::unique_ptr<DashboardClientImplX> impl_;
};

static constexpr const char* MOCK_SHUTDOWN_ENDPOINT = "/universal-robots/robot-api/system/v1/shutdown";

// --- commandShutdown ---
// Spec: PUT /system/v1/shutdown
//   202 → APIResponse  {"message": string|null}
//   403 → APIError     {"message": string, "details": string}  (not in remote-control mode)

TEST_F(DashboardClientImplXMockTest, shutdown_success)
{
  // 202 Accepted
  const std::string body = R"({"message":null})";
  server_.Put(MOCK_SHUTDOWN_ENDPOINT, [&body](const httplib::Request&, httplib::Response& res) {
    res.status = 202;
    res.set_content(body, "application/json");
  });

  auto response = impl_->commandShutdown();
  EXPECT_TRUE(response.ok);
  EXPECT_EQ(response.message, body);
}

TEST_F(DashboardClientImplXMockTest, shutdown_forbidden)
{
  // 403 Forbidden — robot is not in remote-control mode
  const std::string body =
      R"({"message":"Forbidden","details":"Not authorized to perform this operation under the current robot control mode."})";
  server_.Put(MOCK_SHUTDOWN_ENDPOINT, [&body](const httplib::Request&, httplib::Response& res) {
    res.status = 403;
    res.set_content(body, "application/json");
  });

  auto response = impl_->commandShutdown();
  EXPECT_FALSE(response.ok);
  EXPECT_EQ(response.message, body);
}

TEST_F(DashboardClientImplXMockTest, shutdown_endpoint_not_found)
{
  // The server is reachable but no handler is registered for the shutdown
  // endpoint, so httplib returns 404 by default. The client must treat this as
  // a failed (non-ok) response without throwing an exception.
  // Note: no server_.Put(MOCK_SHUTDOWN_ENDPOINT, ...) registration here.

  auto response = impl_->commandShutdown();
  EXPECT_FALSE(response.ok);
  EXPECT_EQ(std::get<int>(response.data.at("status_code")), 404);
}

// --- commandGenerateFlightReport ---
// Spec: POST /supportfiles/v1
//   200 → GenerateFlightReportResponse  {"message": string|null, "details": string|null}
//   408 → APIError                      {"message": string, "details": string}
//   429 → APIError
//   507 → APIError

TEST_F(DashboardClientImplXMockTest, generate_flight_report_success)
{
  // 200 OK with GenerateFlightReportResponse body
  const std::string body = R"({"message":"Flight report generated successfully.","details":null})";
  server_.Post(MOCK_SUPPORTFILES_ENDPOINT,
               [&body](const httplib::Request&, httplib::Response& res) { res.set_content(body, "application/json"); });

  auto response = impl_->commandGenerateFlightReport("");
  EXPECT_TRUE(response.ok);
  EXPECT_EQ(response.message, body);
}

TEST_F(DashboardClientImplXMockTest, generate_flight_report_timeout)
{
  // 408 Request Timeout — flight recorder took too long
  const std::string body =
      R"({"message":"Flight recorder service took longer time to generate the report than expected.","details":""})";
  server_.Post(MOCK_SUPPORTFILES_ENDPOINT, [&body](const httplib::Request&, httplib::Response& res) {
    res.status = 408;
    res.set_content(body, "application/json");
  });

  auto response = impl_->commandGenerateFlightReport("");
  EXPECT_FALSE(response.ok);
  EXPECT_EQ(response.message, body);
}

TEST_F(DashboardClientImplXMockTest, generate_flight_report_insufficient_storage)
{
  // 507 Insufficient Storage
  const std::string body = R"({"message":"Insufficient storage to generate flight report.","details":""})";
  server_.Post(MOCK_SUPPORTFILES_ENDPOINT, [&body](const httplib::Request&, httplib::Response& res) {
    res.status = 507;
    res.set_content(body, "application/json");
  });

  auto response = impl_->commandGenerateFlightReport("");
  EXPECT_FALSE(response.ok);
  EXPECT_EQ(response.message, body);
}

TEST_F(DashboardClientImplXMockTest, generate_flight_report_report_type_ignored)
{
  // PolyScope X ignores the report_type argument (logs a warning). A non-empty
  // value must not prevent a successful call.
  const std::string body = R"({"message":"Flight report generated successfully.","details":null})";
  server_.Post(MOCK_SUPPORTFILES_ENDPOINT,
               [&body](const httplib::Request&, httplib::Response& res) { res.set_content(body, "application/json"); });

  auto response = impl_->commandGenerateFlightReport("blackbox");
  EXPECT_TRUE(response.ok);
}

// --- commandDownloadSupportFiles ---
// Spec: GET /supportfiles/v1
//   200 → application/zip binary
//   204 → No Content (no flight reports available yet)
//   404 → APIError  {"message": string, "details": string}
//   500 → APIError

TEST_F(DashboardClientImplXMockTest, download_support_files_success)
{
  // 200 OK with zip binary body
  const std::string fake_zip(R"(PK)"
                             "\x03\x04"
                             "fake_zip_data",
                             18);
  server_.Get(MOCK_SUPPORTFILES_ENDPOINT, [&fake_zip](const httplib::Request&, httplib::Response& res) {
    res.set_content(fake_zip, "application/zip");
  });

  const std::filesystem::path out = std::filesystem::temp_directory_path() / "urcl_test_support.zip";
  std::filesystem::remove(out);

  auto response = impl_->commandDownloadSupportFiles(out.string());
  EXPECT_TRUE(response.ok);

  std::ifstream file(out, std::ios::binary);
  ASSERT_TRUE(file.is_open());
  const std::string written((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  EXPECT_EQ(written, fake_zip);

  std::filesystem::remove(out);
}

TEST_F(DashboardClientImplXMockTest, download_support_files_no_content)
{
  // 204 No Content — no flight reports are available yet.
  // The implementation treats any 2xx as success, so ok=true and an empty file is created.
  server_.Get(MOCK_SUPPORTFILES_ENDPOINT, [](const httplib::Request&, httplib::Response& res) { res.status = 204; });

  const std::filesystem::path out = std::filesystem::temp_directory_path() / "urcl_test_support_empty.zip";
  std::filesystem::remove(out);

  auto response = impl_->commandDownloadSupportFiles(out.string());
  EXPECT_TRUE(response.ok);
  EXPECT_TRUE(std::filesystem::exists(out));
  EXPECT_EQ(std::filesystem::file_size(out), 0u);

  std::filesystem::remove(out);
}

TEST_F(DashboardClientImplXMockTest, download_support_files_overwrites_existing)
{
  // A second download to the same destination path must overwrite the previous file.
  std::string server_content = "version1_data";
  server_.Get(MOCK_SUPPORTFILES_ENDPOINT, [&server_content](const httplib::Request&, httplib::Response& res) {
    res.set_content(server_content, "application/zip");
  });

  const std::filesystem::path out = std::filesystem::temp_directory_path() / "urcl_test_support_overwrite.zip";
  std::filesystem::remove(out);

  EXPECT_TRUE(impl_->commandDownloadSupportFiles(out.string()).ok);

  server_content = "version2_longer_data";
  EXPECT_TRUE(impl_->commandDownloadSupportFiles(out.string()).ok);

  std::ifstream file(out, std::ios::binary);
  ASSERT_TRUE(file.is_open());
  const std::string result((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  EXPECT_EQ(result, server_content);

  std::filesystem::remove(out);
}

TEST_F(DashboardClientImplXMockTest, download_support_files_not_found)
{
  // 404 Not Found — flight recorder service not found
  const std::string body = R"({"message":"Flight recorder service not found.","details":""})";
  server_.Get(MOCK_SUPPORTFILES_ENDPOINT, [&body](const httplib::Request&, httplib::Response& res) {
    res.status = 404;
    res.set_content(body, "application/json");
  });

  const std::filesystem::path out = std::filesystem::temp_directory_path() / "urcl_test_support_notfound.zip";
  std::filesystem::remove(out);

  auto response = impl_->commandDownloadSupportFiles(out.string());

  EXPECT_FALSE(response.ok);
  EXPECT_EQ(response.message, body);
  EXPECT_FALSE(std::filesystem::exists(out)) << "Final file must not be created on server error";
}

TEST_F(DashboardClientImplXMockTest, download_support_files_server_error)
{
  // 500 Internal Server Error
  const std::string body = R"({"message":"Internal server error.","details":""})";
  server_.Get(MOCK_SUPPORTFILES_ENDPOINT, [&body](const httplib::Request&, httplib::Response& res) {
    res.status = 500;
    res.set_content(body, "application/json");
  });

  const std::filesystem::path out = std::filesystem::temp_directory_path() / "urcl_test_support_err.zip";
  std::filesystem::remove(out);

  auto response = impl_->commandDownloadSupportFiles(out.string());

  EXPECT_FALSE(response.ok);
  EXPECT_EQ(response.message, body);
  EXPECT_FALSE(std::filesystem::exists(out)) << "Final file must not be created on server error";
}

TEST_F(DashboardClientImplXMockTest, download_support_files_invalid_path)
{
  // A non-writable path must produce an error before any HTTP request is sent (mkstemp
  // fails because the parent directory does not exist).
  auto response = impl_->commandDownloadSupportFiles("/nonexistent_directory/support.zip");

  EXPECT_FALSE(response.ok);
  EXPECT_NE(response.message.find("Failed to create temporary file"), std::string::npos);
}

TEST_F(DashboardClientImplXMockTest, download_support_files_connection_error)
{
  // The server is stopped after connect() so the streaming GET has no server to talk to.
  // The httplib::Result is falsy (error != Success) and the client must surface this as a
  // non-ok response containing "HTTP request failed". The final file must not be created.
  const std::filesystem::path out = std::filesystem::temp_directory_path() / "urcl_test_support_conn_err.zip";
  std::filesystem::remove(out);

  server_.stop();
  if (server_thread_.joinable())
  {
    server_thread_.join();
  }

  auto response = impl_->commandDownloadSupportFiles(out.string());

  EXPECT_FALSE(response.ok);
  EXPECT_NE(response.message.find("HTTP request failed"), std::string::npos);
  EXPECT_FALSE(std::filesystem::exists(out)) << "Final file must not be created on connection error";
}

TEST_F(DashboardClientImplXMockTest, download_support_files_rename_failure)
{
  // When the destination path already exists as a directory, std::filesystem::rename
  // fails (EISDIR on POSIX). The implementation must return a non-ok response
  // containing "Failed to rename" and must clean up the temporary file.
  const std::string fake_zip = "fake_zip_data";
  server_.Get(MOCK_SUPPORTFILES_ENDPOINT, [&fake_zip](const httplib::Request&, httplib::Response& res) {
    res.set_content(fake_zip, "application/zip");
  });

  // Create a directory at the save_path so that rename() cannot replace it with a file.
  const std::filesystem::path out = std::filesystem::temp_directory_path() / "urcl_test_rename_fail_dir";
  std::filesystem::remove_all(out);
  std::filesystem::create_directory(out);

  auto response = impl_->commandDownloadSupportFiles(out.string());

  EXPECT_FALSE(response.ok);
  EXPECT_NE(response.message.find("Failed to rename"), std::string::npos);

  // The directory itself must be untouched; no temp file should be left behind.
  EXPECT_TRUE(std::filesystem::is_directory(out));
  EXPECT_TRUE(std::filesystem::is_empty(out)) << "Temp file must be cleaned up after rename failure";

  std::filesystem::remove_all(out);
}

// --- commandGetProgramList ---
// Spec: GET /programs/v1/
//   200 → ProgramListResponse {"programs": [ProgramInformation, ...]}
//
// ProgramInformation declares "name" as the only non-nullable field; the other
// five are "anyOf": [<type>, {"type": "null"}], so a null is a spec-conforming
// value for each of them:
//
//   "name":             {"type": "string"}
//   "description":      {"anyOf": [{"type": "string"},  {"type": "null"}]}
//   "createdDate":      {"anyOf": [{"type": "integer"}, {"type": "null"}]}
//   "lastSavedDate":    {"anyOf": [{"type": "integer"}, {"type": "null"}]}
//   "lastModifiedDate": {"anyOf": [{"type": "integer"}, {"type": "null"}]}
//   "programState":     {"anyOf": [{"$ref": "ProgramState-Output"}, {"type": "null"}]}
static constexpr const char* MOCK_PROGRAMS_ENDPOINT = "/universal-robots/robot-api/programs/v1/";

TEST_F(DashboardClientImplXMockTest, get_program_list_all_fields_populated)
{
  // Baseline: every field carries a value. Reading the nullable fields
  // defensively must not change the result for a fully populated response.
  const std::string body = R"({"programs":[{
    "createdDate":1757000000,
    "description":"a program with all fields set",
    "lastModifiedDate":1757000001,
    "lastSavedDate":1757000002,
    "name":"full_program",
    "programState":"FINAL"
  }]})";
  server_.Get(MOCK_PROGRAMS_ENDPOINT,
              [&body](const httplib::Request&, httplib::Response& res) { res.set_content(body, "application/json"); });

  auto response = impl_->commandGetProgramList();
  ASSERT_TRUE(response.ok);

  const auto programs = std::get<std::vector<ProgramInformation>>(response.data.at("programs"));
  ASSERT_EQ(programs.size(), 1u);
  EXPECT_EQ(programs[0].createdDate, 1757000000u);
  EXPECT_EQ(programs[0].description, "a program with all fields set");
  EXPECT_EQ(programs[0].lastModifiedDate, 1757000001u);
  EXPECT_EQ(programs[0].lastSavedDate, 1757000002u);
  EXPECT_EQ(programs[0].name, "full_program");
  EXPECT_EQ(programs[0].programState, "FINAL");
}

TEST_F(DashboardClientImplXMockTest, get_program_list_tolerates_null_description)
{
  // "description" is nullable per the schema, so a null here is a valid
  // response. Converting it straight into the constructor's std::string
  // parameter threw
  //   [json.exception.type_error.302] type must be string, but is null
  // which failed the whole query, including the "name" field most callers need.
  const std::string body = R"({"programs":[{
    "createdDate":1757000000,
    "description":null,
    "lastModifiedDate":1757000001,
    "lastSavedDate":1757000002,
    "name":"program_without_description",
    "programState":"FINAL"
  }]})";
  server_.Get(MOCK_PROGRAMS_ENDPOINT,
              [&body](const httplib::Request&, httplib::Response& res) { res.set_content(body, "application/json"); });

  auto response = impl_->commandGetProgramList();
  ASSERT_TRUE(response.ok);

  const auto programs = std::get<std::vector<ProgramInformation>>(response.data.at("programs"));
  ASSERT_EQ(programs.size(), 1u);
  EXPECT_EQ(programs[0].name, "program_without_description");
  EXPECT_EQ(programs[0].description, "");
  EXPECT_EQ(programs[0].programState, "FINAL");
}

TEST_F(DashboardClientImplXMockTest, get_program_list_tolerates_all_nullable_fields_null)
{
  // Every field the schema declares nullable is null at once. Only "name" is
  // guaranteed by the spec, so that is the only value that must survive; the
  // rest default to an empty string / 0.
  const std::string body = R"({"programs":[{
    "createdDate":null,
    "description":null,
    "lastModifiedDate":null,
    "lastSavedDate":null,
    "name":"only_a_name",
    "programState":null
  }]})";
  server_.Get(MOCK_PROGRAMS_ENDPOINT,
              [&body](const httplib::Request&, httplib::Response& res) { res.set_content(body, "application/json"); });

  auto response = impl_->commandGetProgramList();
  ASSERT_TRUE(response.ok);

  const auto programs = std::get<std::vector<ProgramInformation>>(response.data.at("programs"));
  ASSERT_EQ(programs.size(), 1u);
  EXPECT_EQ(programs[0].name, "only_a_name");
  EXPECT_EQ(programs[0].description, "");
  EXPECT_EQ(programs[0].createdDate, 0u);
  EXPECT_EQ(programs[0].lastModifiedDate, 0u);
  EXPECT_EQ(programs[0].lastSavedDate, 0u);
  EXPECT_EQ(programs[0].programState, "");
}

TEST_F(DashboardClientImplXMockTest, get_program_list_null_field_does_not_drop_other_programs)
{
  // The conversion happens inside the loop over "programs", so before the fix a
  // single program with a null field made the entire list unavailable. All
  // programs must be returned, in response order.
  const std::string body = R"({"programs":[
    {"createdDate":null,"description":null,"lastModifiedDate":null,"lastSavedDate":null,
     "name":"program_with_nulls","programState":null},
    {"createdDate":1757000000,"description":"fully populated","lastModifiedDate":1757000001,
     "lastSavedDate":1757000002,"name":"program_without_nulls","programState":"DRAFT"}
  ]})";
  server_.Get(MOCK_PROGRAMS_ENDPOINT,
              [&body](const httplib::Request&, httplib::Response& res) { res.set_content(body, "application/json"); });

  auto response = impl_->commandGetProgramList();
  ASSERT_TRUE(response.ok);

  const auto programs = std::get<std::vector<ProgramInformation>>(response.data.at("programs"));
  ASSERT_EQ(programs.size(), 2u);
  EXPECT_EQ(programs[0].name, "program_with_nulls");
  EXPECT_EQ(programs[0].description, "");
  EXPECT_EQ(programs[1].name, "program_without_nulls");
  EXPECT_EQ(programs[1].description, "fully populated");
  EXPECT_EQ(programs[1].programState, "DRAFT");
}

TEST_F(DashboardClientImplXMockTest, get_program_list_empty)
{
  // A robot with no programs answers with an empty array.
  const std::string body = R"({"programs":[]})";
  server_.Get(MOCK_PROGRAMS_ENDPOINT,
              [&body](const httplib::Request&, httplib::Response& res) { res.set_content(body, "application/json"); });

  auto response = impl_->commandGetProgramList();
  ASSERT_TRUE(response.ok);
  EXPECT_TRUE(std::get<std::vector<ProgramInformation>>(response.data.at("programs")).empty());
}

// ---------------------------------------------------------------------------
// Non-JSON Robot API bodies
//
// Dashboard handlers must not let nlohmann::json::parse_error escape. HTTP
// error bodies (HTML / plaintext / empty) should be returned as a failed
// DashboardResponse. HTTP 200 with invalid JSON should throw UrException.
// ---------------------------------------------------------------------------

static constexpr const char* MOCK_PROGRAM_STATE_ENDPOINT = "/universal-robots/robot-api/program/v1/state";
static constexpr const char* MOCK_VERSIONS_ENDPOINT = "/universal-robots/robot-api/versions/v1";

static void expectUrException(const std::function<void()>& fn)
{
  try
  {
    fn();
    FAIL() << "Expected UrException";
  }
  catch (const UrException&)
  {
  }
  catch (const std::exception& e)
  {
    FAIL() << "Non-UrException escaped: " << e.what();
  }
}

TEST_F(DashboardClientImplXMockTest, program_state_valid_json)
{
  server_.Get(MOCK_PROGRAM_STATE_ENDPOINT, [](const httplib::Request&, httplib::Response& res) {
    res.set_content(R"({"state":"STOPPED"})", "application/json");
  });

  auto response = impl_->commandProgramState();
  ASSERT_TRUE(response.ok);
  EXPECT_EQ(std::get<std::string>(response.data.at("program_state")), "STOPPED");
}

TEST_F(DashboardClientImplXMockTest, program_state_html_error_body)
{
  server_.Get(MOCK_PROGRAM_STATE_ENDPOINT, [](const httplib::Request&, httplib::Response& res) {
    res.status = 500;
    res.set_content("<html>Internal Server Error</html>", "text/html");
  });

  DashboardResponse response;
  ASSERT_NO_THROW(response = impl_->commandProgramState());
  EXPECT_FALSE(response.ok);
}

TEST_F(DashboardClientImplXMockTest, program_state_plain_text_error_body)
{
  server_.Get(MOCK_PROGRAM_STATE_ENDPOINT, [](const httplib::Request&, httplib::Response& res) {
    res.status = 500;
    res.set_content("Internal Server Error", "text/plain");
  });

  DashboardResponse response;
  ASSERT_NO_THROW(response = impl_->commandProgramState());
  EXPECT_FALSE(response.ok);
}

TEST_F(DashboardClientImplXMockTest, program_state_empty_error_body)
{
  server_.Get(MOCK_PROGRAM_STATE_ENDPOINT, [](const httplib::Request&, httplib::Response& res) { res.status = 500; });

  DashboardResponse response;
  ASSERT_NO_THROW(response = impl_->commandProgramState());
  EXPECT_FALSE(response.ok);
}

TEST_F(DashboardClientImplXMockTest, program_state_malformed_json_success)
{
  server_.Get(MOCK_PROGRAM_STATE_ENDPOINT, [](const httplib::Request&, httplib::Response& res) {
    res.set_content("NOT-JSON-AT-ALL", "application/json");
  });

  expectUrException([this]() { impl_->commandProgramState(); });
}

TEST_F(DashboardClientImplXMockTest, program_state_empty_success_body)
{
  server_.Get(MOCK_PROGRAM_STATE_ENDPOINT,
              [](const httplib::Request&, httplib::Response& res) { res.set_content("", "application/json"); });

  expectUrException([this]() { impl_->commandProgramState(); });
}

TEST_F(DashboardClientImplXMockTest, polyscope_version_malformed_json)
{
  server_.Get(MOCK_VERSIONS_ENDPOINT, [](const httplib::Request&, httplib::Response& res) {
    res.set_content("NOT-JSON-AT-ALL", "application/json");
  });

  expectUrException([this]() { impl_->commandPolyscopeVersion(); });
}

TEST_F(DashboardClientImplXMockTest, get_program_list_html_error_body)
{
  server_.Get(MOCK_PROGRAMS_ENDPOINT, [](const httplib::Request&, httplib::Response& res) {
    res.status = 500;
    res.set_content("<html>Internal Server Error</html>", "text/html");
  });

  DashboardResponse response;
  ASSERT_NO_THROW(response = impl_->commandGetProgramList());
  EXPECT_FALSE(response.ok);
}

// ---------------------------------------------------------------------------

// Connection-state tests
//
// These tests use TestableDashboardClientMock so that is_connected_ can be
// observed directly. Unlike DashboardClientImplXMockTest, SetUp() does NOT
// call connect() — each test controls the connection lifecycle explicitly.
// ---------------------------------------------------------------------------

class DashboardClientImplXConnectionStateTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    port_ = server_.bind_to_any_port("127.0.0.1");
    server_thread_ = std::thread([this]() { server_.listen_after_bind(); });
    while (!server_.is_running())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    impl_ = std::make_unique<TestableDashboardClientMock>("127.0.0.1:" + std::to_string(port_));
  }

  void TearDown() override
  {
    server_.stop();
    if (server_thread_.joinable())
    {
      server_thread_.join();
    }
  }

  void registerOpenApiEndpoint()
  {
    server_.Get(MOCK_OPENAPI_ENDPOINT, [](const httplib::Request&, httplib::Response& res) {
      res.set_content(MOCK_OPENAPI_RESPONSE, "application/json");
    });
  }

  httplib::Server server_;
  int port_ = 0;
  std::thread server_thread_;
  std::unique_ptr<TestableDashboardClientMock> impl_;
};

TEST_F(DashboardClientImplXConnectionStateTest, initially_not_connected)
{
  // A freshly constructed client must report disconnected before connect() is called.
  EXPECT_FALSE(impl_->isConnected());
}

TEST_F(DashboardClientImplXConnectionStateTest, connect_success_sets_connected)
{
  registerOpenApiEndpoint();
  EXPECT_TRUE(impl_->connect());
  EXPECT_TRUE(impl_->isConnected());
}

TEST_F(DashboardClientImplXConnectionStateTest, connect_failure_leaves_not_connected)
{
  // The server is reachable but openapi.json returns 404, so connect() must fail
  // and leave the client in the disconnected state.
  server_.Get(MOCK_OPENAPI_ENDPOINT, [](const httplib::Request&, httplib::Response& res) { res.status = 404; });

  EXPECT_FALSE(impl_->connect());
  EXPECT_FALSE(impl_->isConnected());
}

TEST_F(DashboardClientImplXConnectionStateTest, connect_oversized_version_returns_false)
{
  server_.Get(MOCK_OPENAPI_ENDPOINT, [](const httplib::Request&, httplib::Response& res) {
    res.set_content(R"({"info":{"version":"99999999999999999999.1.1"}})", "application/json");
  });

  EXPECT_NO_THROW(EXPECT_FALSE(impl_->connect()));
  EXPECT_FALSE(impl_->isConnected());
}

TEST_F(DashboardClientImplXConnectionStateTest, connect_trailing_garbage_version_returns_false)
{
  server_.Get(MOCK_OPENAPI_ENDPOINT, [](const httplib::Request&, httplib::Response& res) {
    res.set_content(R"({"info":{"version":"5.15.0junk"}})", "application/json");
  });

  EXPECT_NO_THROW(EXPECT_FALSE(impl_->connect()));
  EXPECT_FALSE(impl_->isConnected());
}

TEST_F(DashboardClientImplXConnectionStateTest, connect_malformed_json_returns_false)
{
  server_.Get(MOCK_OPENAPI_ENDPOINT,
              [](const httplib::Request&, httplib::Response& res) { res.set_content("not json", "application/json"); });

  EXPECT_NO_THROW(EXPECT_FALSE(impl_->connect()));
  EXPECT_FALSE(impl_->isConnected());
}

TEST_F(DashboardClientImplXConnectionStateTest, assert_has_command_reconnects_when_not_connected)
{
  // When assertHasCommand() is called while disconnected it must trigger connect().
  // commandShutdown() calls assertHasCommand("shutdown") internally, so a successful
  // commandShutdown() proves that reconnection happened.
  registerOpenApiEndpoint();
  server_.Put(MOCK_SHUTDOWN_ENDPOINT, [](const httplib::Request&, httplib::Response& res) {
    res.status = 202;
    res.set_content(R"({"message":null})", "application/json");
  });

  ASSERT_FALSE(impl_->isConnected());
  impl_->commandShutdown();
  EXPECT_TRUE(impl_->isConnected());
}

TEST_F(DashboardClientImplXConnectionStateTest, assert_has_command_throws_when_reconnect_fails)
{
  // assertHasCommand() is called while disconnected and the reconnect attempt fails
  // (openapi.json returns 404). A UrException must be thrown.
  server_.Get(MOCK_OPENAPI_ENDPOINT, [](const httplib::Request&, httplib::Response& res) { res.status = 404; });

  ASSERT_FALSE(impl_->isConnected());
  EXPECT_THROW(impl_->commandShutdown(), UrException);
}

TEST_F(DashboardClientImplXConnectionStateTest, disconnect_clears_connected)
{
  registerOpenApiEndpoint();
  ASSERT_TRUE(impl_->connect());
  ASSERT_TRUE(impl_->isConnected());

  impl_->disconnect();
  EXPECT_FALSE(impl_->isConnected());
}

// ---------------------------------------------------------------------------
// commandLoadProgram serialization
//
// The PUT body must be exactly one JSON string member so quotes, separators,
// escapes, duplicate-key-like names, Unicode, and long/malformed strings cannot
// inject extra fields.
// ---------------------------------------------------------------------------

static constexpr const char* MOCK_LOAD_PROGRAM_ENDPOINT = "/universal-robots/robot-api/program/v1/loaded";

class DashboardClientXProgramNameSerializationTest : public ::testing::TestWithParam<std::string>
{
protected:
  void SetUp() override
  {
    server_.Get(MOCK_OPENAPI_ENDPOINT, [](const httplib::Request&, httplib::Response& response) {
      response.set_content(MOCK_OPENAPI_RESPONSE, "application/json");
    });
    server_.Put(MOCK_LOAD_PROGRAM_ENDPOINT, [this](const httplib::Request& request, httplib::Response& response) {
      request_body_.set_value(request.body);
      response.set_content("{}", "application/json");
    });

    port_ = server_.bind_to_any_port("127.0.0.1");
    server_thread_ = std::thread([this]() { server_.listen_after_bind(); });
    while (!server_.is_running())
    {
      std::this_thread::yield();
    }

    client_ = std::make_unique<DashboardClientImplX>("127.0.0.1:" + std::to_string(port_));
    ASSERT_TRUE(client_->connect());
  }

  void TearDown() override
  {
    client_.reset();
    server_.stop();
    if (server_thread_.joinable())
    {
      server_thread_.join();
    }
  }

  httplib::Server server_;
  int port_ = 0;
  std::thread server_thread_;
  std::promise<std::string> request_body_;
  std::unique_ptr<DashboardClientImplX> client_;
};

TEST_P(DashboardClientXProgramNameSerializationTest, serializes_name_as_exactly_one_json_string_member)
{
  const std::string& program_name = GetParam();
  auto request_body = request_body_.get_future();

  ASSERT_TRUE(client_->commandLoadProgram(program_name).ok);
  const std::string body = request_body.get();

  const std::string expected_body = json{ { "name", program_name } }.dump();
  EXPECT_EQ(body, expected_body);

  const json payload = json::parse(body);
  ASSERT_TRUE(payload.is_object());
  ASSERT_EQ(payload.size(), 1);
  ASSERT_TRUE(payload.contains("name"));
  ASSERT_TRUE(payload.at("name").is_string());
  EXPECT_EQ(payload.at("name").get<std::string>(), program_name);
}

std::vector<std::string> programNames()
{
  return {
    R"(program "with quotes")",
    R"(name,with:separators{}[])",
    "backslash\\newline\ncarriage\rreturn\ttab\bbackspace\fformfeed",
    R"(Default program", "name": "SecondProgram)",
    u8"程序-🤖-é",
    std::string(65536, 'a'),
    R"({"name": [this is not JSON})",
  };
}

INSTANTIATE_TEST_SUITE_P(SecurityRegression, DashboardClientXProgramNameSerializationTest,
                         ::testing::ValuesIn(programNames()));

TEST_F(DashboardClientImplXConnectionStateTest, connect_malformed_json_leaves_not_connected)
{
  // HTTP 200 with a non-JSON body must not throw a nlohmann parse_error; connect()
  // reports failure and leaves the client disconnected.
  server_.Get(MOCK_OPENAPI_ENDPOINT, [](const httplib::Request&, httplib::Response& res) {
    res.set_content("<html>not openapi</html>", "text/html");
  });

  bool connected = true;
  ASSERT_NO_THROW(connected = impl_->connect());
  EXPECT_FALSE(connected);
  EXPECT_FALSE(impl_->isConnected());
}

// ---------------------------------------------------------------------------
// commandDownloadProgram request-target construction
//
// The program name must remain a single path segment. Traversal, encoded
// traversal, and non-program 200 responses must not be saved as a download.
// ---------------------------------------------------------------------------

static constexpr const char* MOCK_DOWNLOAD_PROGRAM_PATTERN = "/universal-robots/robot-api/programs/v1/(.+)";

static std::string programExportBody(const std::string& name)
{
  return json{ { "program", { { "programInformation", { { "name", name } } } } } }.dump();
}

class DashboardClientXDownloadProgramTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    server_.Get(MOCK_OPENAPI_ENDPOINT, [](const httplib::Request&, httplib::Response& response) {
      response.set_content(MOCK_OPENAPI_RESPONSE, "application/json");
    });
    server_.Get(MOCK_DOWNLOAD_PROGRAM_PATTERN, [this](const httplib::Request& request, httplib::Response& response) {
      requested_targets_.push_back(request.target);
      response.status = program_status_;
      response.set_content(program_body_, program_content_type_);
    });
    server_.Get("/universal-robots/robot-api/robotstate/v1/robotmode",
                [this](const httplib::Request& request, httplib::Response& response) {
                  requested_targets_.push_back(request.target);
                  response.set_content(R"({"robotMode":"POWER_OFF"})", "application/json");
                });

    port_ = server_.bind_to_any_port("127.0.0.1");
    server_thread_ = std::thread([this]() { server_.listen_after_bind(); });
    while (!server_.is_running())
    {
      std::this_thread::yield();
    }

    client_ = std::make_unique<DashboardClientImplX>("127.0.0.1:" + std::to_string(port_));
    ASSERT_TRUE(client_->connect());
  }

  void TearDown() override
  {
    client_.reset();
    server_.stop();
    if (server_thread_.joinable())
    {
      server_thread_.join();
    }
    if (!out_path_.empty())
    {
      std::filesystem::remove(out_path_);
    }
  }

  std::filesystem::path makeOutPath()
  {
    out_path_ =
        std::filesystem::temp_directory_path() /
        ("urcl_download_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".urpx");
    std::filesystem::remove(out_path_);
    return out_path_;
  }

  httplib::Server server_;
  int port_ = 0;
  std::thread server_thread_;
  std::unique_ptr<DashboardClientImplX> client_;
  std::vector<std::string> requested_targets_;
  int program_status_ = 200;
  std::string program_body_ = programExportBody("normal-program");
  std::string program_content_type_ = "application/json";
  std::filesystem::path out_path_;
};

TEST_F(DashboardClientXDownloadProgramTest, ordinary_name_downloads_to_temp_then_destination)
{
  const auto out = makeOutPath();
  auto response = client_->commandDownloadProgram("normal-program", out.string());
  ASSERT_TRUE(response.ok);
  ASSERT_EQ(requested_targets_.size(), 1u);
  EXPECT_EQ(requested_targets_.front(), "/universal-robots/robot-api/programs/v1/normal-program");

  std::ifstream file(out);
  ASSERT_TRUE(file.is_open());
  const std::string written((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  EXPECT_EQ(written, program_body_);
}

TEST_F(DashboardClientXDownloadProgramTest, space_in_name_is_allowed_and_percent_encoded)
{
  program_body_ = programExportBody("Default program");
  const auto out = makeOutPath();
  auto response = client_->commandDownloadProgram("Default program", out.string());
  ASSERT_TRUE(response.ok);
  ASSERT_EQ(requested_targets_.size(), 1u);
  EXPECT_EQ(requested_targets_.front(), "/universal-robots/robot-api/programs/v1/Default%20program");

  std::ifstream file(out);
  ASSERT_TRUE(file.is_open());
  const std::string written((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  EXPECT_EQ(written, program_body_);
}

TEST_F(DashboardClientXDownloadProgramTest, traversal_names_are_rejected_without_request)
{
  const std::vector<std::string> names = {
    "../../robotstate/v1/robotmode",
    "%2e%2e%2f%2e%2e%2frobotstate/v1/robotmode",
    "../../../../",
    "../../../urcap-system-api/urcap-system-api/internal/license/v1/license/",
    "../../../../../etc/passwd",
    ".",
    "..",
    "foo/bar",
    "foo\\bar",
    "foo\nbar",
    "%2e%2e",
    "%2E%2E%2F",
    "%252e%252e%252f",
  };

  const auto out = makeOutPath();
  for (const auto& name : names)
  {
    requested_targets_.clear();
    auto response = client_->commandDownloadProgram(name, out.string());
    EXPECT_FALSE(response.ok) << "name: " << name;
    EXPECT_TRUE(requested_targets_.empty())
        << "name: " << name << " target: " << (requested_targets_.empty() ? "" : requested_targets_.front());
    EXPECT_FALSE(std::filesystem::exists(out)) << "name: " << name;
  }
}

TEST_F(DashboardClientXDownloadProgramTest, missing_program_does_not_write_file)
{
  program_status_ = 404;
  program_body_ = R"({"message":"not found"})";
  const auto out = makeOutPath();
  auto response = client_->commandDownloadProgram("missing", out.string());
  EXPECT_FALSE(response.ok);
  EXPECT_FALSE(std::filesystem::exists(out));
}

TEST_F(DashboardClientXDownloadProgramTest, unexpected_json_200_is_not_saved)
{
  program_body_ = R"({"robotMode":"POWER_OFF"})";
  const auto out = makeOutPath();
  const std::string sentinel = "do-not-overwrite";
  {
    std::ofstream existing(out);
    existing << sentinel;
  }

  auto response = client_->commandDownloadProgram("normal-program", out.string());
  EXPECT_FALSE(response.ok);
  EXPECT_NE(response.message.find("program export"), std::string::npos);

  std::ifstream file(out);
  const std::string remaining((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  EXPECT_EQ(remaining, sentinel);
}

TEST_F(DashboardClientXDownloadProgramTest, html_200_is_not_saved)
{
  program_body_ = "<html>root</html>";
  program_content_type_ = "text/html";
  const auto out = makeOutPath();
  auto response = client_->commandDownloadProgram("normal-program", out.string());
  EXPECT_FALSE(response.ok);
  EXPECT_FALSE(std::filesystem::exists(out));
}

TEST_F(DashboardClientXDownloadProgramTest, identity_mismatch_is_not_saved)
{
  program_body_ = programExportBody("other-program");
  const auto out = makeOutPath();
  auto response = client_->commandDownloadProgram("normal-program", out.string());
  EXPECT_FALSE(response.ok);
  EXPECT_NE(response.message.find("does not match"), std::string::npos);
  EXPECT_FALSE(std::filesystem::exists(out));
}
