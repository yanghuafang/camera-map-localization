/// CLI: run regression benchmark suite or micro-benchmarks; optional JSON report.
#include <cam_loc/benchmark/benchmark.hpp>
#include <cam_loc/benchmark/micro_benchmarks.hpp>

#include <cam_loc/kitti/calib_parser.hpp>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#include <nlohmann/json.hpp>

namespace {

struct Options {
  std::string repo_root = ".";
  std::string suite = "default";
  std::string filter;
  std::string output_json;
  bool micro = false;
  bool list = false;
  bool fail_on_regression = true;
};

Options parseArgs(int argc, char** argv) {
  Options opt;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto need = [&](const char* name) {
      if (i + 1 >= argc) throw std::runtime_error(std::string("Missing value for ") + name);
      return std::string(argv[++i]);
    };
    if (arg == "--repo-root") {
      opt.repo_root = need("--repo-root");
    } else if (arg == "--suite") {
      opt.suite = need("--suite");
    } else if (arg == "--filter") {
      opt.filter = need("--filter");
    } else if (arg == "--output-json") {
      opt.output_json = need("--output-json");
    } else if (arg == "--micro") {
      opt.micro = true;
    } else if (arg == "--list") {
      opt.list = true;
    } else if (arg == "--no-fail") {
      opt.fail_on_regression = false;
    } else if (arg == "--help" || arg == "-h") {
      std::cout
          << "Usage: benchmark [options]\n"
          << "  Regression + performance benchmark suite for camera-map-localization.\n"
          << "  --repo-root PATH     Repository root (default .)\n"
          << "  --suite NAME         default (smoke + kitti00 cases)\n"
          << "  --filter PREFIX      Run only cases matching prefix\n"
          << "  --output-json PATH   Write machine-readable results\n"
          << "  --micro              Run DT/pose-grid micro-benchmarks\n"
          << "  --list               List case names and exit\n"
          << "  --no-fail            Always exit 0 (report only)\n";
      std::exit(0);
    }
  }
  return opt;
}

void printCaseResult(const cam_loc::benchmark::BenchmarkResult& r) {
  std::cout << std::fixed << std::setprecision(4);
  std::cout << (r.passed ? "[PASS] " : "[FAIL] ") << r.name;
  if (!r.passed && !r.failure_reason.empty()) {
    std::cout << " — " << r.failure_reason;
  }
  std::cout << "\n";
  if (r.num_frames > 0) {
    std::cout << "  frames=" << r.num_frames << " rmse_m=" << r.summary.pose.rmse_translation_m
              << " yaw_rmse_deg=" << r.summary.pose.rmse_yaw_deg
              << " match_rate=" << (100.0 * r.summary.matching.match_rate) << "%"
              << " flat_rate=" << (100.0 * r.summary.matching.flat_rate) << "%"
              << " mean_ms=" << r.summary.mean_frame_ms
              << " p95_ms=" << r.summary.p95_frame_ms << "\n";
  }
}

void writeJson(const cam_loc::benchmark::BenchmarkSuiteResult& suite,
               const std::vector<cam_loc::benchmark::MicroBenchmarkResult>& micro,
               const std::string& path) {
  nlohmann::json root;
  root["passed"] = suite.passed;
  root["failed"] = suite.failed;
  nlohmann::json cases = nlohmann::json::array();
  for (const auto& r : suite.cases) {
    nlohmann::json c;
    c["name"] = r.name;
    c["passed"] = r.passed;
    c["failure_reason"] = r.failure_reason;
    c["num_frames"] = r.num_frames;
    c["rmse_translation_m"] = r.summary.pose.rmse_translation_m;
    c["rmse_yaw_deg"] = r.summary.pose.rmse_yaw_deg;
    c["match_rate"] = r.summary.matching.match_rate;
    c["flat_rate"] = r.summary.matching.flat_rate;
    c["mean_min_cost"] = r.summary.matching.mean_min_cost;
    c["mean_frame_ms"] = r.summary.mean_frame_ms;
    c["p95_frame_ms"] = r.summary.p95_frame_ms;
    cases.push_back(c);
  }
  root["cases"] = cases;

  if (!micro.empty()) {
    nlohmann::json mj = nlohmann::json::array();
    for (const auto& m : micro) {
      mj.push_back({{"name", m.name},
                    {"use_cuda", m.use_cuda},
                    {"mean_ms", m.mean_ms},
                    {"p95_ms", m.p95_ms},
                    {"iterations", m.iterations}});
    }
    root["micro"] = mj;
  }

  std::ofstream out(path);
  out << root.dump(2) << "\n";
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options opt = parseArgs(argc, argv);
    std::vector<cam_loc::benchmark::BenchmarkCase> cases =
        cam_loc::benchmark::defaultBenchmarkSuite(opt.repo_root);

    if (!opt.filter.empty()) {
      std::vector<cam_loc::benchmark::BenchmarkCase> filtered;
      for (const auto& c : cases) {
        if (c.name.rfind(opt.filter, 0) == 0) filtered.push_back(c);
      }
      cases.swap(filtered);
    }

    if (opt.list) {
      for (const auto& c : cases) {
        std::cout << c.name << "\n";
      }
      return 0;
    }

    if (opt.micro) {
      cam_loc::kitti::Calibration calib;
      const std::string calib_path = cam_loc::kitti::resolveCalibPath(
          opt.repo_root + "/data/smoke_kitti", 0);
      if (cam_loc::kitti::parseCalibrationFile(calib_path, calib) != cam_loc::Status::kOk) {
        std::cerr << "Micro-benchmark requires data/smoke_kitti (run prepare_smoke_kitti.sh)\n";
        return 1;
      }
      const auto micro = cam_loc::benchmark::runMicroBenchmarks(calib, 30);
      std::cout << "Micro-benchmarks (smoke calib, 30 iters):\n";
      for (const auto& m : micro) {
        std::cout << std::fixed << std::setprecision(3) << "  " << m.name
                  << (m.use_cuda ? " [cuda]" : " [cpu]") << " mean=" << m.mean_ms
                  << "ms p95=" << m.p95_ms << "ms\n";
      }
      if (!opt.output_json.empty()) {
        cam_loc::benchmark::BenchmarkSuiteResult empty;
        writeJson(empty, micro, opt.output_json);
        std::cout << "Wrote " << opt.output_json << "\n";
      }
      return 0;
    }

    cam_loc::benchmark::BenchmarkSuiteResult suite;
    cam_loc::benchmark::runBenchmarkSuite(cases, suite);

    std::cout << "Benchmark suite: " << suite.passed << " passed, " << suite.failed
              << " failed\n\n";
    for (const auto& r : suite.cases) {
      printCaseResult(r);
    }

    if (!opt.output_json.empty()) {
      writeJson(suite, {}, opt.output_json);
      std::cout << "\nWrote " << opt.output_json << "\n";
    }

    if (opt.fail_on_regression && suite.failed > 0) {
      return 1;
    }
    return 0;
  } catch (const std::exception& ex) {
    std::cerr << "Error: " << ex.what() << "\n";
    return 1;
  }
}
