/// CLI: run regression benchmark suite or micro-benchmarks; optional JSON
/// report.
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#include <nlohmann/json.hpp>

#include "cam_loc/benchmark/benchmark.h"
#include "cam_loc/benchmark/micro_benchmarks.h"
#include "cam_loc/cuda/distance_transform.h"
#include "cam_loc/kitti/calib_parser.h"

namespace {

struct Options {
  // Datasets live beside the repository, not in it, so there is no useful
  // default here -- scripts/lib.sh camloc_data_dir computes the path and the
  // scripts pass it in.
  std::string data_root;
  std::string suite = "default";
  std::string filter;
  std::string output_json;
  bool micro = false;
  bool list = false;
  bool fail_on_regression = true;
};

Options ParseArgs(int argc, char** argv, bool* ok) {
  Options opt;
  *ok = true;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto need = [&](const char* name) -> std::string {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for " << name << "\n";
        *ok = false;
        return std::string();
      }
      return argv[++i];
    };
    if (arg == "--data-root") {
      opt.data_root = need("--data-root");
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
      std::cout << "Usage: benchmark [options]\n"
                << "  Regression + performance benchmark suite for "
                   "camera-map-localization.\n"
                << "  --data-root PATH     Directory holding smoke_kitti/, "
                   "kitti_odometry/, perception/\n"
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

void PrintCaseResult(const cam_loc::benchmark::BenchmarkResult& r) {
  std::cout << std::fixed << std::setprecision(4);
  std::cout << (r.passed ? "[PASS] " : "[FAIL] ") << r.name;
  if (!r.passed && !r.failure_reason.empty()) {
    std::cout << " — " << r.failure_reason;
  }
  std::cout << "\n";
  if (r.num_frames > 0) {
    std::cout << "  frames=" << r.num_frames
              << " rmse_m=" << r.summary.pose.rmse_translation_m
              << " lat_rmse_m=" << r.summary.pose.rmse_lateral_m
              << " lon_rmse_m=" << r.summary.pose.rmse_longitudinal_m
              << " lon_bias_m=" << r.summary.pose.bias_longitudinal_m
              << " yaw_rmse_deg=" << r.summary.pose.rmse_yaw_deg << "\n";
    std::cout << "  match_rate=" << (100.0 * r.summary.matching.match_rate)
              << "% flat_rate=" << (100.0 * r.summary.matching.flat_rate)
              << "% mean_ms=" << r.summary.mean_frame_ms
              << " p95_ms=" << r.summary.p95_frame_ms << "\n";
  }
}

void WriteJson(
    const cam_loc::benchmark::BenchmarkSuiteResult& suite,
    const std::vector<cam_loc::benchmark::MicroBenchmarkResult>& micro,
    const std::string& path) {
  nlohmann::json root;
  root["passed"] = suite.passed;
  root["failed"] = suite.failed;
  // Empty on a CPU-only host. A timing without the hardware behind it is not a
  // measurement anyone can repeat.
  root["cuda_device"] = cam_loc::cuda::DeviceName();
  nlohmann::json cases = nlohmann::json::array();
  for (const auto& r : suite.cases) {
    nlohmann::json c;
    c["name"] = r.name;
    c["passed"] = r.passed;
    c["failure_reason"] = r.failure_reason;
    c["num_frames"] = r.num_frames;
    c["rmse_translation_m"] = r.summary.pose.rmse_translation_m;
    c["rmse_lateral_m"] = r.summary.pose.rmse_lateral_m;
    c["rmse_longitudinal_m"] = r.summary.pose.rmse_longitudinal_m;
    c["rmse_vertical_m"] = r.summary.pose.rmse_vertical_m;
    c["bias_lateral_m"] = r.summary.pose.bias_lateral_m;
    c["bias_longitudinal_m"] = r.summary.pose.bias_longitudinal_m;
    c["max_abs_lateral_m"] = r.summary.pose.max_abs_lateral_m;
    c["max_abs_longitudinal_m"] = r.summary.pose.max_abs_longitudinal_m;
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
    bool args_ok = false;
    const Options opt = ParseArgs(argc, argv, &args_ok);
    if (!args_ok) return 2;
    if (opt.data_root.empty()) {
      std::cerr << "--data-root is required; it is where the datasets live.\n"
                << "The scripts pass it for you: ./scripts/run_benchmark.sh\n";
      return 2;
    }
    std::vector<cam_loc::benchmark::BenchmarkCase> cases =
        cam_loc::benchmark::DefaultBenchmarkSuite(opt.data_root);

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
      const std::string calib_path =
          cam_loc::kitti::ResolveCalibPath(opt.data_root + "/smoke_kitti", 0);
      if (cam_loc::kitti::ParseCalibrationFile(calib_path, calib) !=
          cam_loc::Status::kOk) {
        std::cerr << "Micro-benchmark requires data/smoke_kitti (run "
                     "prepare_smoke_kitti.sh)\n";
        return 1;
      }
      const auto micro = cam_loc::benchmark::RunMicroBenchmarks(calib, 30);
      std::cout << "Micro-benchmarks (smoke calib, 30 iters):\n";
      for (const auto& m : micro) {
        std::cout << std::fixed << std::setprecision(3) << "  " << m.name
                  << (m.use_cuda ? " [cuda]" : " [cpu]")
                  << " mean=" << m.mean_ms << "ms p95=" << m.p95_ms << "ms\n";
      }
      if (!opt.output_json.empty()) {
        cam_loc::benchmark::BenchmarkSuiteResult empty;
        WriteJson(empty, micro, opt.output_json);
        std::cout << "Wrote " << opt.output_json << "\n";
      }
      return 0;
    }

    cam_loc::benchmark::BenchmarkSuiteResult suite;
    cam_loc::benchmark::RunBenchmarkSuite(cases, suite);

    if (cam_loc::cuda::IsAvailable()) {
      std::cout << "CUDA device: " << cam_loc::cuda::DeviceName() << "\n";
    }
    std::cout << "Benchmark suite: " << suite.passed << " passed, "
              << suite.failed << " failed\n\n";
    for (const auto& r : suite.cases) {
      PrintCaseResult(r);
    }

    if (!opt.output_json.empty()) {
      WriteJson(suite, {}, opt.output_json);
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
