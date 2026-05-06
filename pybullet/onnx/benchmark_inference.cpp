// Benchmark ONNX Runtime CPU inference across session configurations.
//
// Example:
//   ./benchmark_inference ./marl_agent.onnx \
//     --iterations 400 --warmup 80 \
//     --intra-threads 1,2,4 \
//     --inter-threads 1,2 \
//     --modes sequential,parallel \
//     --spinning on,off

#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;
using ExecutionMode = ::ExecutionMode;

struct Options {
    std::string model_path;
    int warmup_runs = 50;
    int iterations = 200;
    int batch_size = 1;
    bool random_inputs = true;
    std::vector<int> intra_threads{1, 2, 4};
    std::vector<int> inter_threads{1};
    std::vector<ExecutionMode> execution_modes{
        ORT_SEQUENTIAL,
        ORT_PARALLEL,
    };
    std::vector<bool> spinning_options{true, false};
};

struct BenchmarkConfig {
    int intra_threads = 1;
    int inter_threads = 1;
    ExecutionMode execution_mode = ORT_SEQUENTIAL;
    bool allow_spinning = true;
};

struct BenchmarkResult {
    BenchmarkConfig config;
    double mean_ms = 0.0;
    double p50_ms = 0.0;
    double p95_ms = 0.0;
    double min_ms = 0.0;
    double max_ms = 0.0;
};

void PrintUsage(const char* argv0) {
    std::cerr
        << "Usage: " << argv0 << " <model.onnx> [options]\n"
        << "Options:\n"
        << "  --warmup N            Warmup runs per config (default: 50)\n"
        << "  --iterations N        Timed runs per config (default: 200)\n"
        << "  --batch N             Batch size for dynamic inputs (default: 1)\n"
        << "  --zeros               Use zero inputs instead of random floats\n"
        << "  --intra-threads LIST  Comma-separated list, e.g. 1,2,4\n"
        << "  --inter-threads LIST  Comma-separated list, e.g. 1,2\n"
        << "  --modes LIST          sequential,parallel or both\n"
        << "  --spinning LIST       on,off or both\n";
}

std::vector<std::string> Split(const std::string& text, char delim) {
    std::vector<std::string> parts;
    std::stringstream ss(text);
    std::string item;
    while (std::getline(ss, item, delim)) {
        if (!item.empty()) {
            parts.push_back(item);
        }
    }
    return parts;
}

std::vector<int> ParseIntList(const std::string& text, const char* flag_name) {
    std::vector<int> values;
    for (const std::string& part : Split(text, ',')) {
        try {
            const int value = std::stoi(part);
            if (value < 0) {
                throw std::invalid_argument("negative");
            }
            values.push_back(value);
        } catch (const std::exception&) {
            throw std::runtime_error(std::string("Invalid value for ") + flag_name + ": " + part);
        }
    }
    if (values.empty()) {
        throw std::runtime_error(std::string("No values provided for ") + flag_name);
    }
    return values;
}

std::vector<ExecutionMode> ParseExecutionModes(const std::string& text) {
    std::vector<ExecutionMode> values;
    for (const std::string& part : Split(text, ',')) {
        if (part == "sequential") {
            values.push_back(ORT_SEQUENTIAL);
        } else if (part == "parallel") {
            values.push_back(ORT_PARALLEL);
        } else {
            throw std::runtime_error("Invalid value for --modes: " + part);
        }
    }
    if (values.empty()) {
        throw std::runtime_error("No values provided for --modes");
    }
    return values;
}

std::vector<bool> ParseSpinningOptions(const std::string& text) {
    std::vector<bool> values;
    for (const std::string& part : Split(text, ',')) {
        if (part == "on") {
            values.push_back(true);
        } else if (part == "off") {
            values.push_back(false);
        } else {
            throw std::runtime_error("Invalid value for --spinning: " + part);
        }
    }
    if (values.empty()) {
        throw std::runtime_error("No values provided for --spinning");
    }
    return values;
}

Options ParseArgs(int argc, char** argv) {
    if (argc < 2) {
        PrintUsage(argv[0]);
        std::exit(EXIT_FAILURE);
    }

    Options options;
    options.model_path = argv[1];
    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        auto require_value = [&](const char* flag_name) -> std::string {
            if (i + 1 >= argc) {
                throw std::runtime_error(std::string("Missing value for ") + flag_name);
            }
            return argv[++i];
        };

        if (arg == "--warmup") {
            options.warmup_runs = std::stoi(require_value("--warmup"));
        } else if (arg == "--iterations") {
            options.iterations = std::stoi(require_value("--iterations"));
        } else if (arg == "--batch") {
            options.batch_size = std::stoi(require_value("--batch"));
        } else if (arg == "--zeros") {
            options.random_inputs = false;
        } else if (arg == "--intra-threads") {
            options.intra_threads = ParseIntList(require_value("--intra-threads"), "--intra-threads");
        } else if (arg == "--inter-threads") {
            options.inter_threads = ParseIntList(require_value("--inter-threads"), "--inter-threads");
        } else if (arg == "--modes") {
            options.execution_modes = ParseExecutionModes(require_value("--modes"));
        } else if (arg == "--spinning") {
            options.spinning_options = ParseSpinningOptions(require_value("--spinning"));
        } else if (arg == "--help" || arg == "-h") {
            PrintUsage(argv[0]);
            std::exit(EXIT_SUCCESS);
        } else {
            throw std::runtime_error("Unknown argument: " + arg);
        }
    }

    if (options.warmup_runs < 0 || options.iterations <= 0 || options.batch_size <= 0) {
        throw std::runtime_error("Warmup must be >= 0, iterations > 0, and batch > 0");
    }
    return options;
}

std::vector<int64_t> ResolveShape(
    const std::vector<int64_t>& raw_shape,
    int batch_size
) {
    std::vector<int64_t> resolved = raw_shape;
    for (std::size_t i = 0; i < resolved.size(); ++i) {
        if (resolved[i] <= 0) {
            resolved[i] = (i == 0) ? batch_size : 1;
        }
    }
    if (!resolved.empty()) {
        resolved[0] = batch_size;
    }
    return resolved;
}

std::size_t NumElements(const std::vector<int64_t>& shape) {
    std::size_t total = 1;
    for (int64_t dim : shape) {
        total *= static_cast<std::size_t>(dim);
    }
    return total;
}

std::vector<float> MakeInputData(
    const std::string& input_name,
    const std::vector<int64_t>& shape,
    bool use_random
) {
    std::vector<float> data(NumElements(shape), 0.0f);
    if (input_name == "move_mask") {
        std::fill(data.begin(), data.end(), 1.0f);
        return data;
    }
    if (!use_random) {
        return data;
    }

    std::mt19937 rng(42);
    std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
    for (float& value : data) {
        value = dist(rng);
    }
    return data;
}

std::vector<std::string> GetOutputNames(
    Ort::Session& session,
    Ort::AllocatorWithDefaultOptions& allocator
) {
    const std::size_t num_outputs = session.GetOutputCount();
    std::vector<std::string> names;
    names.reserve(num_outputs);
    for (std::size_t i = 0; i < num_outputs; ++i) {
        Ort::AllocatedStringPtr name = session.GetOutputNameAllocated(i, allocator);
        names.emplace_back(name.get());
    }
    return names;
}

std::string ModeToString(ExecutionMode mode) {
    return mode == ORT_PARALLEL ? "parallel" : "sequential";
}

void PrintConfig(const BenchmarkConfig& config) {
    std::cout
        << "mode=" << ModeToString(config.execution_mode)
        << " intra=" << config.intra_threads
        << " inter=" << config.inter_threads
        << " spinning=" << (config.allow_spinning ? "on" : "off");
}

Ort::SessionOptions MakeSessionOptions(const BenchmarkConfig& config) {
    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(config.intra_threads);
    session_options.SetInterOpNumThreads(config.inter_threads);
    session_options.SetExecutionMode(config.execution_mode);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
    if (!config.allow_spinning) {
        session_options.AddConfigEntry("session.intra_op.allow_spinning", "0");
        session_options.AddConfigEntry("session.inter_op.allow_spinning", "0");
    }
    return session_options;
}

BenchmarkResult RunBenchmark(
    Ort::Env& env,
    const Options& options,
    const BenchmarkConfig& config
) {
    Ort::SessionOptions session_options = MakeSessionOptions(config);
    Ort::Session session(env, options.model_path.c_str(), session_options);
    Ort::AllocatorWithDefaultOptions allocator;
    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(
        OrtArenaAllocator,
        OrtMemTypeDefault
    );

    const std::size_t num_inputs = session.GetInputCount();
    std::vector<std::string> input_name_storage;
    std::vector<const char*> input_names;
    std::vector<std::vector<int64_t>> input_shapes;
    std::vector<std::vector<float>> input_data_storage;
    input_name_storage.reserve(num_inputs);
    input_names.reserve(num_inputs);
    input_shapes.reserve(num_inputs);
    input_data_storage.reserve(num_inputs);

    for (std::size_t i = 0; i < num_inputs; ++i) {
        Ort::AllocatedStringPtr name = session.GetInputNameAllocated(i, allocator);
        input_name_storage.emplace_back(name.get());
        input_names.push_back(input_name_storage.back().c_str());

        Ort::TypeInfo type_info = session.GetInputTypeInfo(i);
        auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
        input_shapes.push_back(ResolveShape(tensor_info.GetShape(), options.batch_size));
        input_data_storage.push_back(
            MakeInputData(input_name_storage.back(), input_shapes.back(), options.random_inputs)
        );

    }

    static bool printed_input_shapes = false;
    if (!printed_input_shapes) {
        std::cout << "Resolved inputs:\n";
        for (std::size_t i = 0; i < input_name_storage.size(); ++i) {
            std::cout << "  " << input_name_storage[i] << " shape=[";
            for (std::size_t j = 0; j < input_shapes[i].size(); ++j) {
                std::cout << input_shapes[i][j];
                if (j + 1 < input_shapes[i].size()) {
                    std::cout << ", ";
                }
            }
            std::cout << "]\n";
        }
        printed_input_shapes = true;
    }

    std::vector<std::string> output_name_storage = GetOutputNames(session, allocator);
    std::vector<const char*> output_names;

    output_names.reserve(output_name_storage.size());
    for (const std::string& name : output_name_storage) {
        output_names.push_back(name.c_str());
    }

    auto run_once = [&]() {
        std::vector<Ort::Value> input_tensors;
        input_tensors.reserve(input_name_storage.size());
        for (std::size_t i = 0; i < input_name_storage.size(); ++i) {
            input_tensors.emplace_back(
                Ort::Value::CreateTensor<float>(
                    memory_info,
                    input_data_storage[i].data(),
                    input_data_storage[i].size(),
                    input_shapes[i].data(),
                    input_shapes[i].size()
                )
            );
        }

        auto outputs = session.Run(
            Ort::RunOptions{nullptr},
            input_names.data(),
            input_tensors.data(),
            input_tensors.size(),
            output_names.data(),
            output_names.size()
        );
        volatile std::size_t sink = outputs.size();
        (void)sink;
    };

    for (int i = 0; i < options.warmup_runs; ++i) {
        run_once();
    }

    std::vector<double> timings_ms;
    timings_ms.reserve(static_cast<std::size_t>(options.iterations));
    for (int i = 0; i < options.iterations; ++i) {
        const auto start = Clock::now();
        run_once();
        const auto end = Clock::now();
        const auto elapsed_us =
            std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        timings_ms.push_back(static_cast<double>(elapsed_us) / 1000.0);
    }

    std::vector<double> sorted = timings_ms;
    std::sort(sorted.begin(), sorted.end());

    auto percentile = [&](double fraction) {
        const std::size_t idx = static_cast<std::size_t>(fraction * (sorted.size() - 1));
        return sorted[idx];
    };

    const double sum = std::accumulate(timings_ms.begin(), timings_ms.end(), 0.0);
    BenchmarkResult result;
    result.config = config;
    result.mean_ms = sum / static_cast<double>(timings_ms.size());
    result.p50_ms = percentile(0.50);
    result.p95_ms = percentile(0.95);
    result.min_ms = sorted.front();
    result.max_ms = sorted.back();
    return result;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Options options = ParseArgs(argc, argv);
        Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "benchmark_inference");

        std::vector<BenchmarkResult> results;
        for (ExecutionMode mode : options.execution_modes) {
            for (int intra_threads : options.intra_threads) {
                for (int inter_threads : options.inter_threads) {
                    for (bool allow_spinning : options.spinning_options) {
                        const BenchmarkConfig config{
                            intra_threads,
                            inter_threads,
                            mode,
                            allow_spinning,
                        };
                        std::cout << "Running ";
                        PrintConfig(config);
                        std::cout << " ...\n";
                        results.push_back(RunBenchmark(env, options, config));
                    }
                }
            }
        }

        std::sort(
            results.begin(),
            results.end(),
            [](const BenchmarkResult& lhs, const BenchmarkResult& rhs) {
                return lhs.mean_ms < rhs.mean_ms;
            }
        );

        std::cout << "\nResults sorted by mean latency (ms)\n";
        std::cout << std::left
                  << std::setw(12) << "mode"
                  << std::setw(8) << "intra"
                  << std::setw(8) << "inter"
                  << std::setw(10) << "spin"
                  << std::setw(12) << "mean"
                  << std::setw(12) << "p50"
                  << std::setw(12) << "p95"
                  << std::setw(12) << "min"
                  << std::setw(12) << "max"
                  << '\n';

        for (const BenchmarkResult& result : results) {
            std::cout << std::left
                      << std::setw(12) << ModeToString(result.config.execution_mode)
                      << std::setw(8) << result.config.intra_threads
                      << std::setw(8) << result.config.inter_threads
                      << std::setw(10) << (result.config.allow_spinning ? "on" : "off")
                      << std::setw(12) << std::fixed << std::setprecision(3) << result.mean_ms
                      << std::setw(12) << result.p50_ms
                      << std::setw(12) << result.p95_ms
                      << std::setw(12) << result.min_ms
                      << std::setw(12) << result.max_ms
                      << '\n';
        }

        if (!results.empty()) {
            std::cout << "\nFastest config: ";
            PrintConfig(results.front().config);
            std::cout << " mean_ms=" << std::fixed << std::setprecision(3)
                      << results.front().mean_ms << '\n';
        }
    } catch (const Ort::Exception& ex) {
        std::cerr << "ONNX Runtime error: " << ex.what() << '\n';
        return EXIT_FAILURE;
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << '\n';
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
