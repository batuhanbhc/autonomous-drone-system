// Example ONNX Runtime inference driver for the exported deterministic actor.
//
// The current exported model takes:
//   - grid       [B, C, H, W]
//   - local_base [B, base_local_dim]
//   - move_mask  [B, num_move_bins]
//
// and returns:
//   - action     [B, 3] = (vx, vy, yaw_rate)
//
// Example build command (adjust include/lib paths for your machine):
//   g++ -std=c++17 -O2 onnx/run_random_inference.cpp \
//       -I/path/to/onnxruntime/include \
//       -L/path/to/onnxruntime/lib -lonnxruntime \
//       -o onnx/run_random_inference
//
// Example usage:
//   ./onnx/run_random_inference onnx/ckpt_update_280_deterministic.onnx

#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace {

std::vector<int64_t> ResolveShape(const std::vector<int64_t>& raw_shape) {
    std::vector<int64_t> resolved = raw_shape;
    for (std::size_t i = 0; i < resolved.size(); ++i) {
        if (resolved[i] <= 0) {
            resolved[i] = 1;
        }
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

std::vector<float> RandomInputData(
    const std::string& input_name,
    const std::vector<int64_t>& shape
) {
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> uniform_dist(-1.0f, 1.0f);

    std::vector<float> data(NumElements(shape), 0.0f);
    if (input_name == "move_mask") {
        std::fill(data.begin(), data.end(), 1.0f);
        return data;
    }

    for (float& value : data) {
        value = uniform_dist(rng);
    }
    return data;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <model.onnx>\n";
        return EXIT_FAILURE;
    }

    const std::string model_path = argv[1];

    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "deterministic_actor");
    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(1);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

    Ort::Session session(env, model_path.c_str(), session_options);
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
    std::vector<Ort::Value> input_tensors;
    input_name_storage.reserve(num_inputs);
    input_names.reserve(num_inputs);
    input_shapes.reserve(num_inputs);
    input_data_storage.reserve(num_inputs);
    input_tensors.reserve(num_inputs);

    for (std::size_t i = 0; i < num_inputs; ++i) {
        Ort::AllocatedStringPtr name = session.GetInputNameAllocated(i, allocator);
        input_name_storage.emplace_back(name.get());
        input_names.push_back(input_name_storage.back().c_str());

        Ort::TypeInfo type_info = session.GetInputTypeInfo(i);
        auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
        std::vector<int64_t> resolved_shape = ResolveShape(tensor_info.GetShape());
        input_shapes.push_back(resolved_shape);
        std::cout << "input[" << i << "] " << input_name_storage.back() << " shape=[";
        for (std::size_t j = 0; j < resolved_shape.size(); ++j) {
            std::cout << resolved_shape[j];
            if (j + 1 < resolved_shape.size()) {
                std::cout << ", ";
            }
        }
        std::cout << "]\n";

        input_data_storage.push_back(RandomInputData(input_name_storage.back(), resolved_shape));
        input_tensors.emplace_back(
            Ort::Value::CreateTensor<float>(
                memory_info,
                input_data_storage.back().data(),
                input_data_storage.back().size(),
                input_shapes.back().data(),
                input_shapes.back().size()
            )
        );
    }

    const std::size_t num_outputs = session.GetOutputCount();
    std::vector<std::string> output_name_storage;
    std::vector<const char*> output_names;
    output_name_storage.reserve(num_outputs);
    output_names.reserve(num_outputs);

    for (std::size_t i = 0; i < num_outputs; ++i) {
        Ort::AllocatedStringPtr name = session.GetOutputNameAllocated(i, allocator);
        output_name_storage.emplace_back(name.get());
        output_names.push_back(output_name_storage.back().c_str());
    }

    auto outputs = session.Run(
        Ort::RunOptions{nullptr},
        input_names.data(),
        input_tensors.data(),
        input_tensors.size(),
        output_names.data(),
        output_names.size()
    );

    for (std::size_t i = 0; i < outputs.size(); ++i) {
        auto tensor_info = outputs[i].GetTensorTypeAndShapeInfo();
        std::vector<int64_t> shape = tensor_info.GetShape();
        const float* values = outputs[i].GetTensorData<float>();
        const std::size_t count = tensor_info.GetElementCount();

        std::cout << output_name_storage[i] << " shape=[";
        for (std::size_t j = 0; j < shape.size(); ++j) {
            std::cout << shape[j];
            if (j + 1 < shape.size()) {
                std::cout << ", ";
            }
        }
        std::cout << "] values=";

        for (std::size_t j = 0; j < count; ++j) {
            std::cout << values[j];
            if (j + 1 < count) {
                std::cout << ' ';
            }
        }
        std::cout << '\n';
    }

    if (outputs.size() == 1) {
        auto tensor_info = outputs[0].GetTensorTypeAndShapeInfo();
        if (tensor_info.GetElementCount() >= 3) {
            const float* values = outputs[0].GetTensorData<float>();
            std::cout << "first action -> vx=" << values[0]
                      << " vy=" << values[1]
                      << " yaw_rate=" << values[2] << '\n';
        }
    }

    return EXIT_SUCCESS;
}
