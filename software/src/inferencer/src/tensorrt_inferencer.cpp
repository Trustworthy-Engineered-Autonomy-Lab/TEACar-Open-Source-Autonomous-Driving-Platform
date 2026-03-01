// inferencer.cpp (TensorRT backend, ROS 2 compatible, no ROS logging)

#include <cuda_runtime_api.h>
#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <memory>
#include <stdexcept>
#include <string>
#include <filesystem>

struct NvInferDeleter {
    template <typename T>
    void operator()(T* obj) const {
      if (obj) {
        obj->destroy();
      }
    }
  };

struct RTInferencer
{
    RTInferencer() : buffers(1, nullptr) {
        if (cudaStreamCreate(&stream) != cudaSuccess) {
          throw std::runtime_error("Failed to create CUDA stream");
        }
      }

    ~RTInferencer() {
    cudaStreamDestroy(stream);
    for (auto& buf : buffers) {
        if (buf) cudaFree(buf);
    }
    }


    class Logger : public nvinfer1::ILogger {
        void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING) {
            std::cerr << "[TensorRT] " << msg << std::endl;
        }
        }
    } logger;

    std::unique_ptr<nvinfer1::ICudaEngine, NvInferDeleter> engine;
    std::unique_ptr<nvinfer1::IExecutionContext, NvInferDeleter> context;
    std::vector<void*> buffers;
    cudaStream_t stream;
    std::string errorString;

}

namespace
{

std::unique_ptr<nvinfer1::ICudaEngine, NvInferDeleter> loadOnnx(RTInferencer* infer, const std::string& file) {
    auto builder = std::unique_ptr<nvinfer1::IBuilder, NvInferDeleter>{nvinfer1::createInferBuilder(infer->logger)};
    auto network = std::unique_ptr<nvinfer1::INetworkDefinition, NvInferDeleter>{builder->createNetworkV2(1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH))};
    auto parser = std::unique_ptr<nvonnxparser::IParser, NvInferDeleter>{nvonnxparser::createParser(*network, infer->logger)};
    
    if (!parser->parseFromFile(file.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kWARNING))) {
        infer->errorString = "Failed to parse ONNX: " + file;
        return nullptr;
    }
    
    auto config = std::unique_ptr<nvinfer1::IBuilderConfig, NvInferDeleter>{builder->createBuilderConfig()};
    size_t totalMem;
    cudaMemGetInfo(nullptr, &totalMem);
    config->setMaxWorkspaceSize(totalMem / 4);
    
    return std::unique_ptr<nvinfer1::ICudaEngine, NvInferDeleter>{builder->buildEngineWithConfig(*network, *config)};
    }

    std::unique_ptr<nvinfer1::ICudaEngine, NvInferDeleter> loadEngine(RTInferencer* infer, const std::string& path) {
        std::ifstream f(path, std::ios::binary);
        if (!f) return nullptr;
        f.seekg(0, std::ios::end);
        size_t size = f.tellg();
        f.seekg(0);
        std::vector<char> buf(size);
        f.read(buf.data(), size);
        auto runtime = std::unique_ptr<nvinfer1::IRuntime, NvInferDeleter>{nvinfer1::createInferRuntime(infer->logger)};
        return std::unique_ptr<nvinfer1::ICudaEngine, NvInferDeleter>{runtime->deserializeCudaEngine(buf.data(), size)};
      }

    bool saveEngine(RTInferencer* infer, const std::string& path) {
    auto ser = std::unique_ptr<nvinfer1::IHostMemory, NvInferDeleter>{infer->engine->serialize()};
    std::ofstream f(path, std::ios::binary);
    if (!f) return false;
    f.write(static_cast<const char*>(ser->data()), ser->size());
    return true;
    }

    bool saveEngine(RTInferencer* infer, const std::string& path) {
        auto ser = std::unique_ptr<nvinfer1::IHostMemory, NvInferDeleter>{infer->engine->serialize()};
        std::ofstream f(path, std::ios::binary);
        if (!f) return false;
        f.write(static_cast<const char*>(ser->data()), ser->size());
        return true;
      }
    
    size_t getVolume(const nvinfer1::Dims& dims) {
        size_t vol = 1;
        for (int i = 0; i < dims.nbDims; ++i) vol *= dims.d[i];
        return vol;
    }

    size_t getTypeSize(nvinfer1::DataType type) {
        switch (type) {
        case nvinfer1::DataType::kFLOAT: return 4;
        case nvinfer1::DataType::kHALF: return 2;
        case nvinfer1::DataType::kINT8: return 1;
        case nvinfer1::DataType::kINT32: return 4;
        case nvinfer1::DataType::kBOOL: return 1;
        default: return 0;
        }
    }

    void* alloc(RTInferencer* inf, int idx, size_t sz) {
        if (idx >= inf->buffers.size()) inf->buffers.resize(idx + 1, nullptr);
        if (!inf->buffers[idx]) cudaMallocManaged(&inf->buffers[idx], sz);
        return inf->buffers[idx];
      }

    extern "C" void* createInferencer(void*) {
    return new RTInferencer();
    }
    
    extern "C" void deleteInferencer(void* handle) {
    delete static_cast<RTInferencer*>(handle);
    }

    extern "C" bool loadModel(void* h, const char* name) {
        auto* i = static_cast<RTInferencer*>(h);
        std::filesystem::path p(name);
        if (p.extension() == ".engine") i->engine = loadEngine(i, name);
        else if (p.extension() == ".onnx") {
          auto cached = p;
          cached.replace_extension(".engine");
          if (std::filesystem::exists(cached)) i->engine = loadEngine(i, cached);
          if (!i->engine) {
            i->engine = loadOnnx(i, name);
            if (i->engine) saveEngine(i, cached);
          }
        } else {
          i->errorString = "Unsupported model format";
          return false;
        }
        if (!i->engine) return false;
        i->context.reset(i->engine->createExecutionContext());
        return i->context != nullptr;
      }
    
    extern "C" unsigned getInputBuffer(void* h, const char* name, void** buf) {
    auto* i = static_cast<RTInferencer*>(h);
    int idx = i->engine->getBindingIndex(name);
    if (idx == -1) return 0;
    auto dims = i->engine->getBindingDimensions(idx);
    auto dtype = i->engine->getBindingDataType(idx);
    size_t sz = getVolume(dims) * getTypeSize(dtype);
    *buf = alloc(i, idx, sz);
    return sz;
    }
       
    extern "C" unsigned getOutputBuffer(void* h, const char* name, void** buf) {
    auto* i = static_cast<RTInferencer*>(h);
    int idx = i->engine->getBindingIndex(name);
    if (idx == -1) return 0;
    auto dims = i->engine->getBindingDimensions(idx);
    auto dtype = i->engine->getBindingDataType(idx);
    size_t sz = getVolume(dims) * getTypeSize(dtype);
    *buf = alloc(i, idx, sz);
    return sz;
    }
      
    extern "C" bool infer(void* h) {
    auto* i = static_cast<RTInferencer*>(h);
    if (!i->context->enqueueV2(i->buffers.data(), i->stream, nullptr)) {
        i->errorString = "Failed to enqueue inference";
        return false;
    }

    
    cudaStreamSynchronize(i->stream);
    return true;
    }
      
    extern "C" const char* getErrorString(void* h) {
    return static_cast<RTInferencer*>(h)->errorString.c_str();
    }
      


      


}