// inferencer.hpp (ROS 2 Compatible Version)
#ifndef INFERENCER__INFERENCER_HPP_
#define INFERENCER__INFERENCER_HPP_

#include <functional>
#include <string>
#include <stdexcept>
#include <dlfcn.h>

namespace inferencer
{
    class Inferencer
    {

    public:
        explicit Inferencer(const std::string &backend)
        {
            std::string lib_name = "lib" + backend + "_inferencer.so";
            dll_handle_ = dlopen(lib_name.c_str(), RTLD_LAZY);
            if (!dll_handle_) {
                throw std::runtime_error("Failed to load " + lib_name + ": " + dlerror());
              }

            create_inferencer_ = reinterpret_cast<CreateInferencerFunc>(dlsym(dll_handle_, "createInferencer"));
            delete_inferencer_ = reinterpret_cast<DeleteInferencerFunc>(dlsym(dll_handle_, "deleteInferencer"));
            auto load_model_fn = reinterpret_cast<LoadModelFunc>(dlsym(dll_handle_, "loadModel"));
            auto get_input_fn = reinterpret_cast<GetInputBufferFunc>(dlsym(dll_handle_, "getInputBuffer"));
            auto get_output_fn = reinterpret_cast<GetOutputBufferFunc>(dlsym(dll_handle_, "getOutputBuffer"));
            auto infer_fn = reinterpret_cast<InferFunc>(dlsym(dll_handle_, "infer"));
            auto get_error_fn = reinterpret_cast<GetErrorStringFunc>(dlsym(dll_handle_, "getErrorString"));

            if (!create_inferencer_ || !delete_inferencer_ || !load_model_fn ||
                !get_input_fn || !get_output_fn || !infer_fn || !get_error_fn)
            {
              throw std::runtime_error("Failed to load function pointers from " + lib_name);
            }

            handle_ = create_inferencer_(nullptr);
            if (!handle_) {
              throw std::runtime_error("Failed to create inferencer instance");
            }

            load_model_ = std::bind(load_model_fn, handle_, std::placeholders::_1);
            get_input_buffer_ = std::bind(get_input_fn, handle_, std::placeholders::_1, std::placeholders::_2);
            get_output_buffer_ = std::bind(get_output_fn, handle_, std::placeholders::_1, std::placeholders::_2);
            infer_ = std::bind(infer_fn, handle_);
            get_error_string_ = std::bind(get_error_fn, handle_);
        }

        ~Inferencer()
        {
            if (handle_) delete_inferencer_(handle_);
            if (dll_handle_) dlclose(dll_handle_);
        }
    
    private:
        using CreateInferencerFunc = void *(*)(void *);
        using DeleteInferencerFunc = void (*)(void *);
        using LoadModelFunc = bool (*)(void *, const char *);
        using GetInputBufferFunc = size_t (*)(void *, const char *, void **);
        using GetOutputBufferFunc = size_t (*)(void *, const char *, void **);
        using InferFunc = bool (*)(void *);
        using GetErrorStringFunc = const char *(*)(void *);
        void *dll_handle_ = nullptr;
        void *handle_ = nullptr;
        CreateInferencerFunc create_inferencer_ = nullptr;
        DeleteInferencerFunc delete_inferencer_ = nullptr;

        std::function<bool(const char *)> load_model_;
        std::function<size_t(const char *, void **)> get_input_buffer_;
        std::function<size_t(const char *, void **)> get_output_buffer_;
        std::function<bool()> infer_;
        std::function<const char *()> get_error_string_;

    };
}

#endif  // INFERENCER__INFERENCER_HPP_