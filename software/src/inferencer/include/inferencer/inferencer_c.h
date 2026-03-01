#ifdef __cplusplus
extern "C" {
#endif

void* createInferencer(void* options);
void deleteInferencer(void* inferencer);
bool loadModel(void* inferencer, const char* modelName);
unsigned getInputBuffer(void* inferencer, const char* inputName, void** buffer);
unsigned getOutputBuffer(void* inferencer, const char* outputName, void** buffer);
bool infer(void* inferencer);
const char* getErrorString(void* inferencer);

#ifdef __cplusplus
}
#endif