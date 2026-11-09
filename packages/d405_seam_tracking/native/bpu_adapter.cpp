// Original adapter to installed S100 DNN/UCP 3.13.6 API, no SDK source copied.
// External tensors must be F32. Quantized internal ops are compiler responsibility.
#include <hobot/dnn/hb_dnn.h>
#include <hobot/hb_ucp.h>
#include <hobot/hb_ucp_sys.h>
#include <cstring>
#include <string>
#include <sstream>
#include <stdexcept>

struct Adapter {
    hbDNNPackedHandle_t packed=nullptr;
    hbDNNHandle_t model=nullptr;
    hbDNNTensor input{}, output{};
    hbUCPTaskHandle_t task=nullptr;
    bool failed=false;
    std::string info;
};
static thread_local std::string error;
static void check(int code, const char* stage) {
    if(code) throw std::runtime_error(std::string(stage)+": "+std::to_string(code));
}
static size_t elements(const hbDNNTensorProperties& p) {
    size_t count=1;
    for(int i=0;i<p.validShape.numDimensions;++i) {
        if(p.validShape.dimensionSize[i]<=0) throw std::runtime_error("dynamic/invalid tensor shape unsupported");
        count*=p.validShape.dimensionSize[i];
    }
    return count;
}
static size_t offset(size_t flat, const hbDNNTensorProperties& p) {
    size_t result=0;
    for(int i=p.validShape.numDimensions-1;i>=0;--i) {
        if(p.stride[i]<=0) throw std::runtime_error("unknown tensor stride");
        result+=(flat%p.validShape.dimensionSize[i])*p.stride[i];
        flat/=p.validShape.dimensionSize[i];
    }
    return result;
}
static std::string shape(const hbDNNTensorProperties& p) {
    std::ostringstream s;s<<"[";
    for(int i=0;i<p.validShape.numDimensions;++i) {if(i)s<<",";s<<p.validShape.dimensionSize[i];}
    s<<"]";return s.str();
}
static void cleanup(Adapter* a) {
    // Caller MUST establish task completion first. Never free device memory still in use.
    if(a->task) {check(hbUCPReleaseTask(a->task),"release_task");a->task=nullptr;}
    if(a->input.sysMem.virAddr) {check(hbUCPFree(&a->input.sysMem),"free_input");a->input.sysMem={};}
    if(a->output.sysMem.virAddr) {check(hbUCPFree(&a->output.sysMem),"free_output");a->output.sysMem={};}
    if(a->packed) {check(hbDNNRelease(a->packed),"release_model");a->packed=nullptr;}
}
extern "C" const char* d405_bpu_error() {return error.c_str();}
extern "C" void* d405_bpu_open(const char* path) {
    auto* a=new Adapter;
    try {
        check(hbDNNInitializeFromFiles(&a->packed,&path,1),"load_hbm");
        const char** names=nullptr;int count=0;
        check(hbDNNGetModelNameList(&names,&count,a->packed),"model_names");
        if(count!=1) throw std::runtime_error("one model required");
        check(hbDNNGetModelHandle(&a->model,a->packed,names[0]),"model_handle");
        int in=0,out=0;check(hbDNNGetInputCount(&in,a->model),"input_count");check(hbDNNGetOutputCount(&out,a->model),"output_count");
        if(in!=1||out!=1) throw std::runtime_error("one float luminance input and one binary logit output required");
        check(hbDNNGetInputTensorProperties(&a->input.properties,a->model,0),"input_properties");
        check(hbDNNGetOutputTensorProperties(&a->output.properties,a->model,0),"output_properties");
        for(auto* tensor:{&a->input,&a->output}) {
            const auto& p=tensor->properties;
            if(p.tensorType!=HB_DNN_TENSOR_TYPE_F32 || p.quantiType!=NONE) throw std::runtime_error("external nonquantized F32 required; do not guess quantization");
            if(p.alignedByteSize<=0 || offset(elements(p)-1,p)+sizeof(float)>size_t(p.alignedByteSize)) throw std::runtime_error("invalid aligned stride bounds");
            check(hbUCPMallocCached(&tensor->sysMem,p.alignedByteSize,0),"allocate_cached");
        }
        a->info="{\"sdk\":\""+std::string(hbDNNGetVersion())+"\",\"input_shape\":"+shape(a->input.properties)+",\"output_shape\":"+shape(a->output.properties)+"}";
        return a;
    } catch(const std::exception& e) {error=e.what();try{cleanup(a);}catch(...){}delete a;return nullptr;}
}
extern "C" const char* d405_bpu_info(void* handle) {return static_cast<Adapter*>(handle)->info.c_str();}
extern "C" int d405_bpu_run(void* handle,const float* input,size_t input_count,float* output,size_t output_count,int timeout_ms) {
    auto* a=static_cast<Adapter*>(handle);
    try {
        if(a->failed||a->task) throw std::runtime_error("previous device failure/task pending; refuse new inference");
        if(input_count!=elements(a->input.properties)||output_count!=elements(a->output.properties)||timeout_ms<=0) throw std::runtime_error("shape/deadline mismatch");
        std::memset(a->input.sysMem.virAddr,0,a->input.sysMem.memSize);
        for(size_t i=0;i<input_count;++i) std::memcpy(static_cast<char*>(a->input.sysMem.virAddr)+offset(i,a->input.properties),input+i,sizeof(float));
        check(hbUCPMemFlush(&a->input.sysMem,HB_SYS_MEM_CACHE_CLEAN),"clean_input_cache");
        check(hbDNNInferV2(&a->task,&a->output,&a->input,a->model),"create_infer_task");
        hbUCPSchedParam params{};HB_UCP_INITIALIZE_SCHED_PARAM(&params);params.backend=HB_UCP_BPU_CORE_0;
        check(hbUCPSubmitTask(a->task,&params),"submit_task");
        check(hbUCPWaitTaskDone(a->task,timeout_ms),"wait_task_deadline");
        check(hbUCPMemFlush(&a->output.sysMem,HB_SYS_MEM_CACHE_INVALIDATE),"invalidate_output_cache");
        for(size_t i=0;i<output_count;++i) std::memcpy(output+i,static_cast<char*>(a->output.sysMem.virAddr)+offset(i,a->output.properties),sizeof(float));
        check(hbUCPReleaseTask(a->task),"release_task");a->task=nullptr;return 0;
    } catch(const std::exception& e) {a->failed=true;error=e.what();return -1;}
}
extern "C" int d405_bpu_close(void* handle) {
    if(!handle)return 0;auto* a=static_cast<Adapter*>(handle);
    try {
        if(a->task)check(hbUCPWaitTaskDone(a->task,1),"close_task_still_active_memory_retained");
        cleanup(a);delete a;return 0;
    } catch(const std::exception& e) {error=e.what();return -1;}
}
