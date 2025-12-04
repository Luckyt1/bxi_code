/*************************
 * 星火大模型文本交互Demo
 * create by wxw
 * 2024-12-17
 * ***********************/
#include "../../../include/sparkchain.h"
#include <iostream>
#include <string>
#include <atomic>
#include <fstream>
#include <vector>
#include <unistd.h>
#include <regex>
#include <stdio.h>
#include <stdlib.h>



#define GREEN "\033[32m"
#define YELLOW "\033[33m"
#define RED "\033[31m"
#define RESET "\033[0m"

using namespace SparkChain;
using namespace std;

// async status tag
static atomic_bool finish(false);

static FILE * file = nullptr;
int choice = 1; //演示模式选择，0：同步，1：异步。

/*************************************SDK初始化参数**********************************************************/
char * APPID     = "APPID";                        //用户的APPID
char * APIKEY    = "APIKEY";                       //用户的APIKey
char * APISECRET = "APISECRET";                    //用户的APISecret
char * WORKDIR   = "./";                                    //SDK工作目录，要求有读写权限
int logLevel     = 100;                                      //日志等级。0：VERBOSE(日志最全)，1：DEBUG，2：INFO，3：WARN，4：ERROR，5：FATAL，100：OFF(关闭日志)
/*************************************SDK初始化参数**********************************************************/

//交互结果监听回调
class SparkCallbacks : public LLMCallbacks
{
    void onLLMResult(LLMResult *result, void *usrContext)
    {
        if(result->getContentType() == LLMResult::TEXT){
            //解析获取的交互结果，示例展示所有结果获取，开发者可根据自身需要，选择获取。
            const char* content  = result->getContent();//获取返回结果
            int contentLen       = result->getContentLen();//获取返回结果长度
            SparkChain::LLMResult::ContentType type = result->getContentType();//获取返回结果类型,TEXT:返回结果为文本,IMAGE:返回结果为图片
            int status           = result->getStatus();//返回结果状态，0：结果返回第一帧，1：结果返回中间帧，2：结果返回最终帧
            const char* role     = result->getRole();//获取角色信息
            const char* sid      = result->getSid();//获取本次会话的sid
            int completionTokens = result->getCompletionTokens();//获取回答的Token大小
            int promptTokens     = result->getPromptTokens();//包含历史问题的总Tokens大小
            int totalTokens      = result->getTotalTokens();//promptTokens和completionTokens的和，也是本次交互计费的Tokens大小
            const char* function = result->getFunctionCall();//获取FunctionCall结果
            if(choice == 2){
                printf(YELLOW "FunctionCall结果: %s\n" RESET, function);                
            }else{
                printf(YELLOW "%s" RESET, content);
            }
            
            if (status == 2)
            {
                printf(YELLOW "\n" RESET, content);
                finish = true;
            } 
        }   
    }

    void onLLMEvent(LLMEvent *event, void *usrContext)
    {
        //解析获取的事件结果，示例展示所有结果获取，开发者可根据自身需要，选择获取。
        int eventId          = event->getEventID();//获取事件id
        const char* eventMsg = event->getEventMsg();//获取事件信息
        const char* sid      = event->getSid();//获取交互sid
    }

    void onLLMError(LLMError *error, void *usrContext)
    {
        //解析获取的错误结果，示例展示所有结果获取，开发者可根据自身需要，选择获取。
        int errCode        = error->getErrCode();//获取错误码
        const char* errMsg = error->getErrMsg();//获取错误信息
        const char* sid    = error->getSid();//获取交互sid

        printf(RED "请求出错,错误码: %d,错误信息:%s,交互sid:%s\n" RESET, errCode, errMsg, sid);
        finish = true;  
    }
};



/***
 * SDK初始化
 * ***/
int initSDK()
{
    SparkChainConfig *config = SparkChainConfig::builder();
    config->appID(APPID)        // 你的appid
        ->apiKey(APIKEY)        // 你的apikey
        ->apiSecret(APISECRET)   // 你的apisecret
        ->workDir(WORKDIR)
        ->logLevel(logLevel); 
    int ret = SparkChain::init(config);
    return ret;
}

/****************
 * SDK支持通过原始json进行大模型交互
 * **********************/
void run_ArunWithJson(){
    LLMConfig *llmConfig = LLMConfig::builder();
    /****************************************
     * 选择要使用的大模型类型(需开通相应的授权)：
     * general:      通用大模型Spark Lite版本
     * generalv3：   通用大模型Spark Pro版本 
     * generalv3.5:  通用大模型Spark Max版本 
     * 4.0Ultra：    通用大模型Spark4.0 Ultra版本
     * pro-128k：    通用大模型pro128k版本
     * max-32k：     通用大模型max32k版本
     * *************************************/
    llmConfig->domain("4.0Ultra");  
    LLM *json_llm = LLMFactory::textGeneration(llmConfig);
    char* rawJson = "{\"header\": {\"app_id\": \"30dfb58a\",\"uid\": \"12345\"},\"parameter\": {\"chat\": {\"domain\": \"4.0Ultra\",\"temperature\": 0.5,\"max_tokens\": 1024}},\"payload\": {\"message\": {\"text\": [{\"role\": \"user\", \"content\": \"今天天气怎么样?\"}]}}}";
    
    SparkCallbacks *cbs = new SparkCallbacks();
    json_llm->registerLLMCallbacks(cbs);//注册监听回调

    printf(GREEN "协议已输入，等待大模型回复...\n" RESET);
    finish = false;
    int ret = json_llm->arunWithJson(rawJson,nullptr);
    if (ret != 0)
    {
        printf(RED "异步请求失败,错误码: %d\n" RESET, ret);
        finish = true;
    }
    int times = 0;
    while (!finish)
    { // 等待结果返回退出
        sleep(1);
        if (times++ > 10) // 等待十秒如果没有最终结果返回退出
            break;
    }
    // 运行结束，释放实例  
    if (json_llm != nullptr)
    {
        LLM::destroy(json_llm);
    }
    if (cbs != nullptr)
        delete cbs;

    printf(GREEN "原始json协议请求演示完成,请继续输入指令:\n0:同步交互\n1:异步交互\n2:FunctionCall\n3:原始json协议请求\n4:退出\n" RESET);
}

/***
 * FunctionCall功能要求SDK版本号大于等于1.1.5，且使用的星火版本Spark Max/4.0 Ultra。
 * ***/
void run_Function_Call(){
    string function = R"(
        [
            {
                "name": "天气查询",
                "description": "天气插件可以提供天气相关信息。你可以提供指定的地点信息、指定的时间点或者时间段信息，来精准检索到天气信息。",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "location": {
                            "type": "string",
                            "description": "地点，比如北京。"
                        },
                        "date": {
                            "type": "string",
                            "description": "日期。"
                        }
                    },
                    "required": [
                        "location"
                    ]
                }
            }
        ]
    )";
    
    // 配置大模型参数
    LLMConfig *llmConfig = LLMConfig::builder();

    /****************************************
     * 选择要使用的大模型类型(需开通相应的授权)：
     * generalv3.5:  通用大模型V3.5版本 
     * 4.0Ultra：    通用大模型v4.0版本
     * max-32k：     通用大模型max32k版本
     * *************************************/
    llmConfig->domain("4.0Ultra");
    LLM *functionCall_llm = LLMFactory::textGeneration(llmConfig);

    SparkCallbacks *cbs = new SparkCallbacks();
    functionCall_llm->registerLLMCallbacks(cbs);//注册监听回调

    SparkChain::LLMTools tools("functions",function);
    char* input = "合肥今天的天气怎么样？";
    printf(GREEN "用户输入: %s\n" RESET, input);
    finish = false;
    int ret = functionCall_llm->arun(input,tools);
    if (ret != 0)
    {
        printf(RED "异步请求失败,错误码: %d\n" RESET, ret);
        finish = true;
    }
    int times = 0;
    while (!finish)
    { // 等待结果返回退出
        sleep(1);
        if (times++ > 10) // 等待十秒如果没有最终结果返回退出
            break;
    }
    // 运行结束，释放实例  
    if (functionCall_llm != nullptr)
    {
        LLM::destroy(functionCall_llm);
    }
    if (cbs != nullptr)
        delete cbs;
    
    printf(GREEN "FunctionCall演示完成,请继续输入指令:\n0:同步交互\n1:异步交互\n2:FunctionCall\n3:原始json协议请求\n4:退出\n" RESET);
}


/***
 * 同步请求
 * ***/
void run_Generation_Sync()
{
    //输入通过命令行用户自己输入！！！
    // 配置大模型参数
    LLMConfig *llmConfig = LLMConfig::builder();

    /****************************************
     * 选择要使用的大模型类型(需开通相应的授权)：
     * general:      通用大模型Spark Lite版本
     * generalv3：   通用大模型Spark Pro版本 
     * generalv3.5:  通用大模型Spark Max版本 
     * 4.0Ultra：    通用大模型Spark4.0 Ultra版本
     * pro-128k：    通用大模型pro128k版本
     * max-32k：     通用大模型max32k版本
     * *************************************/
    llmConfig->domain("4.0Ultra");

    /***********************
     * url和domain是配合使用的，SDK里预设了general,generalv3,generalv3.5,4.0Ultra,pro-128k和max-32k的url。
     * 当使用这几个domain时，SDK会自动设置url，故开发者可不用额外设置其值。
     * SDK同样支持开发者访问其他未预置的服务，此时则需要开发者同时设置domain和url。
     * ******************************/
    // llmConfig->url("wss://spark-api.xf-yun.com/v4.0/chat");

    /*******************
     * 设置历史上下文
     * WindowMemory：通过会话轮数控制上下文范围，即一次提问和一次回答为一轮会话交互。用户可指定会话关联几轮上下文。
     * TokenMemory： 通过Token总长度控制上下文范围，1 token 约等于1.5个中文汉字 或者 0.8个英文单词。用户可指定历史会话Token长度
     * ***********************/
    Memory* window_memory = Memory::WindowMemory(5);
    LLM *syncllm = LLMFactory::textGeneration(llmConfig,window_memory);//以WindowsMemory创建LLM实例
    // Memory* token_memory = Memory::TokenMemory(500);
    // LLM *syncllm = LLMFactory::textGeneration(llmConfig,token_memory);//以TokenMemory创建LLM实例

    //同步请求，示例中演示两轮会话交互
    char input[1024];
    printf(GREEN "请输入交互内容,如需退出请输入q: \n" RESET);
    while(1){        
        if(fgets(input, sizeof(input), stdin) == NULL){
            printf(RED "读取输入时出错\n" RESET);
            break;
        }
        // 去掉换行符
        size_t len = strlen(input);
        if (len > 0 && input[len - 1] == '\n') {
            input[len - 1] = '\0';
        }
        // 判断是否输入了'q'
        if (!strcmp(input, "q")) {
            break;
        }else if(!strcmp(input, "")){
            continue;
        }
        printf(YELLOW "正在思考中,请稍后... \n" RESET);
        LLMSyncOutput *result = syncllm->run(input);
        /*******************获取交互结果**************************************/
        const char* content  = result->getContent();//获取返回结果
        int contentLen       = result->getContentLen();//获取返回结果长度
        SparkChain::LLMResult::ContentType type = result->getContentType();//获取返回结果类型,TEXT:返回结果为文本,IMAGE:返回结果为图片
        int errCode          = result->getErrCode();//获取结果状态,0:调用成功，非0:调用失败
        const char* errMsg   = result->getErrMsg();//获取失败时的错误信息
        const char* role     = result->getRole();//获取角色信息
        const char* sid      = result->getSid();//获取本次交互的sid
        int completionTokens = result->getCompletionTokens();//获取回答的Token大小
        int promptTokens     = result->getPromptTokens();//包含历史问题的总Tokens大小
        int totalTokens      = result->getTotalTokens();//promptTokens和completionTokens的和，也是本次交互计费的Tokens大小
        /*******************获取交互结果**************************************/
        if (errCode != 0)
        {
            printf(RED "请求出错,错误码: %d,错误信息:%s\n\n" RESET, errCode, errMsg);
            break;
        }
        else
        {
            printf(YELLOW "%s\n" RESET, content);
        }
        printf(GREEN "请输入交互内容,如需退出请输入q: \n" RESET);
    }   
    // 运行结束，释放实例  
    if (syncllm != nullptr)
    {
        LLM::destroy(syncllm);
    }
    printf(GREEN "同步演示完成,请继续输入指令:\n0:同步交互\n1:异步交互\n2:FunctionCall\n3:原始json协议请求\n4:退出\n" RESET);
}


/***
 * 异步请求
 * ***/
void run_Generation_Async()
{
    // 配置大模型参数
    LLMConfig *llmConfig = LLMConfig::builder();

    /****************************************
     * 选择要使用的大模型类型(需开通相应的授权)：
     * general:      通用大模型Spark Lite版本
     * generalv3：   通用大模型Spark Pro版本 
     * generalv3.5:  通用大模型Spark Max版本 
     * 4.0Ultra：    通用大模型Spark4.0 Ultra版本
     * pro-128k：    通用大模型pro128k版本
     * max-32k：     通用大模型max32k版本
     * *************************************/
    llmConfig->domain("4.0Ultra");  

    /****************************************
     * url和domain是配合使用的，SDK里预设了general,generalv3,generalv3.5,4.0Ultra,pro-128k和max-32k的url。
     * 当使用这几个domain时，SDK会自动设置url，故开发者可不用额外设置其值。
     * SDK同样支持开发者访问其他未预置的服务，此时则需要开发者同时设置domain和url。
     * **************************************/
    // llmConfig->url("wss://spark-api.xf-yun.com/v4.0/chat");


    /*******************
     * 设置历史上下文
     * WindowMemory：通过会话轮数控制上下文范围，即一次提问和一次回答为一轮会话交互。用户可指定会话关联几轮上下文。
     * TokenMemory： 通过Token总长度控制上下文范围，1 token 约等于1.5个中文汉字 或者 0.8个英文单词。用户可指定历史会话Token长度
     * ***********************/
    Memory* window_memory = Memory::WindowMemory(5);
    LLM *asyncllm = LLMFactory::textGeneration(llmConfig,window_memory);//以WindowsMemory创建LLM实例

    // Memory* token_memory = Memory::TokenMemory(500);
    // LLM *asyncllm = LLMFactory::textGeneration(llmConfig,token_memory);//以TokenMemory创建LLM实例

    if (asyncllm == nullptr)
    {
        printf(RED "LLM实例创建失败,退出程序\n" RESET);
        return;
    }
    
    SparkCallbacks *cbs = new SparkCallbacks();
    asyncllm->registerLLMCallbacks(cbs);// 注册结果监听回调

    //异步请求，示例中演示两轮会话交互
    char input[1024];
    printf(GREEN "请输入交互内容,如需退出请输入q: \n" RESET);
    while(1){        
        if(fgets(input, sizeof(input), stdin) == NULL){
            printf(RED "读取输入时出错\n" RESET);
            break;
        }
        // 去掉换行符
        size_t len = strlen(input);
        if (len > 0 && input[len - 1] == '\n') {
            input[len - 1] = '\0';
        }
        // 判断是否输入了'q'
        if (!strcmp(input, "q")) {
            break;
        }else if(!strcmp(input, "")){
            continue;
        }
        printf(YELLOW "正在思考中,请稍后... \n" RESET);
        finish = false;
        int ret = asyncllm->arun(input);
        if (ret != 0)
        {
            printf(RED "异步请求失败,错误码: %d\n" RESET, ret);
            finish = true;
            break;
        }
        int times = 0;
        while (!finish)
        { // 等待结果返回退出
            sleep(1);
            if (times++ > 20) // 等待十秒如果没有最终结果返回退出
                break;
        }
        printf(GREEN "请输入交互内容,如需退出请输入q: \n" RESET);
    }

    // 运行结束，释放实例  
    if (asyncllm != nullptr)
    {
        LLM::destroy(asyncllm);
    }
    if (cbs != nullptr)
        delete cbs;
    
    printf(GREEN "异步演示完成,请继续输入指令:\n0:同步交互\n1:异步交互\n2:FunctionCall\n3:原始json协议请求\n4:退出\n" RESET);
}


void uninitSDK()
{
    // SDK逆初始化
    SparkChain::unInit();
}

int main(int argc, char const *argv[])
{
    /* SDK初始化,初始化仅需全局初始化一次。*/
    int ret = initSDK();

    if (ret != 0)
    {
        printf(RED "\nSDK初始化失败!错误码:%d" RESET,ret);
        goto exit; //SDK初始化失败，退出
    }

    printf(GREEN "\n#######################################\n" RESET);
    printf(GREEN "##星火大模型交互:用户和大模型进行问答交互##\n" RESET);
    printf(GREEN "#######################################\n" RESET);
    printf(GREEN "演示示例选择:\n0:同步交互\n1:异步交互\n2:FunctionCall\n3:原始json协议请求\n4:退出\n" RESET);

    while(1)
    {
        scanf("%d", &choice);
        if(choice == 1){
            printf(GREEN "异步交互演示\n" RESET);
            run_Generation_Async();
        }else if(choice == 0){
            printf(GREEN "同步交互演示\n" RESET);
            run_Generation_Sync(); 
        }else if(choice == 2){
            printf(GREEN "FunctionCall演示\n" RESET);
            run_Function_Call();
        }else if(choice == 3){
            printf(GREEN "原始json协议请求演示\n" RESET);
            run_ArunWithJson();
        }else{
            break;
        }
    }

exit:
	printf(RED "已退出演示 ...\n" RESET);
	uninitSDK(); //退出    
}
