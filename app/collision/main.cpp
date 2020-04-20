/**
*
* Copyright(c)<2018>, <Huawei Technologies Co.,Ltd>
*
* @version 1.0
*
* @date 2018-5-19
*/
#include <unistd.h>
#include <thread>
#include <fstream>
#include <algorithm>
#include "main.h"
#include "hiaiengine/api.h"
#include <libgen.h>
#include <string.h>

#include <sstream>
#include <iostream>
#include <cmath>
#include <stack>

static const uint32_t GRAPH_ID = 1875642181;
static int motor_speed = 30;
int cnt = 0;
int send_speed = 20;
std::mutex mt;
ShmServer motor;

/**
* @ingroup FasterRcnnDataRecvInterface
* @brief RecvData RecvData Recursion，save the file
* @param [in]
*/

HIAI_StatusT CustomDataRecvInterface::RecvData
    (const std::shared_ptr<void>& message)
{
    std::shared_ptr<callback_result> data =
        std::static_pointer_cast<callback_result>(message);
    int data_type = data->result_type;
    // printf("%d\n", data_type);
    cnt++;

    // mutex
    mt.lock();
    if (data_type) {
        // MobileNet Case
        motor.GetEnvStatus()->obj_number = data->obj_number;
        // Copy obj
        for(int i = 0; i < data->obj_number; i++) {
            motor.GetEnvStatus()->object[i] = data->object[i];
        }
    }
    else {
        motor.GetEnvStatus()->env_block = data->is_block;
    }
    mt.unlock();
    return HIAI_OK;
}

// Init and create graph
HIAI_StatusT HIAI_InitAndStartGraph()
{
    // Step1: Global System Initialization before using HIAI Engine
    HIAI_StatusT status = HIAI_Init(0);

    // Step2: Create and Start the Graph
    status = hiai::Graph::CreateGraph("./graph.config");
    if (status != HIAI_OK)
    {
        HIAI_ENGINE_LOG(status, "Fail to start graph");
        return status;
    }

    // Step3
    std::shared_ptr<hiai::Graph> graph = hiai::Graph::GetInstance(GRAPH_ID);
    if (nullptr == graph)
    {
        HIAI_ENGINE_LOG("Fail to get the graph-%u", GRAPH_ID);
        return status;
    }
	// int leaf_array[1] = {601};  //leaf node id
    int leaf_array[2] = {601, 602};

	for(int i = 0;i < 2;i++){
		hiai::EnginePortID target_port_config;
			target_port_config.graph_id = GRAPH_ID;
			target_port_config.engine_id = leaf_array[i];  
			target_port_config.port_id = 0;
			graph->SetDataRecvFunctor(target_port_config,
				std::shared_ptr<CustomDataRecvInterface>(
						 new CustomDataRecvInterface("")));
	}
	return HIAI_OK;
}

int main(int argc, char* argv[])
{
    HIAI_StatusT ret = HIAI_OK;
	char * dirc = strdup(argv[0]);
	if (dirc)
	{
	    char * dname = ::dirname(dirc);
	    chdir(dname);
	    HIAI_ENGINE_LOG("chdir to %s", dname);
	    free(dirc);
	}


    // 1.create graph
    ret = HIAI_InitAndStartGraph();
    if (HIAI_OK != ret)
    {
        HIAI_ENGINE_LOG("Fail to start graph");;
        return -1;
    }

    // 2.send data
    std::shared_ptr<hiai::Graph> graph = hiai::Graph::GetInstance(GRAPH_ID);
    if (nullptr == graph)
    {
        HIAI_ENGINE_LOG("Fail to get the graph-%u", GRAPH_ID);
        return -1;
    }
    
    // send data to SourceEngine 0 port 
    hiai::EnginePortID engine_id;
    engine_id.graph_id = GRAPH_ID;
    engine_id.engine_id = 958; 
    engine_id.port_id = 0;
    std::shared_ptr<std::string> src_data(new std::string);
    graph->SendData(engine_id, "string", std::static_pointer_cast<void>(src_data));
    for (;;){
        usleep(1000000);
    }
    hiai::Graph::DestroyGraph(GRAPH_ID);
    return 0;
}
