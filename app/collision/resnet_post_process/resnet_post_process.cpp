/**
 * ============================================================================
 *
 * Copyright (C) 2018, Hisilicon Technologies Co., Ltd. All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1 Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *   2 Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *   3 Neither the names of the copyright holders nor the names of the
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * ============================================================================
 */
#include "resnet_post_process.h"
#include <vector>
#include <sstream>
#include <cmath>
#include <regex>
#include "hiaiengine/log.h"

#include <unistd.h>

using hiai::Engine;
using namespace ascend::presenter;
using namespace std::__cxx11;

// register data type
HIAI_REGISTER_DATA_TYPE("EngineTransT", EngineTransT);
HIAI_REGISTER_DATA_TYPE("OutputT", OutputT);
HIAI_REGISTER_DATA_TYPE("ScaleInfoT", ScaleInfoT);
HIAI_REGISTER_DATA_TYPE("NewImageParaT", NewImageParaT);
HIAI_REGISTER_DATA_TYPE("BatchImageParaWithScaleT", BatchImageParaWithScaleT);


// constants
namespace {
//// parameters for drawing box and label begin////
// face box color
const uint8_t kFaceBoxColorR = 255;
const uint8_t kFaceBoxColorG = 190;
const uint8_t kFaceBoxColorB = 0;

// face box border width
const int kFaceBoxBorderWidth = 2;

// face label color
const uint8_t kFaceLabelColorR = 255;
const uint8_t kFaceLabelColorG = 255;
const uint8_t kFaceLabelColorB = 0;

// face label font
const double kFaceLabelFontSize = 0.7;
const int kFaceLabelFontWidth = 2;

// face label text prefix
const std::string kFaceLabelTextPrefix = "Tree:";
const std::string kFaceLabelTextFree = "FREE:";
const std::string kFaceLabelTextBlocked = "BLOCKED:";
const std::string kFaceLabelTextSuffix = "%";
//// parameters for drawing box and label end////

// port number range
const int32_t kPortMinNumber = 0;
const int32_t kPortMaxNumber = 65535;

// confidence range
const float kConfidenceMin = 0.0;
const float kConfidenceMax = 1.0;

// face detection function return value
const int32_t kFdFunSuccess = 0;
const int32_t kFdFunFailed = -1;

// need to deal results when index is 2
const int32_t kDealResultIndex = 2;

// each results size
const int32_t kEachResultSize = 7;

// attribute index
const int32_t kAttributeIndex = 1;

// score index
const int32_t kScoreIndex = 2;

// anchor_lt.x index
const int32_t kAnchorLeftTopAxisIndexX = 3;

// anchor_lt.y index
const int32_t kAnchorLeftTopAxisIndexY = 4;

// anchor_rb.x index
const int32_t kAnchorRightBottomAxisIndexX = 5;

// anchor_rb.y index
const int32_t kAnchorRightBottomAxisIndexY = 6;

// face attribute
const float kAttributeFaceLabelValue = 1.0;
const float kAttributeFaceDeviation = 0.00001;

// percent
const int32_t kScorePercent = 100;

// IP regular expression
const std::string kIpRegularExpression =
    "^((25[0-5]|2[0-4]\\d|[1]{1}\\d{1}\\d{1}|[1-9]{1}\\d{1}|\\d{1})($|(?!\\.$)\\.)){4}$";

// channel name regular expression
const std::string kChannelNameRegularExpression = "[a-zA-Z0-9/]+";
}

// callback port
const uint32_t kSendDataPort = 0;

// sleep interval when queue full (unit:microseconds)
const __useconds_t kSleepInterval = 200000;

ResNetPostProcess::ResNetPostProcess() {
  fd_post_process_config_ = nullptr;
  presenter_channel_ = nullptr;
}

HIAI_StatusT ResNetPostProcess::Init(
    const hiai::AIConfig& config,
    const std::vector<hiai::AIModelDescription>& model_desc) {
  HIAI_ENGINE_LOG("Begin initialize!");

  // get configurations
  if (fd_post_process_config_ == nullptr) {
    fd_post_process_config_ = std::make_shared<FaceDetectionPostConfig>();
  }

  // get parameters from graph.config
  for (int index = 0; index < config.items_size(); index++) {
    const ::hiai::AIConfigItem& item = config.items(index);
    const std::string& name = item.name();
    const std::string& value = item.value();
    std::stringstream ss;
    ss << value;
    if (name == "Confidence") {
      ss >> (*fd_post_process_config_).confidence;
      // validate confidence
      if (IsInvalidConfidence(fd_post_process_config_->confidence)) {
        HIAI_ENGINE_LOG(HIAI_GRAPH_INVALID_VALUE,
                        "Confidence=%s which configured is invalid.",
                        value.c_str());
        return HIAI_ERROR;
      }
    }
    // else : nothing need to do
  }
  return HIAI_OK;
}

// Send data to main.cpp 
bool ResNetPostProcess::SendSentinel(std::shared_ptr<callback_result> &result) {
  // can not discard when queue full
  HIAI_StatusT hiai_ret = HIAI_OK;
  // std::shared_ptr<string> sentinel_msg(new (std::nothrow) string);
  // std::shared_ptr<callback_result> sentinel_msg = std::make_shared<callback_result>(result);
  do {
    hiai_ret = SendData(kSendDataPort, "callback_result",
                        std::static_pointer_cast<void>(result));
    // when queue full, sleep
    if (hiai_ret == HIAI_QUEUE_FULL) {
      HIAI_ENGINE_LOG("queue full, sleep 200ms");
      usleep(kSleepInterval);
    }
  } while (hiai_ret == HIAI_QUEUE_FULL);

  // send failed
  if (hiai_ret != HIAI_OK) {
    HIAI_ENGINE_LOG(HIAI_ENGINE_RUN_ARGS_NOT_RIGHT,
                    "call SendData failed, err_code=%d", hiai_ret);
    return false;
  }
  return true;
}

bool ResNetPostProcess::IsInvalidConfidence(float confidence) {
  return (confidence <= kConfidenceMin) || (confidence > kConfidenceMax);
}

bool ResNetPostProcess::IsInvalidResults(float attr, float score,
                                                const Point &point_lt,
                                                const Point &point_rb) {
  // attribute is not face (background)
  // if (std::abs(attr - kAttributeFaceLabelValue) > kAttributeFaceDeviation) {
  //   return true;
  // }

  // confidence check
  if ((score < fd_post_process_config_->confidence)
      || IsInvalidConfidence(score)) {
    return true;
  }

  // rectangle position is a point or not: lt == rb
  if ((point_lt.x == point_rb.x) && (point_lt.y == point_rb.y)) {
    return true;
  }
  return false;
}

HIAI_StatusT ResNetPostProcess::HandleResults(
    const std::shared_ptr<EngineTransT> &inference_res) {
  HIAI_StatusT status = HIAI_OK;

  // Result raw image
  std::vector<NewImageParaT> img_vec = inference_res->imgs;

  // Result data
  std::vector<OutputT> output_data_vec = inference_res->output_datas;

  // dealing every image (just one for batch size)
  for (uint32_t ind = 0; ind < inference_res->b_info.batch_size; ind++) {
    int32_t out_index = ind * kDealResultIndex;
    OutputT out = output_data_vec[out_index];
    std::shared_ptr<hiai::AISimpleTensor> result_tensor = std::make_shared<
        hiai::AISimpleTensor>();
    result_tensor->SetBuffer(out.data.get(), out.size);

    // Memory copy
    int32_t size = result_tensor->GetSize() / sizeof(float);
    float result[size];
    errno_t mem_ret = memcpy_s(result, sizeof(result),
                               result_tensor->GetBuffer(),
                               result_tensor->GetSize());

    // memory copy failed, skip this image
    if (mem_ret != EOK) {
      HIAI_ENGINE_LOG(HIAI_ENGINE_RUN_ARGS_NOT_RIGHT,
                      "handle results: memcpy_s() error=%d", mem_ret);
      continue;
    }

    uint32_t width = img_vec[ind].img.width;
    uint32_t height = img_vec[ind].img.height;
    uint32_t img_size = img_vec[ind].img.size;

    // every inference result needs 8 float
    // loop the result for every inference result
    std::vector<DetectionResult> detection_results;

    // Classification result: free_conf, block_conf
    vector<float> varr(result, result + size);
    uint8_t block_flag = 1;
    if (varr[0] > varr[1]) {
      // printf("0:%f\n", varr[0]);
      block_flag = 0;
    }
    else{
      // printf("1:%f\n", varr[1]);
      block_flag = 1;
    }
    
    // Send data
    int32_t ret;
    std::shared_ptr<callback_result> send_buf = std::make_shared<callback_result>();
    send_buf->result_type = 0;
    send_buf->is_block = block_flag;
    ret = SendSentinel(send_buf);
    if (ret == false) {
      HIAI_ENGINE_LOG("Failed to send data to main. Reason: SendData failed.");
      return HIAI_ERROR;
    }

  }
  return status;
}

HIAI_IMPL_ENGINE_PROCESS("resnet_post_process",
    ResNetPostProcess, INPUT_SIZE) {
  // check arg0 is null or not
  if (arg0 == nullptr) {
    HIAI_ENGINE_LOG(HIAI_ENGINE_RUN_ARGS_NOT_RIGHT,
                    "Failed to process invalid message.");
    return HIAI_ERROR;
  }

  // check original image is empty or not
  std::shared_ptr<EngineTransT> inference_res = std::static_pointer_cast<
      EngineTransT>(arg0);
  if (inference_res->imgs.empty()) {
    HIAI_ENGINE_LOG(
        HIAI_ENGINE_RUN_ARGS_NOT_RIGHT,
        "Failed to process invalid message, original image is null.");
    return HIAI_ERROR;
  }

 // inference failed, dealing original images
  if (!inference_res->status) {
    HIAI_ENGINE_LOG(HIAI_OK, inference_res->msg.c_str());
    HIAI_ENGINE_LOG(HIAI_OK, "will handle original image.");
    return HIAI_OK;
  }


  // inference success, dealing inference results
  return HandleResults(inference_res);
}
