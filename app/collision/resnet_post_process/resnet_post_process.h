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
#ifndef RESNET_POST_PROCESS_H_
#define RESNET_POST_PROCESS_H_
#include "face_detection_params.h"
#include "hiaiengine/api.h"
#include "hiaiengine/ai_types.h"
#include "hiaiengine/data_type.h"
#include "hiaiengine/data_type_reg.h"
#include "hiaiengine/engine.h"
#include "ascenddk/presenter/agent/presenter_channel.h"


#define INPUT_SIZE 1
#define OUTPUT_SIZE 1

// face detection configuration
struct FaceDetectionPostConfig {
  float confidence;  // confidence
  std::string presenter_ip;  // presenter server IP
  int32_t presenter_port;  // presenter server port for agent
  std::string channel_name;  // channel name
};

/**
 * @brief: face detection post process
 */
class ResNetPostProcess : public hiai::Engine {
public:
  /**
   * @brief: construction function
   */
  ResNetPostProcess();

  /**
   * @brief: the destruction function
   */
  ~ResNetPostProcess() = default;

  /**
   * @brief: face detection post process engine initialize
   * @param [in]: engine's parameters which configured in graph.config
   * @param [in]: model description
   * @return: HIAI_StatusT
   */
  HIAI_StatusT Init(const hiai::AIConfig& config,
                    const std::vector<hiai::AIModelDescription>& model_desc);

  /**
   * @brief: engine processor
   *         1. dealing results
   *         2. call OSD to draw box and label if needed
   *         3. call DVPP to change YUV420SP to JPEG
   *         4. call presenter agent to send JPEG to server
   * @param [in]: input size
   * @param [in]: output size
   */
  HIAI_DEFINE_PROCESS(INPUT_SIZE, OUTPUT_SIZE);

private:
  // configuration
  std::shared_ptr<FaceDetectionPostConfig> fd_post_process_config_;

  // presenter channel
  std::shared_ptr<ascend::presenter::Channel> presenter_channel_;

  /**
   * @brief: send result
   * @return: true: success; false: failed
   */
  bool SendSentinel(std::shared_ptr<callback_result> &result);

  /**
   * @brief: handle results
   * @param [in]: EngineTransT format data which inference engine send
   * @return: HIAI_StatusT
   */
  HIAI_StatusT HandleResults(
      const std::shared_ptr<EngineTransT> &inference_res);

  /**
   * @brief: validate confidence
   * @param [in]: confidence
   * @return: true: invalid
   *          false: valid
   */
  bool IsInvalidConfidence(float confidence);


  /**
   * @brief: validate results
   * @param [in]: attribute
   * @param [in]: score
   * @param [in]: left top anchor
   * @param [in]: right bottom anchor
   * @return: true: invalid
   *          false: valid
   */
  bool IsInvalidResults(float attr, float score,
                        const ascend::presenter::Point &point_lt,
                        const ascend::presenter::Point &point_rb);

};

#endif /* RESNET_POST_PROCESS_H_ */
