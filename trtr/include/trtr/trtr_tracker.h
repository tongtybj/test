#include <fstream>
#include <iostream>
#include <omp.h>
#include <ros/ros.h>
#include <sstream>


#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>

#ifdef TENSORRT
#include <NvInfer.h>
#include <common.h>
#include <buffers.h>
#endif

namespace fs = boost::filesystem;


class BasicTracker
{
public:
  BasicTracker() {}
  ~BasicTracker() {}

  const cv::Point getBboxLT() const { return cv::Point(cx_ - w_/2, cy_ - h_/2);}
  const cv::Point getBboxRB() const { return cv::Point(cx_ + w_/2, cy_ + h_/2);}

  bool init(std::string encoder_model_file, std::string decoder_model_file, float score_threshold, float cosine_window_factor, int cosine_window_step, float size_lpf_factor);
  void reset(const std::vector<float>& bbox, const cv::Mat& img);
  void track(const cv::Mat& img);

protected:

  void getSiamFCLikeScale(const int& template_img_size,  const float& w, const float& h, float& s_z, float& scale_z);
  void centerCropping(const cv::Mat& img, const std::vector<float>& bbox, const int& out_sz, const cv::Scalar& padding, cv::Mat& cropped_img);
  void makePositionEmbedding(float* p, int feat_size, std::vector<int> mask_bounds = std::vector<int>(0));
  void createCosineWindow();

  virtual bool loadModels(std::string encoder_model_file, std::string decoder_model_file) { return false; }
  virtual void encoderInference(const cv::Mat& img, const std::vector<float>& bounds) = 0;
  virtual void decoderInference(const cv::Mat& img, const std::vector<float>& bounds) = 0;
  virtual void processInput(const cv::Mat& img, const std::vector<float>& bounds, const std::string image_type) = 0;

  float cx_, cy_, w_, h_;
  int template_img_size_, search_img_size_;
  int template_feat_size_, search_feat_size_;
  int transformer_dim_;
  std::vector<float> pos_emded_dim_t_;
  cv::Mat default_template_pos_embed_;
  cv::Mat default_search_pos_embed_;

  cv::Mat heatmap_;
  cv::Mat bbox_reg_;
  cv::Mat bbox_wh_;

  // postproces
  cv::Mat cosine_window_;
  // hyper-parameter
  float cosine_window_factor_;
  int cosine_window_step_;
  float size_lpf_factor_;
  float score_threshold_;
};

#ifdef TENSORRT
class TensorRTTracker: public BasicTracker
{
  template <typename T>
  struct TrtDestroyer
  {
    void operator()(T* t) { t->destroy(); }
  };

  template <typename T> using TrtUniquePtr = std::unique_ptr<T, TrtDestroyer<T> >;

  class Logger : public nvinfer1::ILogger {
  public:
    void log(nvinfer1::ILogger::Severity severity, const char* msg) override
    {
      // suppress information level log
      if (severity == Severity::kINFO) return;
      std::cout << msg << std::endl;
    }
  };

  struct InferDeleter
  {
    template <typename T>
    void operator()(T* obj) const
    {
      if (obj)
        {
          obj->destroy();
        }
    }
  };

public:
  TensorRTTracker() {}
  ~TensorRTTracker(){}

private:
  Logger logger_;
  std::shared_ptr<nvinfer1::ICudaEngine> encoder_engine_;
  std::shared_ptr<nvinfer1::ICudaEngine> decoder_engine_;
  std::shared_ptr<samplesCommon::BufferManager> encoder_buffers_;
  std::shared_ptr<samplesCommon::BufferManager> decoder_buffers_;
  std::shared_ptr<nvinfer1::IExecutionContext> encoder_context_;
  std::shared_ptr<nvinfer1::IExecutionContext> decoder_context_;
  cudaStream_t cuda_stream_;

  nvinfer1::ICudaEngine* loadEngine(const std::string& model_file);
  bool loadModels(std::string encoder_model_file, std::string decoder_model_file) override;
  void encoderInference(const cv::Mat& img, const std::vector<float>& bounds) override;
  void decoderInference(const cv::Mat& img, const std::vector<float>& bounds) override;
  void processInput(const cv::Mat& img, const std::vector<float>& bounds, const std::string image_type) override;
};
#endif
