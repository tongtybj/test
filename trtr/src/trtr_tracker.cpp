#include <trtr/trtr_tracker.h>


TensorRT::TensorRT(std::string encoder_model_file, std::string decoder_model_file, float score_threshold, float cosine_window_factor, int cosine_window_step, float size_lpf_factor):
  encoder_engine_(nullptr), decoder_engine_(nullptr), template_img_size_(0), search_img_size_(0), template_feat_size_(0), search_feat_size_(0), transformer_dim_(0), score_threshold_(score_threshold), cosine_window_factor_(cosine_window_factor), cosine_window_step_(cosine_window_step), size_lpf_factor_(size_lpf_factor)
{
  std::stringstream err_msg;
  encoder_engine_ = std::shared_ptr<nvinfer1::ICudaEngine>(loadEngine(encoder_model_file, err_msg), InferDeleter());
  encoder_context_ = std::shared_ptr<nvinfer1::IExecutionContext>(encoder_engine_->createExecutionContext(), InferDeleter());
  encoder_buffers_ = std::make_shared<samplesCommon::BufferManager>(encoder_engine_);

  decoder_engine_ = std::shared_ptr<nvinfer1::ICudaEngine>(loadEngine(decoder_model_file, err_msg), InferDeleter());
  decoder_context_ = std::shared_ptr<nvinfer1::IExecutionContext>(decoder_engine_->createExecutionContext(), InferDeleter());
  decoder_buffers_ = std::make_shared<samplesCommon::BufferManager>(decoder_engine_);

  if(encoder_engine_)
    {
      std::cout << encoder_engine_->getName() << " has " << encoder_engine_->getNbBindings() << " bindings" << std::endl;
      for(int i = 0; i < encoder_engine_->getNbBindings(); i++)
        {
          if(encoder_engine_->bindingIsInput(i)) std::cout << "Input: ";
          else std::cout << "Output: ";
          int tpye_int = (int)encoder_engine_->getBindingDataType(i);
          std::string type = tpye_int == 0?std::string("Float32"):
            tpye_int == 1?std::string("Float16"):
            tpye_int == 2?std::string("int8"):
            tpye_int == 3?std::string("int32"):std::string("bolean");
          std::cout << encoder_engine_->getBindingName(i) << "; dtype: " << type << "; shape: [";
          auto dims = encoder_engine_->getBindingDimensions(i);
          for (int j = 0; j <  dims.nbDims; j ++)
            std::cout <<  dims.d[j] << ", ";
          std::cout << "]" << " size: " << encoder_buffers_->size(encoder_engine_->getBindingName(i)) << std::endl;

          if(encoder_engine_->getBindingName(i) == std::string("template_image"))
            {
              template_img_size_ = encoder_engine_->getBindingDimensions(i).d[1];
            }
          if(encoder_engine_->getBindingName(i) == std::string("template_pos_embed"))
            {
              template_feat_size_ = encoder_engine_->getBindingDimensions(i).d[1];
              transformer_dim_ = encoder_engine_->getBindingDimensions(i).d[3];
            }

        }

      // default_position_embedding
      for(int i = 0; i < transformer_dim_ / 2; i++)
        pos_emded_dim_t_.push_back(std::pow((float)10000, 2 * (i / 2) / (float)(transformer_dim_ / 2)));

      std::vector<int> template_pos_embed_sizes;
      template_pos_embed_sizes.push_back(template_feat_size_);
      template_pos_embed_sizes.push_back(template_feat_size_);
      template_pos_embed_sizes.push_back(transformer_dim_);
      default_template_pos_embed_ = cv::Mat(template_pos_embed_sizes, CV_32F);
      makePositionEmbedding((float*)default_template_pos_embed_.data, template_feat_size_);
    }
  else
    {
      ROS_ERROR_STREAM(err_msg.str());
    }

  if(decoder_engine_)
    {
      std::cout << decoder_engine_->getName() << " has " << decoder_engine_->getNbBindings() << " bindings" << std::endl;
      for(int i = 0; i < decoder_engine_->getNbBindings(); i++)
        {
          if(decoder_engine_->bindingIsInput(i)) std::cout << "Input: ";
          else std::cout << "Output: ";
          int tpye_int = (int)decoder_engine_->getBindingDataType(i);
          std::string type = tpye_int == 0?std::string("Float32"):
            tpye_int == 1?std::string("Float16"):
            tpye_int == 2?std::string("int8"):
            tpye_int == 3?std::string("int32"):std::string("bolean");
          std::cout << decoder_engine_->getBindingName(i) << "; dtype: " << type << "; shape: [";
          auto dims = decoder_engine_->getBindingDimensions(i);
          for (int j = 0; j <  dims.nbDims; j ++)
            std::cout <<  dims.d[j] << ", ";
          std::cout << "]" << " size: " << decoder_buffers_->size(decoder_engine_->getBindingName(i)) << std::endl;

          if(decoder_engine_->getBindingName(i) == std::string("search_image"))
            {
              search_img_size_ = decoder_engine_->getBindingDimensions(i).d[1];
            }
          if(decoder_engine_->getBindingName(i) == std::string("search_pos_embed"))
            {
              search_feat_size_ = decoder_engine_->getBindingDimensions(i).d[1];
            }
        }

      std::vector<int> search_pos_embed_sizes;
      search_pos_embed_sizes.push_back(search_feat_size_);
      search_pos_embed_sizes.push_back(search_feat_size_);
      search_pos_embed_sizes.push_back(transformer_dim_);
      default_search_pos_embed_ = cv::Mat(search_pos_embed_sizes, CV_32F);
      makePositionEmbedding((float*)default_search_pos_embed_.data, search_feat_size_);
    }
  else
    {
      ROS_ERROR_STREAM(err_msg.str());
    }

  if(encoder_engine_ && decoder_engine_)
    {
      // create stream:
      cudaStreamCreate(&cuda_stream_);

      // create cosine window for distance penalty
      createCosineWindow();
    }
}


void TensorRT::init(const std::vector<float>& bbox, const cv::Mat& img)
{
  cx_ = bbox.at(0) + bbox.at(2) / 2;
  cy_ = bbox.at(1) + bbox.at(3) / 2;
  w_ = bbox.at(2);
  h_ = bbox.at(3);

  auto chan_avg = cv::mean(img);
  float s_z;
  float scale_z;
  getSiamFCLikeScale(template_img_size_, w_, h_, s_z, scale_z);

  cv::Mat template_img = cv::Mat::zeros(cv::Size(template_img_size_, template_img_size_), img.type());

  std::vector<float> crop_bbox(0);
  crop_bbox.push_back(cx_ - s_z/2);
  crop_bbox.push_back(cy_ - s_z/2);
  crop_bbox.push_back(s_z);
  crop_bbox.push_back(s_z);
  centerCropping(img, crop_bbox, template_img_size_, chan_avg, template_img);

  // get mask
  std::vector<float> template_mask{0.0, 0.0, (float)template_img_size_, (float)template_img_size_};
  if(cx_ < s_z/2) template_mask.at(0) = (s_z/2 - cx_) * scale_z;
  if(cy_ < s_z/2) template_mask.at(1) = (s_z/2 - cy_) * scale_z;
  if(cx_ + s_z/2 > img.cols) template_mask.at(2) -= (cx_ + s_z/2 - img.cols) * scale_z;
  if(cy_ + s_z/2 > img.rows) template_mask.at(3) -= (cy_ + s_z/2 - img.rows) * scale_z;
  for (auto& v: template_mask) v *= (float)template_feat_size_ / template_img_size_;

  processInput(template_img, template_mask, std::string("template"));

  //encoder_buffers_->copyInputToDeviceAsync(cuda_stream_);
  encoder_buffers_->copyInputToDevice();

  encoder_context_->executeV2(encoder_buffers_->getDeviceBindings().data());
  //context_->enqueueV2(encoder_buffers_->getDeviceBindings().data(), cuda_stream_, nullptr);

  encoder_buffers_->copyOutputToHost();
  // encoder_buffers_->copyOutputToHostAsync(cuda_stream_);
  // cudaStreamSynchronize(cuda_stream_);

  //const float* encoder_memory  = static_cast<const float*>(encoder_buffers_->getHostBuffer(std::string("encoder_memory")));
  // for(int i = 0; i < transformer_dim_ / 4; i++)
  //   std::cout << encoder_memory[4*i] << " " << encoder_memory[4*i+1] << " " << encoder_memory[4*i+2] << " " << encoder_memory[4*i+3] << std::endl;

  std::string binding_name("encoder_memory");
  std::memcpy(decoder_buffers_->getHostBuffer(binding_name), encoder_buffers_->getHostBuffer(binding_name), encoder_buffers_->size(binding_name));

  binding_name = std::string("template_pos_embed");
  std::memcpy(decoder_buffers_->getHostBuffer(binding_name), encoder_buffers_->getHostBuffer(binding_name), encoder_buffers_->size(binding_name));
}

void TensorRT::track(const cv::Mat& img)
{
  float s_z;
  float scale_z;
  getSiamFCLikeScale(template_img_size_, w_, h_, s_z, scale_z);
  cv::Mat search_img = cv::Mat::zeros(cv::Size(search_img_size_, search_img_size_), img.type());
  float s_x = search_img_size_ / scale_z;

  // get mask
  std::vector<float> search_mask{0.0, 0.0, (float)search_img_size_, (float)search_img_size_};
  if(cx_ < s_x/2) search_mask.at(0) = (s_x/2 - cx_) * scale_z;
  if(cy_ < s_x/2) search_mask.at(1) = (s_x/2 - cy_) * scale_z;
  if(cx_ + s_x/2 > img.cols) search_mask.at(2) -= (cx_ + s_x/2 - img.cols) * scale_z;
  if(cy_ + s_x/2 > img.rows) search_mask.at(3) -= (cy_ + s_x/2 - img.rows) * scale_z;
  for (auto& v: search_mask) v *= (float)search_feat_size_ / search_img_size_;

  cv::Scalar chan_avg(0,0,0);
  if(cx_ - s_x/2 < 1 || cy_ - s_x/2 < 1 || cx_ + s_x/2 > img.cols - 1 || cy_ + s_x/2 > img.rows - 1)
    chan_avg = cv::mean(img);

  std::vector<float> crop_bbox(0);
  crop_bbox.push_back(cx_ - s_x/2);
  crop_bbox.push_back(cy_ - s_x/2);
  crop_bbox.push_back(s_x);
  crop_bbox.push_back(s_x);

  centerCropping(img, crop_bbox, search_img_size_, chan_avg, search_img);

  processInput(search_img, search_mask, std::string("search"));

  decoder_buffers_->copyInputToDevice();
  //decoder_buffers_->copyInputToDeviceAsync(cuda_stream_);

  decoder_context_->executeV2(decoder_buffers_->getDeviceBindings().data());
  //decoder_context_->enqueueV2(decoder_buffers_->getDeviceBindings().data(), cuda_stream_, nullptr);

  decoder_buffers_->copyOutputToHost();
  // decoder_buffers_->copyOutputToHostAsync(cuda_stream_);
  // cudaStreamSynchronize(cuda_stream_);

  cv::Mat heatmap = cv::Mat(cv::Size(search_feat_size_, search_feat_size_), CV_32FC1, decoder_buffers_->getHostBuffer("pred_heatmap"));
  cv::Mat bbox_reg = cv::Mat(cv::Size(search_feat_size_, search_feat_size_), CV_32FC2, decoder_buffers_->getHostBuffer("pred_bbox_reg"));
  cv::Mat bbox_wh = cv::Mat(cv::Size(search_feat_size_, search_feat_size_), CV_32FC2, decoder_buffers_->getHostBuffer("pred_bbox_wh"));


  // do not update tracker result if the inference score is too low
  double max;
  cv::minMaxLoc(heatmap, nullptr, &max);
  if(max < score_threshold_) return;

  // postprocess
  // Note: we remove size change penalty whcih is introduced in the original implementation

  // 1. add cosine window (distance penalty)
  int best_window_step = 0;
  float window_factor = cosine_window_factor_;
  cv::Mat post_heatmap;
  float best_score = 0;
  cv::Point max_loc; // store in (x: col, y: row)
  for(int i = 0; i < cosine_window_step_; i++)
    {
      post_heatmap = heatmap * (1 -  window_factor) + cosine_window_ * window_factor;
      cv::minMaxLoc(post_heatmap, nullptr, nullptr, nullptr, &max_loc);

      best_score = heatmap.at<float>(max_loc.y, max_loc.x);
      if(best_score > score_threshold_)
        break;
      else
        window_factor = std::max(window_factor - cosine_window_factor_ / cosine_window_step_, (float)0);
    }
  //std::cout << best_score << ", "<< max_loc << std::endl;

  cv::Vec2f target_wh = cv::Mat2f(bbox_wh)(max_loc) * search_img_size_ / scale_z;
  cv::Vec2f bbox_ct = (cv::Vec2f(max_loc.x, max_loc.y) + cv::Mat2f(bbox_reg)(max_loc)) * search_img_size_ / float(search_feat_size_);

  //std::cout << bbox_ct << std::endl;
  cv::Vec2f ct_delta = (bbox_ct - cv::Vec2f(search_img_size_, search_img_size_) / 2) / scale_z;
  float cx = cx_ + ct_delta[0];
  float cy = cy_ + ct_delta[1];

  // 2. lpf for size
  float lpf = best_score * size_lpf_factor_;
  float w = w_ * (1 - lpf) + target_wh[0] * lpf;
  float h = h_ * (1 - lpf) + target_wh[1] * lpf;

  // 3. handle boundary
  float x1 = std::max((float)0, cx - w / 2);
  float y1 = std::max((float)0, cy - h / 2);
  float x2 = std::min((float)img.cols, cx + w / 2);
  float y2 = std::min((float)img.rows, cy + h / 2);
  cx_ = (x1 + x2) / 2;
  cy_ = (y1 + y2) / 2;
  w_ = (x2 - x1);
  h_ = (y2 - y1);

  // cv::rectangle(search_img,
  //               cv::Point(bbox_ct[0] - target_wh[0] * scale_z / 2, bbox_ct[1] - target_wh[1] * scale_z / 2),
  //               cv::Point(bbox_ct[0] + target_wh[0] * scale_z / 2, bbox_ct[1] + target_wh[1] * scale_z / 2),
  //               cv::Scalar(0,255,255),3,4);

  // debug
  // cv::Mat heatmap_v;
  // post_heatmap.convertTo(heatmap_v, CV_8UC1, 255);
  // cv::imshow("test", heatmap_v);
}

nvinfer1::ICudaEngine* TensorRT::loadEngine(const std::string& model_file, std::stringstream& err_msg)
{
  std::ifstream engineFile(model_file, std::ios::binary);
  if (!engineFile)
    {
      err_msg << "Error opening engine file: " << model_file << std::endl;
      return nullptr;
    }

  engineFile.seekg(0, engineFile.end);
  long int fsize = engineFile.tellg();
  engineFile.seekg(0, engineFile.beg);

  std::vector<char> engineData(fsize);
  engineFile.read(engineData.data(), fsize);
  if (!engineFile)
    {
      err_msg << "Error loading engine file: " << model_file << std::endl;
      return nullptr;
    }

  TrtUniquePtr<nvinfer1::IRuntime> runtime{nvinfer1::createInferRuntime(logger_)};

  return runtime->deserializeCudaEngine(engineData.data(), fsize, nullptr);
}

void TensorRT::getSiamFCLikeScale(const int& template_img_size,  const float& w, const float& h, float& s_z, float& scale_z)
{
  const float context_amount = 0.5;
  float wc_z = w + context_amount * (w + h);
  float hc_z = h + context_amount * (w + h);
  s_z = sqrt(wc_z * hc_z);
  scale_z = template_img_size / s_z;
}

void TensorRT::centerCropping(const cv::Mat& img, const std::vector<float>& bbox, const int& out_sz, const cv::Scalar& padding, cv::Mat& cropped_img)
{
  float a = (out_sz-1) / bbox[2]; // TODO: check the one pixel operation i.e., -1
  float b = (out_sz-1) / bbox[3];
  float c = -a * bbox[0];
  float d = -b * bbox[1];
  cv::Mat mapping = (cv::Mat_<float>(2,3) << a, 0, c, 0, b, d);
  //mapping = np.array([[a, 0, c], [0, b, d]]).astype(np.float)
  cv::warpAffine(img, cropped_img, mapping, cropped_img.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, padding);
}

void TensorRT::processInput(cv::Mat& input_img, std::vector<float>& mask_bounds, std::string image_type)
{
  /* image */
  cv::Vec3f mean(0.485, 0.456, 0.406);
  cv::Vec3f std(0.229, 0.224, 0.225);

  std::shared_ptr<samplesCommon::BufferManager>  buffers;
  if(image_type == std::string("template"))
    buffers = encoder_buffers_;
  if(image_type == std::string("search"))
    buffers = decoder_buffers_;

  float* hostDataBuffer = static_cast<float*>(buffers->getHostBuffer(image_type + std::string("_image")));
  assert(hostDataBuffer != nullptr);

  int cols = input_img.cols;
  int rows = input_img.rows;
  int channels = input_img.channels();

  input_img.forEach<cv::Vec3b>([&](cv::Vec3b &p, const int position[]) {
      int offset =  (position[0] * rows + position[1]) * channels;
      hostDataBuffer[offset] = (p[0] / 255.0 - mean[0]) / std[0];
      hostDataBuffer[offset + 1] = (p[1] / 255.0 - mean[1]) / std[1];
      hostDataBuffer[offset + 2] = (p[2] / 255.0 - mean[2]) / std[2];
    });

  /* position embedding */
  int feat_size = 0;
  if(image_type == std::string("template")) feat_size = template_feat_size_;
  if(image_type == std::string("search")) feat_size = search_feat_size_;
  std::string binding_name = image_type + std::string("_pos_embed");

  if(mask_bounds[0] >= 1 || mask_bounds[1] >= 1 || mask_bounds[2] < feat_size - 1 || mask_bounds[3] < feat_size - 1)
    {
      std::vector<int> discretized_mask{(int)std::ceil(mask_bounds[0]), (int)std::ceil(mask_bounds[1]), (int)std::floor(mask_bounds[2]), (int)std::floor(mask_bounds[3])};
      makePositionEmbedding(static_cast<float*>(buffers->getHostBuffer(binding_name)), feat_size, discretized_mask);
    }
  else
    {
      if(image_type == std::string("template"))
        std::memcpy(buffers->getHostBuffer(binding_name), (void*)default_template_pos_embed_.data, buffers->size(binding_name)); // TODO: directly use buffer pointer

      if(image_type == std::string("search"))
        std::memcpy(buffers->getHostBuffer(binding_name), (void*)default_search_pos_embed_.data, buffers->size(binding_name)); // TODO: directly use buffer pointer
    }
}

void TensorRT::makePositionEmbedding(float* p, int feat_size, std::vector<int> mask_bounds)
{
  if(mask_bounds.empty()) mask_bounds = std::vector<int>{0, 0, feat_size, feat_size};

  //printf("%d, %d, %d, %d \n", mask_bounds[0], mask_bounds[1], mask_bounds[2], mask_bounds[3]);
  float factor_x =  2 * (float)M_PI / ((mask_bounds.at(2) - mask_bounds.at(0)) + 1e-6);
  float factor_y =  2 * (float)M_PI / ((mask_bounds.at(3) - mask_bounds.at(1)) + 1e-6);
#pragma omp parallel for
  for (int i = 0; i < feat_size; i++)
    {
      for (int j = 0; j < feat_size; j++)
        {
          for (int k = 0; k < 4; k++)
            {
              for(int l = 0; l < transformer_dim_ / 4; l++)
                {
                  int offset = i * feat_size * transformer_dim_ + j * transformer_dim_;
                  switch(k)
                    {
                    case 0:
                      {
                        int index = std::min(std::max(0, i + 1 - mask_bounds.at(1)), mask_bounds.at(3) - mask_bounds.at(1)); // TODO: should be 0 if i > mask_bounds.at(3)
                        if(j < mask_bounds.at(0) || j > mask_bounds.at(2)-1) index = 0;
                        offset += l * 2;
                        p[offset] = sinf(index * factor_y / pos_emded_dim_t_.at(l*2));
                        break;
                      }
                    case 1:
                      {
                        int index = std::min(std::max(0, i + 1 - mask_bounds.at(1)), mask_bounds.at(3) - mask_bounds.at(1)); // TODO: should be 0 if i > mask_bounds.at(3)
                        if(j < mask_bounds.at(0) || j > mask_bounds.at(2)-1) index = 0;
                        offset +=  l * 2 + 1;
                        p[offset] = cosf(index * factor_y / pos_emded_dim_t_.at(l*2 + 1));
                        break;
                      }
                    case 2:
                      {
                        int index = std::min(std::max(0, j + 1 - mask_bounds.at(0)), mask_bounds.at(2) - mask_bounds.at(0)); // TODO: should be 0 if i > mask_bounds.at(2)
                        if(i < mask_bounds.at(1) || i > mask_bounds.at(3)-1) index = 0;
                        offset += l * 2 + transformer_dim_ / 2;
                        p[offset] = sinf(index * factor_x / pos_emded_dim_t_.at(l*2));
                        break;
                      }
                    case 3:
                      {
                        int index = std::min(std::max(0, j + 1 - mask_bounds.at(0)), mask_bounds.at(2) - mask_bounds.at(0)); // TODO: should be 0 if i > mask_bounds.at(2)
                        if(i < mask_bounds.at(1) || i > mask_bounds.at(3)-1) index = 0;
                        offset += l * 2 + transformer_dim_ / 2 + 1;
                        p[offset] = cosf(index * factor_x / pos_emded_dim_t_.at(l*2 + 1));
                        break;
                      }
                    }
                }
            }
        }
    }
}

void TensorRT::createCosineWindow()
{
  // https://numpy.org/doc/stable/reference/generated/numpy.hanning.html
  cosine_window_ = cv::Mat(cv::Size(search_feat_size_, search_feat_size_), CV_32F);
  for(int i = 0; i < search_feat_size_; i++)
    {
      for(int j = 0; j < search_feat_size_; j++)
        {
          float v_i = 0.5 - 0.5 * cosf(2 * M_PI * i / (search_feat_size_ - 1));
          float v_j = 0.5 - 0.5 * cosf(2 * M_PI * j / (search_feat_size_ - 1));

          cosine_window_.at<float>(i,j) = v_i * v_j;
        }
    }
}


int main (int argc, char **argv)
{
  ros::init (argc, argv, "trtr_tracker");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  std::string trt_encoder_model_file;
  nhp.param ("trt_encoder_model_file", trt_encoder_model_file, std::string("model.trt"));

  std::string trt_decoder_model_file;
  nhp.param ("trt_decoder_model_file", trt_decoder_model_file, std::string("model.trt"));

  std::string img_dir;
  nhp.param ("image_dir", img_dir, std::string(""));

  std::string gt_file;
  nhp.param ("gt_file", gt_file, std::string(""));

  // hyper-parameter for postprocess
  // following default values are fine-tuned with various tracker benchmark.
  double cosine_window_factor, size_lpf_factor, score_threshold;
  int cosine_window_step;
  nhp.param ("cosine_window_factor", cosine_window_factor, 0.4);
  nhp.param ("cosine_window_step", cosine_window_step, 3);
  nhp.param ("size_lpf_factor", size_lpf_factor, 0.8);
  nhp.param ("score_threshold", score_threshold, 0.05);

  fs::path path(img_dir);

  std::vector<fs::path> img_files;
  for (const auto& e : boost::make_iterator_range(fs::directory_iterator( path ), { }))
        if ( ! fs::is_directory( e ) )
          img_files.push_back(e.path());
  std::sort(img_files.begin(), img_files.end(),
            [](const fs::path &a, const fs::path &b){ return atoi(a.stem().string().c_str()) < atoi(b.stem().string().c_str()); });

  // ground truth img
  std::ifstream ifs(gt_file);
  std::vector< std::vector<float> > gt_bboxes;
  if (ifs)
    {
      while (!ifs.eof())
        {
          std::string buf;
          std::string buf2;
          std::vector<float> gt_bbox;
          std::getline(ifs, buf);

          std::stringstream ss{buf};

          if (buf.empty()) continue;
          while(std::getline(ss, buf2, ','))
            {
              gt_bbox.push_back(atof(buf2.c_str()));
            }

          gt_bboxes.push_back(gt_bbox);
        }

      TensorRT trt_tracker(trt_encoder_model_file, trt_decoder_model_file, score_threshold, cosine_window_factor, cosine_window_step, size_lpf_factor);

      double sum_t = 0;
      for(int i = 0; i < img_files.size(); i++)
        {
          //std::cout << img_files.at(i).string() << std::endl;
          cv::Mat img = cv::imread(img_files.at(i).string());

          if(i == 0)
            {
              trt_tracker.init(gt_bboxes.at(0), img);
              continue;
            }
          else
            {
              auto start_t = ros::Time::now().toSec();
              trt_tracker.track(img);
              double t = ros::Time::now().toSec() - start_t;
              sum_t += t;
              //std::cout <<  << std::endl;
            }


          cv::rectangle(img,
                        trt_tracker.getBboxLT(), trt_tracker.getBboxRB(),
                        cv::Scalar(0,255,255),3,4);

          cv::imshow("result", img);
          auto k = cv::waitKey(10);

          if(k == 27) break;

#if 0
          auto gt_bbox = gt_bboxes.at(i);
          cv::rectangle(img,
                        cv::Point(gt_bbox.at(0), gt_bbox.at(1)),
                        cv::Point(gt_bbox.at(0) + gt_bbox.at(2), gt_bbox.at(1) + gt_bbox.at(3)),
                        cv::Scalar(0,255,0),3,4);
          cv::imshow("test", img);
          cv::waitKey(10);
#endif
        }

      std::cout << (img_files.size() - 1) / sum_t << " FPS" <<std::endl;
    }


  return 0;
}
