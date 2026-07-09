#include "yolov8.h"

cv::Mat Preprocess(const cv::Mat& image, int inputWidth , int inputHeight ) {
    if (image.empty() || inputWidth <= 0 || inputHeight <= 0) return cv::Mat();
    const int height = image.rows;
    const int width = image.cols;
    const double scale = std::min(static_cast<double>(inputHeight) / height, static_cast<double>(inputWidth) / width);
    const int new_h = static_cast<int>(std::round(height * scale));
    const int new_w = static_cast<int>(std::round(width * scale));
    const int pad_h = (inputHeight - new_h) / 2;
    const int pad_w = (inputWidth - new_w) / 2;
    cv::Mat processed_image;
    if (new_h != height || new_w != width) {
        cv::resize(image, processed_image, cv::Size(new_w, new_h));
    } else {
        processed_image = image;
    }
    cv::copyMakeBorder(processed_image, processed_image, pad_h, inputHeight - new_h - pad_h,
                       pad_w, inputWidth - new_w - pad_w, cv::BORDER_CONSTANT, cv::Scalar(0));
    return cv::dnn::blobFromImage(processed_image, 1.0/255.0, cv::Size(inputWidth, inputHeight), cv::Scalar(0, 0, 0), true, false, CV_32F);
}

void DrawResults(cv::Mat& image, const std::vector<Object>& dets) {        
    std::vector<std::string> labels = ReadLabels();
    for (const auto& det : dets) {        
        int x1 = static_cast<int>(det.x1);
        int y1 = static_cast<int>(det.y1);
        int x2 = static_cast<int>(det.x2);
        int y2 = static_cast<int>(det.y2);
        cv::rectangle(image, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);        
        
        std::string labelText = (labels.size() > det.class_id ? labels[det.class_id] : std::to_string(det.class_id)) + ": " + std::to_string(det.score).substr(0, 4);
        cv::putText(image, labelText, cv::Point(x1, y1 - 10), cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 255, 0), 2);        
    }
}

void Get_Dets(const cv::Mat& image, const float* boxes, const float* scores, const float* score_sum, std::vector<int64_t> dims, int tensor_width, int tensor_height, std::vector<Object>& objects) {
    int grid_w = static_cast<int>(dims[2]);
    int grid_h = static_cast<int>(dims[3]);
    int anchors_per_branch = grid_w * grid_h;
    float scale_w = static_cast<float>(tensor_width) / static_cast<float>(grid_w);
    float scale_h = static_cast<float>(tensor_height) / static_cast<float>(grid_h);
    int orig_height = image.rows;
    int orig_width = image.cols;
    float scale2orign = fmin(static_cast<float>(tensor_height) / static_cast<float>(orig_width), static_cast<float>(tensor_width) / static_cast<float>(orig_height));
    int pad_h = static_cast<int>((tensor_width - orig_height * scale2orign) / 2);
    int pad_w = static_cast<int>((tensor_height - orig_width * scale2orign) / 2);

    for (int anchor_idx = 0; anchor_idx < anchors_per_branch; anchor_idx++) {
        if (score_sum[anchor_idx] < conf_threshold) continue;
        float max_score = -1.0f;
        int classId = -1;
        for (int class_idx = 0; class_idx < classNum; class_idx++) {
            size_t score_offset = class_idx * anchors_per_branch + anchor_idx;
            if ((scores[score_offset] > conf_threshold) & (scores[score_offset] > max_score)) { 
                max_score = *(scores + score_offset);
                classId = class_idx;
            }
        }
        if (classId >= 0) { 
            Object object = Dfl(boxes, anchor_idx, anchors_per_branch, grid_h, scale_w, scale_h, scale2orign, pad_w, pad_h);
            object.class_id = classId;
            object.score = max_score;
            objects.push_back(object);
        }
    }
}

Object Dfl(const float* boxes, int anchor_idx, int anchors,  int grid_w, float scale_w, float scale_h, float scale2orign, int pad_w, int pad_h) {
    float xywh[4] = {0, 0, 0, 0};
    for (int i = 0; i < 4; i++) {
        float exp_sum = 0.0f;
        size_t offset = i * dfl_len * anchors + anchor_idx; 
        float exp_dfl[dfl_len];
        for (int dfl_idx = 0; dfl_idx < dfl_len; dfl_idx++) {
            exp_dfl[dfl_idx] = exp(boxes[offset]);
            exp_sum += exp_dfl[dfl_idx];
            offset += anchors;
        }
        offset = i * dfl_len * anchors + anchors; 
        for (int dfl_idx = 0; dfl_idx < dfl_len; dfl_idx++) {
            xywh[i] += (exp_dfl[dfl_idx] / exp_sum) * dfl_idx;
            offset += anchors;
        }
    }
    int h_idx = anchor_idx / grid_w;
    int w_idx = anchor_idx % grid_w;
    Object object;
    object.x1 = ((w_idx - xywh[0] + 0.5f) * scale_w - pad_w) / scale2orign;
    object.y1 = ((h_idx - xywh[1] + 0.5f) * scale_h - pad_h) / scale2orign;
    object.x2 = ((w_idx + xywh[2] + 0.5f) * scale_w - pad_w) / scale2orign;
    object.y2 = ((h_idx + xywh[3] + 0.5f) * scale_h - pad_h) / scale2orign;
    return object;
}

float Calculate_Iou(const Object& det1, const Object& det2) {
    float x1_inter = std::max(det1.x1, det2.x1);
    float y1_inter = std::max(det1.y1, det2.y1);
    float x2_inter = std::min(det1.x2, det2.x2);
    float y2_inter = std::min(det1.y2, det2.y2);
    float width_inter = std::max(0.0f, x2_inter - x1_inter);
    float height_inter = std::max(0.0f, y2_inter - y1_inter);
    float area_inter = width_inter * height_inter;
    float area1 = (det1.x2 - det1.x1) * (det1.y2 - det1.y1);
    float area2 = (det2.x2 - det2.x1) * (det2.y2 - det2.y1);
    float area_union = area1 + area2 - area_inter;
    if (area_union == 0) return 0;
    return area_inter / area_union;
}

std::vector<Object> Nms(const std::vector<Object>& dets) {
    if (dets.empty()) return std::vector<Object>();
    std::vector<Object> final_dets;
    std::vector<int> unique_labels;
    for (const auto& det : dets) {
        if (std::find(unique_labels.begin(), unique_labels.end(), det.class_id) == unique_labels.end()) {
            unique_labels.push_back(det.class_id);
        }
    }
    for (int label : unique_labels) {
        std::vector<Object> dets_class;
        for (const auto& det : dets) {
            if (det.class_id == label) dets_class.push_back(det);
        }
        std::sort(dets_class.begin(), dets_class.end(), [](const Object& a, const Object& b) {
            return a.score > b.score;
        });
        std::vector<Object> keep;
        while (!dets_class.empty()) {
            keep.push_back(dets_class[0]);
            if (dets_class.size() == 1) break;
            std::vector<Object> new_dets_class;
            for (size_t i = 1; i < dets_class.size(); ++i) {
                float iou = Calculate_Iou(keep.back(), dets_class[i]);
                if (iou < iou_threshold) new_dets_class.push_back(dets_class[i]);
            }
            dets_class = new_dets_class;
        }
        final_dets.insert(final_dets.end(), keep.begin(), keep.end());
    }
    return final_dets;
}

std::vector<std::string> ReadLabels() {
    std::vector<std::string> labels;
    std::ifstream labelFile(labelFilePath);
    if (labelFile.is_open()) {
        std::string line;
        while (std::getline(labelFile, line)) labels.push_back(line);
        labelFile.close();
    }
    return labels;
}

std::vector<Object> Postprocess(cv::Mat &image, std::vector<Ort::Value>& outputs, size_t output_num, const int inputWidth, const int inputHeight, std::vector<Object> &objects) {
    for (int i = 0; i < int(output_num / 3); i++) {        
        const float* boxes = outputs[i * 3].GetTensorMutableData<float>();
        const float* scores = outputs[i * 3 + 1].GetTensorMutableData<float>();
        const float* score_sum = outputs[i * 3 + 2].GetTensorMutableData<float>();
        std::vector<int64_t> dims = outputs[i * 3].GetTensorTypeAndShapeInfo().GetShape();            
        Get_Dets(image, boxes, scores, score_sum, dims, inputHeight, inputWidth, objects); 
    }
    return Nms(objects);
}

cv::Mat Yolov8Inference(cv::Mat& image, const std::string& modelPath) {
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "YOLOv8Inference");
    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(4);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    SessionOptionsSpaceMITEnvInit(session_options);
    Ort::Session session_(env, modelPath.c_str(), session_options);

    Ort::AllocatorWithDefaultOptions allocator;
    std::vector<const char*> input_node_names_;
    std::vector<std::string> input_names_;    
    size_t num_inputs_ = session_.GetInputCount();
    input_node_names_.resize(num_inputs_);
    input_names_.resize(num_inputs_, "");    
    for (size_t i = 0; i < num_inputs_; ++i) {
        auto input_name = session_.GetInputNameAllocated(i, allocator);        
        input_names_[i].append(input_name.get());
        input_node_names_[i] = input_names_[i].c_str();                        
    }
    Ort::TypeInfo input_type_info = session_.GetInputTypeInfo(0);
    auto input_tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
    std::vector<int64_t> input_dims = input_tensor_info.GetShape();
    int inputWidth = input_dims[3];
    int inputHeight = input_dims[2];

    std::vector<const char*> output_node_names_;
    std::vector<std::string> output_names_;
    size_t num_outputs_ = session_.GetOutputCount();
    output_node_names_.resize(num_outputs_);
    output_names_.resize(num_outputs_, "");    
    for (size_t i = 0; i < num_outputs_; ++i) {
        auto output_name = session_.GetOutputNameAllocated(i, allocator);
        output_names_[i].append(output_name.get());        
        output_node_names_[i] = output_names_[i].c_str();
    }

    cv::Mat inputTensor = Preprocess(image, inputWidth, inputHeight);    
    std::vector<int64_t> input_shape = {1, 3, inputHeight, inputWidth};
    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(memory_info, inputTensor.ptr<float>(), inputTensor.total(), input_shape.data(), input_shape.size());

    std::vector<Ort::Value> outputs = session_.Run(Ort::RunOptions{nullptr}, input_node_names_.data(), &input_tensor, 1, output_node_names_.data(), output_node_names_.size());
    std::vector<Object> objects;
    std::vector<Object> detected_objects = Postprocess(image, outputs, num_outputs_, inputWidth, inputHeight, objects);
    DrawResults(image, detected_objects);
    return image;
}

// 💥 我们魔改的常驻内存版极限提取函数
std::vector<Object> Yolov8_Fast_GetBoxes(cv::Mat& image, const std::string& modelPath) {
    static Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "YOLOv8Inference");
    static Ort::SessionOptions session_options;
    static std::unique_ptr<Ort::Session> session_ptr = nullptr;
    
    static std::vector<const char*> input_node_names_;
    static std::vector<const char*> output_node_names_;
    static std::vector<std::string> input_names_;
    static std::vector<std::string> output_names_;
    static int inputWidth = 0;
    static int inputHeight = 0;
    static size_t num_outputs_ = 0;

    if (session_ptr == nullptr) {
        session_options.SetIntraOpNumThreads(4);
        session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        SessionOptionsSpaceMITEnvInit(session_options); 
        
        session_ptr = std::make_unique<Ort::Session>(env, modelPath.c_str(), session_options);

        Ort::AllocatorWithDefaultOptions allocator;
        size_t num_inputs_ = session_ptr->GetInputCount();
        input_node_names_.resize(num_inputs_);
        input_names_.resize(num_inputs_, "");    
        for (size_t i = 0; i < num_inputs_; ++i) {
            auto input_name = session_ptr->GetInputNameAllocated(i, allocator);        
            input_names_[i].append(input_name.get());
            input_node_names_[i] = input_names_[i].c_str();                        
        }
        Ort::TypeInfo input_type_info = session_ptr->GetInputTypeInfo(0);
        auto input_tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
        std::vector<int64_t> input_dims = input_tensor_info.GetShape();
        inputWidth = input_dims[3];
        inputHeight = input_dims[2];

        num_outputs_ = session_ptr->GetOutputCount();
        output_node_names_.resize(num_outputs_);
        output_names_.resize(num_outputs_, "");    
        for (size_t i = 0; i < num_outputs_; ++i) {
            auto output_name = session_ptr->GetOutputNameAllocated(i, allocator);
            output_names_[i].append(output_name.get());        
            output_node_names_[i] = output_names_[i].c_str();
        }
    }

    cv::Mat inputTensor = Preprocess(image, inputWidth, inputHeight);    
    std::vector<int64_t> input_shape = {1, 3, inputHeight, inputWidth};
    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(memory_info, inputTensor.ptr<float>(), inputTensor.total(), input_shape.data(), input_shape.size());

    std::vector<Ort::Value> outputs = session_ptr->Run(Ort::RunOptions{nullptr}, input_node_names_.data(), &input_tensor, 1, output_node_names_.data(), output_node_names_.size());
    
    std::vector<Object> objects;
    return Postprocess(image, outputs, num_outputs_, inputWidth, inputHeight, objects);
}