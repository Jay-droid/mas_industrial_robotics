/*
 * Copyright 2022 Bonn-Rhein-Sieg University
 *
 * Author: Kevin Patel, Vamsi Kalagaturu
 * Adapted for YOLOv11 Segmentation by Jay Patil (2026)
 */

#include <mir_object_recognition/yolo_inference.hpp>

YoloInference::YoloInference(std::string net_file, std::string classes_file)
{
    try
    {
        YAML::Node config = YAML::LoadFile(classes_file);

        if (config["class_names"])
        {
            for (auto class_name : config["class_names"])
            {
                class_list.push_back(class_name.as<std::string>());
            }
        }
        else
        {
            std::cerr << "[Yolo Inference] No class_names found in " << classes_file.c_str() << std::endl;
        }
    }
    catch (const YAML::ParserException &ex)
    {
        std::cerr << ex.what() << std::endl;
        return;
    }

    try
    {
        net = readNet(net_file);
    }
    catch (cv::Exception &e)
    {
        std::cerr << std::endl
                  << std::endl
                  << e.msg << std::endl
                  << std::endl;
        return;
    }
}

YoloInference::~YoloInference()
{
}

RecognizedObjectList YoloInference::run_inference(Mat &input_image)
{
    RecognizedObjectList recoglist;
    if (input_image.empty())
    {
        std::cerr << "Error: Input image is empty!" << std::endl;
        return recoglist;
    }
    vector<Mat> detections = pre_process(input_image);
    recoglist = post_process(input_image, detections);

    return recoglist;
}

void YoloInference::draw_label(Mat &input_image, string label, int left, int top)
{
    int baseLine;
    Size label_size = getTextSize(label, FONT_FACE, FONT_SCALE, THICKNESS, &baseLine);
    top = max(top, label_size.height);
    Point tlc = Point(left, top);
    Point brc = Point(left + label_size.width, top + label_size.height + baseLine);
    rectangle(input_image, tlc, brc, BLACK, FILLED);
    putText(input_image, label, Point(left, top + label_size.height), FONT_FACE, FONT_SCALE, YELLOW, THICKNESS);
}

vector<Mat> YoloInference::pre_process(Mat &input_image)
{
    Mat blob;
    // YOLOv11 normalizes pixels by 1/255.0 and prefers RGB channel order
    std::cout << "Input image size: " << input_image.cols << " x " << input_image.rows << std::endl;
    blobFromImage(input_image, blob, 1. / 255., Size(INPUT_WIDTH, INPUT_HEIGHT), Scalar(), true, false);

    net.setInput(blob);

    vector<Mat> outputs;
    // Ask OpenCV to forward propagate and gather the output layers
    net.forward(outputs, net.getUnconnectedOutLayersNames());

    return outputs;
}

RecognizedObjectList YoloInference::post_process(Mat &input_image, vector<Mat> &outputs)
{
    vector<int> class_ids;
    vector<float> confidences;
    vector<Rect> boxes;
    RecognizedObjectList recognized_object_list;

    // Scaling factors mapping back to the native image space dimensions
    float x_factor = (float)input_image.cols / INPUT_WIDTH;
    float y_factor = (float)input_image.rows / INPUT_HEIGHT;

    // YOLOv11 Segmentation yields the box/class tensor in outputs[0]
    Mat raw_detections = outputs[0];

    // If output is a 3D tensor [1, channels, 8400], squash it down to 2D [channels, 8400]
    if (raw_detections.dims == 3)
    {
        int sizes[2] = {raw_detections.size[1], raw_detections.size[2]};
        raw_detections = Mat(2, sizes, CV_32F, raw_detections.ptr<float>());
    }

    // Modern Ultralytics models output shapes as [channels, 8400].
    // Transpose it so each row represents a separate candidate box [8400 rows]
    Mat detections = raw_detections.t();

    int num_classes = class_list.size();
    int total_candidate_rows = detections.rows; // Exactly 8400 at 640x640 resolution
    int row_stride = detections.cols;           // [x, y, w, h] + [classes] + [32 masks]

    float *data = (float *)detections.data;

    for (int i = 0; i < total_candidate_rows; ++i)
    {
        float *class_scores_ptr = data + 4;
        Mat scores(1, num_classes, CV_32FC1, class_scores_ptr);

        Point class_id;
        double max_class_score;
        minMaxLoc(scores, 0, &max_class_score, 0, &class_id);

        if (max_class_score >= SCORE_THRESHOLD)
        {
            confidences.push_back(max_class_score);
            class_ids.push_back(class_id.x);

            float cx = data[0];
            float cy = data[1];
            float w = data[2];
            float h = data[3];

            // Keep boxes in the base model's 640x640 resolution space for accurate NMS calculations
            int left = int(cx - 0.5f * w);
            int top = int(cy - 0.5f * h);
            int width = int(w);
            int height = int(h);

            boxes.push_back(Rect(left, top, width, height));
        }
        data += row_stride;
    }

    // Run NMS on perfectly scaled base model dimensions
    vector<int> indices;
    NMSBoxes(boxes, confidences, SCORE_THRESHOLD, NMS_THRESHOLD, indices);

    // Apply scaling factor mapping back to the camera frame resolution only for selected objects
    for (size_t i = 0; i < indices.size(); i++)
    {
        int idx = indices[i];
        RecognizedObject obj;
        Rect box = boxes[idx];

        int scaled_left = int(box.x * x_factor);
        int scaled_top = int(box.y * y_factor);
        int scaled_width = int(box.width * x_factor);
        int scaled_height = int(box.height * y_factor);

        // Clamp bounds to prevent image boundary segmentation layout faults
        obj.roi = {
            max(0, scaled_left),
            max(0, scaled_top),
            min(scaled_width, input_image.cols - scaled_left),
            min(scaled_height, input_image.rows - scaled_top)};

        string cls_name = class_list[class_ids[idx]];
        std::transform(cls_name.begin(), cls_name.end(), cls_name.begin(), ::toupper);
        obj.class_name = cls_name;
        obj.confidence = confidences[idx];

        recognized_object_list.push_back(obj);
    }

    return recognized_object_list;
}

int main(int argc, char **argv)
{
    CommandLineParser parser(argc, argv, "{net_file||}{classes_file||}{image_file||}");

    string net_file = parser.get<string>("net_file");
    string classes_file = parser.get<string>("classes_file");
    string image_file = parser.get<string>("image_file");

    Mat frame = cv::imread(image_file);
    if (frame.empty())
    {
        std::cerr << "Error: Could not read image file!" << std::endl;
        return -1;
    }

    YoloInference yolo_inference(net_file, classes_file);

    RecognizedObjectList recoglist = yolo_inference.run_inference(frame);

    for (size_t i = 0; i < recoglist.size(); i++)
    {
        RecognizedObject obj = recoglist[i];
        int left = obj.roi[0];
        int top = obj.roi[1];
        int width = obj.roi[2];
        int height = obj.roi[3];

        rectangle(frame, Point(left, top), Point(left + width, top + height), BLUE, 3 * THICKNESS);
        string label = format("%.2f", obj.confidence);
        label = obj.class_name + ":" + label;
        yolo_inference.draw_label(frame, label, left, top);
    }

    vector<double> layersTimes;
    double freq = getTickFrequency() / 1000;
    double t = yolo_inference.net.getPerfProfile(layersTimes) / freq;
    string label = format("Inference time : %.2f ms", t);
    putText(frame, label, Point(20, 40), FONT_FACE, FONT_SCALE, RED);

    imshow("YOLOv11 Bounding Box Test", frame);
    waitKey(0);
    return 0;
}