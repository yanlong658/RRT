#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <string>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Core>
#include <Eigen/Geometry> 
#include "rclcpp/rclcpp.hpp"
#include <stdio.h>
#include <unistd.h>


void on_Mouse(int event, int x, int y, int flags, void* param) {
    cv::Mat *im = reinterpret_cast<cv::Mat*>(param);

    if (event == cv::EVENT_MOUSEMOVE){
        std::cout << "at(" << x << "," << y << ")pixs value is:" << static_cast<int>
			(im->at<uchar>(cv::Point(x, y))) << std::endl;
    }
    

}

class Map{
    public:
        Map(std::string imagPath){
            originalImage = cv::imread(imagPath, cv::IMREAD_COLOR);
            cv::cvtColor(originalImage, binaryImage, cv::COLOR_BGR2GRAY);
            //cv::threshold( originalImage, binaryImage, 100, 0, 2);
            length = originalImage.rows;
            width = originalImage.cols;
            std::cout<<"length : "<<length<<std::endl; //800
            std::cout<<"width : "<<width<<std::endl; // 1600
        }

        void getObstacle(){

            cv::namedWindow("RRT", cv::WINDOW_NORMAL);
            cv::setMouseCallback("RRT", on_Mouse, reinterpret_cast<void *>(&binaryImage));
            cv::imshow("RRT", binaryImage);
            cv::waitKey(0);

        }
    public:
        cv::Mat originalImage;
        cv::Mat grayImage;
        cv::Mat binaryImage;
        float length, width;
};

class Node{
    public:
        float x;
        float y;
        Node* parent;
    
    public:
        Node(float x_, float y_):parent(nullptr){
            x = x_;
            y = y_;
        }

        void setParent(Node* parent_){
            parent = parent_;
        }
        Node* getParent(){
            return parent;
        }

        float getX(){
            return x;
        }

        float getY(){
            return y;
        }
};

class RRT{
    public:
        RRT(Map map, Node* startNode_, Node* endNode_,
            float stepSize_ = 5.0, int goal_sample_rate_ = 5):
            goal_gen(goal_rd()), goal_dis(std::uniform_int_distribution<int>(0, 100)),
            area_gen(area_rd()), area_dis(std::uniform_real_distribution<float>(0, 1)){

            xLimit = map.width;
            yLimit = map.length;
            background = map.originalImage;
            binaryImage = map.binaryImage;

            startNode = startNode_;
            endNode = endNode_;

            stepSize = stepSize_;
            goal_sample_rate = goal_sample_rate_;

        }

        bool checkLimit(){
            bool flag = true;
            if (startNode->x > xLimit && startNode->x < 0.0){
                flag = false;
            }
            if(endNode->y > yLimit && endNode->y < 0.0){
                flag = false;
            }

            return flag;
        }

        bool checkCollision(Node* newNode){
            if ( static_cast<int>(binaryImage.at<uchar>( cv::Point(int(newNode->getX()), int(newNode->getY())) )) < 255 ){
                cv::circle(background, cv::Point(newNode->getX(), newNode->getY()), 5, cv::Scalar(100, 20, 255), -1);
                return true;
            }
            return false;

        }

        Node* getNearestNode(std::vector<float> randomPosition){
            int minID = -1;
            float minDistance = std::numeric_limits<float>::max(); // 编译器允许的float类型的最大值

            for(int i = 0; i<nodeList.size(); i++){
                float distance = std::pow(nodeList[i]->getX() - randomPosition[0], 2) + std::pow(nodeList[i]->getY() - randomPosition[1], 2);
                if (distance < minDistance) {
                    minDistance = distance;
                    minID = i;
                }
            }
            return nodeList[minID];
        }

        bool planning(){
            if (!checkLimit()){
                std::cout<<"Input is error."<<std::endl;
                return false;
            }
                
            cv::namedWindow("RRT", cv::WINDOW_NORMAL);

            cv::circle(background, cv::Point(startNode->getX(), startNode->getY()), 20, cv::Scalar(0, 0, 255), -1);
            cv::circle(background, cv::Point(endNode->getX(),endNode->getY()), 20, cv::Scalar(255, 0, 0), -1);
 
            nodeList.push_back(startNode);

            while (1)
            {
                std::vector<float> randomPosition;
                if (goal_dis(goal_gen) > goal_sample_rate) { // 这里可以优化成直接用节点来表示
                    float randX = area_dis(goal_gen) * xLimit; //800
                    float randY = area_dis(goal_gen) * yLimit;  //1600
                    randomPosition.push_back(randX);
                    randomPosition.push_back(randY);
                }
                else { // 找到了目標,將目標位置保存
                    randomPosition.push_back(endNode->getX());
                    randomPosition.push_back(endNode->getY());
                }

                // 找到和新生成隨機節點距離最近的節點
                Node* nearestNode = getNearestNode(randomPosition);
                // 利用反正切計算角度,然後利用角度和步長計算新坐標
                float theta = atan2(randomPosition[1] - nearestNode->getY(), randomPosition[0] - nearestNode->getX());
                Node* newNode = new Node(nearestNode->getX() + stepSize * cos(theta), nearestNode->getY() + stepSize * sin(theta));
                newNode->setParent(nearestNode);

                if (checkCollision(newNode)){
                    continue;
                }
                nodeList.push_back(newNode);


                cv::line(background,
                         cv::Point(static_cast<int>(newNode->getX()), static_cast<int>(newNode->getY())),
                         cv::Point(static_cast<int>(nearestNode->getX()), static_cast<int>(nearestNode->getY())),
                         cv::Scalar(0, 255, 0),10);

                cv::imshow("RRT", background);
                cv::waitKey(5);

                if (std::sqrt(pow(newNode->getX() - endNode->getX(), 2) + pow(newNode->getY() - endNode->getY(), 2)) <= stepSize) {
                    std::cout << "The path has been found!" << std::endl;
                    break;
                }
            }

            std::vector<Node*> path;
            path.push_back(endNode);
            Node* tmpNode = nodeList.back();
            while (tmpNode->getParent() != nullptr) {
                cv::line(background,
                    cv::Point(static_cast<int>(tmpNode->getX() ), static_cast<int>(tmpNode->getY() )),
                    cv::Point(static_cast<int>(tmpNode->getParent()->getX() ), static_cast<int>(tmpNode->getParent()->getY())),
                    cv::Scalar(255, 0, 255), 10);
                path.push_back(tmpNode);
                tmpNode = tmpNode->getParent();
            }

            cv::imshow("RRT", background);
            cv::imwrite("test.jpg", background);
            cv::waitKey(0);
            path.push_back(startNode);
        }

    public:
        Node* startNode;
        Node* endNode;
        std::vector<Node*> nodeList;
        std::vector<std::vector<float>> obstacleList;


    private:
        float xLimit;
        float yLimit;
        float stepSize;
        int goal_sample_rate;
        cv::Mat background;
        cv::Mat binaryImage;

        std::random_device goal_rd;
        std::mt19937 goal_gen;
        std::uniform_int_distribution<int> goal_dis;

        std::random_device area_rd;
        std::mt19937 area_gen;
        std::uniform_real_distribution<float> area_dis;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;

    Map map("/home/yanlong/career_ws/src/rrt/map/map.png");
    //map.getObstacle();
    Node* startNode = new Node(30,30);
    Node* endNode = new Node(700,700);

    RRT rrt(map, startNode, endNode);

    if(rrt.planning() == true){
        std::cout<<"Planning is sucessful."<<std::endl;
    }

    rclcpp::shutdown();
    return 0;
}