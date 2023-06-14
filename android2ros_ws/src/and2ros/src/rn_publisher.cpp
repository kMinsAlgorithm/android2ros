#include <iostream>
#include <ros/ros.h> // ROS 관련 헤더 파일 추가
#include <std_msgs/String.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <string.h>
#include <string>
#include <signal.h>
using namespace std;
 
//this script can be used as debug to check if the menssage sent via socket is comming correctlly
//To run, go to the terminal, change the directory (cd) until tcp_cpp.cpp folder
//Then run: 
// g++ tcp_cpp.cpp
// ./a.out
// the script will be waiting for data to come via socket on the port 6000 (edit it if is necessary. It may match with the client port.) 



// Ctrl+C 입력 시 실행될 함수
void sigintHandler(int sig) {
    // 원하는 작업 수행
    // 예를 들어, 파일 저장, 정리 작업 등

    // ROS 종료
    ros::shutdown();
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "minimal_subscriber"); // ROS 초기화

    ros::NodeHandle nh; // 노드 핸들러 생성
    ros::Publisher publisher = nh.advertise<std_msgs::String>("app_command_data", 10); // 퍼블리셔 생성

    // SIGINT 시그널 핸들러 등록
    signal(SIGINT, sigintHandler);

    int listening = socket(AF_INET, SOCK_STREAM, 0);
    if (listening == -1) {
        std::cerr << "Can't create a socket! Quitting" << std::endl;
        return -1;
    } else {
        std::cout << "Socket created" << std::endl;
    }

    sockaddr_in hint;
    memset(&hint, 0, sizeof(hint));

    hint.sin_family = AF_INET;
    hint.sin_port = htons(6000);
    inet_pton(AF_INET, "172.20.10.10", &hint.sin_addr);

    bind(listening, reinterpret_cast<sockaddr*>(&hint), sizeof(hint));
    listen(listening, SOMAXCONN);

    sockaddr_in client;
    socklen_t clientSize = sizeof(client);
    int clientSocket = accept(listening, reinterpret_cast<sockaddr*>(&client), &clientSize);
    ROS_INFO("listening");
    close(listening);

    char buf[4096];
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        memset(buf, 0, 4096);
        int bytesReceived = recv(clientSocket, buf, 4096, 0);

        if (bytesReceived == -1) {
            std::cerr << "Error in recv(). Quitting" << std::endl;
            break;
        }

        if (bytesReceived == 0) {
            std::cerr << "Client disconnected" << std::endl;
            break;
        }

        std_msgs::String message;
        message.data = buf;
        ROS_INFO("Publishing: '%s'", message.data.c_str());
        publisher.publish(message);

        ros::spinOnce();
        loop_rate.sleep();
    }

    close(clientSocket);

    return 0;
}
