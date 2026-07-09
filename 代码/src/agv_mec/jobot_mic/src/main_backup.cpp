/*************************************************************************************
@company: WHEELTEC (Dongguan) Co., Ltd
@product: 4/6mic
@filename: main.c
@brief:
@version:       date:       author:            comments:
@v1.0           22-4-16      hj,choi                main
*************************************************************************************/

#include "com_test.h"
#include "record.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <pthread.h>
#include <atomic>
#include <chrono>

// extern std::atomic<int> angle_int;
// extern std::atomic<bool> if_awake;
extern int if_awake;
extern int angle_int;
extern int record_finish;
extern int init_success;

char *temp;

char *join(char *b, char *s2)
{
    char s1[600] = "";
    strcpy(s1, b);
    char *result = (char *)malloc(strlen(s1) + strlen(s2) + 1);
    if (result == NULL)
        exit(1);
    strcpy(result, s1);
    strcat(result, s2);
    return result;
}

const char *time_name()
{
    char str_a[80];
    char str_b[80];
    char str_c[80] = ".pcm";//pcm文件
    time_t current = time(NULL);
    struct tm* timer = localtime(&current);
    strftime(str_a, 80, "%Y-%m-%d", timer);
    strftime(str_b, 80, " %H:%M:%S", timer);
    temp = (char*)malloc(sizeof(char) * 50);//申请内存空间，此处申请大小为50个字符(char)的大小 
    if (NULL == temp)
        {
            printf("\nout of memory! \n");
            exit (1);
        }
    strcpy(temp,str_a);
    strcat(temp,str_b);
    strcat(temp,str_c);
    return temp;
}

void *com_wakeup(void* args)
{
    int fd=1, read_num = 0;
    unsigned char buffer[1];
    memset(buffer, 0, 1);
    char* uartname="/dev/wheeltec_mic";
    //char* uartname="/dev/ttyACM0";
    if((fd=open_port(uartname))<0)
    {
         printf("open %s is failed\n",uartname);
         return 0;
    }
    else{
            set_opt(fd, 115200, 8, 'N', 1);
            //printf("set_opt fd=%d\n",fd);

            while(1){
                    usleep(1000);
                    memset(buffer, 0, 1);
                    read_num = read(fd, buffer, 1);
                    //printf("read_num=%d\n",read_num);
                    if(read_num>0){
						//printf("%x\n", buffer[0]);
						deal_with(buffer[0]);
                    }
                    //else{
                    //    printf("read error\n");
                    //}
                    if(if_awake){
						//printf("angle : [%d]\n",angle_int);
                        record_finish = 0;
					}    
            }
          fd=close(fd);
    }
    return 0;
}

void *time_count(void* args)
{
    while(1){
        if (!init_success && if_awake){
            sleep(RECORD_TIME);
            record_finish = 1;
            if_awake = 0;     
        }    
    }
    return 0;
}

 
// int main()
// {
//     pthread_t pid1;
//     pthread_t pid2;
 
//     if(pthread_create(&pid1, NULL, com_wakeup, NULL))
//     {
//         return -1;
//     }

//     if(pthread_create(&pid2, NULL, time_count, NULL))
//     {
//         return -1;
//     }
 
//     while(1){

//         if (!if_awake){
//             printf(">>>>>待唤醒，请用唤醒词进行唤醒！\n");
//         }
//         while(!if_awake){
//             sleep(0.1);
//         }
//         //sleep(1);
//         printf("\n>>>>>唤醒成功！\n");
//         printf("angle : [%d]\n",angle_int);
//         // time_name();
//         // while(if_awake){
//         //     const char* file = join(ORIGINAL_SOUND_PATH,temp); 
//         //     printf("file_name : [%s]\n",file);
//         //     get_the_record_sound(file);
//         // }
//         sleep(1);
//     }
//     return 0;
// }

// ROS2 角度发布者节点
class AnglePublisher : public rclcpp::Node {
public:
    AnglePublisher() : Node("angle_publisher") {
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("angle_topic", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
            std::bind(&AnglePublisher::publish_angle, this));
    }

private:
    void publish_angle() {
        auto message = std_msgs::msg::Int32();
        // message.data = angle_int.load();  // 读取 `angle_int`
        message.data = angle_int;  // 读取 `angle_int`

        RCLCPP_INFO(this->get_logger(), "Publishing: %d", message.data);
        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    pthread_t pid1;
    if (pthread_create(&pid1, NULL, com_wakeup, NULL)) {
        return -1;
    }

    auto node = std::make_shared<AnglePublisher>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}