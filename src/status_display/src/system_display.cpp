#include <QApplication>
#include <QLabel>
#include <QString>
#include "rclcpp/rclcpp.hpp"
#include "status_interfaces/msg/system_status.hpp"
//using SystemStatus = status_interfaces::msg::SystemStatus; 给消息类型取别名，方便使用
using SystemStatus = status_interfaces::msg::SystemStatus;
class SysStatusDisplay : public rclcpp::Node
{
    public:
        SysStatusDisplay() : Node("system_display")
        {
            //Lambda 回调函数 ([&](const SystemStatus::SharedPtr msg) -> void {...})：
            //当收到 sys_status 消息时，调用 get_qstr_from_msg(msg) 处理消息，并更新 label_ 上的文本内容
            subscriber_ = this->create_subscription<SystemStatus>(
                "sys_status", 10, [&](const SystemStatus::SharedPtr msg) -> void {
                    label_->setText(get_qstr_from_msg(msg));
                });
        //label_ 是一个 QLabel 对象，用于显示系统状态信息
        //get_qstr_from_msg(std::make_shared<SystemStatus>()) 用一个默认 SystemStatus 消息初始化 label_，避免程序启动时为空。
        label_ = new QLabel(get_qstr_from_msg(std::make_shared<SystemStatus>()));
        //label_->show(); 让 QLabel 显示出来
        label_->show();
            }
    QString get_qstr_from_msg(const SystemStatus::SharedPtr msg) {
        std::stringstream show_str;
        show_str
        <<"基于ROS2的系统状态可视化显示工具\n"
        <<"数据时间：\t"<<msg->stamp.sec<<"\ts\n"
        <<"用户名：\t"<<msg->host_name<<"\t\n"
        <<"CPU使用率:\t"<<msg->cpu_percent<<"\t%\n"
        <<"内存使用率：\t"<<msg->memory_percent<<"\t%\n"
        <<"内存总大小：\t"<<msg->memory_total<<"\tMB\n"
        <<"剩余内存：\t"<<msg->memory_available<<"\tMB\n"
        <<"网络发送量：\t"<<msg->net_sent<<"\tMB\n"
        <<"网络接收量：\t"<<msg->net_recv<<"\tMB\n";
        //将 SystemStatus 消息内容 格式化为字符串，然后转换为 QString，便于 QLabel 显示
        return QString::fromStdString(show_str.str());
    }
    private:
    //ROS2 订阅者，用于接收 sys_status 主题的消息，共享指针类型为 SystemStatus
        rclcpp::Subscription<SystemStatus>::SharedPtr subscriber_;
    //用于显示系统状态信息的 QLabel 对象
        QLabel* label_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    //QLabel 是一个用于显示文本或图片的 Qt 组件
    QApplication app(argc, argv);
    auto node = std::make_shared<SysStatusDisplay>();
    //创建一个新线程 运行 rclcpp::spin(node);，持续监听 sys_status 主题，触发回调函数更新 QLabel 内容
    //使用线程是因为 rclcpp::spin(node); 是一个阻塞函数，会阻止后续代码执行
    //而且 Qt 的 GUI 程序必须在主线程中运行，所以需要新建一个线程运行 rclcpp::spin(node);
    //detach() 函数会将线程分离，使得线程结束时能够自动释放资源
    std::thread([&node]() {
        rclcpp::spin(node);
    }).detach();
    app.exec();
    rclcpp::shutdown();
    return 0;
}
