#include <QApplication>
#include <QLabel>
#include <QString>

int main(int argc, char *argv[])
{
    //QLabel 是一个用于显示文本或图片的 Qt 组件
    QApplication app(argc, argv);
    //使用 new 关键字创建 QLabel 对象 label，它会在堆上分配内存
    QLabel* label = new QLabel();
    //fromStdString("Hello, Qt!") 将标准 C++ 字符串 std::string 转换为 QString，以确保与 Qt 兼容。
    QString message = QString::fromStdString("Hello, Qt!");
    //setText() 设置 QLabel 的显示内容，将 QString message 传递给 label
    label->setText(message);
    label->show();
    //exec() 启动 Qt 事件循环，让应用保持运行，等待用户交互（如鼠标点击、窗口关闭等）。
    //这个函数不会返回，直到应用程序退出（如用户关闭窗口）。
    app.exec();
    return 0;
}
 
 