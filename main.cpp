#include "mainwindow.h"

#include <QApplication>
#include <QMessageBox>
#include <QDateTime>
#include <windows.h>
#include <string>

// 写入Windows事件日志
void writeToWindowsEventLog(const QString& errorMessage)
{
    HANDLE hEventLog = RegisterEventSourceA(NULL, "SLCalibration");
    if (hEventLog) {
        const char* messages[1];
        QByteArray errorMsg = errorMessage.toLocal8Bit();
        messages[0] = errorMsg.constData();
        
        ReportEventA(hEventLog,        // 事件日志句柄
                     EVENTLOG_ERROR_TYPE,  // 事件类型：错误
                     0,                  // 事件类别
                     1,                  // 事件ID
                     NULL,               // 用户安全标识符
                     1,                  // 字符串数量
                     0,                  // 原始数据长度
                     messages,           // 字符串数组
                     NULL);              // 原始数据
        
        DeregisterEventSource(hEventLog);
    }
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    
    MainWindow w;
    w.show();
    return a.exec();
}
