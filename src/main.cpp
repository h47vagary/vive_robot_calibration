#include "vive_wrapper.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <windows.h>
#include <QApplication>
#include "main_window.h"

int main(int argc, char *argv[])
{
    std::cout << "MainWindow constructed" << std::endl;
    QApplication app(argc, argv);
    MainWindow window;
    window.show();

    
    return app.exec();
    std::cout << "main() exited" << std::endl;
}
