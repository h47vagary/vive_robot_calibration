#include "csv_parser.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>

#include "filter.h"

CSVParserWindow::CSVParserWindow(QWidget *parent)
{
    std::cout << __FUNCTION__ << std::endl;
    // 创建主窗口和布局
    QWidget *centralWidget = new QWidget(this);
    QVBoxLayout *layout = new QVBoxLayout(centralWidget);
    
    // 创建QCustomPlot图表
    customPlot = new QCustomPlot(this);
    layout->addWidget(customPlot);
    
    // 设置窗口大小
    setMinimumSize(800, 600);
    setCentralWidget(centralWidget);

    // 启用缩放和拖动功能
    customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
}

CSVParserWindow::~CSVParserWindow()
{
}

void CSVParserWindow::plotData()
{
    std::cout << __FUNCTION__ << std::endl;

    // 构造横轴: 索引（点数量）
    QVector<double> indices;
    int pointCount = x.size();
    for (int i = 0; i < pointCount; ++i)
        indices.append(i);

    std::cout << __FUNCTION__ << " pointCount: " << pointCount << std::endl;

    // 清除旧图
    customPlot->clearGraphs();

    // X 坐标图
    customPlot->addGraph();
    customPlot->graph(0)->setData(indices, toQVector(x));
    customPlot->graph(0)->setName("X");
    customPlot->graph(0)->setPen(QPen(Qt::blue));

    // Y 坐标图
    customPlot->addGraph();
    customPlot->graph(1)->setData(indices, toQVector(y));
    customPlot->graph(1)->setName("Y");
    customPlot->graph(1)->setPen(QPen(Qt::red));

    // Z 坐标图
    customPlot->addGraph();
    customPlot->graph(2)->setData(indices, toQVector(z));
    customPlot->graph(2)->setName("Z");
    customPlot->graph(2)->setPen(QPen(Qt::green));

    // A 角度图
    customPlot->addGraph();
    customPlot->graph(3)->setData(indices, toQVector(a));
    customPlot->graph(3)->setName("A");
    customPlot->graph(3)->setPen(QPen(Qt::magenta));

    // B 角度图
    customPlot->addGraph();
    customPlot->graph(4)->setData(indices, toQVector(b));
    customPlot->graph(4)->setName("B");
    customPlot->graph(4)->setPen(QPen(Qt::cyan));

    // C 角度图
    customPlot->addGraph();
    customPlot->graph(5)->setData(indices, toQVector(c));
    customPlot->graph(5)->setName("C");
    customPlot->graph(5)->setPen(QPen(Qt::darkYellow));

    // 设置坐标轴标签
    customPlot->xAxis->setLabel("点序号");
    customPlot->yAxis->setLabel("值");

    customPlot->legend->setVisible(true);
    customPlot->legend->setFont(QFont("Helvetica", 9));

    // 自适应缩放
    customPlot->rescaleAxes();
    customPlot->replot();
}

void CSVParserWindow::loadData(const QString &filename)
{
    std::cout << __FUNCTION__ << std::endl;
    x.clear();
    y.clear();
    z.clear();
    a.clear();
    b.clear();
    c.clear();
    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qDebug() << "无法打开文件:" << filename;
        return;
    }
    
    QTextStream in(&file);
    bool firstLine = true;
    
    while (!in.atEnd()) {
        QString line = in.readLine();
        if (firstLine) {
            firstLine = false;
            continue; // 跳过标题行
        }
        
        QStringList fields = line.split(',');
        if (fields.size() >= 6) {
            x.push_back(fields[0].toDouble());
            y.push_back(fields[1].toDouble());
            z.push_back(fields[2].toDouble());
            a.push_back(fields[3].toDouble());
            b.push_back(fields[4].toDouble());
            c.push_back(fields[5].toDouble());
        }
    }
    
    file.close();
}

void CSVParserWindow::loadData(const std::string &filename, bool is_filtering)
{
    std::cout << __FUNCTION__ << std::endl;
    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    x.clear();
    y.clear();
    z.clear();
    a.clear();
    b.clear();
    c.clear();

    std::string line;
    std::getline(file, line);

    while (std::getline(file, line))
    {
        std::istringstream ss(line);
        std::string token;
        CartesianPose pose;
        double in_x, in_y, in_z, in_a, in_b, in_c;

        // 依次读取6个字段
        if (!std::getline(ss, token, ',')) return;
        if (!parse_double(token, in_x)) return;

        if (!std::getline(ss, token, ',')) return;
        if (!parse_double(token, in_y)) return;

        if (!std::getline(ss, token, ',')) return;
        if (!parse_double(token, in_z)) return;

        if (!std::getline(ss, token, ',')) return;
        if (!parse_double(token, in_a)) return;

        if (!std::getline(ss, token, ',')) return;
        if (!parse_double(token, in_b)) return;

        if (!std::getline(ss, token, ',')) return;
        if (!parse_double(token, in_c)) return;

        x.push_back(in_x);
        y.push_back(in_y);
        z.push_back(in_z);
        a.push_back(in_a);
        b.push_back(in_b);
        c.push_back(in_c);
    }

    if (is_filtering)
    {
        std::unique_ptr<IPoseFilter> pose_filter = std::make_unique<MovingAverageFilter>(filter_window_size_);
        pose_filter->set_filter_param(11);
        pose_filter->filter_xyzabc(x, y, z, a, b, c);
    }
}

bool CSVParserWindow::parse_double(const std::string &str, double &value)
{
    char* endptr = nullptr;
    value = std::strtod(str.c_str(), &endptr);
    return endptr != str.c_str() && *endptr == '\0';
}

void CSVParserWindow::set_filter_param(int filter_window_size)
{
    filter_window_size_ = filter_window_size;
}

void CSVParserWindow::save_data_to_file(const std::string &filename)
{
    std::cout << __FUNCTION__ << " filename: " << filename << std::endl;
    std::ofstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    file << "x,y,z,A,B,C\n";
    for (size_t i = 0; i < x.size(); ++i)
    {
        file << x[i] << ","
            << y[i] << ","
            << z[i] << ","
            << a[i] << ","
            << b[i] << ","
            << c[i] << "\n";
    }
}
