#include "csv_parser.h"
#include <iostream>

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
    customPlot->graph(0)->setData(indices, x);
    customPlot->graph(0)->setName("X");
    customPlot->graph(0)->setPen(QPen(Qt::blue));

    // Y 坐标图
    customPlot->addGraph();
    customPlot->graph(1)->setData(indices, y);
    customPlot->graph(1)->setName("Y");
    customPlot->graph(1)->setPen(QPen(Qt::red));

    // Z 坐标图
    customPlot->addGraph();
    customPlot->graph(2)->setData(indices, z);
    customPlot->graph(2)->setName("Z");
    customPlot->graph(2)->setPen(QPen(Qt::green));

    // A 角度图
    customPlot->addGraph();
    customPlot->graph(3)->setData(indices, a);
    customPlot->graph(3)->setName("A");
    customPlot->graph(3)->setPen(QPen(Qt::magenta));

    // B 角度图
    customPlot->addGraph();
    customPlot->graph(4)->setData(indices, b);
    customPlot->graph(4)->setName("B");
    customPlot->graph(4)->setPen(QPen(Qt::cyan));

    // C 角度图
    customPlot->addGraph();
    customPlot->graph(5)->setData(indices, c);
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
            x.append(fields[0].toDouble());
            y.append(fields[1].toDouble());
            z.append(fields[2].toDouble());
            a.append(fields[3].toDouble());
            b.append(fields[4].toDouble());
            c.append(fields[5].toDouble());
        }
    }
    
    file.close();
}
