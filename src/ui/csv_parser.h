#pragma once

#include <QApplication>
#include <QMainWindow>
#include <QVBoxLayout>
#include <QFile>
#include <QTextStream>
#include <vector>
#include <string>
#include "qcustomplot.h"
#include "common_header.h"

class CSVParserWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit CSVParserWindow(QWidget *parent = nullptr);
    ~CSVParserWindow();
    
    void loadData(const QString &filename);
    void loadData(const std::string &filename, bool is_filtering);
    void plotData();

private:
    bool parse_double(const std::string &str, double &value);

    inline QVector<double> toQVector(const std::vector<double> &vec)
    {
        return QVector<double>::fromStdVector(vec);
    }

private:
    QCustomPlot *customPlot;
    std::vector<double> x, y, z;
    std::vector<double> a, b, c;
};