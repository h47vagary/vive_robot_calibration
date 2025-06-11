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
#include "filter.h"

class CSVParserWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit CSVParserWindow(QWidget *parent = nullptr);
    ~CSVParserWindow();
    
    void loadData(const QString &filename);
    void loadData(const std::string &filename, bool is_filtering);
    void set_filter_param(int filter_window_size);
    void save_data_to_file(const std::string &filename);
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
    int filter_window_size_ = 11;
};