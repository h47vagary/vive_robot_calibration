#pragma once

#include <QApplication>
#include <QMainWindow>
#include <QVBoxLayout>
#include <QFile>
#include <QTextStream>
#include "qcustomplot.h"

class CSVParserWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit CSVParserWindow(QWidget *parent = nullptr);
    ~CSVParserWindow();
    
    void loadData(const QString &filename);
    void plotData();

private:
    QCustomPlot *customPlot;
    QVector<double> x, y, z;
    QVector<double> a, b, c;
};