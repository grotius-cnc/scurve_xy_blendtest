#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include <QTimer>
#include <QKeyEvent>
#include <QColorDialog>
#include <QDebug>
#include <QMessageBox>
#include <iostream>

#ifdef Success
#undef Success
#endif

#include "pathlib.h"

//! Draw lib.
#include "draw_primitives.h"

//! Opencascade.
#include "OcctQtViewer.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    OcctQtViewer *occ;
    pathlib *plib=new pathlib();

};
#endif // MAINWINDOW_H
