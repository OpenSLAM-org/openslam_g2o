#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include "ui_base_main_window.h"

class MainWindow : public QMainWindow, public Ui::BaseMainWindow
{
  Q_OBJECT
  public:
    MainWindow(QWidget * parent = 0, Qt::WindowFlags flags = 0);
    ~MainWindow();

  public slots:
    void on_actionLoad_triggered(bool);
    void on_actionSave_triggered(bool);
    void on_actionQuit_triggered(bool);
    void on_btnOptimize_clicked();
    void on_btnInitialGuess_clicked();

  protected:
    void fixGraph();

};

#endif
