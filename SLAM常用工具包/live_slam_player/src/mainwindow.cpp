#include "mainwindow.h"
#include "ui_mainwindow.h"

using namespace std;

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui_(new Ui::MainWindow)
{
  my_ros_ = new ROSThread(this, &mutex);
  ui_->setupUi(this);
  my_ros_->start();

  slider_checker_ = false;
  play_flag_ = false;
  pause_flag_ = false;
  loop_flag_ = false;
  stop_skip_flag_ = true;

  connect(my_ros_, SIGNAL(StampShow(quint64)), this, SLOT(SetStamp(quint64)));
  connect(my_ros_, SIGNAL(StartSignal()), this, SLOT(Play()));

  connect(ui_->quitButton, SIGNAL(pressed()), this, SLOT(TryClose()));
  connect(ui_->pushButton, SIGNAL(pressed()), this, SLOT(FilePathSet()));
  connect(ui_->pushButton_2, SIGNAL(pressed()), this, SLOT(Play()));
  connect(ui_->pushButton_3, SIGNAL(pressed()), this, SLOT(Pause()));

  connect(ui_->doubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(PlaySpeedChange(double)));
  ui_->doubleSpinBox->setRange(0.01,20.0);
  ui_->doubleSpinBox->setValue(1.0);
  ui_->doubleSpinBox->setSingleStep(0.01);
  connect(ui_->checkBox, SIGNAL(stateChanged(int)), this, SLOT (LoopFlagChange(int)));
  if(loop_flag_ == true){
    ui_->checkBox->setCheckState(Qt::Checked);
  }else{
    ui_->checkBox->setCheckState(Qt::Unchecked);
  }
  connect(ui_->checkBox_2, SIGNAL(stateChanged(int)), this, SLOT (StopSkipFlagChange(int)));
  if(stop_skip_flag_ == true){
    ui_->checkBox_2->setCheckState(Qt::Checked);
  }else{
    ui_->checkBox_2->setCheckState(Qt::Unchecked);
  }
  connect(ui_->checkBox_3, SIGNAL(stateChanged(int)), this, SLOT (AutoStartFlagChange(int)));
  if(my_ros_->auto_start_flag_ == true){
    ui_->checkBox_3->setCheckState(Qt::Checked);
  }else{
    ui_->checkBox_3->setCheckState(Qt::Unchecked);
  }

  connect(ui_->horizontalSlider, SIGNAL(sliderPressed()), this, SLOT(SliderPressed()));
  connect(ui_->horizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(SliderValueChange(int)));
  connect(ui_->horizontalSlider, SIGNAL(sliderReleased()), this, SLOT(SliderValueApply()));

  ui_->horizontalSlider->setRange(0,10000);
  ui_->horizontalSlider->setValue(0);
  slider_value_ = 0;

}

MainWindow::~MainWindow()
{
  emit setThreadFinished(true); //Tell the thread to finish
  delete ui_;
  my_ros_->quit();
  if(!my_ros_->wait(500)) //Wait until it actually has terminated (max. 3 sec)
  {
      my_ros_->terminate(); //Thread didn't exit in time, probably deadlocked, terminate it!
      my_ros_->wait(); //We have to wait again here!
  }
}


void MainWindow::RosInit(ros::NodeHandle &n)
{
  my_ros_->ros_initialize(n);
}


void MainWindow::TryClose()
{
  close();
}


void MainWindow::FilePathSet()
{
  play_flag_ = false;
  my_ros_->play_flag_ = false;
  this->ui_->pushButton_2->setText(QString::fromStdString("Play"));

  pause_flag_ = false;
  my_ros_->pause_flag_ = false;
  this->ui_->pushButton_3->setText(QString::fromStdString("Pause"));

  QFileDialog dialog;
  this->ui_->label->setText("Data is beging loaded.....");
  data_folder_path_ = dialog.getExistingDirectory();
  my_ros_->data_folder_path_ = data_folder_path_.toUtf8().constData();

  my_ros_->Ready();

  this->ui_->label->setText(data_folder_path_);

}

void MainWindow::SetStamp(quint64 stamp)
{
  this->ui_->label_2->setText(QString::number(stamp));

  //set slide bar
  if(slider_checker_ == false){
    ui_->horizontalSlider->setValue(static_cast<int>(static_cast<float>(stamp - my_ros_->initial_data_stamp_)/static_cast<float>(my_ros_->last_data_stamp_ - my_ros_->initial_data_stamp_)*10000));
  }
}

void MainWindow::Play()
{
  if(my_ros_->play_flag_ == false){
    play_flag_ = true;
    my_ros_->play_flag_ = true;
    this->ui_->pushButton_2->setText(QString::fromStdString("End"));

    pause_flag_ = false;
    my_ros_->pause_flag_ = false;
    this->ui_->pushButton_3->setText(QString::fromStdString("Pause"));

  }else{
    play_flag_ = false;
    my_ros_->play_flag_ = false;
    this->ui_->pushButton_2->setText(QString::fromStdString("Play"));
  }
}

void MainWindow::Pause()
{
  if(pause_flag_ == false){
    pause_flag_ = true;
    my_ros_->pause_flag_ = true;
    this->ui_->pushButton_3->setText(QString::fromStdString("Resume"));
  }else{
    pause_flag_ = false;
    my_ros_->pause_flag_ = false;
    this->ui_->pushButton_3->setText(QString::fromStdString("Pause"));
  }
}


void MainWindow::PlaySpeedChange(double value)
{
  my_ros_->play_rate_ = value;
}

void MainWindow::LoopFlagChange(int value)
{
  if(value == 2){
    loop_flag_ = true;
    my_ros_->loop_flag_ = true;
  }else if(value == 0){
    loop_flag_ = false;
    my_ros_->loop_flag_ = false;
  }
}
void MainWindow::StopSkipFlagChange(int value)
{
  if(value == 2){
    stop_skip_flag_ = true;
    my_ros_->stop_skip_flag_ = true;
  }else if(value == 0){
    stop_skip_flag_ = false;
    my_ros_->stop_skip_flag_ = false;
  }
}
void MainWindow::AutoStartFlagChange(int value)
{
  if(value == 2){
    my_ros_->auto_start_flag_ = true;
  }else if(value == 0){
    my_ros_->auto_start_flag_ = false;
  }
}
void MainWindow::SliderValueChange(int value)
{
  slider_value_ = value;
}

void MainWindow::SliderPressed()
{
  slider_checker_ = true;
}

void MainWindow::SliderValueApply()
{
  my_ros_->ResetProcessStamp(slider_value_);
  slider_checker_ = false;
}
