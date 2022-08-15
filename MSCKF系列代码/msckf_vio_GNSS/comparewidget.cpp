#include "comparewidget.h"
#include "ui_comparewidget.h"
#include <iomanip>
#include <iostream>
#include <QMessageBox>
using namespace std;
using namespace P_Struct;
void pose_estimation_3d3d (
    const QVector<cv::Mat>& pts1,
    const QVector<cv::Mat>& pts2,
    cv::Mat& R, cv::Mat& t
);
QVector<QPair<int,int>> GetMatchingTime (
    const QVector<cv::Mat>& pts1,
    const QVector<cv::Mat>& pts2);

CompareWidget::CompareWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::CompareWidget)
{
    this->setWindowIcon(QIcon(":/Resources/icon4.png"));

    ui->setupUi(this);
    ogw=new OGLWidget_C(this);
    chartView=new QChartView();

    QGridLayout *layout=new QGridLayout();
    layout->setMargin(0);
    layout->addWidget(ogw);
    ui->frame->setLayout(layout);
    ogw->show();

    QGridLayout *layout_2=new QGridLayout();
    layout_2->setMargin(7);
    layout_2->addWidget(chartView);
    ui->groupBox_3->setLayout(layout_2);
    chartView->show();

    connect(ui->pushButton_open1,SIGNAL(clicked(bool)),this,SLOT(OpenTJ_Ref()));
    connect(ui->pushButton_open2,SIGNAL(clicked(bool)),this,SLOT(OpenTJ()));
    connect(ui->pushButton_load1,SIGNAL(clicked(bool)),this,SLOT(LoadTJ_Ref()));
    connect(ui->pushButton_load2,SIGNAL(clicked(bool)),this,SLOT(LoadTJ()));
    connect(ui->pushButton_overlook,SIGNAL(toggled(bool)),ogw,SLOT(Change_Overlook(bool)));
    connect(ui->pushButton_adjust,SIGNAL(clicked(bool)),this,SLOT(AdjustTrajectory()));
    connect(ui->pushButton_translate,SIGNAL(clicked(bool)),this,SLOT(TranslateTrajectory()));


    ui->checkBox->setChecked(true);
    ui->checkBox_2->setChecked(true);
}

CompareWidget::~CompareWidget()
{
    delete ui;
}
QVector<cv::Mat> CompareWidget::ReadTrajetoryFromFile(std::string filename,int index1,int index2,int index3,int index4,int mode,cv::Mat &pos0,bool is_inv,bool is_only_xy)
{
    std::fstream fs(filename);
    std::string buffer;
    std::stringstream ss;
    QVector<cv::Mat> vec;
    double x0,y0,z0;
    bool ref=false;
    while(fs.good())
    {
        cv::Mat m;
        std::getline(fs,buffer);
        if(buffer[0]=='#'||buffer.length()<2) continue;
        for(int i=0;i<buffer.size();i++)
        {
            if(buffer[i]==',') buffer[i]=' ';
        }
        ss.str("");
        ss.clear();
        ss<<buffer;
        double x,y,z,t=0,temp;
        for(int i=0;ss.good();i++)
        {
            ss>>temp;
            if(i==index1) x=temp;
            if(i==index2) y=temp;
            if(i==index3) z=temp;
            if(i==index4)
            {   if(temp>1e18) temp=temp/1e9;
                t=temp;
            }
        }
        if(!ref)
        {
            x0=x;y0=y;z0=z;
            ref=true;
            if(mode==1)
            {
                P_Struct::Cartesian cart=CoordTrans::Geod2Cart({x0,y0,z0});
                pos0=(cv::Mat_<double>(4,1) << cart.X,cart.Y,cart.Z,t);
            }
            if(mode==2)
            {
            pos0=(cv::Mat_<double>(4,1) << x0,y0,z0,t);
            }
        }
        if(mode==0)//NEU（x,y,z）的情形
        {

            m = (cv::Mat_<double>(4,1) << x-x0,y-y0,z-z0,t);
            if(is_only_xy) m.at<double>(2)=0;
        }
        else if(mode==1) //BLH的情形
        {
            P_Struct::Cartesian X0=CoordTrans::Geod2Cart({x0,y0,z0});
            P_Struct::Cartesian X=CoordTrans::Geod2Cart({x,y,z});
            P_Struct::Topocentric T=CoordTrans::Cart2Topo(X0,X);
            m = (cv::Mat_<double>(4,1) << T.N,T.E,T.U,t);

        }
        else if(mode==2) //XYZ的情形
        {
            P_Struct::Topocentric T=CoordTrans::Cart2Topo({x0,y0,z0},{x,y,z});
            m = (cv::Mat_<double>(4,1) << T.N,T.E,T.U,t);
        }
        if(is_inv) m=-m;
        vec.push_back(m);

    }
    fs.close();
    return vec;
}

void pose_estimation_3d3d (
    const QVector<cv::Mat>& pts1,
    const QVector<cv::Mat>& pts2,
    cv::Mat& R, cv::Mat& t
)
{
        cv::Mat p1=(cv::Mat_<double>(3,1) << 0,0,0);
        cv::Mat p2=(cv::Mat_<double>(3,1) << 0,0,0);     // center of mass
        int N = pts1.size();
        for ( int i=0; i<N; i++ )
        {
            p1 += pts1[i].rowRange(0,3);
            p2 += pts2[i].rowRange(0,3);
        }
        p1 = p1*(1.0/N);
        p2 = p2*(1.0/N);
        QVector<cv::Mat>     q1 ( N ), q2 ( N ); // remove the center
        for ( int i=0; i<N; i++ )
        {
            q1[i] = pts1[i].rowRange(0,3) - p1;
            q2[i] = pts2[i].rowRange(0,3) - p2;
        }


        cv::Mat W = cv::Mat::zeros(3,3,CV_64F);
            for ( int i=0; i<N; i++ )
            {

                W += q1[i] * (q2[i].t());
            }
        cv::Mat S,U,VT;
        cv::SVD::compute(W,S,U,VT);
        R=U*VT;
        t=p1-R*p2;
}

QVector<QPair<int,int>> GetMatchingTime (
    const QVector<cv::Mat>& pts1,
    const QVector<cv::Mat>& pts2)
{
    QVector<QPair<int,int>> vec;
    int i=0,j=0;
    double dt=1000000000;
    while(i<pts1.size()&&j<pts2.size())
    {
//        cerr<<setprecision(20)<<pts1[i].at<double>(3)<<" "
//           <<setprecision(20)<<pts2[j].at<double>(3)<<endl;
       if(pts1[i].at<double>(3)<pts2[j].at<double>(3))
       {
          if(pts2[j].at<double>(3)-pts1[i].at<double>(3)<dt+0.00001)
          {
              dt=pts2[j].at<double>(3)-pts1[i].at<double>(3);
              i++;
              continue;
          }
          else
          {
              vec.push_back(QPair<int,int>(i,j-1));
              //cerr<<dt<<endl;
              //cerr<<i<<"/"<<pts1.size()<<" "<<j-1<<"/"<<pts2.size()<<endl;
              i++;
              dt=1000000000;
              continue;
          }
       }
       else
       {
           if(pts1[i].at<double>(3)-pts2[j].at<double>(3)<dt+0.00001)
           {
               dt=pts1[i].at<double>(3)-pts2[j].at<double>(3);
               j++;
               continue;
           }
           else
           {
               vec.push_back(QPair<int,int>(i-1,j));
              // cerr<<dt<<endl;
              // cerr<<i-1<<"/"<<pts1.size()<<" "<<j<<"/"<<pts2.size()<<endl;
               j++;
               dt=1000000;
               continue;
           }
       }
    }
   // cerr<<"!!!"<<endl;
    return vec;
}

void CompareWidget::OpenTJ_Ref()
{
    QString filename=QFileDialog::getOpenFileName();
    ui->lineEdit->setText(filename);
}

void CompareWidget::OpenTJ()
{
    QString filename=QFileDialog::getOpenFileName();
    ui->lineEdit_2->setText(filename);
}
void CompareWidget::LoadTJ_Ref()
{
    QString filename=ui->lineEdit->text();
    int mode=ui->comboBox->currentIndex();
    m_mode_Ref=mode;
    int index1=ui->lineEdit_id1->text().toInt();
    int index2=ui->lineEdit_id2->text().toInt();
    int index3=ui->lineEdit_id3->text().toInt();
    int index4=ui->lineEdit_id4->text().toInt();
    bool is_only_xy=ui->checkBox->isChecked();
    m_TJ_Ref=ReadTrajetoryFromFile(filename.toLocal8Bit().toStdString(),index1,index2,index3,index4,mode,m_pos0_Ref,false,is_only_xy);
    if(m_TJ_Ref.size()>0)
    {
    ui->label_7->setText(QString::number(m_TJ_Ref.size()));
    ui->label_8->setText(QString::number(m_TJ_Ref[0].at<double>(3),'g',15));
    ui->label_9->setText(QString::number(m_TJ_Ref[m_TJ_Ref.size()-1].at<double>(3),'g',15));
    }
    ogw->SetTJ_Ref(m_TJ_Ref);
}

void CompareWidget::LoadTJ()
{
    QString filename=ui->lineEdit_2->text();
    int mode=ui->comboBox_2->currentIndex();
    m_mode=mode;
    int index1=ui->lineEdit_id5->text().toInt();
    int index2=ui->lineEdit_id6->text().toInt();
    int index3=ui->lineEdit_id7->text().toInt();
    int index4=ui->lineEdit_id8->text().toInt();
    double time_diff=ui->lineEdit_timediff->text().toDouble();
    bool is_only_xy=ui->checkBox_2->isChecked();
    m_TJ=ReadTrajetoryFromFile(filename.toLocal8Bit().toStdString(),index1,index2,index3,index4,mode,m_pos0,false,is_only_xy);
    for(int i=0;i<m_TJ.size();i++)
    {
        m_TJ[i].at<double>(3)+=time_diff;
    }
    if(m_TJ.size()>0)
    {
    ui->label_10->setText(QString::number(m_TJ.size()));
    ui->label_11->setText(QString::number(m_TJ[0].at<double>(3),'g',15));
    ui->label_12->setText(QString::number(m_TJ[m_TJ.size()-1].at<double>(3),'g',15));
    }
    ogw->SetTJ(m_TJ);
//    for(int i=0;i<4;i++)
//    {
//        cerr<<m_TJ[i].at<double>(0)<<endl;
//    }
}

void CompareWidget::AdjustTrajectory()
{
    int ratio;
    ratio=ui->horizontalSlider->value();
  //  cerr<<"ratio:"<<ratio<<endl;
        QVector<QPair<int,int>> v=GetMatchingTime(m_TJ,m_TJ_Ref);
//        fstream of("E:/log.txt",ios::out);
//        for(int i=0;i<v.size();i++)
//        {
//            cerr<<v[i].first<<" "<<v[i].second<<endl;
//            of<<setprecision(20)<<m_TJ[v[i].first].at<double>(3)<<" "<<setprecision(15)<<m_TJ_Ref[v[i].second].at<double>(3)<<endl;
//        }
//        of.close();
        if(v.size()>10)
        {
            QVector<cv::Mat> vec1;
            QVector<cv::Mat> vec2;
            //cerr<<v.size()<<endl;
            int count=0,count_Ref=0;
            for(int i=0;i<(int)v.size()*(ratio/100.0);i++)
            {
                vec1.push_back(m_TJ[v[i].first]);
                vec2.push_back(m_TJ_Ref[v[i].second]);
                count=v[i].first;
                count_Ref=v[i].second;
            }
            ogw->SetHighlight(0,count);
            ogw->SetHighlight_Ref(0,count_Ref);
            cv::Mat R,t;
            pose_estimation_3d3d (vec2,vec1,R,t);
            //cerr<<R<<endl<<t<<endl;

            for(int i=0;i<m_TJ.size();i++)
            {
                m_TJ[i].rowRange(0,3)=R*m_TJ[i].rowRange(0,3)+t;
            }
        }
        ogw->SetTJ(m_TJ);

        QLineSeries *series=new QLineSeries();
        for(int i=0;i<v.size();i++)
        {
            double x1=m_TJ[v[i].first].at<double>(0);
            double y1=m_TJ[v[i].first].at<double>(1);
            double z1=m_TJ[v[i].first].at<double>(2);
            double x2=m_TJ_Ref[v[i].second].at<double>(0);
            double y2=m_TJ_Ref[v[i].second].at<double>(1);
            double z2=m_TJ_Ref[v[i].second].at<double>(2);
            double t=m_TJ[v[i].first].at<double>(3);
            double dx=x1-x2,dy=y1-y2,dz=z1-z2;
            double dist=std::sqrt(dx*dx+dy*dy+dz*dz);
            series->append(t,dist);
        }
        QChart *chart=new QChart();
        chart->addSeries(series);
        chart->createDefaultAxes();
        chart->legend()->hide();
        chart->setTitle("Postion Difference");
        chartView->setChart(chart);
}

void CompareWidget::TranslateTrajectory()
{

        QVector<QPair<int,int>> v=GetMatchingTime(m_TJ,m_TJ_Ref);

        if(m_mode==-1||m_mode_Ref==-1)
        {
            QMessageBox::information(this,"Failed","No data.");
            return;
        }
        if(m_mode==0||m_mode_Ref==0)
        {
            QMessageBox::information(this,"Failed","'Compare(BLH/XYZ)' needs both the trajectories to be in the absolute coordinate system.");
            return;
        }
        double x0_Ref=m_pos0_Ref.at<double>(0);
        double y0_Ref=m_pos0_Ref.at<double>(1);
        double z0_Ref=m_pos0_Ref.at<double>(2);
        for(int i=0;i<m_TJ.size();i++)
        {
            double x0=m_pos0.at<double>(0);
            double y0=m_pos0.at<double>(1);
            double z0=m_pos0.at<double>(2);
            double x=m_TJ[i].at<double>(0);
            double y=m_TJ[i].at<double>(1);
            double z=m_TJ[i].at<double>(2);
                Cartesian cart= CoordTrans::Topo2Cart({x0,y0,z0},{x,y,z});
                x=cart.X;
                y=cart.Y;
                z=cart.Z;

            P_Struct::Topocentric topo=CoordTrans::Cart2Topo({x0_Ref,y0_Ref,z0_Ref},{x,y,z});
            m_TJ[i].at<double>(0)=topo.N;
            m_TJ[i].at<double>(1)=topo.E;
            m_TJ[i].at<double>(2)=topo.U;
        }
        m_pos0=m_pos0_Ref;

        ogw->SetTJ(m_TJ);
        QLineSeries *series=new QLineSeries();
        for(int i=0;i<v.size();i++)
        {
            double x1=m_TJ[v[i].first].at<double>(0);
            double y1=m_TJ[v[i].first].at<double>(1);
            double z1=m_TJ[v[i].first].at<double>(2);
            double x2=m_TJ_Ref[v[i].second].at<double>(0);
            double y2=m_TJ_Ref[v[i].second].at<double>(1);
            double z2=m_TJ_Ref[v[i].second].at<double>(2);
            double t=m_TJ[v[i].first].at<double>(3);
            double dx=x1-x2,dy=y1-y2,dz=z1-z2;
            double dist=std::sqrt(dx*dx+dy*dy+dz*dz);
            series->append(t,dist);
        }
        QChart *chart=new QChart();
        chart->addSeries(series);
        chart->createDefaultAxes();
        chart->legend()->hide();
        chart->setTitle("Postion Difference");
        chartView->setChart(chart);
}
