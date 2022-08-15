# python 相关笔记
## 环境配置
```bash
conda create -n rich python=3.7  #创建环境
conda activate rich  #激活环境
conda deactivate rich  #退出环境
conda remove -n rich --all  #删除环境
```
## requirements
```
pip freeze > requirements.txt
```
例如
```
pytest==6.2.5
pytest-json-report==1.4.1
pytest-metadata==1.11.0
pytest-ordering==0.6
PyTestReport==0.2.1
python-dateutil==2.8.2
```
* [解决安装缓慢的问题](https://blog.csdn.net/weixin_42455006/article/details/121957633?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522164964070016780271982907%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=164964070016780271982907&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_ecpm_v1~rank_v31_ecpm-3-121957633.142^v7^article_score_rank,157^v4^control&utm_term=%E5%AE%89%E8%A3%85requirements.txt%E5%A4%AA%E6%85%A2&spm=1018.2226.3001.4187)

```bash
pip install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple/ --trusted-host pypi.douban.com
```
# [mmdetetion](https://github.com/liuqian62/notebook/blob/main/python%E7%9B%B8%E5%85%B3/mmdetection.md)
# OpenCV教程
* [OpenCV中文官方文档](https://woshicver.com/)
* [OpenCV 4.0 中文文档](https://opencv.apachecn.org/#/)

# pyechart
* [pyechart中文文档](https://gallery.pyecharts.org/#/README)
* [pyechart的github地址](https://github.com/pyecharts/pyecharts)
* [pyechart图库](https://github.com/pyecharts/pyecharts-gallery)




