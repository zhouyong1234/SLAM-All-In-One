# mmdetection
## 教程
* [博客链接](https://blog.csdn.net/qq_16137569/article/details/121316235)
* [官网链接](https://github.com/open-mmlab/mmdetection)
* [官方文档](https://mmdetection.readthedocs.io/en/latest/)
## 测试程序用时

```bash
python -m torch.distributed.launch --nproc_per_node=1 --master_port=29500 tools/analysis_tools/benchmark.py $cofig $checkpoint --launcher pytorch
```
## 分布式训练
```python
Export CUDA_VISIBLE_DEVICES=”0,1,2,3”

CUDA_VISIBLE_DEVICES=”0,1,2,3” python -m torch.distributed.launch --nproc_per_node=4 tools/train.py $cofig $checkpoint --launcher pytorch

```

## 训练
```python
Export CUDA_VISIBLE_DEVICES=2

CUDA_VISIBLE_DEVICES=2 python tools/train.py $cofig $checkpoint --gpus 2

```
## 测算模型参数量和计算量
```
python tools/analysis_tools/get_flops.py ${CONFIG_FILE} [--shape ${INPUT_SHAPE}]

```
## 指定GPU运行
```
export CUDA_VISIBLE_DEVICES=1 

CUDA_VISIBLE_DEVICE=1 python xxx.py …

```

## Git
```python
git init (初始化本地仓库)
git remote add origin git@github.com:open-mmlab/mmdetection.git
git add .  (提交缓存)
git status
git commit -m “   备注   ”

git pull --rebase origin master (同步)
or   git pull + git pull origin master --allow-unrelated-histories 【允许不相关历史提交，并强制合并】
git push -u origin master （推送）

```
* 冲突

```python
git add . 

git rebase --continue

git pull --rebase origin master 

git push origin master

```
