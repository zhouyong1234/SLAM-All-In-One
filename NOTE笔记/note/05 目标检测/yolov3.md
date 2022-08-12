

版权声明：本文为CSDN博主「江大白*」的原创文章
原文链接https://blog.csdn.net/nan355655600/article/details/106246625
[参考链接](https://blog.csdn.net/nan355655600/article/details/106246625?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522164187022216780274122966%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=164187022216780274122966&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~top_click~default-9-106246625.pc_search_result_cache&utm_term=yolo&spm=1018.2226.3001.4187)

**Yolov3**是**2018年**发明提出的，这成为了目标检测**one-stage**中非常经典的算法，包含**Darknet-53网络结构**、**anchor锚框**、**FPN**等非常优秀的结构。

# 1 yolov3 网络结构				
![[Pasted image 20220111111036.png]]

Darknetconv2d:
                             CBL:	Conv2d + BatchNorm2d + Leaky relu

Residual：        Conv2d 1 * 1  ->Conv2d 3 * 3   
					    ---------------------------------    +    add  -->
						
上图三个蓝色方框内表示Yolov3的三个基本组件：
## 1.1 CBL
Yolov3网络结构中的最小组件，由Conv+Bn+Leaky_relu激活函数三者组成。
## 1.2 Res unit
借鉴Resnet网络中的残差结构，让网络可以构建的更深。
## 1.3 ResX
由一个CBL和X个残差组件构成，是Yolov3中的大组件。每个Res模块前面的CBL都起到下采样的作用，因此经过5次Res模块后，得到的特征图是608->304->152->76->38->19大小。

## 1.4 其他基础操作
（1）Concat：张量拼接，会扩充两个张量的维度，例如26×26×256和26×26×512两个张量拼接，结果是26×26×768。Concat和cfg文件中的route功能一样。
（2）Add：张量相加，张量直接相加，不会扩充维度，例如104×104×128和104×104×128相加，结果还是104×104×128。add和cfg文件中的shortcut功能一样。

## 1.5 Backbone中卷积层的数量
每个ResX中包含1+2×X个卷积层，因此整个主干网络Backbone中一共包含1+（1+2×1）+（1+2×2）+（1+2×8）+（1+2×8）+（1+2×4）=52，再加上一个FC全连接层，即可以组成一个Darknet53分类网络。不过在目标检测Yolov3中，去掉FC层，不过为了方便称呼，仍然把Yolov3的主干网络叫做Darknet53结构。

# yolov4 网络结构
![[Pasted image 20220111112110.png]]

Yolov4的结构图和Yolov3相比，因为多了CSP结构，PAN结构，如果单纯看可视化流程图，会觉得很绕，但是在绘制出上面的图形后，会觉得豁然开朗，其实整体架构和Yolov3是相同的，不过使用各种新的算法思想对各个子结构都进行了改进。
先整理下Yolov4的五个基本组件：
1. CBM：Yolov4网络结构中的最小组件，由Conv+Bn+Mish激活函数三者组成。
2. CBL：由Conv+Bn+Leaky_relu激活函数三者组成。
3. Res unit：借鉴Resnet网络中的残差结构，让网络可以构建的更深。
4. CSPX：借鉴CSPNet网络结构，由卷积层和X个Res unint模块Concat组成。
5. SPP：采用1×1，5×5，9×9，13×13的最大池化的方式，进行多尺度融合。

其他基础操作：
1. Concat：张量拼接，维度会扩充，和Yolov3中的解释一样，对应于cfg文件中的route操作。
2. Add：张量相加，不会扩充维度，对应于cfg文件中的shortcut操作。

Backbone中卷积层的数量：
和Yolov3一样，再来数一下Backbone里面的卷积层数量。
每个CSPX中包含5+2×X个卷积层，因此整个主干网络Backbone中一共包含1+（5+2×1）+（5+2×2）+（5+2×8）+（5+2×8）+（5+2×4）=72。
————————————————
## 2.1 创新
（1）输入端：这里指的创新主要是训练时对输入端的改进，主要包括**Mosaic数据增强**、cmBN、SAT自对抗训练。
（2）BackBone主干网络：将各种新的方式结合起来，包括：CSPDarknet53、**Mish激活函数**、Dropblock
（3）Neck：目标检测网络在BackBone和最后的输出层之间往往会插入一些层，比如Yolov4中的**SPP模块（多尺度融合）**、**FPN+PAN结构（特征金字塔）**
（4）Prediction：输出层的锚框机制和Yolov3相同，主要改进的是训练时的损失函数CIOU_Loss，以及预测框筛选的nms变为**DIOU_nms**

**FPN**是自顶向下的，将高层的特征信息通过**上采样**的方式进行传递融合，得到进行预测的特征图。
