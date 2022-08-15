# Visual-Based-Localization-Papers

The camera re-localization task aims to estimate the 6-DoF pose of a novel (unseen) frame in the coordinate system given by the prior model of the world.  The most related academic topics are SLAM and SfM and it's widely applied in AR, Robotic, etc.

Feel free to make a PR or contribute. :smile:

---

### Table of Contents
- <a href="#FeatureExtration"> survey</a>
- <a href="#system">system</a>
- <a href="#DirectMethod">direct methods</a>
- <a href="#FeatureExtration">feature extraction</a>
- <a href="#FeatureMatch">feature match</a>
- <a href="#Retrieval">retrieval</a>
- <a href="#Pose">robust pose estimation</a>
- <a href="#Fusion">multi-sensors fusion</a>
- <a href="#SLAM ">slam</a>
- <a href="#SfM ">sfm</a>
- <a href="#Wait ">waiting to sort</a>


---

<h1 id="Surveys">Survey</h1>

- **[Image-based camera localization: an overview]** Yihong Wu. Visual Computing for Industry, Biomedicine, and Art, 2018. [**[paper]**](https://arxiv.org/abs/1610.03660)


<h1 id="system">System</h1>

- **[Wide area localization on mobile phones]** Clemens Arth. ISMAR, 2009. [**[paper]**](https://www.researchgate.net/publication/221221483_Wide_Area_Localization_on_Mobile_Phones)

- **[Parallel Tracking and Mapping on a Camera Phone]** ISMAR, 2009. [**[paper]**](https://ieeexplore.ieee.org/document/5336495)

- **[Real-time self-localization from panoramic images on mobile devices]** Clemens Arth. ISMAR, 2011. [**[paper]**](https://ieeexplore.ieee.org/document/6162870)

- **[Scalable 6-DOF Localization on Mobile Devices]** Iven Middelberg, Torsten Sattler. ECCV, 2014. [**[paper]**](https://www.graphics.rwth-aachen.de/media/**paper**s/ECCV14_preprint.pdf)

- **[6D dynamic camera relocalization from single reference image]** Feng W. CVPR 2016. [**[paper]**](http://openaccess.thecvf.com/content_cvpr_2016/**paper**s/Feng_6D_Dynamic_Camera_CVPR_2016_**paper**.pdf)


- **[Image Matching Across Wide Baselines: From Paper to Practice]** Yuehe, Jin. CVPR, 2020. [**[paper]**](https://arxiv.org/abs/2003.01587) [**[code]**](https://github.com/ubc-vision/image-matching-benchmark) 


- **[GN-Net: The Gauss-Newton Loss for Multi-Weather Relocaliza-tion]** L. von Stumberg, P. Wenzel, Q. Khan, and D. Cremers. ICRA, 2020. [**[paper]**](https://arxiv.org/abs/1904.11932) [**[code]**](https://github.com/Artisense-ai/GN-Net-Benchmark)

- **[Using Image Sequences for Long-Term Visual Localization]** Erik Stenborg, Torsten Sattler and Lars Hammarstrand. 3DV, 2020. [**[paper]**](https://ieeexplore.ieee.org/document/9320360) [**[code]**](https://github.com/rulllars/SequentialVisualLocalization)

- **[LM-Reloc: Levenberg-Marquardt Based Direct Visual Relocalization]** Lukas von Stumberg, Patrick Wenzel, Nan Yang, Daniel Cremers. 3DV, 2020. [**[paper]**](https://arxiv.org/abs/2010.06323)

- **[Efficient 2D-3D Matching for Multi-Camera Visual Localization]** Marcel Geppert, Peidong Liu, Zhaopeng Cui, Marc Pollefeys, Torsten Sattler. ICRL, 2020. [**[paper]**](https://arxiv.org/abs/1809.06445)

- **[KFNet: Learning Temporal Camera Relocalization using Kalman Filtering]** Lei Zhou, Zixin Luo, Tianwei Shen... CVPR, 2020 [**[paper]**](https://arxiv.org/pdf/2003.10629.pdf)

- **[Robust Neural Routing Through Space Partitions for Camera Relocalizationin Dynamic Indoor Environments]** Siyan Dong, Qingnan Fan... CVPR, 2021, oral [**[paper]**](https://arxiv.org/pdf/2012.04746.pdf)

- **[CrossLoc: Scalable Aerial Localization Assisted by Multimodal Synthetic Data]** Qi Yan, Jianhao Zheng, ... CVPR, 2022. [**[paper]**](https://arxiv.org/abs/2112.09081) [**[code]**](https://github.com/TOPO-EPFL/CrossLoc) [**[dataset]**](https://doi.org/10.5061/dryad.mgqnk991c) [**[video]**](https://youtu.be/pytRRXPFqFE) [**[website]**](https://crossloc.github.io/) 

<h1 id="DirectMethod">Direct Method</h1>

- **[PoseNet: A Convolutional Network for Real-Time 6-DOF Camera Relocalization]** A. Kendall, M. ICCV, 2015. [**[code]**](http://mi.eng.cam.ac.uk/projects/relocalisation/)[**[paper]**](https://arxiv.org/pdf/1505.07427.pdf)

<h1 id="FeatureExtration">Feature Extracting</h1>

- **[Semantic Visual Localization]** J. L. Schonberger. CVPR, 2018. [**[paper]**](https://arxiv.org/abs/1712.05773)

- **[R2D2: Repeatable and Reliable Detector and Descriptor]** Jerome Revaud. NeurIPS, 2019. [**[paper]**](https://arxiv.org/pdf/1906.06195.pdf)[**[code]**](https://github.com/naver/r2d2)

- **[Learning Feature Descriptors using Camera Pose Supervision]** Qianqian Wang. ECCV, 2020, oral. [**[paper]**](https://arxiv.org/abs/2004.13324) [**[code]**](https://github.com/qianqianwang68/caps)

- **[ASLFeat: Learning Local Features of Accurate Shape and Localization]** Zixin, Lup. CVPR, 2020. [**[**paper**]**](https://arxiv.org/pdf/2003.10071.pdf) [**[**code**]**](https://github.com/lzx551402/ASLFeat)

- **[DISK: learning local features with policy gradient]** Michał J. Tyszkiewicz. NeurIPS, 2020. [**[**paper**]**](https://arxiv.org/pdf/2006.13566.pdf) [**[**code**]**](https://github.com/cvlab-epfl/disk)

- **[Reinforced Feature Points: Optimizing Feature Detection and Description for a High-Level Task]** Aritra Bhowmik... CVPR oral, 2020. [**[paper]**](https://arxiv.org/abs/1912.00623)[**[**code**]**](https://github.com/aritra0593/Reinforced-Feature-Points)

- **[FisheyeSuperPoint: Keypoint Detection andDescription Network for Fisheye Images]** Anna Konrad. 2021. [**[paper]**](https://arxiv.org/pdf/2103.00191.pdf)

<h1 id="FeatureMatch">Feature Matching</h1>

- **[Learning to Find Good Correspondences]** Kwang Moo Yi. CVPR, 2018, oral. [**[paper]**](https://arxiv.org/abs/1711.05971) [**[code]**](https://github.com/vcg-uvic/learned-correspondence-release)

- **[OANet: Learning Two-View Correspondences and Geometry Using Order-Aware Network]** Zhang, Jiahui and Sun. ICCV, 2019. [**[paper]**](https://arxiv.org/abs/1908.04964) [**[code]**](https://github.com/zjhthu/OANet)

- **[ACNe: Attentive Context Normalization for Robust Permutation Equivariant Learning]** Sun, W. CVPR, 2020. [**[paper]**](https://openaccess.thecvf.com/content_CVPR_2020/papers/Sun_ACNe_Attentive_Context_Normalization_for_Robust_Permutation-Equivariant_Learning_CVPR_2020_paper.pdf) [**[code]**](https://github.com/vcg-uvic/acne)

- **[Is there anything new to say about SIFT matching?]** Fabio Bellavia. IJCV, 2020. [**[paper]**](https://flore.unifi.it/bitstream/2158/1182190/1/ijcv.pdf)

- **[Deep  Keypoint-Based  Camera  Pose  Estimation  with  Geometric Constraints]** You-Yi Jau, Rui Zhu. IROS, 2020 [**[paper]**](https://arxiv.org/pdf/2007.15122.pdf) [**[code]**](https://github.com/eric-yyjau/pytorch-deepFEPE)

- **[SuperGlue: Learning Feature Matching with Graph Neural Networks]** Paul-Edouard Sarlin. CVPR, 2020. [**[paper]**](https://arxiv.org/abs/1911.11763) [**[code]**](https://github.com/magicleap/SuperGluePretrainedNetwork) 

- **[LoFTR: Detector-Free Local Feature Matching with Transformers]** Jiaming Sun, Zehong Shen, Yu'ang Wang. CVPR, 2021. [**[paper]**](https://arxiv.org/pdf/2104.00680.pdf) [**[code]**](https://zju3dv.github.io/loftr)

- **[COTR: Correspondence Transformer for Matching Across Images]** Wei Jiang. ICCV, 2021. [**[paper]**](https://arxiv.org/abs/2103.14167) [**[code]**](https://github.com/ubc-vision/COTR)

- **[Patch2Pix for Accurate Image Correspondence Estimation]** Qunjie Zhou, Torsten Sattle, Laura Leal-Taix ́e. CVPR, 2021. [**[paper]**](https://arxiv.org/pdf/2012.01909.pdf)[**[code]**](https://github.com/GrumpyZhou/patch2pix)

- **[DFM: A Performance Baseline for Deep Feature Matching]** Ufuk Efe, Kutalmis Gokalp Ince, A. Aydin Alatan. CVPR, 2021 [**[paper]**](https://openaccess.thecvf.com/content/CVPR2021W/IMW/papers/Efe_DFM_A_Performance_Baseline_for_Deep_Feature_Matching_CVPRW_2021_paper.pdf)

- **[Back to the Feature: Learning Robust Camera Localization from Pixels to Pose]** Paul-Edouard Sarlin. CVPR, 2021 [**[paper]**](https://arxiv.org/abs/2103.09213) [**[code]**](https://github.com/cvg/pixloc)

- **[Cross-Descriptor Visual Localization and Mapping]** Mihai Dusmanu. ICCV, 2021 [**[paper]**](https://arxiv.org/abs/2012.01377) [**[code]**](https://github.com/mihaidusmanu/cross-descriptor-vis-loc-map)

<h1 id="Retrieval">Retrieval Methods</h1>

- **[Visual Categorization with Bags of Keypoints]** G. Csurka. ECCV, 2004. [**[paper]**](https://www.cs.cmu.edu/~efros/courses/LBMV07/Papers/csurka-eccv-04.pdf)

- **[Total Recall: Automatic Query Expansion with a Generative Feature Model for Object Retrieval]** Chum, O. ICCV, 2007. [**[paper]**](https://www.robots.ox.ac.uk/~vgg/publications/**paper**s/chum07b.pdf)

- **[Fisher Kernels on Visual Vocabularies for Image Categorization]** F. Perronnin and C. Dance. CVPR, 2007. [**[paper]**](https://ieeexplore.ieee.org/document/4270291)

- **[Aggregating Local Descriptors Into a Compact Image Representation]** H. Jegou. CVPR, 2010. [**[paper]**](https://lear.inrialpes.fr/pubs/2010/JDSP10/jegou_compactimagerepresentation.pdf)

- **[Fast image-based localization using direct 2D to-3D matching]** Sattler T. ICCV, 2011. [**[paper]**](https://graphics.rwth-aachen.de/media/**paper**s/sattler_iccv11_preprint_011.pdf) [**[code]**](https://www.graphics.rwth-aachen.de/software/image-localization/)

- **[Improving image-based localization by active correspondence search]** ECCV, 2012. [**[paper]**](https://graphics.rwth-aachen.de/media/**paper**s/sattler_eccv12_preprint_1.pdf)

- **[Aggregating Deep Convolutional Features for Image Retrieval]** A. Babenko and V. Lempitsky. ICCV, 2015. [**[paper]**](https://arxiv.org/abs/1510.07493)

- **[A Vote-and-Verify Strategy for Fast Spatial Verification in Image Retrieval]** Johannes L. Sch¨onberger. ACCV, 2016. [**[paper]**](https://frahm.web.unc.edu/wp-content/uploads/sites/6231/2016/06/schoenberger2016vote.pdf) [**[code]**](https://frahm.web.unc.edu/wp-content/uploads/sites/6231/2016/06/schoenberger2016vote.pdf)

- **[NetVLAD: CNN Architecture for Weakly Supervised Place Recognition]** R. Arandjelovic. CVPR, 2016. [**[paper]**](https://arxiv.org/abs/1511.07247)

- **[Crossdimensional Weighting for Aggregated Deep Convolutional Features]** Y. Kalantidis. ECCV, 2016. [**[paper]**](https://arxiv.org/abs/1512.04065) [**[code]**](https://github.com/yahoo/crow)

- **[Fine-Tuning CNN Image Retrieval with no Human Annotation]** F. Radenovic, G. PAMI, 2017. [**[paper]**](https://arxiv.org/pdf/1711.02512) [**[code]**](https://arxiv.org/pdf/1711.02512)

- **[Efficient diffusion on region manifolds: Recovering small objects with compact cnn representations]** A. Iscen. CVPR, 2017. [**[paper]**](https://arxiv.org/abs/1611.05113)

- **[Revisiting Oxford and Paris: Large-scale Image Retrieval Benchmarking]** F. Radenovic, G. CVPR, 2018.  [**[paper]**](https://arxiv.org/abs/1803.11285)

- **[Learning with Average Precision: Training Image Retrieval with a Listwise Loss]** J. Revaud. ICCV, 2019. [**[paper]**](https://arxiv.org/abs/1906.07589)


- **[Benchmarking Image Retrieval for Visual Localization]** Noé Pion,..., Torsten Sattler. 3DV, 2020. [**[paper]**](https://arxiv.org/abs/2011.11946) <a href="https://github.com/naver/kapture-localization">**[code]**</a> 

<h1 id="Pose">Robust Pose Estimation</h1>

- **[Fixing the Locally Optimized RANSAC]**  Karel Lebeda. BMVC, 2012. [**[paper]**](http://www.bmva.org/bmvc/2012/BMVC/**paper**095/**paper**095.pdf)

- **[Camera Pose Voting for Large-Scale Image-Based Localization]** B. Zeisl, T. Sattler. ICCV, 2015. [**[**paper**]**](https://www.cv-foundation.org/openaccess/content_iccv_2015/papers/Zeisl_Camera_Pose_Voting_ICCV_2015_paper.pdf)

- **[City-Scale Localization for Cameras with Known Vertical Direction]** Linus Svarm. TPAMI, 2016. [**[paper]**](http://www1.maths.lth.se/matematiklth/vision/publdb/reports/pdf/svarm-enqvist-etal-pami-16.pdf) 

- **[DSAC - Differentiable RANSAC for Camera Localization]** E.Brachmann. CVPR, 2017. [**[code]**](https://github.com/cvlab-dresden/DSAC)[**[paper]**](http://www.nowozin.net/sebastian/papers/brachmann2017dsac.pdf)

- **[MAGSAC: marginalizing sample consensus]**  Barath, D. CVPR, 2019. [**[paper]**](https://arxiv.org/abs/1803.07469)

- **[GC-RANSAC: Graph-Cut RANSAC]**  Daniel Barath, Jiri Matas. CVPR, 2020. [**[paper]**](http://openaccess.thecvf.com/content_cvpr_2018/papers/Barath_Graph-Cut_RANSAC_CVPR_2018_paper.pdf)[**[code]**](https://github.com/ducha-aiki/pydegensac)

- **[AdaLAM: Revisiting Handcrafted Outlier Detection]**  Luca Cavalli... ECCV, 2020. [**[paper]**](https://arxiv.org/abs/2006.04250)[**[code]**](https://github.com/cavalli1234/AdaLAM)

- **[DegenSac]** 2021 [**[code]**](https://github.com/ducha-aiki/pydegensac)

- **[Learning Bipartite Graph Matching for Robust Visual Localization]**  Hailin Yu, Weicai Ye. ISMAR, 2020. [**[paper]**](http://www.cad.zju.edu.cn/home/gfzhang/papers/BGNet/BGNet.pdf)


<h1 id="Fusion">Multi-sensors Fusion</h1>

<h2 id="Fusion with IMU">Fusion with IMU</h2>

- **[DARNavi: An Indoor-Outdoor Immersive Navigation System with Augmented Reality]** Didi Chuxing. CVPR, 2020. [**[paper]**](https://**paper**.nweon.com/2688)

<h2 id="Fusion with GPS">Fusion with GPS</h2>

- **[Multi-sensor navigation algorithm using monocular camera, imu and gps for large scale augmented reality]** T. Oskiper. ISMAR, 2012. [**[paper]**](https://ieeexplore.ieee.org/document/6402541)

- **[Gomsf: Graph-optimization based multi-sensor fusion for robust uav pose estimation]** R. Mascaro, L. ICRA 2018. [**[paper]**](https://ieeexplore.ieee.org/document/8460193)

- **[Intermittent GPS-aided VIO: Online Initialization and Calibration]** Woosik Lee. ICRA, 2020. [**[paper]**](https://ieeexplore.ieee.org/document/9197029)

- **[Vins Fusion — A General Optimization-based Framework for Global Pose Estimation with Multiple Sensors]** Tong Qin. [**[paper]**](https://arxiv.org/abs/1901.03642)


<h1 id="SLAM">SLAM</h1>

- **[Towards SLAM-based Outdoor Localization using Poor GPS and 2.5D Building Models]** R.Liu et al. ISMAR, 2019. [**[code]**](https://github.com/luyujiangmiao/Building-GPS-SLAM) [**[paper]**](http://www.arth.co.at/data/papers/ismar19_ruyu.pdf)

- **[Neural Topological SLAM for Visual Navigation]** Devendra Singh Chaplot. CVPR, 2020. [**[paper]**](https://arxiv.org/abs/2005.12256)

- **[TANDEM: Tracking and Dense Mapping in Real-time using Deep Multi-view Stereo]** Lukas Koestler et al. CoRL, 2021. [**[paper]**](https://openreview.net/pdf?id=FzMHiDesj0I) [**[code]**](https://github.com/tum-vision/tandem)

<h1 id="SfM">SfM</h1>

- **[DPSNet: End-to-end Deep Plane Sweep Stereo]** Sunghoon Im. ICLR, 2019. [**[paper]**](https://arxiv.org/abs/1905.00538)

- **[Consistent Video Depth Estimation]** XUAN LUO. SIGGRAPH 2020. [**[paper]**](https://arxiv.org/abs/2004.15021)

- **[DeepSFM: Structure From Motion Via Deep Bundle Adjustment]** ECCV 2020. [**[paper]**](https://arxiv.org/abs/1912.09697)

- **[Multi-View Optimization of Local Feature Geometry]** Mihai Dusmanu et al. ECCV 2020. [**[paper]**](https://arxiv.org/pdf/2003.08348v2.pdf) [**[paper]**](https://github.com/mihaidusmanu/local-feature-refinement)

- **[Deepv2d: Video to depth with differentiable structure from motion]** Zachary Teed, Jia Deng. ICLR, 2020. [**[paper]**](https://iclr.cc/virtual_2020/poster_HJeO7RNKPr.html)

- **[Hybrid Rotation Averaging: A Fast and Robust Rotation Averaging Approach]** Yu Chen. CVPR, 2021. [**[paper]**](https://arxiv.org/pdf/2101.09116.pdf) [**[code]**](https://github.com/AIBluefisher/GraphOptim)

- **[Pixel-Perfect Structure-from-Motion with Featuremetric Refinement]** Philipp Lindenberger,* Paul-Edouard Sarlin,* Viktor Larsson, Marc Pollefeys. ICCV, oral, 2021. [**[paper]**](https://arxiv.org/abs/2108.08291)[**[code]**](https://github.com/cvg/pixel-perfect-sfm)


<h1 id="Wait">Waiting to sort</h1>

- **[FREAK:Fast Retina Keypoint.]** A. Amit. CVPR, 2012.

- **[Three things evereyone should know to improve object retrieval]** R. Arandjelovic. CVPR, 2012. 

- **[Learning local feature descriptors with triplets and shallow convolutional neural networks]** V. Balntas. BMVC, 2016. 

- **[Learning 6D Object Pose Estimation Using 3D Objet Coordinates]** E.Brachmann. ECCV, 2014.

- **[Discriminative Learning of Local Image Descriptors]** TPAMI, 2011.

- **[MatchNet: Unifying feature and metric learning for patch-based matching]** X. Han. CVPR, 2015. 

- **[Comparative evaluation of binary features]** J. Heinly. ECCV. 

- **[LIFT: Learned Invariant Feature Transform]** M.Kwang. ECCV, 2016.
