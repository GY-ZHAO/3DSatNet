-------------------------------18 March 2022------------------------------------  
## 1、PointNet++ 
Weights_trained_on_ShapeNet shows the [PointNet++](https://github.com/charlesq34/pointnet2) network we used to gain pretrained models.   
The checkpoint floder contains the weights we trained on [ShapeNet](https://shapenet.org/) Dataset.   
Please refer the [PointNet++](https://arxiv.org/abs/1706.02413) article for more details.

## 2、mmdection3D
### I found [mmdetection3D](https://github.com/open-mmlab/mmdetection3d) contain some 3D compoment segmentation backbone, but the dataset they used are ScanNet, SUNRGB-D, Waymo, nuScenes, Lyft, and KITTI.

## 3、3D Part Segmentation on ShapeNet
I found a leadboard : [3D Part Segmentation](https://paperswithcode.com/sota/3d-part-segmentation-on-shapenet-part) on ShapeNet dataset.

|Model|Backbone|Url|Journal|
|:---|:---|:--|:--|
|KPConv|CNN-based|https://github.com/HuguesTHOMAS/KPConv|ICCV 2019|
|GDANet|CNN-based|https://github.com/mutianxu/GDANet|AAAI2021|
|Spherical Kernel|Graph-Based|https://github.com/EnyaHermite/SPH3D-GCN|CVPR2019|
|PointCNN|CNN-Based|https://github.com/yangyanli/PointCNN|NeurIPS 2018 |
|DRNet|CNN-Based|https://github.com/ShiQiu0419/DRNet|WACV 2021 |
|ConvPoint|CNN-Based|https://github.com/aboulch/ConvPoint/tree/master/examples/shapenet|Computers & Graphics 2019|
|SpiderCNN|CNN-Based|https://github.com/xyf513/SpiderCNN|ECCV 2018|
|3DGCN|Graph-Based|https://github.com/zhihao-lin/3dgcn/tree/master/segmentation|CVPR2020|
|Splatnet|CNN-Based|https://github.com/NVlabs/splatnet|CVPR2018|
|Point Transformer|Transformer-Based|https://github.com/engelnico/point-transformer|IEEE Access 2021|
|CurveNet|CNN-Based|https://github.com/tiangexiang/CurveNet#point-cloud-part-segmentation|ICCV 2021|
|PVT|Transformer-Based|https://github.com/HaochengWan/PVT|2021|
|PVCNN volumetric|CNN-Based|https://github.com/Yochengliu/Relation-Shape-CNN|CVPR 2019|
|DGCNN|Graph-Based|https://github.com/WangYueFt/dgcnn|2018|
|PCNN|CNN-Based|https://github.com/matanatz/pcnn|SIGGRAPH 2018|
|SO-Net|Graph-Based|https://github.com/lijx10/SO-Net|CVPR 2018|
|Pointnet++-SSG|MLP-Based|https://github.com/charlesq34/pointnet2|NIPS 2017|
|PointNet++-MSG|MLP-Based|https://github.com/charlesq34/pointnet2|NIPS 2017|
|Pointnet|MLP-Based|https://github.com/charlesq34/pointnet|CVPR2017|

-------------------------------26 March 2022------------------------------------  
We published the code we used to gain lidar point clouds in Lidar_SLAM.  
The code borrows a lot from [Open3d](http://www.open3d.org/docs/release/jupyter/pipelines/multiway_registration.html?highlight=registration) 



## Acknowledgement
Our code borrows a lot from:  
[PointNet++](https://github.com/charlesq34/pointnet2)   
[PointNet](https://github.com/charlesq34/pointnet)  
[Open3d](http://www.open3d.org/)  


## The code is constantly being updated
