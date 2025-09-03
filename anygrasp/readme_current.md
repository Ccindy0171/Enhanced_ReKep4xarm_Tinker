# MinkowskiEngine
**一定记得要export MAX_JOBS=2！** 
* OOM导致install失败是小时，管理器未能及时察觉OOM导致电脑卡死需暴力重启是大事
使用CUDA toolkit12.1环境（12.2也许也可以，但>12.4一定会报错）
遵循这份指令：https://github.com/CiSong10/MinkowskiEngine/blob/cuda12-installation/installation_note.md
最后运行`python diagnostics.py`的结果如下：

```
==========System==========
Linux-6.8.0-78-generic-x86_64-with-glibc2.35
DISTRIB_ID=Ubuntu
DISTRIB_RELEASE=22.04
DISTRIB_CODENAME=jammy
DISTRIB_DESCRIPTION="Ubuntu 22.04.3 LTS"
3.10.18 | packaged by conda-forge | (main, Jun  4 2025, 14:45:41) [GCC 13.3.0]
==========Pytorch==========
2.5.1+cu121
torch.cuda.is_available(): True
==========NVIDIA-SMI==========
/usr/bin/nvidia-smi
Driver Version 560.35.03
CUDA Version 12.6
VBIOS Version 94.07.82.40.11
Image Version G001.0000.94.01
GSP Firmware Version 560.35.03
==========NVCC==========
/home/cindy/anaconda3/envs/AnyGrasp/bin/nvcc
nvcc: NVIDIA (R) Cuda compiler driver
Copyright (c) 2005-2023 NVIDIA Corporation
Built on Mon_Apr__3_17:16:06_PDT_2023
Cuda compilation tools, release 12.1, V12.1.105
Build cuda_12.1.r12.1/compiler.32688072_0
==========CC==========
CC=/home/cindy/anaconda3/envs/AnyGrasp/bin/x86_64-conda-linux-gnu-c++
/home/cindy/anaconda3/envs/AnyGrasp/bin/x86_64-conda-linux-gnu-c++
x86_64-conda-linux-gnu-c++ (conda-forge gcc 11.4.0-13) 11.4.0
Copyright (C) 2021 Free Software Foundation, Inc.
This is free software; see the source for copying conditions.  There is NO
warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

==========MinkowskiEngine==========
0.5.4
MinkowskiEngine compiled with CUDA Support: True
NVCC version MinkowskiEngine is compiled: 12010
CUDART version MinkowskiEngine is compiled: 12010
```

且numpy==1.26.4

# AnyGraspSDK
**一定记得要export MAX_JOBS=2！** 
按照 https://zhuanlan.zhihu.com/p/1924881466229233373 本地安装GraspnetAPI后再安装SDK

