# 主要看0.9的计时器版本我写了详细注释
 对智能车的代码进行修改注释，计时器版本是添加了斑马线误识别的计时器和救援版本就多了一个东西
## image2video用于存图合成视频  res文件夹下一定要有train文件夹才能存图命令是
make image2video -j
## camera_display用于打开图像调整前瞻，我们前瞻为1.5
make camera_display -j
## camera_calibration相机标定用于去畸变
make camera_calibration -j
# 运行都是./应用程序名字

# 模型训练的话可以加我qq或者微信具体询问，我们没有改过人工智能模型，也没换过模型学弟学妹们可以尝试一下，误识别的情况很严重特别是斑马线计时器就是为了防斑马线误判。

# 文件最好把 sasu_intelligentcar_fz3b_c++放在workspace里面最好把不要套文件夹否则编译会有问题，或者更改makefile文件也可以
# !!!每次改代码都编译，改motion.json文件里面的值不需要编译

# https://ai.baidu.com/ai-doc/HWCE/Yk3b86gvp  开发平台和系统烧录都在里面根据教程一步一步走就行
