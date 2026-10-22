独立推理接口测试
================
入口：/home/nvidia/agile/fly/test_inference_link.sh
实现：/home/nvidia/agile/fly/inference_link_probe.py

脚本只读取输入并发出一次HTTP推理请求；不切换飞控模式、不解锁、不发布控制目标，
不启动或停止ROS/Odin/MAVROS。现有test_model_chain.sh属于完整控制链测试，本脚本独立运行。
按用户要求，交付时未运行本脚本或自动发送推理请求；实际测试由用户执行。

一、先检查服务端口（不需要ROS或图片）

cd /home/nvidia/agile
bash ./fly/test_inference_link.sh --check-connection

默认服务端：http://192.168.1.10:5008/predict
此模式只检查TCP，成功不代表模型已成功推理。
ping通但connection_failed/Connection refused，仍需检查服务是否监听5008及监听地址/防火墙。
最近一次已有诊断记录显示该主机可ping通，但5008端口拒绝连接；交付脚本时未重新测试。

二、发送当前ROS相机图像

bash ./fly/test_inference_link.sh --instruction "fly to the chair"

前提：已有ROS master和Odin图像发布者。
默认图像话题：/uav1/odin1/image/compressed
图像必须有有效时间戳且在0.5秒内，最多等待8秒。
主节点不存在时明确报ros_unavailable，不无限等待注册。
图像发布者不存在时报no_image_publisher；设备占用/未出图/时间戳异常会导致image_timeout。
可用--image-topic、--master-uri指定其他已有话题或主节点。
脚本不会因没有实时图像而自动用旧图片代替。

三、不依赖ROS，用指定的图片测试推理

bash ./fly/test_inference_link.sh --image /绝对路径/图片.jpg --instruction "fly to the chair"

该方式只需要Pillow和HTTP服务。图片由用户明确选择，报告标记saved_file，不宣称是当前帧。
JPEG/PNG等Pillow支持的格式均可，输入上限8MiB。
如果暂时没有ROS图像，用此方式可单独验证模型服务。

四、位姿与输入协议

与vla_direct_bridge.py一致：
  image：224x224 RGB，JPEG quality=85，再进行Base64编码
  instr：指令文本
  proprio：[x_cm, y_cm, z_cm, yaw_deg]
  request_id：本次请求的毫秒时间编号

默认proprio=[0,0,0,0]，明确以本次观测为新原点，适用于一次独立接口测试；
不冒充飞控绝对坐标，也不沿用上一轮飞行任务原点。
如需指定模型坐标系下的位姿：
bash ./fly/test_inference_link.sh --image /绝对路径/图片.jpg --proprio 10 0 0 30

实时模式若收到与图像时间相差不超过50ms的Odin里程计，会把实际xyz_m、四元数和frame_id
记入报告作为观测证据。它不会隐式改变用户指定的proprio，也不依赖MAVROS。

五、更换服务器/测试已存在的本地转发器

bash ./fly/test_inference_link.sh --url http://服务器IP:端口/predict --image /绝对路径/图片.jpg
bash ./fly/test_inference_link.sh --url http://127.0.0.1:15017/predict --image /绝对路径/图片.jpg

第二条要求转发器已经在运行；本脚本不会自动启动转发器。
HTTP请求不使用环境代理，也不跟随重定向。

六、结果和超时

每次创建独立目录：fly/runtime/inference_link/日期时间_随机编号/
终端最后打印报告的完整路径。

report.json：当前阶段、错误原因、HTTP状态、耗时、图像来源、输入位姿和解析结果。
request.json：实际发送的完整请求，包含Base64图像。
source_image.bin：原始输入图像内容。
model_input.jpg：实际发送的224x224 JPEG。
response_body.txt：服务端原始返回正文，HTTP 4xx/5xx时也保留。
settings.json：本次参数。

网络检查失败时尚未读取图片，所以不会产生request.json或图像文件。
图像采集失败时不会发送POST。错误发生在哪一步，就保留该步之前已取得的证据。

--connect-timeout 3：TCP连接超时，默认3秒。
--capture-timeout 8：ROS图像采集等待时间，另有4秒初始化余量的子进程硬期限。
--http-timeout 20：HTTP等待时间，另有1秒余量的子进程硬期限。
不会自动重试POST；请求超时只说明客户端已停止等待，服务端可能仍在处理该次请求。

完整成功需要HTTP成功、JSON status=success、action和action_ori均为4个有限数值，
并通过与现有桥接相同的坐标组合校验（误差阈值1e-3）。
HTTP 200但返回错误状态、无效JSON或错误坐标时，仍以非零退出码结束并保存具体原因。
成功仅证明这一次图片推理请求及返回格式通过，不能替代控制链、坐标标定或飞行验证。
