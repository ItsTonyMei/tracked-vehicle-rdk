1. For detailed information about the SDK's components, features, and development, please refer to the SDK documentation for the corresponding chip series in the Software Development section of the Docume
   Taking CI13060 as an example, the reference link is as follows: https://document.chipintelli.com/%E8%BD%AF%E4%BB%B6%E5%BC%80%E5%8F%91/SDK/CI130X%E8%8A%AF%E7%89%87SDK/CI-SDK-Offline/

2. If you need to modify and adjust the relevant algorithm parameters in the SDK, please refer to the SDK documentation for the corresponding chip series in the Software Development section of the Documentation Center.
   Taking AEC as an example, the reference link is as follows:：https://document.chipintelli.com/%E8%BD%AF%E4%BB%B6%E5%BC%80%E5%8F%91/SDK/CI130X%E8%8A%AF%E7%89%87SDK/components/%E5%9B%9E%E5%A3%B0%E6%B6%88%E9%99%A4%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E/

3. If the firmware size exceeds the flash capacity of the selected chip, please try again after switching to a smaller acoustic model, reducing the number of command words or voice prompts, or using a chip with larger flash capacity.

4. If algorithms are enabled, the system provides relatively less memory space for language model. To ensure normal recognition operation, please reduce the number of command words.


The firmware communication protocol is as follows:
你好瓦力:A5 FA 00 81 01 00 21 FB:A5 FA 00 82 01 00 22 FB
增大音量:A5 FA 00 81 04 00 24 FB:A5 FA 00 82 04 00 25 FB
减小音量:A5 FA 00 81 05 00 25 FB:A5 FA 00 82 05 00 26 FB
小车停车:A5 FA 00 81 06 00 26 FB:A5 FA 00 82 06 00 27 FB
小车停止:A5 FA 00 81 06 00 26 FB:A5 FA 00 82 06 00 27 FB
小车停止前进:A5 FA 00 81 06 00 26 FB:A5 FA 00 82 06 00 27 FB
小车前进:A5 FA 00 81 07 00 27 FB:A5 FA 00 82 07 00 28 FB
小车向前移动:A5 FA 00 81 07 00 27 FB:A5 FA 00 82 07 00 28 FB
小车后退:A5 FA 00 81 08 00 28 FB:A5 FA 00 82 08 00 29 FB
小车向后移动:A5 FA 00 81 08 00 28 FB:A5 FA 00 82 08 00 29 FB
小车左转:A5 FA 00 81 09 00 29 FB:A5 FA 00 82 09 00 2A FB
小车左转弯:A5 FA 00 81 09 00 29 FB:A5 FA 00 82 09 00 2A FB
小车右转:A5 FA 00 81 0A 00 2A FB:A5 FA 00 82 0A 00 2B FB
小车右转弯:A5 FA 00 81 0A 00 2A FB:A5 FA 00 82 0A 00 2B FB
小车左旋:A5 FA 00 81 0B 00 2B FB:A5 FA 00 82 0B 00 2C FB
小车逆时针转:A5 FA 00 81 0B 00 2B FB:A5 FA 00 82 0B 00 2C FB
小车右旋:A5 FA 00 81 0C 00 2C FB:A5 FA 00 82 0C 00 2D FB
小车顺时针转:A5 FA 00 81 0C 00 2C FB:A5 FA 00 82 0C 00 2D FB
跟我走:A5 FA 00 81 0D 00 2D FB:A5 FA 00 82 0D 00 2E FB
跟过来:A5 FA 00 81 0D 00 2D FB:A5 FA 00 82 0D 00 2E FB
开启跟随:A5 FA 00 81 0D 00 2D FB:A5 FA 00 82 0D 00 2E FB
启动跟随:A5 FA 00 81 0D 00 2D FB:A5 FA 00 82 0D 00 2E FB
别跟我:A5 FA 00 81 0E 00 2E FB:A5 FA 00 82 0E 00 2F FB
不要跟:A5 FA 00 81 0E 00 2E FB:A5 FA 00 82 0E 00 2F FB
关闭跟随:A5 FA 00 81 0E 00 2E FB:A5 FA 00 82 0E 00 2F FB
结束跟随:A5 FA 00 81 0E 00 2E FB:A5 FA 00 82 0E 00 2F FB
<欢迎语>:A5 FA 00 81 02 00 22 FB:A5 FA 00 82 02 00 23 FB
<休息语>:A5 FA 00 81 03 00 23 FB:A5 FA 00 82 03 00 24 FB
