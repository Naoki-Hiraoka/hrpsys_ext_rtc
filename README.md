# hrpsys_ext_rtc

This repository is deprecated and for choreonoid Release-1.7.

For choreonoid Release-2.1, use https://github.com/Naoki-Hiraoka/hrpsys_ext_rtc2

このrtcを使用するためには、[hrpsys_tools](http://wiki.ros.org/hrpsys_tools)は外部のRTCをプラグインとしてロードする機能に乏しいので、代わりに[rtcloader](https://github.com/Naoki-Hiraoka/rtcloader)を使用してください。

- SoftErrorLimiter2: https://github.com/fkanehiro/hrpsys-base/tree/master/rtc/SoftErrorLimiter とは、https://github.com/fkanehiro/hrpsys-base/pull/1294 を反映していて、choreonoidモデルを使用している点が異なる.
- CollisionDetector2: https://github.com/fkanehiro/hrpsys-base/tree/master/rtc/CollisionDetector とは、vclip以外の干渉検出ライブラリを利用可能な点(特に非凸形状を扱える点)と、choreonoidモデルを使用している点が異なる.