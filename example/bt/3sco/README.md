# BT 3SCO示例

源码路径: example/bt/3sco

## 概述
本工程用于同时建立3条eSCO链路的示例，目前只有58x支持3SCO
## 用法

### 编译方法
- 编译方法同其他common 工程，scons --board=xxxx  -j8

### 支持情况说明
- 该工程当然也能编译非58x的board，只是只支持1个eSCO

### L2CAP Profile操作
- 上层采用自定义L2CAP profile，在btskey main Menu下输入btskey a进入bt_l2cap_profile Menu

- 进入bt_l2cap_profile Menu后需要先输入btskey 1注册L2CAP profile，才能进行后续的建立ACL/SCO链接

### eSCO数据通路
- 3条eSCO的数据通路，目前只有第一条建立的eSCO通路的数据跟本地的音频通路进行了对接，后面建立的两条eSCO的数据默认采用回环的方式发送回对方

  ​    

