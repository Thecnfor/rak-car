# Frame.msg
#
# 一个完整 MC602 串口帧(0x77 0x68 | len | payload | 0x0A)。
# 用于 Mc602Transaction.srv 的批量事务:请求/响应各携带 N 个帧,
# 桥节点逐帧执行、响应按序对应。不跨帧重编码 —— 帧格式保持单帧原样。

uint8[] data
