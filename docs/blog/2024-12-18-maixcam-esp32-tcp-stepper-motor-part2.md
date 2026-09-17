# Maixcam Pro 与 ESP32 通过 socket TCP 通信来控制步进电机（二）

> 原文发布于 CSDN（2024-12-18）：<https://blog.csdn.net/2401_84550508/article/details/144566530>
> 本文由 CSDN 迁移至 GitHub。

## 前言

上文通过 socket TCP 通信，实现 Maixcam 与 ESP32 之间相互发送信息的功能，本文主要讲述 **Maixcam 端** 的主要操作和代码（Maixcam 作为 TCP 服务器，向 ESP32 发送数据）。

可拓展功能：Maixcam 识别物体坐标信息和种类后，传输给 ESP32，ESP32 控制步进电机带动机械臂到物体上方实现抓取功能……

## 一、代码及操作

这段代码通过在 Maixcam 上创建一个简单的服务器，在指定的 IP 地址和端口上接收客户端连接，并向连接的 ESP32 客户端发送一组数据。

### （一）具体步骤

#### 1. 导入必要模块
```python
import socket
import threading
import time
```

#### 2. 服务器配置
`local_ip` 和 `local_port` 定义了服务器监听的 IP 地址和端口。`data_set` 是一个包含四个字符串的列表，每个字符串由三个数字组成，以逗号分隔。
```python
local_ip = "192.168.10.5"
local_port = 8080

# 数据集，包含四个由三个数字构成的组合
data_set = [
    "2560,2560,1",
    "2560,2560,2",
    "2560,2560,3",
    "2560,2560,4",
]
```
> 这里为了方便调试只是简单列了四组数据，实际上可以用 Maixcam 的识别功能进行拓展，如：识别完物体种类、尺寸以及坐标后，发送给 ESP32，ESP32 控制电机驱动机械臂实现对物体的精准抓取（多个物体可以考虑优先级）。

#### 3. 接收线程函数 receiveThread
负责处理来自单个客户端的连接：遍历 `data_set` 逐条发送；每条发送后等待 18 秒再尝试从客户端接收数据；若收到数据则打印，否则继续发送下一条；异常则打印错误；最后关闭连接。
```python
def receiveThread(conn, addr):
    try:
        for message in data_set:
            # 发送数据给客户端
            conn.sendall(message.encode('utf-8') + b"\n")
            print(f"Sent {message} to {addr}")
            # 等待一段时间再尝试接收来自客户端的数据
            time.sleep(18)
            # 尝试接收来自客户端的数据
            client_data = conn.recv(1024)
            if client_data:
                print(f"Received {client_data.decode('utf-8')} from {addr}")
                print("Received!")
            else:
                print("No data received from client, continuing to send next message.")
    except Exception as e:
        print(f"Error in communication with {addr}: {e}")
    finally:
        print(f"Client {addr} disconnected")
        conn.close()
```

#### 4. 服务器设置和监听
```python
ip_port = (local_ip, local_port)
sk = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sk.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sk.bind(ip_port)
sk.listen(50)
```

#### 5. 主循环
无限循环等待客户端连接；每接受一个新的客户端连接，起一个守护线程运行 `receiveThread`。
```python
print("Server is listening on", ip_port)
while True:
    conn, addr = sk.accept()
    print(f"Client {addr} connected")
    t = threading.Thread(target=receiveThread, args=(conn, addr))
    t.daemon = True
    t.start()
```

#### 6. 完整代码
```python
import socket
import threading
import time

local_ip = "192.168.10.5"
local_port = 8080

data_set = [
    "2560,2560,1",
    "2560,2560,2",
    "2560,2560,3",
    "2560,2560,4",
]

def receiveThread(conn, addr):
    try:
        for message in data_set:
            # 发送数据给客户端
            conn.sendall(message.encode('utf-8') + b"\n")
            print(f"Sent {message} to {addr}")
            # 等待一段时间再尝试接收来自客户端的数据
            time.sleep(18)
            # 尝试接收来自客户端的数据
            client_data = conn.recv(1024)
            if client_data:
                print(f"Received {client_data.decode('utf-8')} from {addr}")
                print("Received!")
            else:
                print("No data received from client, continuing to send next message.")
    except Exception as e:
        print(f"Error in communication with {addr}: {e}")
    finally:
        print(f"Client {addr} disconnected")
        conn.close()

ip_port = (local_ip, local_port)
sk = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sk.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sk.bind(ip_port)
sk.listen(50)

print("Server is listening on", ip_port)
while True:
    conn, addr = sk.accept()
    print(f"Client {addr} connected")
    t = threading.Thread(target=receiveThread, args=(conn, addr))
    t.daemon = True
    t.start()
```

### （二）注意事项
1. 服务器在发送完 `data_set` 中的所有数据后，不会主动关闭连接。如果客户端没有发送任何数据或关闭连接，服务器将无限期地等待下一个 `recv` 调用接收数据，可能导致资源占用；
2. 代码中没有处理客户端发送的数据，只是简单地检查是否收到了数据。根据实际需求，可能需要添加逻辑来处理客户端的响应；
3. 服务器没有实现任何形式的并发控制或数据完整性检查；
4. 硬编码的 IP 地址和端口可能限制了服务器的灵活性；
5. Maixcam 在重新连接 Wi-Fi 以传输代码时，会出现 IP 改变的情况，需同步修改目标 IP，否则 ESP32 将无法连接到 Maixcam！

## 三、总结
以上就是 Maixcam 和 ESP32 通信之间有关 Maixcam 的内容。本文仅仅简单介绍了 TCP 通信、ESP32 与 Maixcam 相互通信的功能。如有错误，敬请批评指正！
