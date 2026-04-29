请使用Boost.Interprocess实现共享内存通信的Server与Client，其中类ShmServer为共享内存通信的Server，用于发送信息，类ShmClient为共享内存通信的Client,用于接收信息。
其中ShmServer类的头文件内容如下：
```cpp
namespace robot_bridge {

class ShmServerParams {
public:
    // 根据需要来定义
};

/*
 * 这个一个共享内存通信的Server类，用于发送信息，其中ShmServerParams为共享内存通信的Server的参数类，用于配置共享内存通信的Server的参数。
 * 当有信息需要发送时，调用write函数将信息写入共享内存中，同时使用条件变量通知ShmClient有新的信息到来。一个ShmServer与ShmClient的关系为
 * 一对多的关系，即一个ShmServer可以与多个ShmClient进行通信，当ShmServer发送完信息后，会通知所有的ShmClient有新的信息到来。
 */
class ShmServer {
public:
    ShmServer(std::shared_ptr<ShmServerParams> params_ptr);
    virtual ~ShmServer();

    // 初始化整个Server,如果初始化失败，则返回一个非0值，如果初始化成功，则返回0
    int init();

    // 将需要发送的信息写入共享内存中，同时使用条件变量通知ShmClient有新的信息到来。如果操作成功，则返回0，如果操作失败，则返回一个非0值，
    int write(const std::string &buf_str);
    // int write(std::shared_ptr<MsgType> msg_ptr);
};
}// namespace robot_bridge

```

ShmClient类的头文件内容如下：
```cpp
namespace robot_bridge {

class ShmClientParams {
public:
    // 根据需要来定义
};


class ShmClient {
public:
    ShmClient(std::shared_ptr<ShmClientParams> params_ptr);
    virtual ~ShmClient();

    // 初始化整个Client,如果初始化失败，则返回一个非0值，如果初始化成功，则返回0
    int init();

    // 从共享内存中读取信息，如果操作成功，则返回0，如果操作失败，则返回一个非0值。当调用这个函数时，程序会一直等待，直到有新的信息到来，妆有新的信息到来时，会将信息读取到buf_str中
    int read(std::string &buf_str);
};
}// namespace robot_bridge
```