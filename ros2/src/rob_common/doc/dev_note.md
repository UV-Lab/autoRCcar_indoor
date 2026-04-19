# Dev Note

## 环境准备

### 安装protobuf

```shell
apt update 
apt install -y protobuf-c-compiler libprotobuf-dev
apt install -y protobuf-c-compiler protobuf-compiler

```

### 安装iceoryx库

iceoryx是一个cpp的icp库，当前使用的版本为v2.95.0,其编译方法如下：


```shell

git clone https://github.com/eclipse-iceoryx/iceoryx.git
cd iceoryx
git checkout v2.95.0

# 安装依赖库
apt-get install libacl1-dev libssl-dev zlib1g-dev

mkdir build
cd build
cmake -B. -H../iceoryx_meta -DBUILD_ALL=ON
make -j24
make install 

```